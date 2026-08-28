#include <string.h>
#include <stddef.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "espnow_link.h"
#include "config_link_proto.h"
#include "link_secrets.h"
#include "basestation_config.h"
#include "store.h"

static const char *TAG = "ESPNOW_LINK";

static uint8_t s_fc_mac[6] = CONFIG_LINK_PEER_MAC;

// Guards a single send-and-wait-for-ack exchange at a time. There's only
// ever one basestation user, so no need for anything fancier than "one
// outstanding request" -- see BASESTATION_SPEC.md's Concurrency section.
static SemaphoreHandle_t s_ack_sem;
static volatile uint8_t s_waiting_seq;      // 0..255; only meaningful while s_awaiting_ack
static volatile bool s_awaiting_ack;
static volatile cl_ack_status_t s_last_ack_status;

// Serializes whole sync passes so the periodic background retry and an
// explicit "sync now" request never interleave their sends.
static SemaphoreHandle_t s_sync_mutex;

static TaskHandle_t s_sync_task_handle;
static uint8_t s_next_seq;

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
    if (len != (int)sizeof(cl_ack_t)) {
        return;
    }
    const cl_ack_t *ack = (const cl_ack_t *)data;
    if (ack->hdr.magic != CONFIG_LINK_PROTO_MAGIC || ack->hdr.type != (uint8_t)CL_PKT_ACK) {
        return;
    }
    if (!s_awaiting_ack || ack->hdr.seq != s_waiting_seq) {
        return; // stray/late ack -- we've already moved on, ignore
    }
    s_last_ack_status = (cl_ack_status_t)ack->status;
    xSemaphoreGive(s_ack_sem);
}

// Sends `payload` (already-populated header included) and waits up to
// ESPNOW_LINK_ACK_TIMEOUT_MS for the matching ack. Returns true and fills
// *out_status if an ack for this exact seq arrived in time; returns false
// (leave the item pending, try again later) on send failure or timeout --
// both are the FC's link simply not being open right now, not an error.
static bool send_and_wait(const void *payload, size_t len, uint8_t seq, cl_ack_status_t *out_status) {
    xSemaphoreTake(s_ack_sem, 0); // drain any stale/unconsumed signal
    s_waiting_seq = seq;
    s_awaiting_ack = true;

    esp_err_t ret = esp_now_send(s_fc_mac, (const uint8_t *)payload, len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_send failed: %d", ret);
        s_awaiting_ack = false;
        return false;
    }

    bool got = xSemaphoreTake(s_ack_sem, pdMS_TO_TICKS(ESPNOW_LINK_ACK_TIMEOUT_MS)) == pdTRUE;
    s_awaiting_ack = false;
    if (!got) {
        return false; // FC unreachable right now -- normal, not an error
    }
    *out_status = s_last_ack_status;
    return true;
}

static const char *pid_target_name(cl_pid_target_t t) {
    switch (t) {
        case CL_PID_ROLL:     return "roll";
        case CL_PID_PITCH:    return "pitch";
        case CL_PID_HEADING:  return "heading";
        case CL_PID_AIRSPEED: return "airspeed";
        default:              return "?";
    }
}

static void sync_pass(void) {
    xSemaphoreTake(s_sync_mutex, portMAX_DELAY);

    cl_waypoint_t pts[CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET];
    size_t count;
    bool loop;
    if (store_mission_is_pending(pts, &count, &loop)) {
        cl_set_mission_t pkt = {0};
        uint8_t seq = s_next_seq++;
        pkt.hdr.magic = CONFIG_LINK_PROTO_MAGIC;
        pkt.hdr.type = (uint8_t)CL_PKT_SET_MISSION;
        pkt.hdr.seq = seq;
        pkt.hdr.count = (uint16_t)count;
        pkt.loop = loop ? 1 : 0;
        memcpy(pkt.waypoints, pts, count * sizeof(cl_waypoint_t));
        // NOT sizeof(cl_hdr_t) -- the loop byte sits between hdr and
        // waypoints[], see shared/config_link_proto.h's cl_set_mission_t.
        size_t wire_len = offsetof(cl_set_mission_t, waypoints) + count * sizeof(cl_waypoint_t);

        cl_ack_status_t status;
        if (send_and_wait(&pkt, wire_len, seq, &status)) {
            ESP_LOGI(TAG, "mission (%u pts, loop=%d): %s", (unsigned)count, (int)loop, status == CL_ACK_OK ? "confirmed" : "rejected");
            store_mark_mission_result(status == CL_ACK_OK ? STORE_STATUS_CONFIRMED : STORE_STATUS_FAILED, status);
        } else {
            ESP_LOGD(TAG, "mission send: no ack (FC unreachable), staying pending");
        }
    }

    for (cl_pid_target_t target = CL_PID_ROLL; target <= CL_PID_AIRSPEED; target++) {
        cl_pid_gains_t gains;
        uint8_t mask = store_pid_pending_mask(target, &gains);
        if (mask == 0) {
            continue;
        }

        cl_set_pid_t pkt = {0};
        uint8_t seq = s_next_seq++;
        pkt.hdr.magic = CONFIG_LINK_PROTO_MAGIC;
        pkt.hdr.type = (uint8_t)CL_PKT_SET_PID;
        pkt.hdr.seq = seq;
        pkt.hdr.count = 0;
        pkt.target = (uint8_t)target;
        pkt.fields_present = mask;
        pkt.gains = gains;

        cl_ack_status_t status;
        if (send_and_wait(&pkt, sizeof(pkt), seq, &status)) {
            ESP_LOGI(TAG, "pid %s (fields 0x%x): %s", pid_target_name(target), mask,
                     status == CL_ACK_OK ? "confirmed" : "rejected");
            store_mark_pid_result(target, mask, status == CL_ACK_OK ? STORE_STATUS_CONFIRMED : STORE_STATUS_FAILED, status);
        } else {
            ESP_LOGD(TAG, "pid %s send: no ack (FC unreachable), staying pending", pid_target_name(target));
        }
    }

    xSemaphoreGive(s_sync_mutex);
}

static void sync_task(void *arg) {
    (void)arg;
    while (1) {
        // Wakes early on espnow_link_kick_async(), otherwise retries on its
        // own cadence -- covers the case where the FC's link gate opens
        // with no new local edit to trigger a kick.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ESPNOW_LINK_RETRY_PERIOD_MS));
        sync_pass();
    }
}

void espnow_link_init(void) {
    s_ack_sem = xSemaphoreCreateBinary();
    s_sync_mutex = xSemaphoreCreateMutex();

    // WiFi/netif/event-loop bring-up already happened in wifi_ap_init() --
    // this only adds ESP-NOW on top of the already-running STA interface.
    ESP_ERROR_CHECK(esp_now_init());

    uint8_t pmk[16] = CONFIG_LINK_PMK;
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));

    esp_now_peer_info_t peer = {0};
    uint8_t lmk[16] = CONFIG_LINK_LMK;
    memcpy(peer.peer_addr, s_fc_mac, 6);
    memcpy(peer.lmk, lmk, 16);
    peer.channel = CONFIG_LINK_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = true;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    xTaskCreate(sync_task, "espnow_sync", 4096, NULL, 3, &s_sync_task_handle);

    ESP_LOGI(TAG, "ESP-NOW link up (never gated -- basestation doesn't fly). "
                   "FC peer MAC %02x:%02x:%02x:%02x:%02x:%02x",
             s_fc_mac[0], s_fc_mac[1], s_fc_mac[2], s_fc_mac[3], s_fc_mac[4], s_fc_mac[5]);
}

void espnow_link_kick_async(void) {
    if (s_sync_task_handle != NULL) {
        xTaskNotifyGive(s_sync_task_handle);
    }
}

void espnow_link_sync_now(void) {
    sync_pass();
}
