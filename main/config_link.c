#include <string.h>
#include <stddef.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "pwm_output.h"
#include "config_link.h"
#include "config_link_proto.h"
#include "config_link_secrets.h"
#include "config_store.h"
#include "nav.h"

static const char *TAG = "CONFIG_LINK";

// How long the gate's conditions must hold continuously before RE-opening a
// closed link. Does not apply to the very first open at boot (see
// config_link.h). Placeholder, bench-tunable like other timing constants in
// this codebase.
#define CONFIG_LINK_OPEN_DEBOUNCE_US ((int64_t)30 * 1000000)

// How far above SERVO_MIN_PULSEWIDTH_US the throttle channel can sit and
// still count as "idle". Placeholder.
#define CONFIG_LINK_IDLE_THROTTLE_MARGIN_US (50)

// GPS-reported speed is noisy even when genuinely stationary -- consecutive-
// fix position jitter alone can show up as an apparent drift of a knot or
// more at rest. A near-zero threshold in the tenths-of-a-knot range would
// spuriously read as "moving" from that noise alone and block re-opening
// after a real landing, so this needs real headroom: multiple m/s, not a
// tiny fraction. Placeholder, bench-tunable from real GPS behavior.
#define CONFIG_LINK_GPS_IDLE_SPEED_MPS (2.5f)
#define KNOTS_TO_MPS (0.514444f)

// Loose sanity bound on an individual PID gain field -- rejects obviously
// corrupted/garbage floats (NaN, Inf, wild magnitudes from a bad packet),
// not a judgment about what's a "reasonable" tuning value; that's left to
// the human sending the update.
#define CONFIG_LINK_PID_GAIN_ABS_MAX (10000.0f)

#define CONFIG_LINK_RECV_QUEUE_DEPTH (4)

extern bool set_roll_pid_gains(uint8_t fields_present, const cl_pid_gains_t *g);
extern bool set_pitch_pid_gains(uint8_t fields_present, const cl_pid_gains_t *g);
extern bool set_airspeed_pid_gains(uint8_t fields_present, const cl_pid_gains_t *g);

typedef struct {
    size_t len;
    uint8_t data[sizeof(cl_set_mission_t)]; // largest packet type
} cl_queued_pkt_t;

static QueueHandle_t recv_queue;
static volatile bool link_active = false;
static int64_t conditions_met_since_us = 0;

static void send_ack(const uint8_t *peer_mac, uint8_t seq, cl_ack_status_t status) {
    cl_ack_t ack = {
        .hdr = { .magic = CONFIG_LINK_PROTO_MAGIC, .type = CL_PKT_ACK, .seq = seq, .count = 0 },
        .status = (uint8_t)status,
    };
    esp_err_t ret = esp_now_send(peer_mac, (const uint8_t *)&ack, sizeof(ack));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to send ack: %d", ret);
    }
}

static bool gain_ok(uint8_t fields_present, uint8_t bit, float v) {
    if (!(fields_present & bit)) {
        return true; // field not present, nothing to validate
    }
    return isfinite(v) && fabsf(v) <= CONFIG_LINK_PID_GAIN_ABS_MAX;
}

static void handle_set_pid(const uint8_t *peer_mac, const cl_set_pid_t *pkt, size_t len) {
    if (len != sizeof(cl_set_pid_t)) {
        send_ack(peer_mac, pkt->hdr.seq, CL_ACK_BAD_LEN);
        return;
    }
    if (pkt->target > CL_PID_AIRSPEED) {
        send_ack(peer_mac, pkt->hdr.seq, CL_ACK_OUT_OF_RANGE);
        return;
    }
    const cl_pid_gains_t *g = &pkt->gains;
    bool ok = gain_ok(pkt->fields_present, CL_FIELD_KP, g->k_p)
        && gain_ok(pkt->fields_present, CL_FIELD_KI, g->k_i)
        && gain_ok(pkt->fields_present, CL_FIELD_KD, g->k_d)
        && gain_ok(pkt->fields_present, CL_FIELD_ILIMIT, g->i_limit);
    if (!ok) {
        send_ack(peer_mac, pkt->hdr.seq, CL_ACK_OUT_OF_RANGE);
        return;
    }

    bool persisted;
    switch ((cl_pid_target_t)pkt->target) {
        case CL_PID_ROLL:     persisted = set_roll_pid_gains(pkt->fields_present, g); break;
        case CL_PID_PITCH:    persisted = set_pitch_pid_gains(pkt->fields_present, g); break;
        case CL_PID_HEADING:  persisted = nav_set_heading_pid_gains(pkt->fields_present, g); break;
        case CL_PID_AIRSPEED: persisted = set_airspeed_pid_gains(pkt->fields_present, g); break;
        default:
            send_ack(peer_mac, pkt->hdr.seq, CL_ACK_OUT_OF_RANGE);
            return;
    }
    send_ack(peer_mac, pkt->hdr.seq, persisted ? CL_ACK_OK : CL_ACK_NVS_WRITE_FAILED);
}

static void handle_set_mission(const uint8_t *peer_mac, const cl_set_mission_t *pkt, size_t len) {
    size_t expected_len = offsetof(cl_set_mission_t, waypoints) + (size_t)pkt->hdr.count * sizeof(cl_waypoint_t);
    if (pkt->hdr.count > CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET || pkt->hdr.count > NAV_MAX_WAYPOINTS
        || len != expected_len) {
        send_ack(peer_mac, pkt->hdr.seq, CL_ACK_BAD_LEN);
        return;
    }

    waypoint_t wps[CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET];
    for (uint16_t i = 0; i < pkt->hdr.count; i++) {
        double lat = pkt->waypoints[i].lat_deg;
        double lon = pkt->waypoints[i].lon_deg;
        if (!isfinite(lat) || !isfinite(lon) || lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            send_ack(peer_mac, pkt->hdr.seq, CL_ACK_OUT_OF_RANGE);
            return;
        }
        wps[i].lat_deg = lat;
        wps[i].lon_deg = lon;
    }

    bool persisted = nav_set_mission(wps, pkt->hdr.count, pkt->loop != 0);
    send_ack(peer_mac, pkt->hdr.seq, persisted ? CL_ACK_OK : CL_ACK_NVS_WRITE_FAILED);
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len <= 0 || (size_t)len > sizeof(cl_set_mission_t)) {
        return; // can't safely copy, drop
    }
    cl_queued_pkt_t item;
    item.len = (size_t)len;
    memcpy(item.data, data, item.len);
    // Non-blocking: this runs in the WiFi driver's own task, not a context
    // to block in. Drop on a full queue rather than stall the radio.
    if (xQueueSend(recv_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "recv queue full, dropping packet");
    }
    (void)info;
}

static void config_link_task(void *arg) {
    cl_queued_pkt_t item;
    while (1) {
        if (xQueueReceive(recv_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (item.len < sizeof(cl_hdr_t)) {
            continue; // too short to even have a header, nothing to ack
        }
        const cl_hdr_t *hdr = (const cl_hdr_t *)item.data;

        uint8_t peer_mac[6] = CONFIG_LINK_PEER_MAC;

        if (hdr->magic != CONFIG_LINK_PROTO_MAGIC) {
            send_ack(peer_mac, hdr->seq, CL_ACK_BAD_MAGIC);
            continue;
        }

        switch ((cl_pkt_type_t)hdr->type) {
            case CL_PKT_SET_PID:
                handle_set_pid(peer_mac, (const cl_set_pid_t *)item.data, item.len);
                break;
            case CL_PKT_SET_MISSION:
                handle_set_mission(peer_mac, (const cl_set_mission_t *)item.data, item.len);
                break;
            default:
                send_ack(peer_mac, hdr->seq, CL_ACK_BAD_TYPE);
                break;
        }
    }
}

static void config_link_bringup(void) {
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LINK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());

    uint8_t pmk[16] = CONFIG_LINK_PMK;
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));

    esp_now_peer_info_t peer = {0};
    uint8_t peer_mac[6] = CONFIG_LINK_PEER_MAC;
    uint8_t lmk[16] = CONFIG_LINK_LMK;
    memcpy(peer.peer_addr, peer_mac, 6);
    memcpy(peer.lmk, lmk, 16);
    peer.channel = CONFIG_LINK_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = true;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "config-link: OPEN");
}

static void config_link_teardown(void) {
    esp_now_deinit();
    // esp_now_deinit() alone can leave the WiFi driver initialized without
    // actually quiescing the radio -- esp_wifi_stop() is what gets full RF
    // silence while flying, which is the actual point of this gate.
    esp_wifi_stop();
    ESP_LOGI(TAG, "config-link: CLOSED");
}

void config_link_init(void) {
    recv_queue = xQueueCreate(CONFIG_LINK_RECV_QUEUE_DEPTH, sizeof(cl_queued_pkt_t));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    xTaskCreate(config_link_task, "config_link_task", 4096, NULL, 3, NULL);

    // Open immediately at boot -- see the reasoning in config_link.h.
    config_link_bringup();
    link_active = true;
    conditions_met_since_us = esp_timer_get_time();
}

void config_link_update_gate(bool want_autonomous, uint32_t throttle_pulse_us,
                              bool gps_fix_valid, float gps_speed_kts) {
    bool idle_throttle = throttle_pulse_us <= (SERVO_MIN_PULSEWIDTH_US + CONFIG_LINK_IDLE_THROTTLE_MARGIN_US);
    bool speed_ok = !gps_fix_valid || (gps_speed_kts * KNOTS_TO_MPS <= CONFIG_LINK_GPS_IDLE_SPEED_MPS);
    bool met = !want_autonomous && idle_throttle && speed_ok;

    if (!met) {
        conditions_met_since_us = 0; // no debounce on the closing edge
        if (link_active) {
            config_link_teardown();
            link_active = false;
        }
        return;
    }

    if (conditions_met_since_us == 0) {
        conditions_met_since_us = esp_timer_get_time();
    }
    if (!link_active && (esp_timer_get_time() - conditions_met_since_us) >= CONFIG_LINK_OPEN_DEBOUNCE_US) {
        config_link_bringup();
        link_active = true;
    }
}
