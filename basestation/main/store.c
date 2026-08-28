#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "store.h"

static const char *TAG = "STORE";

#define STORE_NVS_NAMESPACE "bscfg"

// NVS keys. Kept short -- NVS keys are capped at 15 chars.
#define KEY_MISSION_N    "m_n"
#define KEY_MISSION_PTS  "m_pts"
#define KEY_MISSION_ST   "m_st"
#define KEY_MISSION_FR   "m_fr"
#define KEY_MISSION_LOOP "m_lp"

typedef struct {
    const char *prefix;
} pid_key_info_t;

// One short prefix per PID target, used to build this target's NVS keys
// (e.g. "rl_g", "rl_st", "rl_fr" for roll).
static const char *s_pid_key_prefix[4] = { "rl", "pt", "hd", "as" };

static SemaphoreHandle_t s_mutex;
static store_mission_t s_mission;
static store_pid_target_t s_pid[4];

static void lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mutex); }

static bool waypoint_valid(const cl_waypoint_t *w) {
    return isfinite(w->lat_deg) && isfinite(w->lon_deg)
        && w->lat_deg >= -90.0 && w->lat_deg <= 90.0
        && w->lon_deg >= -180.0 && w->lon_deg <= 180.0;
}

// ---- NVS load/save helpers (best-effort: log and continue on failure --
// the in-memory state, already updated by the caller, remains the live
// source of truth for the rest of this boot regardless) ----

static void load_mission_from_nvs(void) {
    nvs_handle_t h;
    if (nvs_open(STORE_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return; // nothing stored yet -- keep zeroed/UNSET defaults
    }
    uint8_t count = 0;
    size_t len = sizeof(count);
    if (nvs_get_blob(h, KEY_MISSION_N, &count, &len) == ESP_OK
        && len == sizeof(count) && count <= CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET) {
        size_t pts_len = (size_t)count * sizeof(cl_waypoint_t);
        if (count == 0 || (nvs_get_blob(h, KEY_MISSION_PTS, s_mission.points, &pts_len) == ESP_OK
                            && pts_len == (size_t)count * sizeof(cl_waypoint_t))) {
            s_mission.count = count;
        }
    }
    uint8_t st = STORE_STATUS_UNSET, fr = CL_ACK_OK;
    size_t st_len = sizeof(st), fr_len = sizeof(fr);
    if (nvs_get_blob(h, KEY_MISSION_ST, &st, &st_len) == ESP_OK && st_len == sizeof(st)) {
        s_mission.state.status = (store_status_t)st;
    }
    if (nvs_get_blob(h, KEY_MISSION_FR, &fr, &fr_len) == ESP_OK && fr_len == sizeof(fr)) {
        s_mission.state.fail_reason = (cl_ack_status_t)fr;
    }
    uint8_t loop = 0;
    size_t loop_len = sizeof(loop);
    if (nvs_get_blob(h, KEY_MISSION_LOOP, &loop, &loop_len) == ESP_OK && loop_len == sizeof(loop)) {
        s_mission.loop = (loop != 0);
    }
    nvs_close(h);
}

static esp_err_t save_mission_to_nvs(void) {
    nvs_handle_t h;
    esp_err_t ret = nvs_open(STORE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for mission: %d", ret);
        return ret;
    }
    uint8_t count = (uint8_t)s_mission.count;
    uint8_t st = (uint8_t)s_mission.state.status;
    uint8_t fr = (uint8_t)s_mission.state.fail_reason;
    uint8_t loop = s_mission.loop ? 1 : 0;
    ret = nvs_set_blob(h, KEY_MISSION_N, &count, sizeof(count));
    if (ret == ESP_OK) {
        ret = nvs_set_blob(h, KEY_MISSION_PTS, s_mission.points, count * sizeof(cl_waypoint_t));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_blob(h, KEY_MISSION_ST, &st, sizeof(st));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_blob(h, KEY_MISSION_FR, &fr, sizeof(fr));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_blob(h, KEY_MISSION_LOOP, &loop, sizeof(loop));
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist mission: %d", ret);
    }
    return ret;
}

static void load_pid_from_nvs(cl_pid_target_t target) {
    nvs_handle_t h;
    if (nvs_open(STORE_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    char key[16];
    snprintf(key, sizeof(key), "%s_g", s_pid_key_prefix[target]);
    size_t len = sizeof(cl_pid_gains_t);
    nvs_get_blob(h, key, &s_pid[target].gains, &len);

    uint8_t st[STORE_PID_FIELD_COUNT];
    snprintf(key, sizeof(key), "%s_st", s_pid_key_prefix[target]);
    len = sizeof(st);
    if (nvs_get_blob(h, key, st, &len) == ESP_OK && len == sizeof(st)) {
        for (int i = 0; i < STORE_PID_FIELD_COUNT; i++) {
            s_pid[target].field[i].status = (store_status_t)st[i];
        }
    }

    uint8_t fr[STORE_PID_FIELD_COUNT];
    snprintf(key, sizeof(key), "%s_fr", s_pid_key_prefix[target]);
    len = sizeof(fr);
    if (nvs_get_blob(h, key, fr, &len) == ESP_OK && len == sizeof(fr)) {
        for (int i = 0; i < STORE_PID_FIELD_COUNT; i++) {
            s_pid[target].field[i].fail_reason = (cl_ack_status_t)fr[i];
        }
    }
    nvs_close(h);
}

static esp_err_t save_pid_to_nvs(cl_pid_target_t target) {
    nvs_handle_t h;
    esp_err_t ret = nvs_open(STORE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for pid target %d: %d", target, ret);
        return ret;
    }
    char key[16];
    snprintf(key, sizeof(key), "%s_g", s_pid_key_prefix[target]);
    ret = nvs_set_blob(h, key, &s_pid[target].gains, sizeof(cl_pid_gains_t));

    if (ret == ESP_OK) {
        uint8_t st[STORE_PID_FIELD_COUNT];
        for (int i = 0; i < STORE_PID_FIELD_COUNT; i++) {
            st[i] = (uint8_t)s_pid[target].field[i].status;
        }
        snprintf(key, sizeof(key), "%s_st", s_pid_key_prefix[target]);
        ret = nvs_set_blob(h, key, st, sizeof(st));
    }
    if (ret == ESP_OK) {
        uint8_t fr[STORE_PID_FIELD_COUNT];
        for (int i = 0; i < STORE_PID_FIELD_COUNT; i++) {
            fr[i] = (uint8_t)s_pid[target].field[i].fail_reason;
        }
        snprintf(key, sizeof(key), "%s_fr", s_pid_key_prefix[target]);
        ret = nvs_set_blob(h, key, fr, sizeof(fr));
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist pid target %d: %d", target, ret);
    }
    return ret;
}

esp_err_t store_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    memset(&s_mission, 0, sizeof(s_mission));
    memset(s_pid, 0, sizeof(s_pid));

    load_mission_from_nvs();
    for (int t = 0; t < 4; t++) {
        load_pid_from_nvs((cl_pid_target_t)t);
    }
    ESP_LOGI(TAG, "loaded: mission %u pts (status=%d), pid gains restored from NVS where present",
             (unsigned)s_mission.count, (int)s_mission.state.status);
    return ESP_OK;
}

void store_get_mission(store_mission_t *out) {
    lock();
    *out = s_mission;
    unlock();
}

void store_get_pid(cl_pid_target_t target, store_pid_target_t *out) {
    lock();
    *out = s_pid[target];
    unlock();
}

bool store_mission_is_pending(cl_waypoint_t *out_points, size_t *out_count, bool *out_loop) {
    bool pending;
    lock();
    pending = (s_mission.state.status == STORE_STATUS_PENDING);
    if (pending) {
        memcpy(out_points, s_mission.points, s_mission.count * sizeof(cl_waypoint_t));
        *out_count = s_mission.count;
        *out_loop = s_mission.loop;
    }
    unlock();
    return pending;
}

uint8_t store_pid_pending_mask(cl_pid_target_t target, cl_pid_gains_t *out_gains) {
    uint8_t mask = 0;
    lock();
    for (int i = 0; i < STORE_PID_FIELD_COUNT; i++) {
        if (s_pid[target].field[i].status == STORE_STATUS_PENDING) {
            mask |= (1u << i);
        }
    }
    *out_gains = s_pid[target].gains;
    unlock();
    return mask;
}

esp_err_t store_set_mission(const cl_waypoint_t *points, size_t count, bool loop) {
    if (count > CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < count; i++) {
        if (!waypoint_valid(&points[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    lock();
    memcpy(s_mission.points, points, count * sizeof(cl_waypoint_t));
    s_mission.count = count;
    s_mission.loop = loop;
    s_mission.state.status = STORE_STATUS_PENDING;
    s_mission.state.fail_reason = CL_ACK_OK;
    esp_err_t nvs_ret = save_mission_to_nvs();
    unlock();

    // A local NVS write failure doesn't change the delivery-status contract
    // with the FC (that's what CL_ACK_NVS_WRITE_FAILED means) -- it's a
    // basestation-side durability problem, logged loudly in
    // save_mission_to_nvs() already. The edit still applies in memory for
    // the rest of this boot and is still attempted over the link.
    (void)nvs_ret;
    return ESP_OK;
}

esp_err_t store_set_pid_fields(cl_pid_target_t target, uint8_t fields_present, const cl_pid_gains_t *gains) {
    if (fields_present & CL_FIELD_KP     && !isfinite(gains->k_p))     return ESP_ERR_INVALID_ARG;
    if (fields_present & CL_FIELD_KI     && !isfinite(gains->k_i))     return ESP_ERR_INVALID_ARG;
    if (fields_present & CL_FIELD_KD     && !isfinite(gains->k_d))     return ESP_ERR_INVALID_ARG;
    if (fields_present & CL_FIELD_ILIMIT && !isfinite(gains->i_limit)) return ESP_ERR_INVALID_ARG;

    lock();
    if (fields_present & CL_FIELD_KP)     s_pid[target].gains.k_p = gains->k_p;
    if (fields_present & CL_FIELD_KI)     s_pid[target].gains.k_i = gains->k_i;
    if (fields_present & CL_FIELD_KD)     s_pid[target].gains.k_d = gains->k_d;
    if (fields_present & CL_FIELD_ILIMIT) s_pid[target].gains.i_limit = gains->i_limit;

    for (int i = 0; i < STORE_PID_FIELD_COUNT; i++) {
        if (fields_present & (1u << i)) {
            s_pid[target].field[i].status = STORE_STATUS_PENDING;
            s_pid[target].field[i].fail_reason = CL_ACK_OK;
        }
    }
    save_pid_to_nvs(target);
    unlock();
    return ESP_OK;
}

void store_mark_mission_result(store_status_t status, cl_ack_status_t fail_reason) {
    lock();
    // Only apply if still PENDING -- if the user edited again since this
    // send went out, a newer edit already re-marked it PENDING and this
    // (now-stale) result shouldn't clobber that.
    if (s_mission.state.status == STORE_STATUS_PENDING) {
        s_mission.state.status = status;
        s_mission.state.fail_reason = fail_reason;
        save_mission_to_nvs();
    }
    unlock();
}

void store_mark_pid_result(cl_pid_target_t target, uint8_t fields_sent_mask, store_status_t status, cl_ack_status_t fail_reason) {
    lock();
    bool changed = false;
    for (int i = 0; i < STORE_PID_FIELD_COUNT; i++) {
        if ((fields_sent_mask & (1u << i)) && s_pid[target].field[i].status == STORE_STATUS_PENDING) {
            s_pid[target].field[i].status = status;
            s_pid[target].field[i].fail_reason = fail_reason;
            changed = true;
        }
    }
    if (changed) {
        save_pid_to_nvs(target);
    }
    unlock();
}
