#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config_store.h"

static const char *TAG = "CONFIG_STORE";

#define CONFIG_STORE_NAMESPACE "cfg"
#define CONFIG_STORE_MISSION_KEY "mission"

esp_err_t config_store_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Standard ESP-IDF boilerplate: a partition-layout/version change
        // leaves stale NVS data that must be erased before it can be used.
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

bool config_store_load_pid_gains(const char *key, cl_pid_gains_t *out) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(cl_pid_gains_t);
    esp_err_t ret = nvs_get_blob(handle, key, out, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != sizeof(cl_pid_gains_t)) {
        return false;
    }
    return true;
}

esp_err_t config_store_save_pid_gains(const char *key, const cl_pid_gains_t *g) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for '%s': %d", key, ret);
        return ret;
    }
    ret = nvs_set_blob(handle, key, g, sizeof(cl_pid_gains_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist PID gains for '%s': %d", key, ret);
    }
    return ret;
}

bool config_store_load_mission(waypoint_t *out, size_t max_count, size_t *out_count, bool *out_loop) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    // {count, loop} packed together so a mission and its loop flag can never
    // desync in NVS (e.g. a partial write leaving one updated but not the other).
    uint8_t meta[2] = {0, 0};
    size_t meta_len = sizeof(meta);
    esp_err_t ret = nvs_get_blob(handle, CONFIG_STORE_MISSION_KEY "_n", meta, &meta_len);
    uint8_t count = meta[0];
    if (ret != ESP_OK || meta_len != sizeof(meta) || count == 0 || count > max_count) {
        nvs_close(handle);
        return false;
    }

    size_t wps_len = (size_t)count * sizeof(waypoint_t);
    ret = nvs_get_blob(handle, CONFIG_STORE_MISSION_KEY, out, &wps_len);
    nvs_close(handle);
    if (ret != ESP_OK || wps_len != (size_t)count * sizeof(waypoint_t)) {
        return false;
    }

    *out_count = count;
    *out_loop = meta[1] != 0;
    return true;
}

esp_err_t config_store_save_mission(const waypoint_t *wps, size_t count, bool loop) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for mission: %d", ret);
        return ret;
    }

    uint8_t meta[2] = {(uint8_t)count, (uint8_t)(loop ? 1 : 0)};
    ret = nvs_set_blob(handle, CONFIG_STORE_MISSION_KEY "_n", meta, sizeof(meta));
    if (ret == ESP_OK) {
        ret = nvs_set_blob(handle, CONFIG_STORE_MISSION_KEY, wps, count * sizeof(waypoint_t));
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist mission: %d", ret);
    }
    return ret;
}
