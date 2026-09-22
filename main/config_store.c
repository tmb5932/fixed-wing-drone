#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config_store.h"

static const char *TAG = "CONFIG_STORE";

#define CONFIG_STORE_NAMESPACE "cfg"
#define CONFIG_STORE_MISSION_KEY "mission"
#define CONFIG_STORE_AIRFRAME_KEY "airframe"
#define CONFIG_STORE_AIRSPEED_KEY "aspd_cfg"
#define CONFIG_STORE_MOTOR_KEY "motor_cfg"

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

bool config_store_load_pid_gains(const char *key, pid_gains_t *out) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(pid_gains_t);
    esp_err_t ret = nvs_get_blob(handle, key, out, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != sizeof(pid_gains_t)) {
        return false;
    }
    return true;
}

esp_err_t config_store_save_pid_gains(const char *key, const pid_gains_t *g) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for '%s': %d", key, ret);
        return ret;
    }
    ret = nvs_set_blob(handle, key, g, sizeof(pid_gains_t));
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
    // count==0 is a valid loaded state (an explicitly-cleared mission, see
    // nav_set_mission()) distinct from "nothing ever persisted" (meta blob
    // absent entirely).
    uint8_t meta[2] = {0, 0};
    size_t meta_len = sizeof(meta);
    esp_err_t ret = nvs_get_blob(handle, CONFIG_STORE_MISSION_KEY "_n", meta, &meta_len);
    if (ret != ESP_OK || meta_len != sizeof(meta)) {
        nvs_close(handle);
        return false;
    }
    uint8_t count = meta[0];
    if (count > max_count) {
        nvs_close(handle);
        return false;
    }

    if (count > 0) {
        size_t wps_len = (size_t)count * sizeof(waypoint_t);
        ret = nvs_get_blob(handle, CONFIG_STORE_MISSION_KEY, out, &wps_len);
        if (ret != ESP_OK || wps_len != (size_t)count * sizeof(waypoint_t)) {
            nvs_close(handle);
            return false;
        }
    }
    nvs_close(handle);

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
    if (ret == ESP_OK && count > 0) {
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

bool config_store_load_output_cfg(const char *key, output_cfg_t *out) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(output_cfg_t);
    esp_err_t ret = nvs_get_blob(handle, key, out, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != sizeof(output_cfg_t)) {
        return false;
    }
    return true;
}

esp_err_t config_store_save_output_cfg(const char *key, const output_cfg_t *cfg) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for '%s': %d", key, ret);
        return ret;
    }
    ret = nvs_set_blob(handle, key, cfg, sizeof(output_cfg_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist output cfg for '%s': %d", key, ret);
    }
    return ret;
}

bool config_store_load_airframe_mode(airframe_mode_t *out) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    uint8_t raw;
    size_t len = sizeof(raw);
    esp_err_t ret = nvs_get_blob(handle, CONFIG_STORE_AIRFRAME_KEY, &raw, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != sizeof(raw)) {
        return false;
    }
    *out = (airframe_mode_t)raw;
    return true;
}

esp_err_t config_store_save_airframe_mode(airframe_mode_t mode) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for airframe mode: %d", ret);
        return ret;
    }
    uint8_t raw = (uint8_t)mode;
    ret = nvs_set_blob(handle, CONFIG_STORE_AIRFRAME_KEY, &raw, sizeof(raw));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist airframe mode: %d", ret);
    }
    return ret;
}

bool config_store_load_airspeed_cfg(airspeed_cfg_t *out) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(airspeed_cfg_t);
    esp_err_t ret = nvs_get_blob(handle, CONFIG_STORE_AIRSPEED_KEY, out, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != sizeof(airspeed_cfg_t)) {
        return false;
    }
    return true;
}

esp_err_t config_store_save_airspeed_cfg(const airspeed_cfg_t *cfg) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for airspeed cfg: %d", ret);
        return ret;
    }
    ret = nvs_set_blob(handle, CONFIG_STORE_AIRSPEED_KEY, cfg, sizeof(airspeed_cfg_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist airspeed cfg: %d", ret);
    }
    return ret;
}

bool config_store_load_motor_cfg(motor_cfg_t *out) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(motor_cfg_t);
    esp_err_t ret = nvs_get_blob(handle, CONFIG_STORE_MOTOR_KEY, out, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != sizeof(motor_cfg_t)) {
        return false;
    }
    return true;
}

esp_err_t config_store_save_motor_cfg(const motor_cfg_t *cfg) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed for motor cfg: %d", ret);
        return ret;
    }
    ret = nvs_set_blob(handle, CONFIG_STORE_MOTOR_KEY, cfg, sizeof(motor_cfg_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to persist motor cfg: %d", ret);
    }
    return ret;
}
