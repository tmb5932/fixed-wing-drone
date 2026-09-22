#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "pid_types.h"
#include "nav.h"

// Stateless NVS marshalling helpers for persisted config (PID gains,
// mission, per-channel output ranges, airframe mode, airspeed target). Not a
// state owner itself -- callers (main.c, nav.c, setup_mode.c) hold the live
// values and call these to load an override at boot / save after an applied
// change, same role pid.c plays for the PID math itself.
//
// pid_gains_t is reused directly as the NVS blob format for PID gains,
// rather than a separate storage-only struct -- keeps the type/file count
// down. If its layout ever changes later, old NVS blobs could misparse;
// that's cheaply fixed by bumping the key names below if/when it actually
// happens, not worth designing around now.

// nvs_flash_init() plus the standard ESP_ERR_NVS_NO_FREE_PAGES erase-retry.
// Call once, first thing in app_main(), before anything else touches NVS.
esp_err_t config_store_init(void);

// Returns false (leaving *out untouched) if `key` isn't present in NVS --
// the caller should keep its compiled-in default in that case.
bool config_store_load_pid_gains(const char *key, pid_gains_t *out);
esp_err_t config_store_save_pid_gains(const char *key, const pid_gains_t *g);

// Returns false (leaving *out_count/*out_loop untouched) if no mission is persisted.
bool config_store_load_mission(waypoint_t *out, size_t max_count, size_t *out_count, bool *out_loop);
esp_err_t config_store_save_mission(const waypoint_t *wps, size_t count, bool loop);

// Per-output-channel calibrated PWM range + direction. `reversed` mirrors the
// applied pulse width around (min_us+max_us)/2 rather than swapping min/max,
// so min_us < max_us always holds regardless of direction -- see
// apply_output_cfg() in main.c.
typedef struct {
    uint16_t min_us;
    uint16_t max_us;
    bool reversed;
} output_cfg_t;

// Returns false (leaving *out untouched) if `key` isn't present in NVS --
// the caller should fall back to the channel's full type range (servo/motor)
// with reversed=false in that case.
bool config_store_load_output_cfg(const char *key, output_cfg_t *out);
esp_err_t config_store_save_output_cfg(const char *key, const output_cfg_t *cfg);

typedef enum {
    AIRFRAME_CONVENTIONAL = 0,
    AIRFRAME_AILEVON_MODE = 1,
} airframe_mode_t;

// Returns false (leaving *out untouched) if nothing is persisted -- the
// caller should default to AIRFRAME_CONVENTIONAL in that case.
bool config_store_load_airframe_mode(airframe_mode_t *out);
esp_err_t config_store_save_airframe_mode(airframe_mode_t mode);

// target_cms: commanded cruise airspeed for AIRSPEED_PID_CFG. fallback_pct:
// throttle fraction (0.0-1.0) used when no airspeed sensor reading is
// available at all -- see update_autonomous_outputs() in main.c.
typedef struct {
    float target_cms;
    float fallback_pct;
} airspeed_cfg_t;

// Returns false (leaving *out untouched) if nothing is persisted -- the
// caller should keep its compiled-in default in that case.
bool config_store_load_airspeed_cfg(airspeed_cfg_t *out);
esp_err_t config_store_save_airspeed_cfg(const airspeed_cfg_t *cfg);

// motor_count: 1 (single motor, ESC1 only -- ESC2 held at motor-off) or 2
// (twin motor, both ESCs mirror the same throttle command -- there's no
// differential-thrust mixing yet, see output_ctl.h). esc1_is_left: only
// meaningful when motor_count == 2, purely a label for a future mixer.
typedef struct {
    uint8_t motor_count;
    bool esc1_is_left;
} motor_cfg_t;

// Returns false (leaving *out untouched) if nothing is persisted -- the
// caller should default to {motor_count=1, esc1_is_left=true} in that case.
bool config_store_load_motor_cfg(motor_cfg_t *out);
esp_err_t config_store_save_motor_cfg(const motor_cfg_t *cfg);

#endif // CONFIG_STORE_H
