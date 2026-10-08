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

// Which physical RC input pin (0-5, i.e. capture-channel index -- CH1_IN_GPIO
// is 0, CH6_IN_GPIO is 5, see main.c) each logical RC_* function
// (output_ctl.h's RC_THROTTLE/RC_AILERON/etc.) actually reads from. Lets a
// receiver whose channel order doesn't match this project's default
// (RC_AILERON=CH1, RC_ELEVATOR=CH2, RC_THROTTLE=CH3, RC_DIAL=CH4,
// RC_RUDDER=CH5, RC_SWITCH=CH6) be remapped from setup mode instead of
// re-wiring anything. Indexed by logical channel -- same count as
// output_ctl.h's NUM_RC_CHANNELS, kept as its own literal here since
// output_ctl.h already includes this header (a circular include the other
// way isn't possible).
#define RC_INPUT_MAP_CHANNELS (6)
typedef struct {
    uint8_t phys_ch[RC_INPUT_MAP_CHANNELS];
} rc_input_map_cfg_t;

// Returns false (leaving *out untouched) if nothing is persisted -- the
// caller should default to the identity mapping {0,1,2,3,4,5} in that case.
bool config_store_load_rc_input_map(rc_input_map_cfg_t *out);
esp_err_t config_store_save_rc_input_map(const rc_input_map_cfg_t *cfg);

// Output-side counterpart to rc_input_map_cfg_t: which physical servo output
// pin (0-5, i.e. CH1_OUT_GPIO is 0 .. CH6_OUT_GPIO is 5, see main.c) each
// logical RC_* function drives. Lets e.g. the aileron servo be plugged into
// servo_out_6 without it being driven as "Aux (switch)". Always a
// permutation of {0..5} -- two functions sharing one pin would fight over
// the same comparator -- so setters swap rather than overwrite. Indexed by
// logical channel; ESC1/ESC2 have dedicated connectors and aren't remappable.
// Per-channel calibration (output_cfg_t) stays keyed by logical function, so
// a servo's range/reversal follows it when it moves to another pin.
#define SERVO_OUTPUT_MAP_CHANNELS (6)
typedef struct {
    uint8_t phys_out[SERVO_OUTPUT_MAP_CHANNELS];
} servo_output_map_cfg_t;

// Returns false (leaving *out untouched) if nothing is persisted -- the
// caller should default to the identity mapping {0,1,2,3,4,5} in that case.
bool config_store_load_servo_output_map(servo_output_map_cfg_t *out);
esp_err_t config_store_save_servo_output_map(const servo_output_map_cfg_t *cfg);

// Autonomous-mode trim: the neutral pulse width (in the same "stick space"
// as a live RC input, i.e. before output_cfg_t reversal/clipping) each
// logical RC_* function is commanded around when the PID loop takes over,
// in place of a flat 1500us. Manual pass-through never uses this -- the
// transmitter's own trim is already baked into the live input there. Kept
// out of output_cfg_t on purpose so adding it didn't change that blob's
// size and invalidate every already-persisted calibration. Indexed by
// logical channel; only aileron/elevator/rudder are actually trimmable (see
// main.c's trim_channel_allowed()), the rest stay at 1500.
#define TRIM_CHANNELS (6)
typedef struct {
    uint16_t center_us[TRIM_CHANNELS];
} trim_cfg_t;

// Returns false (leaving *out untouched) if nothing is persisted -- the
// caller should default every channel to 1500us in that case.
bool config_store_load_trim_cfg(trim_cfg_t *out);
esp_err_t config_store_save_trim_cfg(const trim_cfg_t *cfg);

// Returns false (leaving *out untouched) if nothing valid is persisted -- the
// caller should default to NAV_MODE_WAYPOINT in that case.
bool config_store_load_nav_mode(nav_mode_t *out);
esp_err_t config_store_save_nav_mode(nav_mode_t mode);

#endif // CONFIG_STORE_H
