#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"
#include "pwm_output.h"
#include "rc_capture.h"
#include "imu.h"
#include "gps.h"
#include "pid.h"
#include "nav.h"
#include "airspeed.h"
#include "config_store.h"
#include "i2c_bus.h"
#include "output_ctl.h"
#include "setup_mode.h"

static const char *TAG = "MAIN";

#define CONTROL_TASK_HZ (100)
#define CONTROL_TASK_MS (1000 / CONTROL_TASK_HZ)

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define CAPTURE_RESOLUTION (80000000) // This is unchangeable on the esp32s3, so its always 80MHz

// GPIO assignments for RC receiver inputs, confirmed against the v2.1 board
// pinout (GPIO 35/36/37 are NC on this board -- not usable for anything).
#define CH1_IN_GPIO  GPIO_NUM_9
#define CH2_IN_GPIO  GPIO_NUM_38
#define CH3_IN_GPIO  GPIO_NUM_14
#define CH4_IN_GPIO  GPIO_NUM_21
#define CH5_IN_GPIO  GPIO_NUM_7
#define CH6_IN_GPIO  GPIO_NUM_8

// GPIO assignments for outputs to the peripherals
#define CH1_OUT_GPIO  GPIO_NUM_17
#define CH2_OUT_GPIO  GPIO_NUM_18
#define CH3_OUT_GPIO  GPIO_NUM_46
#define CH4_OUT_GPIO  GPIO_NUM_45
#define CH5_OUT_GPIO  GPIO_NUM_48
#define CH6_OUT_GPIO  GPIO_NUM_47

// Dedicated ESC/motor outputs -- separate physical connectors from the 6
// channels above. Confirmed against the v2.1 board pinout (the KiCad
// sources in this repo are a future v3.1 revision and don't reflect it).
#define ESC1_OUT_GPIO GPIO_NUM_16
#define ESC2_OUT_GPIO GPIO_NUM_15

// NUM_RC_CHANNELS, NUM_OUTPUT_CHANNELS, CH*_NUM, RC_*, and ESC1_CH/ESC2_CH
// channel-index macros live in output_ctl.h now, shared with setup_mode.c.

// Airframe control-surface layout is now a runtime setting (g_airframe_mode,
// loaded from NVS at boot) instead of this compile-time define -- see
// AIRFRAME_CONVENTIONAL/AIRFRAME_AILEVON_MODE in config_store.h.

static output_group_t out_groups[SOC_MCPWM_GROUPS];
static rc_capture_group_t cap_groups[SOC_MCPWM_GROUPS];

// Per-output-channel calibrated PWM range + reversal, loaded from NVS at
// boot (config_store.h's output_cfg_t), defaulting to PULSEWIDTH_DEFAULT_MIN/
// MAX_US with reversed=false when nothing is persisted yet. See
// channel_output_type()/apply_output_cfg() below. Sized for all 8 output
// channels (6 RC-mirrored + ESC1 + ESC2), not just NUM_RC_CHANNELS.
static output_cfg_t channel_cfgs[NUM_OUTPUT_CHANNELS];

// Which physical RC input pin (capture-channel index, 0-5) each logical
// RC_* function actually reads from -- see config_store.h's
// rc_input_map_cfg_t. Defaults to the identity mapping (matching this
// project's compiled-in default channel order: RC_AILERON=CH1,
// RC_ELEVATOR=CH2, RC_THROTTLE=CH3, RC_DIAL=CH4, RC_RUDDER=CH5,
// RC_SWITCH=CH6), overridable from setup mode for a receiver whose channel
// order doesn't match, without re-wiring anything.
static uint8_t rc_input_map[NUM_RC_CHANNELS] = {0, 1, 2, 3, 4, 5};

static airframe_mode_t g_airframe_mode = AIRFRAME_CONVENTIONAL;

// Single vs twin motor, and which physical ESC connector is "left" when
// twin. Defaults to a single motor on ESC1 -- see apply_throttle_to_escs().
static motor_cfg_t g_motor_cfg = { .motor_count = 1, .esc1_is_left = true };

// Target cruise airspeed (target_cms) for AIRSPEED_PID_CFG, and the throttle
// fraction (fallback_pct) used when no airspeed sensor reading is available
// at all. Loaded from NVS at boot; these defaults match the previous
// compile-time AIRSPEED_TARGET_CMS placeholder and the user's spec of a 75%
// cruise fallback.
static airspeed_cfg_t g_airspeed_cfg = { .target_cms = 1000.0f, .fallback_pct = 0.75f };

// i_limit=250 reserves at least half of the actuator's +/-500us range
// (SERVO_MIN/MAX_PULSEWIDTH_US relative to MIDDLE_SERVO_VAL) for P/D, so a
// wound-up I-term can't eat the whole output on its own. Placeholder like the
// gains themselves; re-tune once k_i is actually set to something nonzero.
// SITL-validated (sitl/nav_sim.exe + sitl/sitl_sim.exe, see sitl/README.md):
// k_p=10 k_d=0.3 -- this project's gains before this pass -- is only stable
// for small corrections; it diverges under a full-authority step (the ±45deg
// commanded by set_goal_roll_deg's clamp, which the new waypoint-following
// nav_task actually issues in practice, not just small disturbance recovery).
// k_p=5 k_d=0.4 was the most aggressive gain that stayed stable across the
// entire commanded range (10/20/30/45deg steps, both signs) AND across 20
// noise seeds at up to 2deg IMU noise stddev -- higher k_p consistently
// failed the noise sweep before it failed the clean-signal sweep.
pid_cfg_t ROLL_PID_CFG = {
    .k_p = 5,
    .k_i = 0,
    .k_d = 0.4,
    .i_limit = 250,
    .integral = 0,
    .last_err = 0,
    .first = true
};

// Same SITL sweep as ROLL_PID_CFG, run separately against the pitch axis's
// own plant model (it has a nonzero restoring term -- see sitl/plant.h --
// unlike roll) and its own ±20deg commanded range. Same winning gain.
pid_cfg_t PITCH_PID_CFG = {
    .k_p = 5,
    .k_i = 0,
    .k_d = 0.4,
    .i_limit = 250,
    .integral = 0,
    .last_err = 0,
    .first = true
};

// Desk-derived, not SITL-validated like ROLL/PITCH_PID_CFG above -- there's
// no airspeed plant model (thrust curve, drag, mass) to sweep against yet.
// This replaces an untested k_p=5 placeholder with a number grounded in the
// MS4525DO's own accuracy spec instead of a guess; still needs real bench/
// flight tuning.
//
// The sensor's +-0.25%-of-span spec is +-0.005 PSI (+-34Pa) of noise on the
// underlying differential-pressure reading, filtered here (see
// AIRSPEED_FILTER_ALPHA in airspeed.c) down to roughly a 4x std-dev
// reduction -- but noise on a *pressure* reading translates to noise on the
// *speed* it implies (dv/dp = 1/(rho*v)) that shrinks with airspeed, not a
// fixed cm/s figure. At g_airspeed_cfg's default 1000cm/s (10m/s) cruise
// target, that's dp =~ 61Pa for the target speed itself -- comparable to the
// sensor's own noise floor -- giving an estimated post-filter residual noise
// of roughly +-65cm/s even at cruise (worse nearer zero airspeed, where the
// same pressure noise implies a larger speed swing).
//
// Sizing k_p so that residual noise alone doesn't swamp the output: the
// throttle channel's usable PWM range is ~1000us (SERVO_MIN/MAX_PULSEWIDTH_US
// via channel_cfgs[ESC1_CH]), so k_p=2 keeps noise-driven throttle
// chatter to roughly +-130us (2 * 65cm/s) -- a small, tolerable fraction of
// that range -- while still giving a real 2m/s (200cm/s) airspeed error 400us
// of correction, which is a meaningful, not negligible, throttle response.
pid_cfg_t AIRSPEED_PID_CFG = {
    .k_p = 2,
    .k_i = 0,
    .k_d = 0,
    .i_limit = 250,
    .integral = 0,
    .last_err = 0,
    .first = true
};

// Conservative caps on commanded attitude. Retune once the airframe is
// flight-characterized / tested. These exist so nothing (i.e. a bug)
// can command a full 180 degree pitch through set_goal_roll_deg/set_goal_pitch_deg;
#define MAX_ROLL_GOAL_DEG  45.0f
#define MAX_PITCH_GOAL_DEG 20.0f

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float goal_roll_deg = 0.0f;
float goal_pitch_deg = 0.0f;

void set_goal_roll_deg(float deg) {
    goal_roll_deg = clampf(deg, -MAX_ROLL_GOAL_DEG, MAX_ROLL_GOAL_DEG);
}

void set_goal_pitch_deg(float deg) {
    goal_pitch_deg = clampf(deg, -MAX_PITCH_GOAL_DEG, MAX_PITCH_GOAL_DEG);
}

// Merges the fields marked present in `fields_present` (a pid_field_mask_t
// bitmask) into *cfg's live gains, leaving absent fields untouched, then
// persists the merged 4-gain result to NVS under `nvs_key`. No mutex needed:
// these three PID configs are only ever read inside update_autonomous_outputs(),
// which never runs concurrently with a setup-mode HTTP request -- flight mode
// and setup mode are mutually exclusive per boot (see the boot-time branch in
// app_main()).
static bool apply_pid_field_update(pid_cfg_t *cfg, const char *nvs_key, uint8_t fields_present, const pid_gains_t *g) {
    if (fields_present & PID_FIELD_KP)     cfg->k_p = g->k_p;
    if (fields_present & PID_FIELD_KI)     cfg->k_i = g->k_i;
    if (fields_present & PID_FIELD_KD)     cfg->k_d = g->k_d;
    if (fields_present & PID_FIELD_ILIMIT) cfg->i_limit = g->i_limit;
    pid_gains_t merged = { cfg->k_p, cfg->k_i, cfg->k_d, cfg->i_limit };
    return config_store_save_pid_gains(nvs_key, &merged) == ESP_OK;
}

bool set_roll_pid_gains(uint8_t fields_present, const pid_gains_t *g) {
    return apply_pid_field_update(&ROLL_PID_CFG, "pid_roll", fields_present, g);
}

bool set_pitch_pid_gains(uint8_t fields_present, const pid_gains_t *g) {
    return apply_pid_field_update(&PITCH_PID_CFG, "pid_pitch", fields_present, g);
}

bool set_airspeed_pid_gains(uint8_t fields_present, const pid_gains_t *g) {
    return apply_pid_field_update(&AIRSPEED_PID_CFG, "pid_aspd", fields_present, g);
}

pid_gains_t get_roll_pid_gains(void) {
    return (pid_gains_t){ ROLL_PID_CFG.k_p, ROLL_PID_CFG.k_i, ROLL_PID_CFG.k_d, ROLL_PID_CFG.i_limit };
}

pid_gains_t get_pitch_pid_gains(void) {
    return (pid_gains_t){ PITCH_PID_CFG.k_p, PITCH_PID_CFG.k_i, PITCH_PID_CFG.k_d, PITCH_PID_CFG.i_limit };
}

pid_gains_t get_airspeed_pid_gains(void) {
    return (pid_gains_t){ AIRSPEED_PID_CFG.k_p, AIRSPEED_PID_CFG.k_i, AIRSPEED_PID_CFG.k_d, AIRSPEED_PID_CFG.i_limit };
}

bool autonomous_mode_enabled(uint32_t mode_us) {
    return (mode_us < 1500);
}

/* INPUT CONVERSIONS (2 groups with 3 capture channels each) */

/**
 * Convert rc channel number to capture channel number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in) 
*/
int rc_channel_to_capture_channel(int ch)
{
    if (ch < 0 || ch >= NUM_RC_CHANNELS) { ESP_LOGE(TAG, "Invalid channel number: %d; should be in [0, %d)", ch, NUM_RC_CHANNELS); abort(); }
    return ch % MCPWM_CAPTURE_CHANNELS_PER_GROUP; 
}

/**
 * Convert rc channel number to capture group number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in) 
*/
int rc_channel_to_capture_group(int ch)
{
    if (ch < 0 || ch >= NUM_RC_CHANNELS) { ESP_LOGE(TAG, "Invalid channel number: %d; should be in [0, %d)", ch, NUM_RC_CHANNELS); abort(); }
    return ch / MCPWM_CAPTURE_CHANNELS_PER_GROUP;
}

/* OUTPUT CONVERSIONS (2 groups with 3 operators each, with each operator having 2 triggers) */

/**
 * Convert output channel number to output group number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in)
*/
int rc_channel_to_output_group(int ch)
{
    if (ch < 0 || ch >= NUM_OUTPUT_CHANNELS) { ESP_LOGE(TAG, "Invalid output channel number: %d; should be in [0, %d)", ch, NUM_OUTPUT_CHANNELS); abort(); }
    return ch / MCPWM_TRIGGERS_PER_GROUP;
}

/**
 * Convert output channel number to output operator number (local to its own
 * group -- e.g. channel 6 is the first channel of group 1, not the 4th
 * operator of a single flat sequence, so the channel index is first reduced
 * to its position within its own group before dividing).
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in)
*/
int rc_channel_to_output_op(int ch)
{
    if (ch < 0 || ch >= NUM_OUTPUT_CHANNELS) { ESP_LOGE(TAG, "Invalid output channel number: %d; should be in [0, %d)", ch, NUM_OUTPUT_CHANNELS); abort(); }
    int local_ch = ch % MCPWM_TRIGGERS_PER_GROUP;
    return local_ch / SOC_MCPWM_GENERATORS_PER_OPERATOR;
}

/**
 * Convert output channel number to output comparator / generator number
 * (also local to its own group -- see rc_channel_to_output_op()).
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in)
*/
int rc_channel_to_output_cmpr(int ch)
{
    if (ch < 0 || ch >= NUM_OUTPUT_CHANNELS) { ESP_LOGE(TAG, "Invalid output channel number: %d; should be in [0, %d)", ch, NUM_OUTPUT_CHANNELS); abort(); }
    int local_ch = ch % MCPWM_TRIGGERS_PER_GROUP;
    return local_ch % SOC_MCPWM_COMPARATORS_PER_OPERATOR;
}

// Resolves a logical RC_* channel through rc_input_map to the rc_input_t
// it's actually wired to right now. Single choke point for the remap --
// every caller that needs to reach into a capture channel (pulse width,
// staleness, "ever heard from" checks) goes through this instead of
// indexing cap_groups directly, so the mapping can't accidentally be
// bypassed in one place and honored in another.
static rc_input_t *capture_input_for(int ch) {
    int phys = rc_input_map[ch];
    return &cap_groups[rc_channel_to_capture_group(phys)].inputs[rc_channel_to_capture_channel(phys)];
}

uint32_t get_channel_pulse_width(int ch) {
    return capture_input_for(ch)->pulse_width_us;
}

/**
 * Check if the given input_capture is stale, aka haven't recieved any signal from reciever in 200 milliseconds.
 * Returns true if the input_capture is stale, else false
*/
bool is_stale(rc_input_t input_capture) {
    return (esp_timer_get_time() - input_capture.last_update_us) > 200000;
}

// Single source of truth for "would control_task() currently choose
// autonomous or manual", used both by control_task() itself and by setup
// mode's HTTP API (get_rc_mode_status() below) so a bench tester can see,
// before ever leaving setup mode, which way their switch is set -- without
// duplicating the decision logic in two places and risking them drifting
// apart. Deliberately excludes critical_fault_latched: that's a control_task-
// only concept (never true in setup mode, which never calls
// update_autonomous_outputs()), so control_task ANDs it in separately.
rc_mode_status_t get_rc_mode_status(void) {
    rc_input_t *sw_input = capture_input_for(RC_SWITCH);
    rc_mode_status_t s;
    s.radio_connected = sw_input->last_update_us != 0;
    s.signal_stale = s.radio_connected && is_stale(*sw_input);
    s.autonomous = s.radio_connected && (s.signal_stale || autonomous_mode_enabled(get_channel_pulse_width(RC_SWITCH)));
    return s;
}

/**
 * Turns given channel number into its matching comparator handle
 * Returns handle of comparator for given channel
*/
mcpwm_cmpr_handle_t channel_to_comparator(int ch) {
    return out_groups[rc_channel_to_output_group(ch)].operators[rc_channel_to_output_op(ch)].comparators[rc_channel_to_output_cmpr(ch)];
}

int clip(int num, int min, int max) {
    if (num < min) {
        return min;
    } else if (num > max) {
        return max;
    } else {
        return num;
    }
}

// RC_THROTTLE's own output slot (servo_out_3) used to double as a second,
// fully redundant copy of the ESC signal -- now it's a spare SERVO_TYPE pin
// like the other 5 RC-mirrored channels (see pass_through_inputs()), not
// tied to throttle at all. Only ESC1_CH/ESC2_CH are still MOTOR_TYPE.
item_type_t channel_output_type(int ch) {
    return (ch == ESC1_CH || ch == ESC2_CH) ? MOTOR_TYPE : SERVO_TYPE;
}

// Applies channel ch's calibrated reversal + range (channel_cfgs[ch]) to a
// desired pulse width and writes it out. Reversal mirrors the value around
// the channel's own midpoint rather than swapping min/max, so min_us < max_us
// always holds regardless of direction.
static void apply_output_cfg(int ch, int desired_us) {
    const output_cfg_t *cfg = &channel_cfgs[ch];
    int val = cfg->reversed ? ((int)cfg->min_us + (int)cfg->max_us - desired_us) : desired_us;
    update_comparator_value(channel_to_comparator(ch), clip(val, cfg->min_us, cfg->max_us));
}

output_cfg_t get_channel_output_cfg(int ch) {
    return channel_cfgs[ch];
}

bool set_channel_output_cfg(int ch, const output_cfg_t *cfg) {
    if (ch < 0 || ch >= NUM_OUTPUT_CHANNELS || cfg->min_us >= cfg->max_us) {
        return false;
    }

    output_cfg_t validated = {
        .min_us = (uint16_t)clip(cfg->min_us, PULSEWIDTH_ABS_MIN_US, PULSEWIDTH_ABS_MAX_US),
        .max_us = (uint16_t)clip(cfg->max_us, PULSEWIDTH_ABS_MIN_US, PULSEWIDTH_ABS_MAX_US),
        .reversed = cfg->reversed,
    };
    if (validated.min_us >= validated.max_us) {
        return false;
    }

    channel_cfgs[ch] = validated;

    char key[16];
    snprintf(key, sizeof(key), "outcfg%d", ch + 1);
    return config_store_save_output_cfg(key, &validated) == ESP_OK;
}

airframe_mode_t get_airframe_mode(void) {
    return g_airframe_mode;
}

bool set_airframe_mode(airframe_mode_t mode) {
    g_airframe_mode = mode;
    return config_store_save_airframe_mode(mode) == ESP_OK;
}

airspeed_cfg_t get_airspeed_cfg(void) {
    return g_airspeed_cfg;
}

bool set_airspeed_cfg(const airspeed_cfg_t *cfg) {
    if (!isfinite(cfg->target_cms) || !isfinite(cfg->fallback_pct) || cfg->fallback_pct < 0.0f || cfg->fallback_pct > 1.0f) {
        return false;
    }
    g_airspeed_cfg = *cfg;
    return config_store_save_airspeed_cfg(cfg) == ESP_OK;
}

motor_cfg_t get_motor_cfg(void) {
    return g_motor_cfg;
}

bool set_motor_cfg(const motor_cfg_t *cfg) {
    if (cfg->motor_count != 1 && cfg->motor_count != 2) {
        return false;
    }
    g_motor_cfg = *cfg;
    return config_store_save_motor_cfg(cfg) == ESP_OK;
}

rc_input_map_cfg_t get_rc_input_map(void) {
    rc_input_map_cfg_t cfg;
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        cfg.phys_ch[i] = rc_input_map[i];
    }
    return cfg;
}

bool set_rc_input_map(const rc_input_map_cfg_t *cfg) {
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        if (cfg->phys_ch[i] >= NUM_RC_CHANNELS) {
            return false;
        }
    }
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        rc_input_map[i] = cfg->phys_ch[i];
    }
    return config_store_save_rc_input_map(cfg) == ESP_OK;
}

uint32_t get_physical_pulse_width(int phys) {
    return cap_groups[rc_channel_to_capture_group(phys)].inputs[rc_channel_to_capture_channel(phys)].pulse_width_us;
}

// Mirrors throttle_us to ESC1 always, and to ESC2 only in twin-motor mode --
// there's no differential-thrust mixing yet (see output_ctl.h), just an
// identical copy of whatever the single throttle command is. In single-motor
// mode ESC2 is explicitly held at motor-off rather than left at a stale
// value, in case something's plugged into it by mistake.
static void apply_throttle_to_escs(int throttle_us) {
    apply_output_cfg(ESC1_CH, throttle_us);
    if (g_motor_cfg.motor_count == 2) {
        apply_output_cfg(ESC2_CH, throttle_us);
    } else {
        apply_output_cfg(ESC2_CH, (int)PULSEWIDTH_DEFAULT_MIN_US);
    }
}

void pass_through_inputs(uint32_t ch[NUM_RC_CHANNELS]) {
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        if (capture_input_for(i)->last_update_us == 0) {
            continue;
        }

        if (i == RC_THROTTLE) {
            // RC_THROTTLE's own output slot (servo_out_3) is a spare pin, not
            // mirrored from the throttle stick -- see channel_output_type()'s
            // comment. Only the dedicated ESC outputs get the throttle value.
            apply_throttle_to_escs((int)ch[i]);
            continue;
        }
        apply_output_cfg(i, (int)ch[i]);
    }
}

/**
 * Runs the autonomous PID outputs for one control cycle.
 * Returns false (and leaves the outputs untouched) if the IMU isn't ready or
 * its data couldn't be read this cycle, so the caller can fall back to manual.
 */
bool update_autonomous_outputs(void)
{
    if (!imu_ready) {
        return false;
    }

    const int MIDDLE_SERVO_VAL = 1500;
    const float dt_s = 1.0f / CONTROL_TASK_HZ;

    // Grab values from the IMU
    BaseType_t ret = xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT));

    // If we fail to take the mutex, log an error and return early. This is a critical failure, as we can't safely read the IMU data without the mutex.
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take imu_data_mutex! Error: %d", ret);
        return false;
    }

    float roll_deg = imu_data.roll;
    float pitch_deg = imu_data.pitch;
    xSemaphoreGive(imu_data_mutex);

    // Airspeed-hold throttle. airspeed_reading() is only ever true once a
    // real driver calls airspeed_enable() after validating its hardware is
    // actually present (mirroring imu_init()'s WHO_AM_I-gated pattern), so
    // today -- with no sensor driver written yet -- this always falls
    // through to the fallback-percent throttle below (see g_airspeed_cfg,
    // configurable in setup mode).
    //
    // Turn compensation: a banked turn needs more lift than level flight to
    // hold altitude (load factor n = 1/cos(bank)), which raises stall speed
    // by sqrt(n) = 1/sqrt(cos(bank)) -- so holding the same speed margin
    // above stall through a turn means flying faster than the level-flight
    // cruise target, not the same speed. goal_roll_deg is already clamped to
    // +/-MAX_ROLL_GOAL_DEG (45deg) by set_goal_roll_deg(), safely away from
    // the singularity at 90deg, so no extra bound is needed here: at the max
    // commanded bank this scales the target up by ~19% (1/sqrt(cos(45deg))).
    // Only applies to the closed-loop airspeed-PID path below -- the no-
    // sensor fallback just outputs a fixed throttle percentage with no speed
    // feedback at all, so there's no target to compensate there.
    float bank_rad = fabsf(goal_roll_deg) * ((float)M_PI / 180.0f);
    float target_cms_effective = g_airspeed_cfg.target_cms / sqrtf(cosf(bank_rad));

    // Throttle range comes from ESC1_CH's own calibrated output_cfg_t --
    // ESC1 is always driven (single or twin motor, see
    // apply_throttle_to_escs()), and RC_THROTTLE's own output slot
    // (servo_out_3) is a spare pin now, no longer tied to throttle at all --
    // see pass_through_inputs() and channel_output_type()'s comments.
    int16_t airspeed_cms = airspeed_get();
    int throttle_out;
    if (airspeed_reading() && airspeed_cms != INT16_MIN) {
        float throttle_cmd = pid_step(&AIRSPEED_PID_CFG, (float)airspeed_cms, target_cms_effective, dt_s);
        throttle_out = clip((int)(1000 + throttle_cmd), channel_cfgs[ESC1_CH].min_us, channel_cfgs[ESC1_CH].max_us);
    } else {
        int span = (int)channel_cfgs[ESC1_CH].max_us - (int)channel_cfgs[ESC1_CH].min_us;
        throttle_out = (int)channel_cfgs[ESC1_CH].min_us + (int)(span * g_airspeed_cfg.fallback_pct);
    }
    apply_throttle_to_escs(throttle_out);

    float roll_cmd = pid_step(&ROLL_PID_CFG, roll_deg, goal_roll_deg, dt_s);
    float pitch_cmd = pid_step(&PITCH_PID_CFG, pitch_deg, goal_pitch_deg, dt_s);

    if (g_airframe_mode == AIRFRAME_AILEVON_MODE) {
        // Combined surfaces: mix roll and pitch into left/right elevon
        // outputs. Sign convention (which physical output is "left" vs
        // "right", and +/- for roll) depends on servo mounting; verify
        // direction on the bench, same as the existing single-purpose
        // aileron/elevator outputs already require.
        apply_output_cfg(RC_AILERON, (int)(MIDDLE_SERVO_VAL + pitch_cmd - roll_cmd));
        apply_output_cfg(RC_ELEVATOR, (int)(MIDDLE_SERVO_VAL + pitch_cmd + roll_cmd));
    } else {
        apply_output_cfg(RC_AILERON, (int)(MIDDLE_SERVO_VAL + roll_cmd));
        apply_output_cfg(RC_ELEVATOR, (int)(MIDDLE_SERVO_VAL + pitch_cmd));
    }

    apply_output_cfg(RC_RUDDER, MIDDLE_SERVO_VAL);
    // apply_output_cfg(5-1, 1500);
    // apply_output_cfg(4-1, 1500);

    return true;
}

// Once any critical autonomous-path failure happens (e.g. IMU not ready, or
// a failed mutex take), autonomous mode is locked out for the rest of this
// boot. There's no in-flight recovery for "the IMU never came up" or similar,
// so the plane needs a reset before autonomous can be trusted again.
static bool critical_fault_latched = false;

void control_task(void *arg) {
    uint32_t ch[NUM_RC_CHANNELS];
    bool was_autonomous = false;
    while (1) {
        ch[RC_THROTTLE] = get_channel_pulse_width(RC_THROTTLE);
        ch[RC_AILERON] = get_channel_pulse_width(RC_AILERON);
        ch[RC_ELEVATOR] = get_channel_pulse_width(RC_ELEVATOR);
        ch[RC_RUDDER] = get_channel_pulse_width(RC_RUDDER);
        ch[RC_SWITCH] = get_channel_pulse_width(RC_SWITCH);
        ch[RC_DIAL] = get_channel_pulse_width(RC_DIAL);

        // printf("CH: %lu, %lu, %lu, %lu, %lu, %lu\n", ch[0], ch[1], ch[2], ch[3], ch[4], ch[5]);

        rc_mode_status_t mode_status = get_rc_mode_status();
        bool want_autonomous = !critical_fault_latched && mode_status.autonomous;

        // On the manual->autonomous transition edge, clear out accumulated
        // PID/nav state so a nav task that's been idling in the background
        // (or a previous autonomous run) doesn't hand this engagement stale
        // integral/derivative history.
        if (want_autonomous && !was_autonomous) {
            pid_reset(&ROLL_PID_CFG);
            pid_reset(&PITCH_PID_CFG);
            pid_reset(&AIRSPEED_PID_CFG);
            nav_reset();
        }
        was_autonomous = want_autonomous;

        if (want_autonomous && !imu_ready) {
            // Not a fault -- the IMU can still be mid-boot (e.g. inside
            // calibrate_gyro_bias()'s up-to-15s+ settle window, restarted
            // if it detects motion) well after control_task is already
            // running. Falling back to manual for this cycle and retrying
            // once imu_ready actually goes true avoids permanently latching
            // autonomous off over a race the operator can trivially lose by
            // flipping the switch early -- there's no external indication
            // that window is even in progress. A real failure (imu_ready
            // was true, then update_autonomous_outputs() itself fails, e.g.
            // a mutex take) below still latches, since that's an actual
            // runtime fault, not an expected startup race.
            want_autonomous = false;
        } else if (want_autonomous && !update_autonomous_outputs()) {
            critical_fault_latched = true;
            want_autonomous = false;
        }

        if (!want_autonomous) {
            // manual pass-through mode from remote controller
            pass_through_inputs(ch);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_MS));
    }
}

// Brings up the MCPWM output groups (servo/ESC) and capture groups (RC
// input) and starts both. Called from exactly one of app_main() (normal
// flight boot) or setup_mode_run() (setup-mode boot) per boot.
void io_hardware_init(void) {
    // OUTPUT SETUP
    ESP_LOGI(TAG, "Creating output group 0.");
    out_groups[0] = create_group(0);

    mcpwm_timer_handle_t timer0 = NULL;
    initialize_timer(&timer0, 0, SERVO_TIMEBASE_RESOLUTION_HZ, SERVO_TIMEBASE_PERIOD);

    ESP_LOGI(TAG, "Adding operator and generator/comparator.");
    add_operator(&out_groups[0], 0, timer0);
    add_gen_cmpr(&out_groups[0].operators[0], 0, CH1_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);
    add_gen_cmpr(&out_groups[0].operators[0], 1, CH2_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);

    add_operator(&out_groups[0], 1, timer0);
    // Spare output (see channel_output_type()'s comment) -- SERVO_TYPE/
    // DEFAULT_STARTING_VALUE like the other 5 RC-mirrored channels, not
    // MOTOR_TYPE, since nothing drives this pin as a throttle anymore.
    add_gen_cmpr(&out_groups[0].operators[1], 0, CH3_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);
    add_gen_cmpr(&out_groups[0].operators[1], 1, CH4_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);

    add_operator(&out_groups[0], 2, timer0);
    add_gen_cmpr(&out_groups[0].operators[2], 0, CH5_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);
    add_gen_cmpr(&out_groups[0].operators[2], 1, CH6_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);
    mcpwm_timer_start(&timer0);

    // Dedicated ESC1/ESC2 outputs (channel indices ESC1_CH/ESC2_CH, output_ctl.h)
    // -- a separate MCPWM group from the 6 channels above, since group 0's 3
    // operators are already fully used. Only 1 of group 1's 3 operators is
    // used here, leaving room for more future output channels.
    ESP_LOGI(TAG, "Creating output group 1 (ESC1/ESC2).");
    out_groups[1] = create_group(1);

    mcpwm_timer_handle_t timer1 = NULL;
    initialize_timer(&timer1, 1, SERVO_TIMEBASE_RESOLUTION_HZ, SERVO_TIMEBASE_PERIOD);

    add_operator(&out_groups[1], 0, timer1);
    add_gen_cmpr(&out_groups[1].operators[0], 0, ESC1_OUT_GPIO, MOTOR_TYPE, MIN_STARTING_VALUE);
    add_gen_cmpr(&out_groups[1].operators[0], 1, ESC2_OUT_GPIO, MOTOR_TYPE, MIN_STARTING_VALUE);
    mcpwm_timer_start(&timer1);

    // INPUT SETUP
    ESP_LOGI(TAG, "Creating capture group 0.");
    cap_groups[0] = create_rc_capture_group(0);
    rc_capture_init_timer(&cap_groups[0], CAPTURE_RESOLUTION); // Capture resolution not used on esp32s3

    rc_capture_add_channel(&cap_groups[0], 0, CH1_IN_GPIO);
    rc_capture_add_channel(&cap_groups[0], 1, CH2_IN_GPIO);
    rc_capture_add_channel(&cap_groups[0], 2, CH3_IN_GPIO);

    rc_capture_start(&cap_groups[0]);

    ESP_LOGI(TAG, "Creating capture group 1.");
    cap_groups[1] = create_rc_capture_group(1);
    rc_capture_init_timer(&cap_groups[1], CAPTURE_RESOLUTION); // Capture resolution not used on esp32s3

    rc_capture_add_channel(&cap_groups[1], 0, CH4_IN_GPIO);
    rc_capture_add_channel(&cap_groups[1], 1, CH5_IN_GPIO);
    rc_capture_add_channel(&cap_groups[1], 2, CH6_IN_GPIO);

    rc_capture_start(&cap_groups[1]);
}

// How long after power-on the BOOT button (GPIO0) is watched for a press
// that diverts boot into setup mode instead of normal flight init. GPIO0 is
// only sampled as a strapping pin by the ROM bootloader during the actual
// reset -- by the time app_main() is running that window has already
// passed, so reading it here as a plain input is safe and doesn't risk
// UART download mode. GPIO0 is deliberately not exposed as an auxiliary GPIO
// (see auxiliary.h) precisely so init_auxiliary_gpio() can never reconfigure
// it as an output and fight this input configuration.
#define SETUP_MODE_BOOT_WINDOW_MS (5000)
#define SETUP_MODE_POLL_MS (50)

// Returns true if the BOOT button was pressed (read low) at any point during
// the SETUP_MODE_BOOT_WINDOW_MS window after power-on.
static bool setup_mode_requested(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << GPIO_NUM_0,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Press BOOT within %dms to enter setup mode...", SETUP_MODE_BOOT_WINDOW_MS);
    for (int waited_ms = 0; waited_ms < SETUP_MODE_BOOT_WINDOW_MS; waited_ms += SETUP_MODE_POLL_MS) {
        if (gpio_get_level(GPIO_NUM_0) == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(SETUP_MODE_POLL_MS));
    }
    return false;
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the system to stabilize

    // Must run before anything else touches NVS (config_store.c, and
    // nav_init()'s own persisted-mission/heading-PID load below).
    ESP_ERROR_CHECK(config_store_init());

    // Must run here, in this single-threaded setup phase, before imu_task or
    // airspeed_init() (below) create any task that might call
    // i2c_bus_add_device() -- see i2c_bus.h's threading contract.
    i2c_bus_init();
    i2c_bus_scan(); // bring-up diagnostic -- see i2c_bus.h

    // Override compiled-in PID defaults with whatever was last persisted via
    // setup mode, if anything. HEADING_PID_CFG's own load happens inside
    // nav_init() itself, since it's nav.c's own private state.
    pid_gains_t g;
    if (config_store_load_pid_gains("pid_roll", &g)) {
        ROLL_PID_CFG.k_p = g.k_p; ROLL_PID_CFG.k_i = g.k_i; ROLL_PID_CFG.k_d = g.k_d; ROLL_PID_CFG.i_limit = g.i_limit;
        ESP_LOGI(TAG, "Loaded persisted roll PID gains from NVS");
    }
    if (config_store_load_pid_gains("pid_pitch", &g)) {
        PITCH_PID_CFG.k_p = g.k_p; PITCH_PID_CFG.k_i = g.k_i; PITCH_PID_CFG.k_d = g.k_d; PITCH_PID_CFG.i_limit = g.i_limit;
        ESP_LOGI(TAG, "Loaded persisted pitch PID gains from NVS");
    }
    if (config_store_load_pid_gains("pid_aspd", &g)) {
        AIRSPEED_PID_CFG.k_p = g.k_p; AIRSPEED_PID_CFG.k_i = g.k_i; AIRSPEED_PID_CFG.k_d = g.k_d; AIRSPEED_PID_CFG.i_limit = g.i_limit;
        ESP_LOGI(TAG, "Loaded persisted airspeed PID gains from NVS");
    }

    // Per-channel output range/reversal: default to the standard 1000-2000us
    // range with reversed=false when nothing is persisted yet -- setup mode
    // can widen this up to PULSEWIDTH_ABS_MIN/MAX_US per channel. Covers all
    // 8 output channels (6 RC-mirrored + ESC1 + ESC2).
    for (int ch = 0; ch < NUM_OUTPUT_CHANNELS; ch++) {
        channel_cfgs[ch].min_us = PULSEWIDTH_DEFAULT_MIN_US;
        channel_cfgs[ch].max_us = PULSEWIDTH_DEFAULT_MAX_US;
        channel_cfgs[ch].reversed = false;

        char key[16];
        snprintf(key, sizeof(key), "outcfg%d", ch + 1);
        output_cfg_t loaded;
        if (config_store_load_output_cfg(key, &loaded)) {
            channel_cfgs[ch] = loaded;
            ESP_LOGI(TAG, "Loaded persisted output cfg for channel %d from NVS", ch + 1);
        }
    }

    airframe_mode_t loaded_airframe_mode;
    if (config_store_load_airframe_mode(&loaded_airframe_mode)) {
        g_airframe_mode = loaded_airframe_mode;
        ESP_LOGI(TAG, "Loaded persisted airframe mode from NVS: %d", (int)g_airframe_mode);
    }

    airspeed_cfg_t loaded_airspeed_cfg;
    if (config_store_load_airspeed_cfg(&loaded_airspeed_cfg)) {
        g_airspeed_cfg = loaded_airspeed_cfg;
        ESP_LOGI(TAG, "Loaded persisted airspeed cfg from NVS");
    }

    motor_cfg_t loaded_motor_cfg;
    if (config_store_load_motor_cfg(&loaded_motor_cfg)) {
        g_motor_cfg = loaded_motor_cfg;
        ESP_LOGI(TAG, "Loaded persisted motor cfg from NVS: count=%d esc1_is_left=%d",
                 g_motor_cfg.motor_count, g_motor_cfg.esc1_is_left);
    }

    // Pure in-memory update (rc_input_map isn't touched by io_hardware_init()
    // until later), safe to load this early regardless of ordering.
    rc_input_map_cfg_t loaded_rc_map;
    if (config_store_load_rc_input_map(&loaded_rc_map)) {
        for (int i = 0; i < NUM_RC_CHANNELS; i++) {
            rc_input_map[i] = loaded_rc_map.phys_ch[i];
        }
        ESP_LOGI(TAG, "Loaded persisted RC input map from NVS");
    }

    // Checked only after every persisted setting above is loaded into its
    // live global, so setup mode's HTTP API reflects actually-persisted
    // state rather than compiled-in defaults.
    if (setup_mode_requested()) {
        setup_mode_run(); // never returns -- back to flight mode is a physical reset
    }

    // Create the IMU and GPS tasks, pinned to core 1 -- WiFi's own driver/
    // interrupt handling on the ESP32-S3 is tied to core 0, and an unpinned
    // I2C/UART-polling task landing there under WiFi load can starve the I2C
    // driver's recovery path long enough to trip the interrupt watchdog (see
    // setup_mode.c's identical reasoning; flight mode never runs WiFi, so
    // this can't happen here today, but pin it anyway for the same defensive
    // reason control_task already is).
    BaseType_t result = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task! Error: %d", result);
        return;
    } else {
        ESP_LOGI(TAG, "IMU task created successfully");
    }

    result = xTaskCreatePinnedToCore(
        gps_task,
        "gps_task",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS task! Error: %d", result);
        return;
    } else {
        ESP_LOGI(TAG, "GPS task created successfully");
    }

    io_hardware_init();

    nav_init();

    // Scaffolding only: this wires up the mutex+task so airspeed_get()/
    // airspeed_reading() are live, but airspeed_enable() is deliberately
    // NOT called here. That should happen from inside the real sensor
    // driver's init, only after it self-validates the hardware is present
    // (mirroring imu_init()'s WHO_AM_I-gated pattern) -- so throttle stays
    // on the known-good hardcoded path until real, validated hardware
    // exists.
    airspeed_init();

    xTaskCreatePinnedToCore(control_task, "control", 8096, NULL, 1, NULL, 1);
    return;
}
