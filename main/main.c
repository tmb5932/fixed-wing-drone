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
#include "flight_log.h"

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

// Last pulse width actually written for each logical output channel, after
// reversal and clipping -- what the servo/ESC is really being sent, as
// opposed to the RC input it was derived from. 0 until first written.
// Reported by the setup-mode API so reversal/range/mapping can be verified
// on screen, not just by watching the servo.
static volatile uint16_t last_output_us[NUM_OUTPUT_CHANNELS];

// Which physical RC input pin (capture-channel index, 0-5) each logical
// RC_* function actually reads from -- see config_store.h's
// rc_input_map_cfg_t. Defaults to the identity mapping (matching this
// project's compiled-in default channel order: RC_AILERON=CH1,
// RC_ELEVATOR=CH2, RC_THROTTLE=CH3, RC_DIAL=CH4, RC_RUDDER=CH5,
// RC_SWITCH=CH6), overridable from setup mode for a receiver whose channel
// order doesn't match, without re-wiring anything.
static uint8_t rc_input_map[NUM_RC_CHANNELS] = {0, 1, 2, 3, 4, 5};

// Which physical servo output pin (0-5 = CH1_OUT_GPIO..CH6_OUT_GPIO) each
// logical RC_* function drives -- see config_store.h's
// servo_output_map_cfg_t. Defaults to the identity mapping (aileron on
// servo_out_1, etc.), overridable from setup mode. Always a permutation.
static uint8_t servo_output_map[NUM_RC_CHANNELS] = {0, 1, 2, 3, 4, 5};

// Autonomous-mode trim centres -- see config_store.h's trim_cfg_t and
// output_ctl.h's TRIM_* bounds. Filled with TRIM_NEUTRAL_US in app_main()
// before any persisted override is loaded.
static trim_cfg_t g_trim_cfg;

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

// Pitch attitude held during the failsafe spiral descent (motors off, see
// nav.h's NAV_FAILSAFE_DESCEND). Slightly nose-down so the glide keeps
// enough airspeed in the bank instead of the pitch loop holding the nose up
// into a stall. Not SITL-validated -- worth confirming on a real airframe.
#define FAILSAFE_DESCENT_PITCH_DEG (-5.0f)

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

// How long the RC link must be silent before it counts as lost (failsafe,
// no-radio safe outputs, trim-capture refusal). 1s rather than the original
// 200ms: brief dropouts of a few frames are normal at range (antenna
// orientation in a bank), and a receiver itself typically holds the last
// values through them -- 200ms was turning ordinary dropouts into failsafe
// manoeuvres in manual flight. Shorter gaps just hold the last inputs.
#define RC_SIGNAL_LOSS_US (1000000u)

// RC gaps longer than this get recorded in the flight log (as a dropout, or
// as a full loss once past RC_SIGNAL_LOSS_US). A healthy PWM receiver pulses
// every ~20ms, so 100ms is ~5 missed frames.
#define RC_DROPOUT_LOG_US (100000u)

// Microseconds since the input's last edge (see rc_input_t.last_update_us
// for why this is 32-bit).
static uint32_t us_since_update(const rc_input_t *in) {
    return (uint32_t)esp_timer_get_time() - in->last_update_us;
}

/**
 * True if the input has been silent for longer than RC_SIGNAL_LOSS_US.
 */
bool is_stale(const rc_input_t *in) {
    return us_since_update(in) > RC_SIGNAL_LOSS_US;
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
    s.radio_connected = sw_input->ever_updated;
    s.signal_stale = s.radio_connected && is_stale(sw_input);
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

// Resolves a logical output channel to the physical output slot it's wired
// to: the 6 RC-mirrored functions go through servo_output_map, ESC1/ESC2
// have dedicated connectors and map to themselves. Single choke point for
// the output remap, same role capture_input_for() plays on the input side.
static int output_slot_for(int ch) {
    return (ch < NUM_RC_CHANNELS) ? servo_output_map[ch] : ch;
}

// Applies channel ch's calibrated reversal + range (channel_cfgs[ch]) to a
// desired pulse width and writes it out to whichever physical pin ch is
// mapped to. Reversal mirrors the value around the channel's own midpoint
// rather than swapping min/max, so min_us < max_us always holds regardless
// of direction.
static void apply_output_cfg(int ch, int desired_us) {
    const output_cfg_t *cfg = &channel_cfgs[ch];
    int val = cfg->reversed ? ((int)cfg->min_us + (int)cfg->max_us - desired_us) : desired_us;
    val = clip(val, cfg->min_us, cfg->max_us);
    update_comparator_value(channel_to_comparator(output_slot_for(ch)), val);
    last_output_us[ch] = (uint16_t)val;
}

uint16_t get_channel_output_us(int ch) {
    return last_output_us[ch];
}

output_cfg_t get_channel_output_cfg(int ch) {
    return channel_cfgs[ch];
}

esp_err_t set_channel_output_cfg(int ch, const output_cfg_t *cfg) {
    if (ch < 0 || ch >= NUM_OUTPUT_CHANNELS || cfg->min_us >= cfg->max_us) {
        return ESP_ERR_INVALID_ARG;
    }

    output_cfg_t validated = {
        .min_us = (uint16_t)clip(cfg->min_us, PULSEWIDTH_ABS_MIN_US, PULSEWIDTH_ABS_MAX_US),
        .max_us = (uint16_t)clip(cfg->max_us, PULSEWIDTH_ABS_MIN_US, PULSEWIDTH_ABS_MAX_US),
        .reversed = cfg->reversed,
    };
    if (validated.min_us >= validated.max_us) {
        return ESP_ERR_INVALID_ARG;
    }

    channel_cfgs[ch] = validated;

    char key[16];
    snprintf(key, sizeof(key), "outcfg%d", ch + 1);
    return config_store_save_output_cfg(key, &validated);
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

// True if map[] holds each of 0..NUM_RC_CHANNELS-1 exactly once.
static bool is_output_permutation(const uint8_t map[NUM_RC_CHANNELS]) {
    bool seen[NUM_RC_CHANNELS] = {0};
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        if (map[i] >= NUM_RC_CHANNELS || seen[map[i]]) {
            return false;
        }
        seen[map[i]] = true;
    }
    return true;
}

servo_output_map_cfg_t get_servo_output_map(void) {
    servo_output_map_cfg_t cfg;
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        cfg.phys_out[i] = servo_output_map[i];
    }
    return cfg;
}

bool set_servo_output_map(const servo_output_map_cfg_t *cfg) {
    if (!is_output_permutation(cfg->phys_out)) {
        return false;
    }
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        servo_output_map[i] = cfg->phys_out[i];
    }
    return config_store_save_servo_output_map(cfg) == ESP_OK;
}

bool trim_channel_allowed(int ch) {
    return ch == RC_AILERON || ch == RC_ELEVATOR || ch == RC_RUDDER;
}

static bool trim_in_bounds(int center_us) {
    return center_us >= TRIM_NEUTRAL_US - TRIM_MAX_OFFSET_US &&
           center_us <= TRIM_NEUTRAL_US + TRIM_MAX_OFFSET_US;
}

trim_cfg_t get_trim_cfg(void) {
    return g_trim_cfg;
}

bool set_trim_center(int ch, uint16_t center_us) {
    if (ch < 0 || ch >= NUM_RC_CHANNELS || !trim_channel_allowed(ch) || !trim_in_bounds(center_us)) {
        return false;
    }
    g_trim_cfg.center_us[ch] = center_us;
    return config_store_save_trim_cfg(&g_trim_cfg) == ESP_OK;
}

// 10 samples, one per 20ms RC frame -- long enough to catch a stick that's
// still being moved, short enough to block an HTTP handler for.
#define TRIM_CAPTURE_SAMPLES (10)
#define TRIM_CAPTURE_SAMPLE_MS (20)
// Max min-to-max spread across the samples before a channel counts as
// "stick being moved" instead of "stick at rest" -- a resting stick jitters
// by only a few us on a typical receiver.
#define TRIM_CAPTURE_MAX_SPREAD_US (10)

bool capture_trim_from_inputs(char *err, size_t err_len) {
    static const char *const names[NUM_RC_CHANNELS] = {
        [RC_AILERON] = "aileron", [RC_ELEVATOR] = "elevator", [RC_RUDDER] = "rudder",
    };
    uint32_t sum[NUM_RC_CHANNELS] = {0};
    uint32_t lo[NUM_RC_CHANNELS], hi[NUM_RC_CHANNELS];
    for (int c = 0; c < NUM_RC_CHANNELS; c++) { lo[c] = UINT32_MAX; hi[c] = 0; }

    for (int s = 0; s < TRIM_CAPTURE_SAMPLES; s++) {
        for (int c = 0; c < NUM_RC_CHANNELS; c++) {
            if (!trim_channel_allowed(c)) continue;
            rc_input_t *in = capture_input_for(c);
            if (!in->ever_updated || is_stale(in)) {
                snprintf(err, err_len, "no live radio signal on %s -- is the transmitter on?", names[c]);
                return false;
            }
            uint32_t us = in->pulse_width_us;
            sum[c] += us;
            if (us < lo[c]) lo[c] = us;
            if (us > hi[c]) hi[c] = us;
        }
        vTaskDelay(pdMS_TO_TICKS(TRIM_CAPTURE_SAMPLE_MS));
    }

    trim_cfg_t next = g_trim_cfg;
    for (int c = 0; c < NUM_RC_CHANNELS; c++) {
        if (!trim_channel_allowed(c)) continue;
        if (hi[c] - lo[c] > TRIM_CAPTURE_MAX_SPREAD_US) {
            snprintf(err, err_len, "%s stick moved during capture (%lu-%lu us) -- let go of the sticks and retry",
                     names[c], (unsigned long)lo[c], (unsigned long)hi[c]);
            return false;
        }
        int avg = (int)((sum[c] + TRIM_CAPTURE_SAMPLES / 2) / TRIM_CAPTURE_SAMPLES);
        if (!trim_in_bounds(avg)) {
            snprintf(err, err_len, "%s reads %d us, more than %d us from %d -- stick not centred, or channel mapping wrong?",
                     names[c], avg, TRIM_MAX_OFFSET_US, TRIM_NEUTRAL_US);
            return false;
        }
        next.center_us[c] = (uint16_t)avg;
    }

    // All-or-nothing: only reached once every channel passed.
    g_trim_cfg = next;
    if (config_store_save_trim_cfg(&g_trim_cfg) != ESP_OK) {
        snprintf(err, err_len, "trim applied live but failed to persist to NVS");
        return false;
    }
    return true;
}

uint32_t get_physical_pulse_width(int phys) {
    return cap_groups[rc_channel_to_capture_group(phys)].inputs[rc_channel_to_capture_channel(phys)].pulse_width_us;
}

// Mirrors throttle_us to ESC1 always, and to ESC2 only in twin-motor mode --
// there's no differential-thrust mixing yet (see output_ctl.h), just an
// identical copy of whatever the single throttle command is. In single-motor
// mode ESC2 is explicitly held at motor-off rather than left at a stale
// value, in case something's plugged into it by mistake.
//
// While a setup-mode ESC test (start_esc_test()) is running, it overrides
// throttle_us entirely: only the ESC under test gets the test throttle and
// the other is held at motor-off, regardless of motor count or stick.
static volatile int esc_test_ch = -1;
static volatile int64_t esc_test_until_us = 0;

// Deadline for a setup-mode control test (start_direction_test() /
// start_level_test()),
// declared up here so start_esc_test() can cancel one -- the two tests
// never run at the same time.
static volatile int64_t ctrl_test_until_us = 0;

static void apply_throttle_to_escs(int throttle_us) {
    if (esc_test_ch >= 0 && esp_timer_get_time() < esc_test_until_us) {
        int test_ch = esc_test_ch;
        int other_ch = (test_ch == ESC1_CH) ? ESC2_CH : ESC1_CH;
        const output_cfg_t *cfg = &channel_cfgs[test_ch];
        int test_us = (int)cfg->min_us + (int)((cfg->max_us - cfg->min_us) * ESC_TEST_THROTTLE_PCT);
        apply_output_cfg(test_ch, test_us);
        apply_output_cfg(other_ch, (int)channel_cfgs[other_ch].min_us);
        return;
    }

    apply_output_cfg(ESC1_CH, throttle_us);
    if (g_motor_cfg.motor_count == 2) {
        apply_output_cfg(ESC2_CH, throttle_us);
    } else {
        apply_output_cfg(ESC2_CH, (int)PULSEWIDTH_DEFAULT_MIN_US);
    }
}

bool start_esc_test(int esc_ch, int duration_ms) {
    if ((esc_ch != ESC1_CH && esc_ch != ESC2_CH) || duration_ms <= 0 || duration_ms > ESC_TEST_MAX_MS) {
        return false;
    }
    // Channel first, deadline second: the pass-through task only acts once
    // the deadline is in the future, so it never pairs a fresh deadline with
    // a stale channel.
    ctrl_test_until_us = 0;
    esc_test_ch = esc_ch;
    esc_test_until_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    return true;
}

void pass_through_inputs(uint32_t ch[NUM_RC_CHANNELS]) {
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        bool never_heard = !capture_input_for(i)->ever_updated;

        if (i == RC_THROTTLE) {
            // Never skipped like the other channels: with no radio yet, the
            // ESCs are actively held at motor-off instead of left untouched,
            // so an ESC test (see apply_throttle_to_escs()) still runs with
            // no receiver connected and is reliably cut back to off once it
            // ends, rather than leaving the motor at test throttle.
            int throttle_us = never_heard ? (int)channel_cfgs[ESC1_CH].min_us : (int)ch[i];

            // RC_THROTTLE's own output slot is a spare pin, not mirrored from
            // the throttle stick -- see channel_output_type()'s comment. Only
            // the dedicated ESC outputs get the throttle value. The spare is
            // explicitly held centered rather than just skipped, since a
            // setup-mode remap can swap it onto a pin that was just driving
            // a real servo, which would otherwise freeze at its last value.
            apply_throttle_to_escs(throttle_us);
            apply_output_cfg(RC_THROTTLE, (int)starting_to_pulse_width(SERVO_TYPE, DEFAULT_STARTING_VALUE));
            continue;
        }
        if (never_heard) {
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
// Reads the current fused attitude. Returns false if the IMU isn't ready or
// its mutex couldn't be taken (callers treat that as a fault).
static bool read_attitude(float *roll_deg, float *pitch_deg)
{
    if (!imu_ready) {
        return false;
    }
    BaseType_t ret = xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT));
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take imu_data_mutex! Error: %d", ret);
        return false;
    }
    *roll_deg = imu_data.roll;
    *pitch_deg = imu_data.pitch;
    xSemaphoreGive(imu_data_mutex);
    return true;
}

// Drives the control surfaces from roll/pitch commands (us offsets from
// trim; positive roll_cmd = roll right, positive pitch_cmd = nose up -- the
// autopilot's convention, since the PIDs compute goal - current with
// positive roll = bank right). Mixing, trim, reversal and output mapping
// all happen here, so it's the single path every autonomous surface
// command takes -- setup mode's direction test drives it directly to check
// that path on the bench.
static void apply_surface_commands(float roll_cmd, float pitch_cmd)
{
    // Trim centres (g_trim_cfg) replace a flat 1500us neutral: with k_i=0 a
    // P/D-only loop can only hold a needed constant surface offset by
    // holding a constant attitude error (offset/k_p degrees), so an
    // untrimmed airframe would fly persistently off its commanded attitude.
    const int ail_center = g_trim_cfg.center_us[RC_AILERON];
    const int ele_center = g_trim_cfg.center_us[RC_ELEVATOR];
    const int rud_center = g_trim_cfg.center_us[RC_RUDDER];

    if (g_airframe_mode == AIRFRAME_AILEVON_MODE) {
        // Combined surfaces: mix roll and pitch into left/right elevon
        // outputs. Sign convention (which physical output is "left" vs
        // "right", and +/- for roll) depends on servo mounting; verify
        // direction on the bench with setup mode's control-direction test.
        // Each elevon's trim centre is captured from its own (transmitter-
        // mixed) input channel, so it's that surface's own neutral.
        apply_output_cfg(RC_AILERON, (int)(ail_center + pitch_cmd - roll_cmd));
        apply_output_cfg(RC_ELEVATOR, (int)(ele_center + pitch_cmd + roll_cmd));
    } else {
        apply_output_cfg(RC_AILERON, (int)(ail_center + roll_cmd));
        apply_output_cfg(RC_ELEVATOR, (int)(ele_center + pitch_cmd));
    }

    apply_output_cfg(RC_RUDDER, rud_center);
}

// The attitude half of the autonomous control law: roll/pitch PIDs toward
// the given goals, then apply_surface_commands(). Shared by real autonomous
// flight and setup mode's level test. Optionally reports the PID commands.
static void apply_attitude_control(float roll_deg, float pitch_deg, float goal_roll, float goal_pitch,
                                   float dt_s, float *out_roll_cmd, float *out_pitch_cmd)
{
    float roll_cmd = pid_step(&ROLL_PID_CFG, roll_deg, goal_roll, dt_s);
    float pitch_cmd = pid_step(&PITCH_PID_CFG, pitch_deg, goal_pitch, dt_s);
    apply_surface_commands(roll_cmd, pitch_cmd);

    if (out_roll_cmd) *out_roll_cmd = roll_cmd;
    if (out_pitch_cmd) *out_pitch_cmd = pitch_cmd;
}

bool update_autonomous_outputs(void)
{
    const float dt_s = 1.0f / CONTROL_TASK_HZ;

    float roll_deg, pitch_deg;
    if (!read_attitude(&roll_deg, &pitch_deg)) {
        return false;
    }

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
    // Failsafe spiral descent (signal lost, home reached or unreachable):
    // motors off, slight nose-down glide; the bank comes from nav.c via
    // goal_roll_deg as usual.
    if (nav_get_failsafe() == NAV_FAILSAFE_DESCEND) {
        apply_throttle_to_escs((int)channel_cfgs[ESC1_CH].min_us);
        apply_attitude_control(roll_deg, pitch_deg, goal_roll_deg, FAILSAFE_DESCENT_PITCH_DEG, dt_s, NULL, NULL);
        return true;
    }

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

    apply_attitude_control(roll_deg, pitch_deg, goal_roll_deg, goal_pitch_deg, dt_s, NULL, NULL);

    return true;
}

// ---- Setup-mode control tests ----
//
// Two bench tests, both with the motors held off:
//  - Direction test (CONTROL_TEST_DIRECTION): open loop, no IMU. Feeds a
//    fixed roll/pitch command straight into apply_surface_commands(), so the
//    operator can check that what the autopilot means by "roll right" / "pull
//    up" really moves the surfaces that way (mixing, reversal, mapping).
//  - Level test (CONTROL_TEST_LEVEL): closed loop. Runs the real attitude
//    PIDs toward wings-level / pitch-zero on live IMU data; tilting the board
//    should make the surfaces push it back toward level. This is what
//    catches an IMU sign mismatch, which the direction test can't.
static volatile control_test_kind_t ctrl_test_kind = CONTROL_TEST_DIRECTION;
static volatile float ctrl_test_roll_cmd = 0.0f;   // direction test only
static volatile float ctrl_test_pitch_cmd = 0.0f;  // direction test only
static volatile bool ctrl_test_reset_pending = false;
static bool ctrl_test_running = false;  // only touched by the stepping task
static volatile control_test_status_t ctrl_test_status;

// Shared start sequence: cancel the ESC test, flag a fresh start, and set
// the deadline last (same ordering rule as start_esc_test()).
static void begin_control_test(control_test_kind_t kind, int duration_ms) {
    esc_test_until_us = 0;
    ctrl_test_kind = kind;
    ctrl_test_reset_pending = true;
    ctrl_test_until_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
}

bool start_direction_test(float roll_cmd_us, float pitch_cmd_us, int duration_ms) {
    if (duration_ms <= 0 || duration_ms > CONTROL_TEST_MAX_MS) {
        return false;
    }
    ctrl_test_roll_cmd = clampf(roll_cmd_us, -CONTROL_TEST_FULL_CMD_US, CONTROL_TEST_FULL_CMD_US);
    ctrl_test_pitch_cmd = clampf(pitch_cmd_us, -CONTROL_TEST_FULL_CMD_US, CONTROL_TEST_FULL_CMD_US);
    begin_control_test(CONTROL_TEST_DIRECTION, duration_ms);
    return true;
}

bool start_level_test(int duration_ms) {
    if (!imu_ready || duration_ms <= 0 || duration_ms > CONTROL_TEST_MAX_MS) {
        return false;
    }
    begin_control_test(CONTROL_TEST_LEVEL, duration_ms);
    return true;
}

void stop_control_test(void) {
    ctrl_test_until_us = 0;
}

control_test_status_t get_control_test_status(void) {
    control_test_status_t s = ctrl_test_status;
    int64_t remaining = ctrl_test_until_us - esp_timer_get_time();
    s.active = remaining > 0;
    s.remaining_ms = s.active ? (int)(remaining / 1000) : 0;
    s.kind = ctrl_test_kind;
    return s;
}

bool control_test_step(float dt_s) {
    if (esp_timer_get_time() >= ctrl_test_until_us) {
        if (ctrl_test_running) {
            // Recentre on the way out: with no radio connected,
            // pass_through_inputs() leaves surface channels untouched, so
            // they'd otherwise sit at the last test deflection.
            ctrl_test_running = false;
            apply_surface_commands(0.0f, 0.0f);
            ESP_LOGI(TAG, "Control test ended");
        }
        return false;
    }

    control_test_kind_t kind = ctrl_test_kind;
    if (ctrl_test_reset_pending) {
        ctrl_test_reset_pending = false;
        pid_reset(&ROLL_PID_CFG);
        pid_reset(&PITCH_PID_CFG);
        ESP_LOGI(TAG, "Control test started: %s", kind == CONTROL_TEST_LEVEL ? "level (IMU + PID)" : "direction (fixed command)");
        ctrl_test_running = true;
    }

    float roll_deg = 0.0f, pitch_deg = 0.0f;
    if (kind == CONTROL_TEST_LEVEL && !read_attitude(&roll_deg, &pitch_deg)) {
        ctrl_test_until_us = 0;
        return false;
    }

    // Motors stay off for the whole test, whatever the throttle stick says.
    apply_output_cfg(ESC1_CH, (int)channel_cfgs[ESC1_CH].min_us);
    apply_output_cfg(ESC2_CH, (int)channel_cfgs[ESC2_CH].min_us);

    float roll_cmd, pitch_cmd;
    if (kind == CONTROL_TEST_LEVEL) {
        apply_attitude_control(roll_deg, pitch_deg, 0.0f, 0.0f, dt_s, &roll_cmd, &pitch_cmd);
    } else {
        roll_cmd = ctrl_test_roll_cmd;
        pitch_cmd = ctrl_test_pitch_cmd;
        apply_surface_commands(roll_cmd, pitch_cmd);
    }

    ctrl_test_status.roll_deg = roll_deg;
    ctrl_test_status.pitch_deg = pitch_deg;
    ctrl_test_status.roll_cmd_us = roll_cmd;
    ctrl_test_status.pitch_cmd_us = pitch_cmd;
    return true;
}

// Records RC link gaps on the switch channel (the one signal loss is judged
// by) in the flight log: short ones as FLOG_RC_DROPOUT, ones that crossed
// RC_SIGNAL_LOSS_US as FLOG_RC_LOST + FLOG_RC_REGAINED. Called every
// control_task cycle (10ms, so gap lengths are accurate to ~10ms).
static void log_rc_gaps(bool signal_stale)
{
    static bool gap_open = false;
    static bool lost_logged = false;
    static uint32_t gap_max_us = 0;

    const rc_input_t *sw = capture_input_for(RC_SWITCH);
    if (!sw->ever_updated) {
        return;
    }
    uint32_t gap_us = us_since_update(sw);
    if (gap_us > RC_DROPOUT_LOG_US) {
        gap_open = true;
        if (gap_us > gap_max_us) gap_max_us = gap_us;
        if (signal_stale && !lost_logged) {
            flight_log_event(FLOG_RC_LOST, 0);
            lost_logged = true;
        }
    } else if (gap_open) {
        flight_log_event(lost_logged ? FLOG_RC_REGAINED : FLOG_RC_DROPOUT, gap_max_us / 1000);
        gap_open = false;
        lost_logged = false;
        gap_max_us = 0;
    }
}

// Radio lost but autonomous can't run (IMU not ready / faulted, or never
// armed): pass-through would keep replaying the last received pulses --
// surfaces frozen mid-deflection and the motor still at the last throttle.
// Instead hold every surface at its trim centre and cut the motors, a
// hands-off glide.
static void apply_no_radio_safe_outputs(void)
{
    apply_surface_commands(0.0f, 0.0f);
    apply_throttle_to_escs((int)channel_cfgs[ESC1_CH].min_us);
}

// Once any critical autonomous-path failure happens (e.g. IMU not ready, or
// a failed mutex take), autonomous mode is locked out for the rest of this
// boot. There's no in-flight recovery for "the IMU never came up" or similar,
// so the plane needs a reset before autonomous can be trusted again.
static bool critical_fault_latched = false;

// Autonomous mode (including the stale-signal failsafe) stays locked out
// until the RC switch has been seen in its manual position, with a live
// signal, at least once since boot. Without this, powering up with the
// switch already in autonomous engages it on its own the moment imu_ready
// goes true -- on the bench that's the throttle jumping to the fallback
// percentage with no pilot action at all. Trade-off: an in-air brownout
// reboot mid-autonomous comes back in manual until the pilot flips the
// switch to manual and back, which is the safe direction to fail.
static bool autonomous_armed = false;

// Locks (non-failsafe) autonomous out until the switch is seen in manual
// again, so the plane never re-engages autonomous on its own:
//  - the switch asks for autonomous but there's no GPS fix (lost mid-flight,
//    or never had one) -- a fix coming back doesn't re-engage it;
//  - the radio comes back during the failsafe spiral descent -- the pilot
//    gets manual control, whatever the switch was left at.
static bool autonomous_lockout = false;

// Latched while the failsafe has been in its spiral descent during the
// current signal loss, so the moment the radio returns can be detected
// without racing nav_task (which clears the failsafe state on its own
// cycle as soon as it sees the signal back).
static bool saw_failsafe_descent = false;

void control_task(void *arg) {
    uint32_t ch[NUM_RC_CHANNELS];
    bool was_autonomous = false;
    bool was_safe_outputs = false;  // only for logging the transition once
    while (1) {
        ch[RC_THROTTLE] = get_channel_pulse_width(RC_THROTTLE);
        ch[RC_AILERON] = get_channel_pulse_width(RC_AILERON);
        ch[RC_ELEVATOR] = get_channel_pulse_width(RC_ELEVATOR);
        ch[RC_RUDDER] = get_channel_pulse_width(RC_RUDDER);
        ch[RC_SWITCH] = get_channel_pulse_width(RC_SWITCH);
        ch[RC_DIAL] = get_channel_pulse_width(RC_DIAL);

        // printf("CH: %lu, %lu, %lu, %lu, %lu, %lu\n", ch[0], ch[1], ch[2], ch[3], ch[4], ch[5]);

        rc_mode_status_t mode_status = get_rc_mode_status();
        log_rc_gaps(mode_status.signal_stale);
        if (!autonomous_armed && mode_status.radio_connected && !mode_status.signal_stale &&
            !autonomous_mode_enabled(ch[RC_SWITCH])) {
            autonomous_armed = true;
            ESP_LOGI(TAG, "RC switch seen in manual -- autonomous mode armed");
        }
        // Signal-loss failsafe (return home, then spiral down -- see nav.h).
        // Only once armed, same gate as autonomous itself: a link that drops
        // before the pilot ever selected manual stays in pass-through.
        bool signal_lost = autonomous_armed && mode_status.signal_stale;
        nav_set_signal_lost(signal_lost);

        bool want_autonomous;
        if (signal_lost) {
            // Failsafe always runs, GPS or not -- without GPS nav.c picks the
            // in-place spiral descent rather than return-to-home.
            want_autonomous = !critical_fault_latched;
            if (nav_get_failsafe() == NAV_FAILSAFE_DESCEND) {
                saw_failsafe_descent = true;
            }
        } else {
            if (saw_failsafe_descent) {
                saw_failsafe_descent = false;
                autonomous_lockout = true;
                flight_log_event(FLOG_AUTONOMOUS_LOCKOUT, 1);
                ESP_LOGW(TAG, "Radio regained during failsafe descent -- manual control; autonomous disabled until the switch goes back to manual");
            }

            bool switch_auto = autonomous_armed && mode_status.autonomous;
            if (!switch_auto) {
                autonomous_lockout = false;  // switch in manual: re-allow
            } else if (!autonomous_lockout && !nav_gps_fix_ok()) {
                autonomous_lockout = true;
                flight_log_event(FLOG_AUTONOMOUS_LOCKOUT, 0);
                ESP_LOGW(TAG, "No GPS fix -- autonomous disabled until the switch goes back to manual");
            }
            want_autonomous = switch_auto && !autonomous_lockout && !critical_fault_latched;
        }

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
            flight_log_event(FLOG_IMU_FAULT, 0);
            want_autonomous = false;
        }

        if (!want_autonomous) {
            if (mode_status.signal_stale) {
                if (!was_safe_outputs) {
                    flight_log_event(FLOG_NO_RADIO_SAFE, 0);
                    ESP_LOGW(TAG, "No radio and autonomous unavailable -- surfaces at trim, motors off");
                }
                was_safe_outputs = true;
                apply_no_radio_safe_outputs();
            } else {
                was_safe_outputs = false;
                // manual pass-through mode from remote controller
                pass_through_inputs(ch);
            }
        } else {
            was_safe_outputs = false;
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
//
// Manual pass-through runs for the whole window (polled at the servo frame
// rate), so a brownout reboot in the air hands the pilot their surfaces back
// as soon as the outputs come up, instead of after the window expires.
#define SETUP_MODE_BOOT_WINDOW_MS (5000)
#define SETUP_MODE_POLL_MS (20)

// Returns true if the BOOT button was pressed (read low) at any point during
// the SETUP_MODE_BOOT_WINDOW_MS window after power-on. Requires
// io_hardware_init() to have already run -- passes RC input through to the
// outputs on every poll.
static bool setup_mode_requested(void) {
    uint32_t ch[NUM_RC_CHANNELS];
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
        for (int i = 0; i < NUM_RC_CHANNELS; i++) {
            ch[i] = get_channel_pulse_width(i);
        }
        pass_through_inputs(ch);
        vTaskDelay(pdMS_TO_TICKS(SETUP_MODE_POLL_MS));
    }
    return false;
}

void app_main(void)
{
    // Nothing slow may run before io_hardware_init() + the pass-through in
    // setup_mode_requested() below: this path is also an in-air brownout
    // recovery, and every millisecond here is a millisecond the pilot has
    // no control surfaces. Only the fast NVS loads the pass-through itself
    // depends on (calibration, channel maps) go first.

    // First: records why we (re)booted -- e.g. a brownout mid-flight --
    // before anything else can go wrong. RAM-only, no NVS needed.
    flight_log_boot();

    // Must run before anything else touches NVS (config_store.c, and
    // nav_init()'s own persisted-mission/heading-PID load below).
    ESP_ERROR_CHECK(config_store_init());

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

    for (int ch = 0; ch < TRIM_CHANNELS; ch++) {
        g_trim_cfg.center_us[ch] = TRIM_NEUTRAL_US;
    }
    trim_cfg_t loaded_trim;
    if (config_store_load_trim_cfg(&loaded_trim)) {
        // Per-channel, so one corrupt entry doesn't throw away the others.
        for (int ch = 0; ch < TRIM_CHANNELS; ch++) {
            if (trim_channel_allowed(ch) && trim_in_bounds(loaded_trim.center_us[ch])) {
                g_trim_cfg.center_us[ch] = loaded_trim.center_us[ch];
            } else if (trim_channel_allowed(ch)) {
                ESP_LOGW(TAG, "Persisted trim for channel %d out of bounds (%u us), using %d",
                         ch + 1, loaded_trim.center_us[ch], TRIM_NEUTRAL_US);
            }
        }
        ESP_LOGI(TAG, "Loaded persisted trim from NVS");
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

    servo_output_map_cfg_t loaded_servo_map;
    if (config_store_load_servo_output_map(&loaded_servo_map)) {
        if (is_output_permutation(loaded_servo_map.phys_out)) {
            for (int i = 0; i < NUM_RC_CHANNELS; i++) {
                servo_output_map[i] = loaded_servo_map.phys_out[i];
            }
            ESP_LOGI(TAG, "Loaded persisted servo output map from NVS");
        } else {
            ESP_LOGW(TAG, "Persisted servo output map is invalid, using default identity map");
        }
    }

    // After every persisted setting above (pass-through needs the
    // calibration and channel maps), before anything slow. Shared by both
    // boot paths -- setup_mode_run() relies on it having already run.
    io_hardware_init();

    // Checked only after every persisted setting above is loaded into its
    // live global, so setup mode's HTTP API reflects actually-persisted
    // state rather than compiled-in defaults. Runs manual pass-through for
    // the whole window.
    if (setup_mode_requested()) {
        setup_mode_run(); // never returns -- back to flight mode is a physical reset
    }

    // Must run here, in this single-threaded setup phase, before imu_task or
    // airspeed_init() (below) create any task that might call
    // i2c_bus_add_device() -- see i2c_bus.h's threading contract. No longer
    // preceded by a 1s settle delay: the 5s BOOT window above already gives
    // the sensors far longer than that to power up.
    i2c_bus_init();
    i2c_bus_scan(); // bring-up diagnostic -- see i2c_bus.h

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
