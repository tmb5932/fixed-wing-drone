#ifndef OUTPUT_CTL_H
#define OUTPUT_CTL_H

#include <stdint.h>
#include "pwm_output.h"
#include "config_store.h"

// Shared RC-channel I/O surface between main.c (normal flight boot) and
// setup_mode.c (setup-mode boot): both bring up the same MCPWM capture/output
// hardware and drive the same input->output pass-through, so this exposes
// exactly what setup_mode.c needs to reuse main.c's implementation instead of
// duplicating it.

#define NUM_RC_CHANNELS (6)

// translating from channel numbers to what it controls
#define CH1_NUM 1
#define CH2_NUM 2
#define CH3_NUM 3
#define CH4_NUM 4
#define CH5_NUM 5
#define CH6_NUM 6

// Used for indexing related values
#define RC_THROTTLE (CH3_NUM - 1)
#define RC_AILERON  (CH1_NUM - 1)
#define RC_ELEVATOR (CH2_NUM - 1)
#define RC_RUDDER   (CH5_NUM - 1)
#define RC_SWITCH   (CH6_NUM - 1)
#define RC_DIAL     (CH4_NUM - 1)

// Two dedicated ESC/motor outputs, separate from the 6 RC-mirrored channels
// above -- there is no corresponding RC *input* channel for these (the
// receiver only has 6), so they're output-only indices. Never pass these to
// any rc_channel_to_capture_*() function -- those remain bounded by
// NUM_RC_CHANNELS. GPIO assignments (confirmed against the current board's
// pinout) live in main.c.
#define NUM_ESC_CHANNELS (2)
#define NUM_OUTPUT_CHANNELS (NUM_RC_CHANNELS + NUM_ESC_CHANNELS)
#define ESC1_CH (NUM_RC_CHANNELS)
#define ESC2_CH (NUM_RC_CHANNELS + 1)

// Brings up the MCPWM capture groups (RC input) and MCPWM output groups
// (servo/ESC), and starts both. Called exactly once per boot, from
// app_main() before the setup-mode BOOT-button window, for both boot paths.
void io_hardware_init(void);

// Only MOTOR_TYPE for ESC1_CH/ESC2_CH; SERVO_TYPE for all 6 RC-mirrored
// channels, including RC_THROTTLE's own output slot (servo_out_3) -- that
// pin used to be a redundant second copy of the ESC signal, now it's a spare
// servo output like the rest (see main.c's pass_through_inputs()). Single
// source of truth for which absolute pulse-width bounds
// (SERVO_MIN/MAX_PULSEWIDTH_US vs MOTOR_MIN/MAX_PULSEWIDTH_US) apply to a
// given channel, used for boot-time output_cfg_t defaulting and for clamping
// calibration input from the setup-mode API.
item_type_t channel_output_type(int ch);

uint32_t get_channel_pulse_width(int ch);

// Passes each channel's current input pulse width straight through to its
// output, through that channel's calibrated range/reversal (see
// config_store.h's output_cfg_t). Used for manual flight control and for
// setup mode's live bench pass-through.
void pass_through_inputs(uint32_t ch[NUM_RC_CHANNELS]);

// Read-only snapshot of channel `ch`'s live calibrated output_cfg_t, for the
// setup-mode HTTP API's GET /api/outputs.
output_cfg_t get_channel_output_cfg(int ch);

// Validates `cfg` (min_us < max_us, clamped to channel_output_type(ch)'s
// absolute bounds), applies it live, and persists it to NVS. Returns
// ESP_ERR_INVALID_ARG (nothing applied) if cfg was rejected, or the NVS
// error (still applied live) if only persisting failed -- distinguished so
// the API doesn't report a storage failure as "invalid range".
esp_err_t set_channel_output_cfg(int ch, const output_cfg_t *cfg);

// Last pulse width actually written to logical output channel `ch`, after
// reversal and clipping (0 if never written yet).
uint16_t get_channel_output_us(int ch);

// Live airframe mixing mode / airspeed-hold config accessors, for the
// setup-mode HTTP API. Setters apply live and persist to NVS, returning
// false (still applies live) if the NVS write failed or the value was
// rejected as invalid.
airframe_mode_t get_airframe_mode(void);
bool set_airframe_mode(airframe_mode_t mode);

airspeed_cfg_t get_airspeed_cfg(void);
bool set_airspeed_cfg(const airspeed_cfg_t *cfg);

// Read accessors for the attitude/airspeed PID gains main.c owns (their
// setters -- set_roll_pid_gains() etc. -- already exist for the old
// config-link caller and are declared extern where needed; these are new,
// for the setup-mode API to prefill its forms with actually-live values).
pid_gains_t get_roll_pid_gains(void);
pid_gains_t get_pitch_pid_gains(void);
pid_gains_t get_airspeed_pid_gains(void);

// Live motor-count / left-right-role config for ESC1/ESC2, for the
// setup-mode HTTP API. esc1_is_left is inert bookkeeping today (both ESCs
// always receive the identical mirrored throttle command -- see
// apply_throttle_to_escs() in main.c); it exists so a future differential-
// thrust mixer has a place to read "which physical connector is which side"
// without a schema change.
motor_cfg_t get_motor_cfg(void);
bool set_motor_cfg(const motor_cfg_t *cfg);

// Setup-mode bench test for identifying which physical motor is ESC1 vs
// ESC2: spins only `esc_ch` (ESC1_CH or ESC2_CH) at ESC_TEST_THROTTLE_PCT of
// its calibrated range for `duration_ms`, holding the other ESC at
// motor-off, overriding the throttle stick for the duration. Takes effect
// through pass_through_inputs(), so only does anything while something is
// calling that (setup mode's pass-through task). Returns false if esc_ch
// isn't an ESC or duration_ms is outside (0, ESC_TEST_MAX_MS].
#define ESC_TEST_THROTTLE_PCT (0.15f)
#define ESC_TEST_MAX_MS (3000)
bool start_esc_test(int esc_ch, int duration_ms);

// Setup-mode control tests (see main.c), both holding the ESCs at
// motor-off and taking over the surfaces from RC pass-through:
//
//  - start_direction_test(): open loop, no IMU. Applies fixed roll/pitch
//    commands (us offsets from trim, clamped to +/-CONTROL_TEST_FULL_CMD_US)
//    through the real surface mixing/trim/reversal/mapping path. Autopilot
//    convention: positive roll = roll right, positive pitch = nose up.
//  - start_level_test(): closed loop. Runs the real roll/pitch PIDs toward
//    level on live IMU data. Returns false if the IMU isn't ready yet.
//
// Both return false if duration_ms is outside (0, CONTROL_TEST_MAX_MS].
// Starting one cancels any running ESC test (and vice versa).
#define CONTROL_TEST_MAX_MS (20000)
#define CONTROL_TEST_FULL_CMD_US (500.0f)
typedef enum {
    CONTROL_TEST_DIRECTION = 0,
    CONTROL_TEST_LEVEL = 1,
} control_test_kind_t;
bool start_direction_test(float roll_cmd_us, float pitch_cmd_us, int duration_ms);
bool start_level_test(int duration_ms);
void stop_control_test(void);

// Call once per pass-through cycle from setup mode, in place of
// pass_through_inputs() whenever it returns true (a test is running and it
// drove the outputs this cycle).
bool control_test_step(float dt_s);

typedef struct {
    bool active;
    int remaining_ms;
    control_test_kind_t kind;
    float roll_deg, pitch_deg;            // live IMU attitude (level test only)
    float roll_cmd_us, pitch_cmd_us;      // command sent, us offset from trim
} control_test_status_t;
control_test_status_t get_control_test_status(void);

// Live RC input channel mapping (which physical pin backs each logical RC_*
// function), for the setup-mode HTTP API. Setter validates each entry is a
// valid capture-channel index (0..NUM_RC_CHANNELS), applies it live, and
// persists to NVS, returning false (still applies live) if the NVS write
// failed or the value was rejected as invalid.
rc_input_map_cfg_t get_rc_input_map(void);
bool set_rc_input_map(const rc_input_map_cfg_t *cfg);

// Live servo output mapping (which physical servo_out pin each logical RC_*
// function drives), for the setup-mode HTTP API. Setter rejects anything
// that isn't a permutation of 0..NUM_RC_CHANNELS-1 (see config_store.h's
// servo_output_map_cfg_t), applies it live, and persists to NVS, returning
// false if rejected (nothing applied) or if the NVS write failed (still
// applied live).
servo_output_map_cfg_t get_servo_output_map(void);
bool set_servo_output_map(const servo_output_map_cfg_t *cfg);

// Autonomous-mode trim centres (see config_store.h's trim_cfg_t). Every
// centre is bounded to TRIM_NEUTRAL_US +/- TRIM_MAX_OFFSET_US: real trim is
// a small correction, so anything past that is treated as a mistake (a
// deflected stick, a mis-mapped channel) rather than silently flown.
#define TRIM_NEUTRAL_US (1500)
#define TRIM_MAX_OFFSET_US (200)

trim_cfg_t get_trim_cfg(void);

// Whether logical channel `ch` is one autonomous mode actually commands
// around a trim centre (aileron, elevator, rudder).
bool trim_channel_allowed(int ch);

// Sets one channel's centre, applies it live and persists it. Returns
// false (nothing applied) if `ch` isn't trimmable or `center_us` is out of
// bounds, or (still applied live) if the NVS write failed.
bool set_trim_center(int ch, uint16_t center_us);

// Samples every trimmable channel's live RC input over ~200ms and, only if
// all of them are present, steady, and in bounds, applies and persists the
// averages as the new trim centres. Intended use: trim the plane out on the
// transmitter in manual flight, land, enter setup mode, centre the sticks,
// capture. On failure nothing is changed and `err` describes why. Blocks
// the caller for the sampling window.
bool capture_trim_from_inputs(char *err, size_t err_len);

// Raw live pulse width for physical capture-channel index `phys` (0-5),
// bypassing rc_input_map entirely -- for setup mode's channel-mapping UI,
// so a bench tester can wiggle a stick and see which physical pin number
// reacts, independent of whatever logical function it's currently mapped to.
uint32_t get_physical_pulse_width(int phys);

// Whether control_task() would currently choose autonomous or manual mode,
// exposed for setup mode's HTTP API so a bench tester can see which way
// their RC switch is set without leaving setup mode (which never runs
// control_task itself). See get_rc_mode_status()'s own comment in main.c.
typedef struct {
    bool autonomous;      // would engage autonomous mode right now
    bool radio_connected; // false until RC_SWITCH's physical pin has ever updated
    bool signal_stale;    // radio_connected but no update in >1s (RC_SIGNAL_LOSS_US, main.c -- the failsafe path)
} rc_mode_status_t;

rc_mode_status_t get_rc_mode_status(void);

#endif // OUTPUT_CTL_H
