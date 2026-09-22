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
// NUM_RC_CHANNELS. GPIO assignments live in main.c pending an exact pinout
// for the current board rev.
#define NUM_ESC_CHANNELS (2)
#define NUM_OUTPUT_CHANNELS (NUM_RC_CHANNELS + NUM_ESC_CHANNELS)
#define ESC1_CH (NUM_RC_CHANNELS)
#define ESC2_CH (NUM_RC_CHANNELS + 1)

// Brings up the MCPWM capture groups (RC input) and MCPWM output groups
// (servo/ESC), and starts both. Called once from app_main() during normal
// flight boot, or from setup_mode_run() during setup-mode boot -- exactly
// one of the two runs per boot.
void io_hardware_init(void);

// Only MOTOR_TYPE for RC_THROTTLE (the ESC channel); SERVO_TYPE for all
// other channels. Single source of truth for which absolute pulse-width
// bounds (SERVO_MIN/MAX_PULSEWIDTH_US vs MOTOR_MIN/MAX_PULSEWIDTH_US) apply
// to a given channel, used for boot-time output_cfg_t defaulting and for
// clamping calibration input from the setup-mode API.
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
// absolute bounds), applies it live, and persists it to NVS. Returns false
// (still applies live) if the NVS write failed, or if cfg was rejected as
// invalid.
bool set_channel_output_cfg(int ch, const output_cfg_t *cfg);

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

#endif // OUTPUT_CTL_H
