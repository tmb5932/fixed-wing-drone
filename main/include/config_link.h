#ifndef CONFIG_LINK_H
#define CONFIG_LINK_H

#include <stdbool.h>
#include <stdint.h>

/**
 * One-time boot setup: brings up the WiFi driver (STA mode, never actually
 * associates to an AP -- this is the standard ESP-NOW-only usage pattern)
 * and opens the ESP-NOW config link immediately. Call once from app_main(),
 * after nav_init() (the link's receive handlers depend on nav's mutex/ready
 * gate already existing).
 *
 * The link opens immediately at boot rather than waiting through the usual
 * 30s-sustained-conditions debounce (see config_link_update_gate()) --
 * powering on the FC is inherently a ground/pre-flight event, so there's no
 * reason to make the user wait here. The debounce still applies to every
 * *re*-open after a close, which is what actually matters for telling
 * "genuinely parked" apart from "momentarily idle mid-flight".
 */
void config_link_init(void);

/**
 * Call once per control_task cycle with this cycle's freshest inputs. Owns
 * the open/close gating state machine: closes the link immediately (no
 * debounce) the instant any required condition breaks, and reopens only
 * after all of them have held continuously for CONFIG_LINK_OPEN_DEBOUNCE_US.
 *
 * gps_fix_valid/gps_speed_kts are ignored (treated as "condition satisfied")
 * when there's no GPS fix at all, since that's a real bench-testing
 * scenario (see the caller in main.c) and not evidence of being airborne.
 */
void config_link_update_gate(bool want_autonomous, uint32_t throttle_pulse_us,
                              bool gps_fix_valid, float gps_speed_kts);

#endif // CONFIG_LINK_H
