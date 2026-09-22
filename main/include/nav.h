#ifndef NAV_H
#define NAV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "pid_types.h"

typedef struct {
    double lat_deg;
    double lon_deg;
} waypoint_t;

// Max waypoints per mission. Used to be capped at whatever fit in one
// ESP-NOW config-link SET_MISSION packet; now that missions are set over the
// local setup-mode HTTP API instead, this is a free-standing limit (kept at
// the same value as before, but can be raised later -- NVS/RAM cost is
// trivial, ~16 bytes/waypoint).
#define NAV_MAX_WAYPOINTS (15)

/**
 * Spawns the waypoint-following nav task. Mirrors airspeed.c's
 * self-contained _init() pattern (creates its own resources and task).
 * Also creates nav_config_mutex and attempts to load a persisted
 * mission/heading-PID from NVS, overriding the compiled-in defaults --
 * http_server_start() (setup_mode.c) must run AFTER this, since its mission/
 * heading-PID request handlers depend on nav_config_mutex already existing.
 */
void nav_init(void);

/**
 * Clears HEADING_PID_CFG's accumulated history (integral, last_err) without
 * touching mission progress. Call this on the manual->autonomous transition
 * edge, same as the attitude PIDs, so stale error history from a nav task
 * that's been idling in the background doesn't leak into a fresh engage.
 */
void nav_reset(void);

/**
 * Replaces the whole mission with `wps` (0..NAV_MAX_WAYPOINTS entries;
 * count=0 clears the mission entirely, which step_nav() already treats as a
 * safe wings-level-hold state) and resets mission progress to the first
 * waypoint. `loop`: once the last
 * waypoint is reached, wrap back to the first one and keep going, instead
 * of holding an orbit at the last waypoint indefinitely. Persists to NVS.
 * Returns false (still applies live) if the NVS write failed, or true on
 * full success. Called from http_server.c in response to a POST
 * /api/mission request -- mutex-guarded since nav_task reads the mission
 * every cycle regardless of autonomous mode.
 */
bool nav_set_mission(const waypoint_t *wps, size_t count, bool loop);

/**
 * Merges the fields marked present in `fields_present` (a
 * pid_field_mask_t bitmask) into HEADING_PID_CFG's live gains, leaving
 * absent fields untouched, and persists the merged result to NVS. Returns
 * false (still applies live) if the NVS write failed. Mutex-guarded for the
 * same reason as nav_set_mission().
 */
bool nav_set_heading_pid_gains(uint8_t fields_present, const pid_gains_t *gains);

/**
 * Read-only snapshot of the current mission for the setup-mode HTTP API's
 * GET /api/state. Copies up to max_count waypoints into `out`. Sets
 * *out_count = 0 if the mutex couldn't be taken.
 */
void nav_get_mission(waypoint_t *out, size_t max_count, size_t *out_count, bool *out_loop);

/**
 * Read-only snapshot of HEADING_PID_CFG's current gains, for the setup-mode
 * HTTP API's GET /api/state.
 */
pid_gains_t nav_get_heading_pid_gains(void);

#endif // NAV_H
