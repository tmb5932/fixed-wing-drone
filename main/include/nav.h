#ifndef NAV_H
#define NAV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config_link_proto.h"

typedef struct {
    double lat_deg;
    double lon_deg;
} waypoint_t;

// Mission size is capped at whatever fits in one config-link SET_MISSION
// packet -- see shared/config_link_proto.h.
#define NAV_MAX_WAYPOINTS (CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET)

/**
 * Spawns the waypoint-following nav task. Mirrors airspeed.c's
 * self-contained _init() pattern (creates its own resources and task).
 * Also creates nav_config_mutex and attempts to load a persisted
 * mission/heading-PID from NVS, overriding the compiled-in defaults --
 * config_link_init() (main.c) must run AFTER this, since the config-link
 * receive path depends on nav_config_mutex already existing.
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
 * Replaces the whole mission with `wps` (1..NAV_MAX_WAYPOINTS entries) and
 * resets mission progress to the first waypoint. `loop`: once the last
 * waypoint is reached, wrap back to the first one and keep going, instead
 * of holding an orbit at the last waypoint indefinitely. Persists to NVS.
 * Returns false (still applies live) if the NVS write failed, or true on
 * full success. Called from config_link.c in response to a SET_MISSION
 * packet -- mutex-guarded since nav_task reads the mission every cycle
 * regardless of autonomous mode.
 */
bool nav_set_mission(const waypoint_t *wps, size_t count, bool loop);

/**
 * Merges the fields marked present in `fields_present` (a
 * cl_pid_field_mask_t bitmask) into HEADING_PID_CFG's live gains, leaving
 * absent fields untouched, and persists the merged result to NVS. Returns
 * false (still applies live) if the NVS write failed. Mutex-guarded for the
 * same reason as nav_set_mission().
 */
bool nav_set_heading_pid_gains(uint8_t fields_present, const cl_pid_gains_t *gains);

#endif // NAV_H
