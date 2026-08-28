#ifndef NAV_H
#define NAV_H

#include <stdbool.h>

typedef struct {
    double lat_deg;
    double lon_deg;
} waypoint_t;

/**
 * Spawns the waypoint-following nav task. Mirrors airspeed.c's
 * self-contained _init() pattern (creates its own resources and task).
 */
void nav_init(void);

/**
 * Clears HEADING_PID_CFG's accumulated history (integral, last_err) without
 * touching mission progress. Call this on the manual->autonomous transition
 * edge, same as the attitude PIDs, so stale error history from a nav task
 * that's been idling in the background doesn't leak into a fresh engage.
 */
void nav_reset(void);

#endif // NAV_H
