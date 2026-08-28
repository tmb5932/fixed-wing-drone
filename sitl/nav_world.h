#ifndef SITL_NAV_WORLD_H
#define SITL_NAV_WORLD_H

// Turns a simulated bank angle into a heading change and a lat/lon track,
// so the waypoint-following stack (nav.c's HEADING_PID_CFG cascaded into
// main.c's ROLL_PID_CFG) can be exercised against something other than a
// hand comment. Deliberately as simple as plant.c: a standard-rate
// coordinated turn plus flat-earth position integration, consistent with
// the equirectangular approximation main/gps_math.c already uses for
// distance_to_target()/heading_to_target() -- not a substitute for flight
// testing, just enough fidelity to sanity-check the heading loop's gains
// and the waypoint-acceptance radius before the plane exists to fly.

typedef struct {
    double lat_deg;
    double lon_deg;
    float heading_deg; // true-north-clockwise, [0, 360)
} world_state_t;

world_state_t world_init(double lat_deg, double lon_deg, float heading_deg);

// Advances position/heading by dt_s, assuming a constant-airspeed,
// constant-altitude coordinated turn at the given bank angle (positive =
// right bank = turn right, matching set_goal_roll_deg()'s sign convention).
void world_step(world_state_t *world, float bank_deg, float airspeed_mps, float dt_s);

#endif // SITL_NAV_WORLD_H
