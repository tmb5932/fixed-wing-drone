#include <math.h>
#include "nav_world.h"
#include "gps_math.h"

#define GRAVITY_MPS2 (9.81f)

// Matches EARTH_RADIUS in main/gps_math.c -- kept in step with the same
// equirectangular approximation distance_to_target()/heading_to_target()
// use, so a straight-line true track through world_step() and a
// distance_to_target()/heading_to_target() call against the same two points
// agree with each other.
#define EARTH_RADIUS_M (6371000.0)

world_state_t world_init(double lat_deg, double lon_deg, float heading_deg)
{
    world_state_t world = { .lat_deg = lat_deg, .lon_deg = lon_deg, .heading_deg = heading_deg };
    return world;
}

void world_step(world_state_t *world, float bank_deg, float airspeed_mps, float dt_s)
{
    // Clamp well short of the +-90deg asymptote in tan() -- set_goal_roll_deg()
    // already clamps commanded bank to +-45deg, but the plant's actual angle
    // can transiently overshoot that under a large heading error, and tan()
    // blowing up there would be a simulation artifact, not a real airplane
    // behavior worth modeling.
    float bank_clamped = bank_deg;
    if (bank_clamped > 80.0f) bank_clamped = 80.0f;
    if (bank_clamped < -80.0f) bank_clamped = -80.0f;

    // Standard-rate coordinated turn: turn_rate = g * tan(bank) / airspeed.
    double turn_rate_rad_s = (GRAVITY_MPS2 * tanf(degrees_to_rads(bank_clamped))) / airspeed_mps;
    float turn_rate_dps = (float)rads_to_degrees(turn_rate_rad_s);

    world->heading_deg = fmodf(world->heading_deg + turn_rate_dps * dt_s + 360.0f, 360.0f);

    double heading_rad = degrees_to_rads(world->heading_deg);
    double north_mps = airspeed_mps * cos(heading_rad);
    double east_mps = airspeed_mps * sin(heading_rad);

    double lat_rad = degrees_to_rads(world->lat_deg);
    world->lat_deg += rads_to_degrees((north_mps * dt_s) / EARTH_RADIUS_M);
    world->lon_deg += rads_to_degrees((east_mps * dt_s) / (EARTH_RADIUS_M * cos(lat_rad)));
}
