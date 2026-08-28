#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "actuator.h"
#include "airframe_params.h"
#include "gps_math.h"
#include "nav_world.h"
#include "noise.h"
#include "pid.h"
#include "plant.h"

// Mirrors main/include/pwm_output.h's defines (see sitl_main.c)
#define SERVO_MIN_PULSEWIDTH_US 1000
#define SERVO_MAX_PULSEWIDTH_US 2000
#define SERVO_MIN_DEGREE        -90
#define SERVO_MAX_DEGREE        90

// Mirrors main.c/nav.c's own #defines -- see sitl/README.md's note on why
// these live as duplicated constants rather than a shared header: nav.c and
// main.c are FreeRTOS/hardware tasks that can't be linked into a host
// binary directly, the same reason sitl_main.c already reimplements
// control_task's PID/actuator cascade instead of linking main.c.
#define CONTROL_TASK_HZ 100
#define NAV_TASK_HZ 10
#define WAYPOINT_ACCEPTANCE_RADIUS_M 30.0
#define MAX_ROLL_GOAL_DEG 45.0f

#define MAX_WAYPOINTS 16

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Computes a waypoint at (bearing_deg, dist_m) from (lat0, lon0), using the
// same equirectangular approximation as distance_to_target()/
// heading_to_target() -- a convenience for sweep testing so scenarios can be
// specified as "45deg right turn, 200m out" instead of hand-picked lat/lon
// pairs that mean nothing to a human reader.
#define EARTH_RADIUS_M (6371000.0)
static void offset_latlon(double lat0, double lon0, float bearing_deg, float dist_m,
                           double *lat_out, double *lon_out)
{
    double bearing_rad = degrees_to_rads(bearing_deg);
    double lat0_rad = degrees_to_rads(lat0);
    *lat_out = lat0 + rads_to_degrees((dist_m * cos(bearing_rad)) / EARTH_RADIUS_M);
    *lon_out = lon0 + rads_to_degrees((dist_m * sin(bearing_rad)) / (EARTH_RADIUS_M * cos(lat0_rad)));
}

int main(int argc, char **argv)
{
    // Heading-loop gains -- default matches nav.c's HEADING_PID_CFG.
    float heading_kp = 1.0f, heading_ki = 0.0f, heading_kd = 0.0f, heading_ilimit = 45.0f;
    // Roll-loop gains -- default matches main.c's ROLL_PID_CFG.
    float roll_kp = 5.0f, roll_ki = 0.0f, roll_kd = 0.4f, roll_ilimit = 250.0f;

    double start_lat = 0.0, start_lon = 0.0;
    float start_heading_deg = 0.0f;
    float airspeed_mps = 12.0f;
    float duration_s = 60.0f;

    // Convenience single-waypoint spec, used when no --wp is given.
    float target_bearing_deg = 90.0f;
    float target_dist_m = 300.0f;

    float roll_noise_deg = 0.0f;
    float heading_noise_deg = 0.0f;
    float servo_rate_dps = 600.0f;
    long seed = 1;

    double wp_lat[MAX_WAYPOINTS], wp_lon[MAX_WAYPOINTS];
    int num_wp = 0;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--heading_kp=", 13) == 0) {
            heading_kp = atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--heading_ki=", 13) == 0) {
            heading_ki = atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--heading_kd=", 13) == 0) {
            heading_kd = atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--heading_ilimit=", 17) == 0) {
            heading_ilimit = atof(argv[i] + 17);
        } else if (strncmp(argv[i], "--roll_kp=", 10) == 0) {
            roll_kp = atof(argv[i] + 10);
        } else if (strncmp(argv[i], "--roll_ki=", 10) == 0) {
            roll_ki = atof(argv[i] + 10);
        } else if (strncmp(argv[i], "--roll_kd=", 10) == 0) {
            roll_kd = atof(argv[i] + 10);
        } else if (strncmp(argv[i], "--roll_ilimit=", 14) == 0) {
            roll_ilimit = atof(argv[i] + 14);
        } else if (strncmp(argv[i], "--start_lat=", 12) == 0) {
            start_lat = atof(argv[i] + 12);
        } else if (strncmp(argv[i], "--start_lon=", 12) == 0) {
            start_lon = atof(argv[i] + 12);
        } else if (strncmp(argv[i], "--heading0=", 11) == 0) {
            start_heading_deg = atof(argv[i] + 11);
        } else if (strncmp(argv[i], "--airspeed=", 11) == 0) {
            airspeed_mps = atof(argv[i] + 11);
        } else if (strncmp(argv[i], "--duration_s=", 13) == 0) {
            duration_s = atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--target_bearing=", 17) == 0) {
            target_bearing_deg = atof(argv[i] + 17);
        } else if (strncmp(argv[i], "--target_dist_m=", 16) == 0) {
            target_dist_m = atof(argv[i] + 16);
        } else if (strncmp(argv[i], "--roll_noise_deg=", 17) == 0) {
            roll_noise_deg = atof(argv[i] + 17);
        } else if (strncmp(argv[i], "--heading_noise_deg=", 20) == 0) {
            heading_noise_deg = atof(argv[i] + 20);
        } else if (strncmp(argv[i], "--servo_rate=", 13) == 0) {
            servo_rate_dps = atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--seed=", 7) == 0) {
            seed = atol(argv[i] + 7);
        } else if (strncmp(argv[i], "--wp=", 5) == 0) {
            if (num_wp >= MAX_WAYPOINTS) {
                fprintf(stderr, "Too many --wp (max %d)\n", MAX_WAYPOINTS);
                return 1;
            }
            if (sscanf(argv[i] + 5, "%lf,%lf", &wp_lat[num_wp], &wp_lon[num_wp]) != 2) {
                fprintf(stderr, "Bad --wp=%s (expected lat,lon)\n", argv[i] + 5);
                return 1;
            }
            num_wp++;
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            fprintf(stderr, "Usage: %s [--start_lat=deg] [--start_lon=deg] [--heading0=deg] "
                            "[--airspeed=mps] [--duration_s=s] [--target_bearing=deg] "
                            "[--target_dist_m=m] [--wp=lat,lon (repeatable, overrides target_bearing/dist)] "
                            "[--heading_kp=] [--heading_ki=] [--heading_kd=] [--heading_ilimit=] "
                            "[--roll_kp=] [--roll_ki=] [--roll_kd=] [--roll_ilimit=] "
                            "[--roll_noise_deg=] [--heading_noise_deg=] [--servo_rate=deg_per_s] [--seed=n]\n",
                    argv[0]);
            return 1;
        }
    }

    if (num_wp == 0) {
        offset_latlon(start_lat, start_lon, target_bearing_deg, target_dist_m,
                       &wp_lat[0], &wp_lon[0]);
        num_wp = 1;
    }

    srand(seed == 0 ? (unsigned)time(NULL) : (unsigned)seed);

    pid_cfg_t heading_pid = pid_init(heading_kp, heading_ki, heading_kd, heading_ilimit);
    pid_cfg_t roll_pid = pid_init(roll_kp, roll_ki, roll_kd, roll_ilimit);
    plant_state_t plant = plant_init(0.0f);
    actuator_state_t actuator = actuator_init();
    world_state_t world = world_init(start_lat, start_lon, start_heading_deg);

    int wp_idx = 0;
    float goal_roll_deg = 0.0f;

    const float inner_dt_s = 1.0f / CONTROL_TASK_HZ;
    const int nav_divider = CONTROL_TASK_HZ / NAV_TASK_HZ;
    const int steps = (int)(duration_s * CONTROL_TASK_HZ);

    // Summary stats, gathered as the run progresses.
    double min_dist_m = 1e18;
    float max_abs_heading_err_deg = 0.0f;
    float first_acceptance_t_s = -1.0f;
    bool reached_final_wp = false;

    FILE *out = fopen("nav_output.csv", "w");
    if (!out) {
        perror("fopen nav_output.csv");
        return 1;
    }
    fprintf(out, "t_s,lat,lon,heading_deg,desired_heading_deg,heading_err_deg,"
                 "goal_roll_deg,roll_deg,dist_to_wp_m,wp_idx\n");

    for (int i = 0; i < steps; i++) {
        float t = i * inner_dt_s;

        if (i % nav_divider == 0) {
            double dist_m = distance_to_target(world.lat_deg, world.lon_deg,
                                                wp_lat[wp_idx], wp_lon[wp_idx]);
            if (dist_m <= WAYPOINT_ACCEPTANCE_RADIUS_M) {
                if (first_acceptance_t_s < 0.0f) first_acceptance_t_s = t;
                if (wp_idx + 1 < num_wp) {
                    wp_idx++;
                    dist_m = distance_to_target(world.lat_deg, world.lon_deg,
                                                 wp_lat[wp_idx], wp_lon[wp_idx]);
                } else {
                    reached_final_wp = true;
                }
            }

            float desired_heading_deg = heading_to_target(world.lat_deg, world.lon_deg,
                                                            wp_lat[wp_idx], wp_lon[wp_idx]);
            float measured_heading_deg = fmodf(world.heading_deg + gaussian_noise(heading_noise_deg) + 360.0f, 360.0f);
            float wrapped_error = heading_error_deg(measured_heading_deg, desired_heading_deg);

            float roll_cmd_deg = pid_step(&heading_pid, 0.0f, wrapped_error, 1.0f / NAV_TASK_HZ);
            goal_roll_deg = clampf(roll_cmd_deg, -MAX_ROLL_GOAL_DEG, MAX_ROLL_GOAL_DEG);

            if (fabsf(wrapped_error) > max_abs_heading_err_deg) max_abs_heading_err_deg = fabsf(wrapped_error);
        }

        float measured_roll_deg = plant.angle_deg + gaussian_noise(roll_noise_deg);
        float pid_output_us = pid_step(&roll_pid, measured_roll_deg, goal_roll_deg, inner_dt_s);

        float clipped_us = clampf(pid_output_us,
                                   SERVO_MIN_PULSEWIDTH_US - 1500,
                                   SERVO_MAX_PULSEWIDTH_US - 1500);
        float commanded_deflection_deg = clipped_us * (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)
                                          / (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US);

        actuator_step(&actuator, commanded_deflection_deg, servo_rate_dps, inner_dt_s);
        plant_step(&plant, &ROLL_PARAMS, actuator.deflection_deg, inner_dt_s);

        world_step(&world, plant.angle_deg, airspeed_mps, inner_dt_s);

        double dist_now_m = distance_to_target(world.lat_deg, world.lon_deg,
                                                wp_lat[wp_idx], wp_lon[wp_idx]);
        if (dist_now_m < min_dist_m) min_dist_m = dist_now_m;

        float desired_heading_now = heading_to_target(world.lat_deg, world.lon_deg,
                                                        wp_lat[wp_idx], wp_lon[wp_idx]);
        float err_now = heading_error_deg(world.heading_deg, desired_heading_now);

        fprintf(out, "%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
                t, world.lat_deg, world.lon_deg, world.heading_deg, desired_heading_now,
                err_now, goal_roll_deg, plant.angle_deg, dist_now_m, wp_idx);
    }

    fclose(out);

    printf("Wrote nav_output.csv: %d waypoints, %.1fs, heading k_p=%.3f k_i=%.3f k_d=%.3f, "
           "roll k_p=%.3f k_i=%.3f k_d=%.3f, airspeed=%.1fmps, roll_noise=%.2fdeg, "
           "heading_noise=%.2fdeg, servo_rate=%.1fdps, seed=%ld\n",
           num_wp, duration_s, heading_kp, heading_ki, heading_kd, roll_kp, roll_ki, roll_kd,
           airspeed_mps, roll_noise_deg, heading_noise_deg, servo_rate_dps, seed);
    printf("Result: final_wp_idx=%d reached_final=%s min_dist_m=%.2f max_abs_heading_err_deg=%.2f "
           "first_acceptance_t_s=%.2f\n",
           wp_idx, reached_final_wp ? "yes" : "no", min_dist_m, max_abs_heading_err_deg,
           first_acceptance_t_s);

    return 0;
}
