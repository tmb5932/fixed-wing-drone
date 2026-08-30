#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "actuator.h"
#include "airframe_params.h"
#include "noise.h"
#include "pid.h"
#include "plant.h"

// Mirrors main/include/pwm_output.h's defines to avoid esp-idf dependencies
#define SERVO_MIN_PULSEWIDTH_US 1000
#define SERVO_MAX_PULSEWIDTH_US 2000
#define SERVO_MIN_DEGREE        -90
#define SERVO_MAX_DEGREE        90
#define CONTROL_TASK_HZ 100

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int main(int argc, char **argv)
{
    const char *axis = "roll";
    float goal_deg = 0.0f;
    float i_limit = 250.0f;         // matches ROLL_PID_CFG/PITCH_PID_CFG's default in main.c
    float noise_deg = 0.0f;         // off by default
    float servo_rate_dps = 600.0f;  // fast enough to be near-ideal
    long seed = 1; 
    char *positional[5];
    int npos = 0;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--axis=", 7) == 0) {
            axis = argv[i] + 7;
        } else if (strncmp(argv[i], "--goal=", 7) == 0) {
            goal_deg = atof(argv[i] + 7);
        } else if (strncmp(argv[i], "--i_limit=", 10) == 0) {
            i_limit = atof(argv[i] + 10);
        } else if (strncmp(argv[i], "--noise_deg=", 12) == 0) {
            noise_deg = atof(argv[i] + 12);
        } else if (strncmp(argv[i], "--servo_rate=", 13) == 0) {
            servo_rate_dps = atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--seed=", 7) == 0) {
            seed = atol(argv[i] + 7);
        } else if (npos < 5) {
            positional[npos++] = argv[i];
        }
    }

    plant_params_t params;
    if (strcmp(axis, "roll") == 0) {
        params = ROLL_PARAMS;
    } else if (strcmp(axis, "pitch") == 0) {
        params = PITCH_PARAMS;
    } else {
        fprintf(stderr, "Unknown --axis=%s (expected roll or pitch)\n", axis);
        return 1;
    }

    float k_p = npos > 0 ? atof(positional[0]) : 10.0f;
    float k_i = npos > 1 ? atof(positional[1]) : 0.0f;
    float k_d = npos > 2 ? atof(positional[2]) : 0.0f;
    float duration_s = npos > 3 ? atof(positional[3]) : 5.0f;
    float initial_disturbance_deg = npos > 4 ? atof(positional[4]) : 20.0f;

    if (npos == 0) {
        fprintf(stderr, "Usage: %s [k_p] [k_i] [k_d] [duration_s] [initial_disturbance_deg] "
                        "[--axis=roll|pitch] [--goal=deg] [--i_limit=us] [--noise_deg=stddev] "
                        "[--servo_rate=deg_per_s] [--seed=n]\n", argv[0]);
        fprintf(stderr, "Using defaults: axis=%s k_p=%.2f k_i=%.2f k_d=%.2f duration=%.1fs "
                        "disturbance=%.1fdeg goal=%.1fdeg i_limit=%.1fus noise=%.2fdeg servo_rate=%.1fdps seed=%ld\n",
                axis, k_p, k_i, k_d, duration_s, initial_disturbance_deg, goal_deg, i_limit,
                noise_deg, servo_rate_dps, seed);
    }

    srand(seed == 0 ? (unsigned)time(NULL) : (unsigned)seed);

    const float dt_s = 1.0f / CONTROL_TASK_HZ;
    const int steps = (int)(duration_s / dt_s);
    // main.c's goal_roll_deg/goal_pitch_deg default to 0 (level) and nothing
    // sets them yet, same as this sim's default -- but set_goal_roll_deg() /
    // set_goal_pitch_deg() now exist, so --goal lets you test what the PID
    // does when commanded to bank/pitch to something other than level.

    pid_cfg_t pid = pid_init(k_p, k_i, k_d, i_limit);
    plant_state_t plant = plant_init(initial_disturbance_deg);
    actuator_state_t actuator = actuator_init();

    FILE *out = fopen("output.csv", "w");
    if (!out) {
        perror("fopen output.csv");
        return 1;
    }
    fprintf(out, "t_s,goal_deg,angle_deg,measured_angle_deg,rate_dps,pid_output_us,"
                 "commanded_deflection_deg,actual_deflection_deg\n");

    // Summary stats, gathered as the run progresses -- mirrors nav_sitl_main.c's
    // "Result:" line so both binaries expose pass/fail metrics the same way.
    const float SETTLE_BAND_DEG = 2.0f;
    const float DIVERGED_BOUND_DEG = 150.0f;
    float max_abs_angle_deg = 0.0f;
    float last_outside_t_s = -1.0f;
    bool diverged = false;

    for (int i = 0; i < steps; i++) {
        float t = i * dt_s;

        float true_angle_deg = plant.angle_deg;
        float true_rate_dps = plant.rate_dps;
        float measured_angle_deg = true_angle_deg + gaussian_noise(noise_deg);

        float pid_output_us = pid_step(&pid, measured_angle_deg, goal_deg, dt_s);

        float clipped_us = clampf(pid_output_us,
                                   SERVO_MIN_PULSEWIDTH_US - 1500,
                                   SERVO_MAX_PULSEWIDTH_US - 1500);

        float commanded_deflection_deg = clipped_us * (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE)
                                          / (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US);

        // The physical surface chases the command instead of snapping to it
        actuator_step(&actuator, commanded_deflection_deg, servo_rate_dps, dt_s);

        fprintf(out, "%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.3f,%.3f\n",
                t, goal_deg, true_angle_deg, measured_angle_deg, true_rate_dps, pid_output_us,
                commanded_deflection_deg, actuator.deflection_deg);

        float abs_angle = fabsf(true_angle_deg);
        if (abs_angle > max_abs_angle_deg) max_abs_angle_deg = abs_angle;
        if (!isfinite(true_angle_deg) || abs_angle > DIVERGED_BOUND_DEG) diverged = true;
        if (fabsf(true_angle_deg - goal_deg) > SETTLE_BAND_DEG) last_outside_t_s = t;

        plant_step(&plant, &params, actuator.deflection_deg, dt_s);
    }

    fclose(out);

    float final_error_deg = fabsf(plant.angle_deg - goal_deg);
    float settle_time_s;
    if (diverged) {
        settle_time_s = -1.0f;
    } else if (last_outside_t_s < 0.0f) {
        settle_time_s = 0.0f; // within band for the whole run
    } else if (last_outside_t_s >= (steps - 1) * dt_s) {
        settle_time_s = -1.0f; // never settled within duration_s
    } else {
        settle_time_s = last_outside_t_s + dt_s;
    }

    printf("Wrote output.csv: axis=%s %d steps, %.1fs, k_p=%.3f k_i=%.3f k_d=%.3f i_limit=%.1fus, "
           "initial disturbance=%.1fdeg, goal=%.1fdeg, noise=%.2fdeg, servo_rate=%.1fdps, seed=%ld\n",
           axis, steps, duration_s, k_p, k_i, k_d, i_limit, initial_disturbance_deg, goal_deg,
           noise_deg, servo_rate_dps, seed);
    printf("Result: max_abs_angle_deg=%.2f final_error_deg=%.2f settle_time_s=%.2f diverged=%s\n",
           max_abs_angle_deg, final_error_deg, settle_time_s, diverged ? "yes" : "no");
    return 0;
}
