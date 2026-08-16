#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float k_p;
    float k_i;
    float k_d;
    float i_limit;
    float integral;
    float last_err;
    bool first;
} pid_cfg_t;

/**
 * Build a fresh pid_cfg_t with the given gains, I-term limit, and zeroed state.
 */
pid_cfg_t pid_init(float k_p, float k_i, float k_d, float i_limit);

/**
 * Clear accumulated state (integral, last_err) without touching the gains.
 * Call this when re-arming a controller (e.g. switching into autonomous mode)
 * so old error history doesn't leak into the new run.
 */
void pid_reset(pid_cfg_t *cfg);

/**
 * Advance the controller by one timestep.
 *
 * current/goal are in whatever unit the caller is regulating (this project
 * uses degrees). dt_s is the time since the last call, in seconds -- passed
 * in explicitly (rather than assumed) so this module has no dependency on
 * any particular loop rate, which is what makes it possible to run the exact
 * same code in the SITL at a different rate than the firmware if needed.
 */
float pid_step(pid_cfg_t *cfg, float current, float goal, float dt_s);

#endif // PID_H
