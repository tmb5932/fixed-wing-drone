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
 * Build fresh new pid_cfg_t with the given values.
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
 */
float pid_step(pid_cfg_t *cfg, float current, float goal, float dt_s);

#endif // PID_H
