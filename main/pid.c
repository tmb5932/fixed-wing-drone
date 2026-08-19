#include <math.h>
#include "pid.h"

pid_cfg_t pid_init(float k_p, float k_i, float k_d, float i_limit)
{
    pid_cfg_t cfg = {
        .k_p = k_p,
        .k_i = k_i,
        .k_d = k_d,
        .i_limit = i_limit,
        .integral = 0,
        .last_err = 0,
        .first = true
    };
    return cfg;
}

void pid_reset(pid_cfg_t *cfg)
{
    cfg->integral = 0;
    cfg->last_err = 0;
    cfg->first = true;
}

float pid_step(pid_cfg_t *cfg, float current, float goal, float dt_s)
{
    float err = goal - current;

    if (cfg->first) {
        cfg->first = false;
        cfg->last_err = err;
    }

    cfg->integral += err * dt_s;

    if (cfg->k_i != 0.0f) {
        float i_bound = fabsf(cfg->i_limit / cfg->k_i);
        if (cfg->integral > i_bound) { cfg->integral = i_bound; }
        if (cfg->integral < -i_bound) { cfg->integral = -i_bound; }
    }

    float p = cfg->k_p * err;
    float i = cfg->k_i * cfg->integral;
    float d = cfg->k_d * ((err - cfg->last_err) / dt_s);

    cfg->last_err = err;

    return p + i + d;
}
