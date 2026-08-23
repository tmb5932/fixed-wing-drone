#include "plant.h"

plant_state_t plant_init(float initial_angle_deg)
{
    plant_state_t state = { .angle_deg = initial_angle_deg, .rate_dps = 0.0f };
    return state;
}

void plant_step(plant_state_t *state, const plant_params_t *params, float deflection_deg, float dt_s)
{
    float accel_dps2 = params->control_effectiveness * deflection_deg
                        - params->damping * state->rate_dps
                        - params->restoring * state->angle_deg;

    // Semi-implicit (a.k.a. symplectic) Euler: update rate first, then use the
    // NEW rate to advance angle, rather than using the old rate for both (that's
    // plain forward Euler). One extra line, but noticeably more stable for an
    // oscillatory system at a fixed timestep -- forward Euler tends to leak
    // energy in and slowly blow up a lightly-damped system. RK4 would be more
    // accurate still, but is overkill for tuning gains on a model this simple.
    state->rate_dps += accel_dps2 * dt_s;
    state->angle_deg += state->rate_dps * dt_s;
}
