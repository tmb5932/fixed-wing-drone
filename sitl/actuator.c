#include "actuator.h"

actuator_state_t actuator_init(void)
{
    actuator_state_t state = { .deflection_deg = 0.0f };
    return state;
}

void actuator_step(actuator_state_t *state, float commanded_deg, float max_rate_dps, float dt_s)
{
    float max_step = max_rate_dps * dt_s;
    float error = commanded_deg - state->deflection_deg;

    if (error > max_step) error = max_step;
    if (error < -max_step) error = -max_step;

    state->deflection_deg += error;
}
