#ifndef SITL_ACTUATOR_H
#define SITL_ACTUATOR_H

// A real servo cannot teleport to a newly commanded position every control
// loop tick. It has physically rotates there, at some maximum angular speed.

typedef struct {
    float deflection_deg; // where the control surface actually is right now
} actuator_state_t;

actuator_state_t actuator_init(void);

// Moves `state->deflection_deg` toward `commanded_deg` by at most
// `max_rate_dps * dt_s` degrees this step. Pass a very large max_rate_dps to
// approximate an instantaneous (ideal) actuator.
void actuator_step(actuator_state_t *state, float commanded_deg, float max_rate_dps, float dt_s);

#endif // SITL_ACTUATOR_H
