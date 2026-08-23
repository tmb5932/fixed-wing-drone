#ifndef SITL_PLANT_H
#define SITL_PLANT_H

// A deliberately simple single-axis rotational model -- not full 6-DOF aero.
// See sitl/README.md for the reasoning. Good enough for picking sane PID
// starting gains before first flight; not a substitute for flight testing.

typedef struct {
    float angle_deg;  // current attitude angle (roll or pitch, whichever axis this instance represents)
    float rate_dps;   // angular rate, degrees/second
} plant_state_t;

typedef struct {
    float control_effectiveness; // deg/s^2 of angular accel per degree of control surface deflection
    float damping;                // 1/s -- how strongly angular rate is damped by the airframe itself
    float restoring;              // 1/s^2 -- how strongly angle itself is pulled back toward 0

    // `restoring` is 0 for an axis that's neutrally stable in angle (roll: a
    // banked aircraft doesn't un-bank itself, only rate gets damped) and
    // nonzero for an axis with genuine static stability (pitch: a
    // conventional tail produces a restoring moment toward trim AoA). See
    // sitl/README.md.
} plant_params_t;

plant_state_t plant_init(float initial_angle_deg);

void plant_step(plant_state_t *state, const plant_params_t *params, float deflection_deg, float dt_s);

#endif // SITL_PLANT_H
