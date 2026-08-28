#ifndef SITL_AIRFRAME_PARAMS_H
#define SITL_AIRFRAME_PARAMS_H

#include "plant.h"

// Shared by sitl_main.c (attitude-loop sim) and nav_sitl_main.c (waypoint-
// loop sim) so the two binaries can't quietly drift apart on what the
// airframe's roll/pitch dynamics are assumed to be.
extern const plant_params_t ROLL_PARAMS;
extern const plant_params_t PITCH_PARAMS;

#endif // SITL_AIRFRAME_PARAMS_H
