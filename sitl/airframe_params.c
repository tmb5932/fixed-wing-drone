#include "airframe_params.h"

const plant_params_t ROLL_PARAMS  = { .control_effectiveness = 400.0f, .damping = 3.0f, .restoring = 0.0f };
const plant_params_t PITCH_PARAMS = { .control_effectiveness = 400.0f, .damping = 3.0f, .restoring = 40.0f };
