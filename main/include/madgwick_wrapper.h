#ifndef MADGWICK_WRAPPER_H
#define MADGWICK_WRAPPER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* madgwick_t;

madgwick_t madgwick_create(void);
void madgwick_begin(madgwick_t handle, float sampleFrequency);
void madgwick_set_beta(madgwick_t handle, float beta);
// Returns false if the update was rejected (non-finite input) or the filter's
// internal state had to be reset to level after going non-finite -- see
// MadgwickAHRS.h's own comment. Callers should log this, since it means this
// cycle's roll/pitch/yaw isn't a real reading of the sensor's orientation.
bool madgwick_update(madgwick_t handle, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
bool madgwick_update_imu(madgwick_t handle, float gx, float gy, float gz, float ax, float ay, float az);

float madgwick_get_roll(madgwick_t handle);
float madgwick_get_pitch(madgwick_t handle);
float madgwick_get_yaw(madgwick_t handle);

#ifdef __cplusplus
}
#endif

#endif
