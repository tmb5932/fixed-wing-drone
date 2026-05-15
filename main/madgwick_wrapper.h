#ifndef MADGWICK_WRAPPER_H
#define MADGWICK_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void* madgwick_t;

madgwick_t madgwick_create(void);
void madgwick_begin(madgwick_t handle, float sampleFrequency);
void madgwick_update(madgwick_t handle, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void madgwick_update_imu(madgwick_t handle, float gx, float gy, float gz, float ax, float ay, float az);

float madgwick_get_roll(madgwick_t handle);
float madgwick_get_pitch(madgwick_t handle);
float madgwick_get_yaw(madgwick_t handle);

#ifdef __cplusplus
}
#endif

#endif
