#include "madgwick_wrapper.h"
#include "MadgwickAHRS.h"

extern "C" {

madgwick_t madgwick_create(void) {
    return static_cast<madgwick_t>(new Madgwick());
}

void madgwick_begin(madgwick_t handle, float sampleFrequency) {
    static_cast<Madgwick*>(handle)->begin(sampleFrequency);
}

void madgwick_update(madgwick_t handle, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    static_cast<Madgwick*>(handle)->update(gx, gy, gz, ax, ay, az, mx, my, mz);
}

void madgwick_update_imu(madgwick_t handle, float gx, float gy, float gz, float ax, float ay, float az) {
    static_cast<Madgwick*>(handle)->updateIMU(gx, gy, gz, ax, ay, az);
}

float madgwick_get_roll(madgwick_t handle) {
    return static_cast<Madgwick*>(handle)->getRoll();
}

float madgwick_get_pitch(madgwick_t handle) {
    return static_cast<Madgwick*>(handle)->getPitch();
}

float madgwick_get_yaw(madgwick_t handle) {
    return static_cast<Madgwick*>(handle)->getYaw();
}

}
