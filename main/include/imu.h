#ifndef IMU_H
#define IMU_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ICM-20948 I2C address (AD0 low = 0x68, AD0 high = 0x69)
#define ICM20948_I2C_ADDR   0x69

// ICM-20948 registers (Bank 0)
#define ICM20948_WHO_AM_I           0x00
#define ICM20948_USER_CTRL          0x03
#define ICM20948_LP_CONFIG          0x05
#define ICM20948_PWR_MGMT_1         0x06
#define ICM20948_PWR_MGMT_2         0x07
#define ICM20948_INT_PIN_CFG        0x0F
#define ICM20948_ACCEL_XOUT_H       0x2D
#define ICM20948_GYRO_XOUT_H        0x33
#define ICM20948_REG_BANK_SEL       0x7F

// ICM-20948 registers (Bank 2)
#define ICM20948_GYRO_SMPLRT_DIV    0x00
#define ICM20948_GYRO_CONFIG_1      0x01
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG       0x14

// AK09916 magnetometer
#define AK09916_I2C_ADDR            0x0C
#define AK09916_WHO_AM_I            0x01
#define AK09916_ST1                 0x10
#define AK09916_HXL                 0x11
#define AK09916_HXH                 0x12
#define AK09916_HYL                 0x13
#define AK09916_HYH                 0x14
#define AK09916_HZL                 0x15
#define AK09916_HZH                 0x16
#define AK09916_ST2                 0x18
#define AK09916_CNTL2               0x31
#define AK09916_CNTL3               0x32
#define AK09916_MODE_CONT_10HZ      0x02

#define AK09916_ST1_DRDY_BIT        0x01
#define AK09916_ST2_HOFL_BIT        0x08

// Magnetometer sensitivity, per the AK09916 datasheet
#define MAG_SCALE_UT_PER_LSB        (0.15f)

// Plausibility band for a |B| reading, in microtesla. Earth's field is
// roughly 25-65uT depending on location; this is intentionally loose since
// it's only meant to reject garbage reads (bad I2C, saturated sensor, nearby
// magnetic interference), not to be a precise calibration check.
#define MAG_MIN_VALID_UT            (10.0f)
#define MAG_MAX_VALID_UT            (150.0f)

// How long a previously-good magnetometer sample stays trusted before we
// stop feeding it into the filter and fall back to zeros (6-axis fusion).
#define MAG_STALE_TIMEOUT_US        (500000)

// Bench-tunable correction for the AK09916's yaw zero-reference/direction
// vs. heading_to_target()'s true-north-clockwise convention. Starts at 0;
// calibrate by pointing the nose at known headings and adjusting.
#define MAG_YAW_OFFSET_DEG          (0.0f)

// Hard-iron calibration: a fixed offset from nearby ferrous/current-
// carrying material (servos, motor, wiring, the board itself) that adds
// onto the true field and makes the raw reading's magnitude vary with
// orientation instead of staying constant. Subtracted from the axis-
// remapped reading before use. Bench-derived via (min+max)/2 per axis while
// tumbling the board through as many orientations as possible -- redo this
// if the board's position relative to nearby servos/motor/wiring changes
// (e.g. once mounted in the actual airframe).
#define MAG_HARD_IRON_OFFSET_X_UT   (29.0f)
#define MAG_HARD_IRON_OFFSET_Y_UT   (-9.4f)
#define MAG_HARD_IRON_OFFSET_Z_UT   (-8.6f)

#define IMU_SAMPLE_RATE_HZ  (100)
#define IMU_MUTEX_WAIT (15)

typedef struct {
    // Madgwick filter output (in degrees)
    float roll;
    float pitch;
    float yaw;
    // True when yaw was fused with a trustworthy, non-stale magnetometer
    // sample this cycle (i.e. real compass heading, not gyro-only drift).
    bool mag_valid;
} imu_data_t;

extern imu_data_t imu_data;
extern SemaphoreHandle_t imu_data_mutex;
extern volatile bool imu_ready;

bool imu_init(void);
void imu_task(void *pvParameters);

#endif
