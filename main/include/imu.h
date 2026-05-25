#ifndef IMU_H
#define IMU_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// I2C config
#define IMU_I2C_PORT        I2C_NUM_0
#define IMU_I2C_SDA_PIN     1
#define IMU_I2C_SCL_PIN     2
#define IMU_I2C_FREQ_HZ     400000

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
#define AK09916_CNTL2               0x31
#define AK09916_CNTL3               0x32
#define AK09916_MODE_CONT_10HZ      0x02

#define IMU_SAMPLE_RATE_HZ  (100)
#define IMU_MUTEX_WAIT (15)

typedef struct {
    // Madgwick filter output (in degrees)
    float roll;
    float pitch;
    float yaw;
} imu_data_t;

extern imu_data_t imu_data;
extern SemaphoreHandle_t imu_data_mutex;

bool imu_init(void);
void imu_task(void *pvParameters);

#endif