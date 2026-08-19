#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "madgwick_wrapper.h"
#include "imu.h"
#include "globals.h"

#define GYRO_SCALE (131.0f)
#define ACCEL_SCALE (8192.0f)

static const char *TAG = "IMU";
SemaphoreHandle_t imu_data_mutex;
imu_data_t imu_data = {0};
volatile bool imu_ready = false;
static madgwick_t filter;

// Low level I2C helpers

static esp_err_t imu_write(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, addr, buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t imu_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, buf, len, pdMS_TO_TICKS(10));
}

static esp_err_t icm_write(uint8_t reg, uint8_t val) {
    return imu_write(ICM20948_I2C_ADDR, reg, val);
}

static esp_err_t icm_read(uint8_t reg, uint8_t *buf, size_t len) {
    return imu_read(ICM20948_I2C_ADDR, reg, buf, len);
}

static esp_err_t icm_select_bank(uint8_t bank) {
    return icm_write(ICM20948_REG_BANK_SEL, (bank << 4) & 0x30);
}

// I2C initializaton for the IMU
static void i2c_bus_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

// Magnometer setup (this was awful to do, and im not even using it rn)
static void mag_init() {
    // Enable bypass so AK09916 is visible on I2C bus
    icm_select_bank(0);
    icm_write(ICM20948_USER_CTRL, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    icm_write(ICM20948_INT_PIN_CFG, 0x02);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Reset AK09916
    imu_write(AK09916_I2C_ADDR, AK09916_CNTL3, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify
    uint8_t mag_id = 0;
    imu_read(AK09916_I2C_ADDR, AK09916_WHO_AM_I, &mag_id, 1);
    ESP_LOGI(TAG, "AK09916 WHO_AM_I = 0x%02X (expect 0x09)", mag_id);
    if (mag_id != 0x09) {
        ESP_LOGE(TAG, "AK09916 not found");
        return;
    }

    // Continuous 100Hz mode
    imu_write(AK09916_I2C_ADDR, AK09916_CNTL2, AK09916_MODE_CONT_10HZ);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Magnetometer initialized");
}

bool imu_init() {
    i2c_bus_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify the IMU
    icm_select_bank(0);
    uint8_t who_am_i = 0;
    icm_read(ICM20948_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0xEA) {
        ESP_LOGE(TAG, "ICM-20948 not found: 0x%02X", who_am_i);
        return false;
    }
    ESP_LOGI(TAG, "ICM-20948 found");

    // Reset
    icm_write(ICM20948_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake, auto clock
    icm_write(ICM20948_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Enable accel + gyro
    icm_write(ICM20948_PWR_MGMT_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure accel
    icm_select_bank(2);
    icm_write(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00);
    icm_write(ICM20948_ACCEL_SMPLRT_DIV_2, 10);
    icm_write(ICM20948_ACCEL_CONFIG, 0x01);

    // Configure gyro
    icm_write(ICM20948_GYRO_SMPLRT_DIV, 10);
    // this should have been +- 500 but seems to actually be 250 (or maybe this just command doesnt work)
    icm_write(ICM20948_GYRO_CONFIG_1, 0x01);

    icm_select_bank(0);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "IMU initialized");
    mag_init();

    imu_data_mutex = xSemaphoreCreateMutex();
    if (imu_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU Mutex!");
        return false;
    };

    filter = madgwick_create();
    madgwick_begin(filter, IMU_SAMPLE_RATE_HZ);

    imu_ready = true;
    return true;
}

// Capture loop (task function)
void imu_task(void *pvParameters) {
    bool ret = imu_init();

    if (!ret) {
        // imu_ready stays false, so the control task will never engage
        // autonomous mode; manual RC pass-through doesn't need the IMU, so
        // there's no reason to take the whole system down over this.
        ESP_LOGE(TAG, "Failed to initialize IMU! Autonomous mode unavailable.");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Loop task started");
    uint8_t raw[12];

    // Do precise sleep timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ);

    while (1) {
        // Precise frequency control
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        icm_select_bank(0);
        esp_err_t ret = icm_read(ICM20948_ACCEL_XOUT_H, raw, 12);

        if (ret == ESP_OK) {
            int16_t raw_ax = (int16_t)((raw[0]  << 8) | raw[1]);
            int16_t raw_ay = (int16_t)((raw[2]  << 8) | raw[3]);
            int16_t raw_az = (int16_t)((raw[4]  << 8) | raw[5]);
            int16_t raw_gx = (int16_t)((raw[6]  << 8) | raw[7]);
            int16_t raw_gy = (int16_t)((raw[8]  << 8) | raw[9]);
            int16_t raw_gz = (int16_t)((raw[10] << 8) | raw[11]);

            // 2. Scale to physical units
            float ax = (float)raw_ax / ACCEL_SCALE;
            float ay = (float)raw_ay / ACCEL_SCALE;
            float az = (float)raw_az / ACCEL_SCALE;

            // Gyro to rad/s
            float gx = (float)raw_gx / GYRO_SCALE;
            float gy = (float)raw_gy / GYRO_SCALE;
            float gz = (float)raw_gz / GYRO_SCALE;

            madgwick_update_imu(filter, gx, gy, gz, ax, ay, az);

            BaseType_t ret = xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT));
            if (ret == pdTRUE) {
                imu_data.roll  = madgwick_get_roll(filter);
                imu_data.pitch = madgwick_get_pitch(filter);
                imu_data.yaw = madgwick_get_yaw(filter);
                xSemaphoreGive(imu_data_mutex);
            }
        } else {
            ESP_LOGE(TAG, "failed to read IMU");
        }
    }
}
