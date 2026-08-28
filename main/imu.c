#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "madgwick_wrapper.h"
#include "imu.h"
#include "globals.h"
#include "sim_config.h"

// 32768/1000 LSB per deg/s, matching the +-1000dps range set via
// GYRO_CONFIG_1 below.
#define GYRO_SCALE (32.8f)
// 32768/4 LSB per g, matching the +-4g range set via ACCEL_CONFIG below.
#define ACCEL_SCALE (8192.0f)

// Madgwick's correction step (accel for roll/pitch, mag for yaw) is a
// fixed-size step per sample, not proportional to how wrong the estimate
// is -- so it converges roughly exponentially, with a time constant set by
// this gain. The library's default (0.1) turned out to take 20-30+ seconds
// to settle a real ~90 degree yaw error at this 100Hz sample rate on the
// bench, which is unusably slow for anything that gets disturbed. Bumped
// up as a first attempt at a much faster settle time; retune from real
// bench behavior (too twitchy/noisy -> lower it, still too slow -> raise
// it further) rather than assuming this value is final.
#define MADGWICK_BETA (1.0f)

static const char *TAG = "IMU";
SemaphoreHandle_t imu_data_mutex;
imu_data_t imu_data = {0};
volatile bool imu_ready = false;
static madgwick_t filter;

// Gated once at boot in imu_init(), same pattern as imu_ready. If the
// AK09916 isn't found, we simply never feed magnetometer data into the
// filter (it falls back to 6-axis fusion internally); there's no need to
// take autonomous mode down over a missing compass.
static bool mag_available = false;

// Last known-good magnetometer sample (microtesla) and when it was taken.
// Fed into the filter every cycle while still fresh, so the filter always
// sees a continuous signal instead of a sample that only updates at the
// magnetometer's own ~10Hz rate.
static float last_mx = 0.0f, last_my = 0.0f, last_mz = 0.0f;
static int64_t last_good_mag_us = 0;

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
static bool mag_init() {
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
        return false;
    }

    // Continuous 100Hz mode
    imu_write(AK09916_I2C_ADDR, AK09916_CNTL2, AK09916_MODE_CONT_10HZ);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Magnetometer initialized");
    return true;
}

/**
 * Reads one magnetometer sample if a fresh one is ready, and caches it into
 * last_mx/my/mz + last_good_mag_us on success. Returns false (and leaves the
 * cache untouched) if there's no new data yet, the read failed, the sensor
 * reports overflow, or the reading is outside a plausible field-strength
 * range for Earth's magnetic field.
 */
static bool read_mag() {
    uint8_t st1 = 0;
    if (imu_read(AK09916_I2C_ADDR, AK09916_ST1, &st1, 1) != ESP_OK) {
        return false;
    }
    if (!(st1 & AK09916_ST1_DRDY_BIT)) {
        return false; // no new sample since we last read
    }

    // HXL..HZH + ST2 read as one burst: ST2 must be read to latch/release
    // the measurement per the AK09916 datasheet, even though we only care
    // about its HOFL bit here. That's registers 0x11-0x18 inclusive (HXL,
    // HXH, HYL, HYH, HZL, HZH, a reserved byte at 0x17, then ST2 at 0x18)
    // -- 8 bytes, not 7; a 7-byte read stops one short of ST2, so the
    // sensor's internal latch never actually releases and it stops posting
    // new DRDY events after the first sample.
    uint8_t raw[8];
    if (imu_read(AK09916_I2C_ADDR, AK09916_HXL, raw, sizeof(raw)) != ESP_OK) {
        return false;
    }
    uint8_t st2 = raw[7];
    if (st2 & AK09916_ST2_HOFL_BIT) {
        return false; // magnetic sensor overflow, reading not trustworthy
    }

    // AK09916 registers are little-endian (low byte first), the opposite of
    // the big-endian ICM-20948 accel/gyro registers read elsewhere in this
    // file -- easy to get backwards.
    int16_t raw_hx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t raw_hy = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t raw_hz = (int16_t)((raw[5] << 8) | raw[4]);

    // Axis remap from the AK09916 die to the ICM-20948 accel/gyro frame.
    // This follows the common convention for this part, but is NOT verified
    // against this specific board -- bench-check against a reference
    // compass before trusting compass-based navigation.
    float mx = (float)raw_hy * MAG_SCALE_UT_PER_LSB;
    float my = (float)raw_hx * MAG_SCALE_UT_PER_LSB;
    float mz = -(float)raw_hz * MAG_SCALE_UT_PER_LSB;

    // Hard-iron correction: subtract the fixed offset from nearby
    // ferrous/current-carrying material before this reading's magnitude is
    // trusted or used. See MAG_HARD_IRON_OFFSET_*_UT in imu.h.
    mx -= MAG_HARD_IRON_OFFSET_X_UT;
    my -= MAG_HARD_IRON_OFFSET_Y_UT;
    mz -= MAG_HARD_IRON_OFFSET_Z_UT;

    float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    if (mag_norm < MAG_MIN_VALID_UT || mag_norm > MAG_MAX_VALID_UT) {
        return false; // implausible for Earth's field, likely bad read/interference
    }

    last_mx = mx;
    last_my = my;
    last_mz = mz;
    last_good_mag_us = esp_timer_get_time();
    return true;
}

// Every MEMS gyro reads a small nonzero rate at rest (a manufacturing/
// thermal offset), and nothing else in this filter corrects it out: roll/
// pitch get pulled back toward gravity every cycle so they only show a
// small steady-state error from it, but yaw only has the magnetometer's
// correction to fight with, and that correction is a fixed-size step per
// sample (scaled by Madgwick's beta) rather than something that scales
// with how wrong the estimate is -- if gyro bias drives error faster than
// that fixed step can claw back, yaw drifts effectively forever. Averaging
// a couple hundred stationary samples at boot and subtracting that offset
// from every later reading is the standard fix.
//
// Bias genuinely shifts boot-to-boot with temperature, so a fresh per-boot
// calibration is more accurate than one hardcoded number -- but that only
// holds if the board is actually still while it happens. In the field the
// plane gets carried/handled right around power-on, so this can't just
// blindly average whatever the first GYRO_CAL_SAMPLES readings are: it
// waits for a rolling window of samples to look genuinely still (low
// max-min range) before it starts accumulating the average, and restarts
// the average (not just the stillness wait) if motion is detected partway
// through, so a bump mid-calibration can't contaminate the result. If it
// never settles within GYRO_CAL_TIMEOUT_MS, falls back to a bench-derived
// hardcoded default (see GYRO_BIAS_*_DPS_DEFAULT in imu.h) rather than
// hanging boot indefinitely or using a garbage in-motion average.
#define GYRO_CAL_SAMPLES (200)
// ~200ms rolling window at IMU_SAMPLE_RATE_HZ, used only to detect
// stillness -- not necessarily the same samples that end up in the average.
#define GYRO_CAL_STILL_WINDOW (20)
#define GYRO_CAL_STILL_THRESHOLD_DPS (1.0f)
#define GYRO_CAL_TIMEOUT_MS (15000)

static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

static void calibrate_gyro_bias() {
    ESP_LOGI(TAG, "Calibrating gyro bias -- waiting for the board to go still...");

    float win_x[GYRO_CAL_STILL_WINDOW], win_y[GYRO_CAL_STILL_WINDOW], win_z[GYRO_CAL_STILL_WINDOW];
    int win_count = 0, win_idx = 0;

    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    int good = 0;
    int64_t start_us = esp_timer_get_time();
    uint8_t raw[12];

    while (good < GYRO_CAL_SAMPLES) {
        if ((esp_timer_get_time() - start_us) > (int64_t)GYRO_CAL_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "Board never settled within %dms -- falling back to bench-derived gyro bias defaults", GYRO_CAL_TIMEOUT_MS);
            gyro_bias_x = GYRO_BIAS_X_DPS_DEFAULT;
            gyro_bias_y = GYRO_BIAS_Y_DPS_DEFAULT;
            gyro_bias_z = GYRO_BIAS_Z_DPS_DEFAULT;
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ));
        icm_select_bank(0);
        if (icm_read(ICM20948_ACCEL_XOUT_H, raw, 12) != ESP_OK) {
            continue;
        }
        int16_t raw_gx = (int16_t)((raw[6]  << 8) | raw[7]);
        int16_t raw_gy = (int16_t)((raw[8]  << 8) | raw[9]);
        int16_t raw_gz = (int16_t)((raw[10] << 8) | raw[11]);
        float gx = (float)raw_gx / GYRO_SCALE;
        float gy = (float)raw_gy / GYRO_SCALE;
        float gz = (float)raw_gz / GYRO_SCALE;

        win_x[win_idx] = gx; win_y[win_idx] = gy; win_z[win_idx] = gz;
        win_idx = (win_idx + 1) % GYRO_CAL_STILL_WINDOW;
        if (win_count < GYRO_CAL_STILL_WINDOW) win_count++;

        bool still = false;
        if (win_count == GYRO_CAL_STILL_WINDOW) {
            float min_x = win_x[0], max_x = win_x[0];
            float min_y = win_y[0], max_y = win_y[0];
            float min_z = win_z[0], max_z = win_z[0];
            for (int i = 1; i < GYRO_CAL_STILL_WINDOW; i++) {
                if (win_x[i] < min_x) { min_x = win_x[i]; }
                if (win_x[i] > max_x) { max_x = win_x[i]; }
                if (win_y[i] < min_y) { min_y = win_y[i]; }
                if (win_y[i] > max_y) { max_y = win_y[i]; }
                if (win_z[i] < min_z) { min_z = win_z[i]; }
                if (win_z[i] > max_z) { max_z = win_z[i]; }
            }
            still = (max_x - min_x) <= GYRO_CAL_STILL_THRESHOLD_DPS
                 && (max_y - min_y) <= GYRO_CAL_STILL_THRESHOLD_DPS
                 && (max_z - min_z) <= GYRO_CAL_STILL_THRESHOLD_DPS;
        }

        if (!still) {
            if (good > 0) {
                ESP_LOGI(TAG, "Motion detected mid-calibration (%d/%d samples) -- restarting average", good, GYRO_CAL_SAMPLES);
            }
            sum_gx = sum_gy = sum_gz = 0.0;
            good = 0;
            continue;
        }

        sum_gx += (double)gx;
        sum_gy += (double)gy;
        sum_gz += (double)gz;
        good++;
    }

    gyro_bias_x = (float)(sum_gx / good);
    gyro_bias_y = (float)(sum_gy / good);
    gyro_bias_z = (float)(sum_gz / good);
    ESP_LOGI(TAG, "Gyro bias: x=%.3f y=%.3f z=%.3f deg/s (%d/%d samples, settled after %lldms)",
        gyro_bias_x, gyro_bias_y, gyro_bias_z, good, GYRO_CAL_SAMPLES,
        (long long)((esp_timer_get_time() - start_us) / 1000));
}

bool imu_init() {
#ifdef IMU_SIMULATED
    ESP_LOGW(TAG, "IMU_SIMULATED is defined (sim_config.h) -- using fake IMU data, no real I2C/hardware I/O");
    imu_data_mutex = xSemaphoreCreateMutex();
    if (imu_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU Mutex!");
        return false;
    }
    imu_ready = true;
    return true;
#else
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
    // ACCEL_CONFIG is [DLPFCFG 2:0][FS_SEL 1:0][FCHOICE]; the old 0x01 here
    // only set FCHOICE (enabled the DLPF) and left FS_SEL at its default 00
    // (+-2g) -- same bug as the old gyro config. +-2g clips on a hard bench
    // jerk or any real in-flight maneuver/gust beyond 1g of extra load.
    // FS_SEL=01 (+-4g) with FCHOICE still set is DLPFCFG(000) FS_SEL(01)
    // FCHOICE(1) = 0b00000011. ACCEL_SCALE above matches this range.
    icm_write(ICM20948_ACCEL_CONFIG, 0x03);

    // Configure gyro
    icm_write(ICM20948_GYRO_SMPLRT_DIV, 10);
    // GYRO_CONFIG_1 is [DLPFCFG 2:0][FS_SEL 1:0][FCHOICE], same layout as
    // ACCEL_CONFIG above -- the old 0x01 here only set FCHOICE and left
    // FS_SEL at its default 00 (+-250dps), which is easy to saturate on a
    // fast hand-flip during bench testing (131 LSB/dps * 250dps is right at
    // the int16 ceiling), and is a real risk in flight too. FS_SEL=10
    // (+-1000dps) with FCHOICE still set is DLPFCFG(000) FS_SEL(10)
    // FCHOICE(1) = 0b00000101. GYRO_SCALE above matches this range.
    icm_write(ICM20948_GYRO_CONFIG_1, 0x05);

    icm_select_bank(0);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "IMU initialized");
    mag_available = mag_init();

    imu_data_mutex = xSemaphoreCreateMutex();
    if (imu_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU Mutex!");
        return false;
    };

    filter = madgwick_create();
    madgwick_begin(filter, IMU_SAMPLE_RATE_HZ);
    madgwick_set_beta(filter, MADGWICK_BETA);

    calibrate_gyro_bias();

    imu_ready = true;
    return true;
#endif
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

    // Do precise sleep timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ);

#ifdef IMU_SIMULATED
    float fake_yaw_deg = 0.0f;
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        fake_yaw_deg = fmodf(fake_yaw_deg + (IMU_SIM_YAW_RATE_DPS / IMU_SAMPLE_RATE_HZ) + 360.0f, 360.0f);

        BaseType_t mret = xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT));
        if (mret == pdTRUE) {
            imu_data.roll = 0.0f;
            imu_data.pitch = 0.0f;
            imu_data.yaw = fake_yaw_deg;
            imu_data.mag_valid = true;
            xSemaphoreGive(imu_data_mutex);
        }
    }
#else
    uint8_t raw[12];

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

            // Gyro to deg/s, with the boot-time bias calibration removed
            // (see calibrate_gyro_bias()) -- otherwise even a small
            // per-axis DC offset integrates into steady, unbounded drift.
            float gx = (float)raw_gx / GYRO_SCALE - gyro_bias_x;
            float gy = (float)raw_gy / GYRO_SCALE - gyro_bias_y;
            float gz = (float)raw_gz / GYRO_SCALE - gyro_bias_z;

            // Pull a fresh magnetometer sample if the AK09916 has one ready
            // (it updates internally at ~10Hz, so most 100Hz cycles won't).
            // Keep feeding the last known-good sample into the filter while
            // it's still fresh, so the filter doesn't see a signal that
            // only updates 1 out of every ~10 cycles; fall back to zeros
            // (which madgwick_update() treats as "no magnetometer", per
            // MadgwickAHRS.cpp) once the cache goes stale.
            if (mag_available) {
                read_mag();
            }
            bool mag_fresh = mag_available &&
                (esp_timer_get_time() - last_good_mag_us) < MAG_STALE_TIMEOUT_US;
            float mx = mag_fresh ? last_mx : 0.0f;
            float my = mag_fresh ? last_my : 0.0f;
            float mz = mag_fresh ? last_mz : 0.0f;

            madgwick_update(filter, gx, gy, gz, ax, ay, az, mx, my, mz);

            BaseType_t ret = xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT));
            if (ret == pdTRUE) {
                imu_data.roll  = madgwick_get_roll(filter);
                imu_data.pitch = madgwick_get_pitch(filter);
                // Madgwick's yaw is positive-counter-clockwise (standard
                // right-handed math convention); heading_to_target() and
                // the rest of the nav stack use positive-clockwise compass
                // bearings (0=N, 90=E, ...). Negate here, after fusion, to
                // convert between the two -- NOT by flipping gz or any
                // other raw sensor input, since those all need to stay in
                // one consistent physical frame for the gyro/accel/mag
                // fusion math itself to stay correct.
                imu_data.yaw = fmodf(-madgwick_get_yaw(filter) + MAG_YAW_OFFSET_DEG + 360.0f, 360.0f);
                imu_data.mag_valid = mag_fresh;
                xSemaphoreGive(imu_data_mutex);
            }
        } else {
            ESP_LOGE(TAG, "failed to read IMU");
        }
    }
#endif
}
