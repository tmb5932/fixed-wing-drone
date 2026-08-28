#include <string.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "globals.h"
#include "airspeed.h"
#include "sim_config.h"

static const char *TAG = "AIRSPEED";
bool airspeed_enabled = false;
int16_t airspeed_g = 0;
SemaphoreHandle_t airspeed_mutex = NULL;

// MS4525DO digital differential-pressure sensor, as used on the RMRC/Matek
// ASPD4525-style pitot module. No register addressing -- a plain 4-byte I2C
// read returns [pressure_hi, pressure_lo, temp_hi, temp_lo].
#define MS4525DO_I2C_ADDR (0x28)

// Sensor status is packed into the top 2 bits of the first output byte.
// 0b00 = normal, 0b01 = reserved/command mode, 0b10 = stale (no new
// conversion since last read), 0b11 = fault. Only 0b00 is trusted.
#define MS4525DO_STATUS_MASK   (0xC0)
#define MS4525DO_STATUS_NORMAL (0x00)

// 14-bit pressure count, 10%-90% of full scale maps to [PSI_MIN, PSI_MAX].
#define MS4525DO_COUNT_10PCT (0.1f * 16383.0f)
#define MS4525DO_COUNT_SPAN  (0.8f * 16383.0f)

// Transfer-function-A, bidirectional +/-1 PSI part (MS4525DO-DS5AI001DP),
// the same chip used on the Matek ASPD4525 this board is a clone of. If a
// different pressure-range part ever replaces it, these two must change to
// match its datasheet.
#define MS4525DO_PSI_MIN (-1.0f)
#define MS4525DO_PSI_MAX (1.0f)

#define PSI_TO_PA (6894.757f)

// Fixed sea-level air density. There's no barometer/altitude source wired
// up yet (see README/nav.c), so this doesn't correct for altitude or
// temperature -- a deliberate simplification, worth revisiting once a
// barometer is in place.
#define AIR_DENSITY_KG_M3 (1.225f)

// Differential pressure sensors read a small nonzero bias at true zero
// airspeed (mounting/soldering tolerance), so we average this many samples
// at boot -- while the plane is assumed stationary on the bench -- and
// subtract that offset from every later reading.
#define ZERO_CAL_SAMPLES (50)

// Gated once at boot by airspeed_task(); same permanent-gate pattern as
// imu_ready/mag_available in imu.c. If the sensor never validates,
// airspeed_enable() is simply never called, and the caller's existing
// airspeed_reading()-gated fallback keeps throttle on the known-good path.
static float zero_offset_psi = 0.0f;

/**
 * Gets the current airspeed value in cm/s.
 * Returns INT16_MIN on failure.
 */
int16_t airspeed_get() {
    int16_t local_airspeed = INT16_MIN;
    BaseType_t ret = xSemaphoreTake(airspeed_mutex, pdMS_TO_TICKS(AIRSPEED_MUTEX_WAIT_MS));
    if (ret == pdTRUE) {
        local_airspeed = airspeed_g;
        xSemaphoreGive(airspeed_mutex);
    }
    return local_airspeed;
}

/**
 * Returns true if airspeed sensor is being read, else false
 */
bool airspeed_reading() {
    return airspeed_enabled;
}

/**
 * Enables airspeed reading
 */
void airspeed_enable() {
    airspeed_enabled = true;
}

/**
 * Disables airspeed reading
 */
void airspeed_disable() {
    airspeed_enabled = false;
}

/**
 * Reads one raw sample from the MS4525DO. Returns false (leaving the
 * outputs untouched) on an I2C failure, or if the sensor's own status bits
 * report anything other than "normal" (stale/fault/reserved).
 */
static bool ms4525_read_raw(uint16_t *out_pressure_counts, uint16_t *out_temp_counts) {
    uint8_t raw[4];
    esp_err_t ret = i2c_master_read_from_device(I2C_PORT, MS4525DO_I2C_ADDR, raw, sizeof(raw), pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        return false;
    }

    if ((raw[0] & MS4525DO_STATUS_MASK) != MS4525DO_STATUS_NORMAL) {
        return false;
    }

    *out_pressure_counts = (uint16_t)(((raw[0] & 0x3F) << 8) | raw[1]);
    *out_temp_counts = (uint16_t)(((raw[2] << 8) | raw[3]) >> 5);
    return true;
}

static float ms4525_counts_to_psi(uint16_t pressure_counts) {
    return ((float)pressure_counts - MS4525DO_COUNT_10PCT) * (MS4525DO_PSI_MAX - MS4525DO_PSI_MIN) / MS4525DO_COUNT_SPAN + MS4525DO_PSI_MIN;
}

/**
 * Reads the current airspeed value in cm/s.
 * Returns INT16_MIN on failure.
 */
static int16_t read_airspeed() {
    uint16_t pressure_counts, temp_counts;
    if (!ms4525_read_raw(&pressure_counts, &temp_counts)) {
        return INT16_MIN;
    }

    float diff_press_psi = ms4525_counts_to_psi(pressure_counts) - zero_offset_psi;
    float diff_press_pa = diff_press_psi * PSI_TO_PA;

    // v = sign(dp) * sqrt(2*|dp|/rho) -- signed so a reversed/backwards
    // pitot install reads as a (wrong-signed) speed instead of NaN, which is
    // easier to notice and debug on the bench than a silent stuck value.
    float sign = (diff_press_pa < 0.0f) ? -1.0f : 1.0f;
    float speed_ms = sign * sqrtf(2.0f * fabsf(diff_press_pa) / AIR_DENSITY_KG_M3);

    return (int16_t)(speed_ms * 100.0f);
}

/**
 * Waits for the MS4525DO to report a valid (non-stale, non-fault) sample,
 * then zero-calibrates against ZERO_CAL_SAMPLES readings. Returns false if
 * the sensor never comes up or too many calibration samples fail -- e.g. if
 * the board isn't populated, or the shared I2C bus isn't up yet.
 */
static bool ms4525_init_and_calibrate() {
    uint16_t pressure_counts, temp_counts;
    bool found = false;
    for (int attempt = 0; attempt < 20 && !found; attempt++) {
        found = ms4525_read_raw(&pressure_counts, &temp_counts);
        if (!found) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "MS4525DO not responding, giving up on airspeed");
        return false;
    }

    float sum_psi = 0.0f;
    int good = 0;
    for (int i = 0; i < ZERO_CAL_SAMPLES; i++) {
        if (ms4525_read_raw(&pressure_counts, &temp_counts)) {
            sum_psi += ms4525_counts_to_psi(pressure_counts);
            good++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (good < ZERO_CAL_SAMPLES / 2) {
        ESP_LOGE(TAG, "MS4525DO zero calibration failed (%d/%d good samples)", good, ZERO_CAL_SAMPLES);
        return false;
    }

    zero_offset_psi = sum_psi / (float)good;
    ESP_LOGI(TAG, "MS4525DO zero-calibrated: offset=%.4f PSI over %d/%d samples", zero_offset_psi, good, ZERO_CAL_SAMPLES);
    return true;
}

/**
 * Airspeed reading task. Infinite loop
 */
static void airspeed_task(void *_params) {
    int16_t local_airspeed = 0;

#ifdef AIRSPEED_SIMULATED
    ESP_LOGW(TAG, "AIRSPEED_SIMULATED is defined (sim_config.h) -- using fake airspeed data, no real I2C/hardware I/O");
    airspeed_enable();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(AIRSPEED_SAMPLE_PERIOD_MS));

        local_airspeed = AIRSPEED_SIM_CMS;

        BaseType_t ret = xSemaphoreTake(airspeed_mutex, pdMS_TO_TICKS(AIRSPEED_MUTEX_WAIT_MS));
        if (ret == pdTRUE) {
            airspeed_g = local_airspeed;
            xSemaphoreGive(airspeed_mutex);
        }
    }
#else
    // Validate + zero-calibrate the sensor before ever calling
    // airspeed_enable(). This mirrors imu_init()'s WHO_AM_I-gated pattern:
    // main.c's throttle path only trusts airspeed_get() once
    // airspeed_reading() is true, so until this succeeds, throttle stays on
    // its existing hardcoded fallback. Runs from inside this task (not
    // app_main) so it doesn't matter whether imu_task has finished bringing
    // up the shared I2C bus yet -- the retry loop above tolerates that.
    if (ms4525_init_and_calibrate()) {
        airspeed_enable();
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(AIRSPEED_SAMPLE_PERIOD_MS));
        if (!airspeed_enabled) {
            continue;
        }

        local_airspeed = read_airspeed();

        BaseType_t ret = xSemaphoreTake(airspeed_mutex, pdMS_TO_TICKS(AIRSPEED_MUTEX_WAIT_MS));
        if (ret == pdTRUE) {
            airspeed_g = local_airspeed;
            xSemaphoreGive(airspeed_mutex);
        }

    }
#endif
}

/**
 * Initializes the airspeed reading task and mutex
 */
void airspeed_init() {
    airspeed_mutex = xSemaphoreCreateMutex();
    if (airspeed_mutex == NULL) {
        ESP_LOGE(TAG, "Airspeed mutex creation failed, aborting");
        abort();
    }
    // 2048 was enough when this task did nothing but a stub read; real I2C
    // reads plus float-formatting ESP_LOG calls (e.g. "%.4f") need more
    // headroom than that, matching the other sensor tasks' 4096.
    BaseType_t ret = xTaskCreate(airspeed_task, "airspeed_task", 4096, NULL, 5, NULL);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Airspeed task creation failed, aborting");
        abort();
    }
}
