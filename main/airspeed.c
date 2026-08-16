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

static const char *TAG = "AIRSPEED";
bool airspeed_enabled = false;
int16_t airspeed_g = 0;
SemaphoreHandle_t airspeed_mutex = NULL;

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
 * Reads the current airspeed value in cm/s.
 * Returns INT16_MIN on failure.
 */
static int16_t read_airspeed() {
    return 0; // TODO: implement read airspeed function
}

/**
 * Airspeed reading task. Infinite loop
 */
static void airspeed_task(void *_params) {
    int16_t local_airspeed = 0;
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
    BaseType_t ret = xTaskCreate(airspeed_task, "airspeed_task", 2048, NULL, 5, NULL);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Airspeed task creation failed, aborting");
        abort();
    }
}
