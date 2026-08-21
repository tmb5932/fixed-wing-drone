#include "esp_log.h"
#include "auxiliary.h"

/**
 * This file is for the auxiliary GPIOs for the flight controller.
 * There are 10 GPIOs that can be used for various purposes, such as controlling LEDs, buzzers, or other peripherals.
 *     These pins are labeled as AUX_0 to AUX_9 and the number corresponds to the GPIO number on the PCB.
 * There is also a QT port that can be used for anything, and is not connected to the same I2C bus as the other QT ports on the board.
 *     These are labeled as AUX_QT_YELLOW_GPIO and AUX_QT_BLUE_GPIO, matching the respective colors from the PCB.
 *     Ground and 3v3 are still connected in the same way as the other QT ports.
 */

static const char *TAG = "AUX";

static const gpio_num_t aux_gpio_pins[AUX_COUNT] = {
    AUX_0, AUX_1, AUX_2, AUX_3, AUX_4,
    AUX_5, AUX_6, AUX_7, AUX_8, AUX_9
};

void init_auxiliary_gpio(void)
{
    // Initialize the auxiliary GPIOs as outputs
    uint64_t pin_bit_mask = (1ULL << AUX_QT_YELLOW_GPIO) | (1ULL << AUX_QT_BLUE_GPIO);
    for (int i = 0; i < AUX_COUNT; i++) {
        pin_bit_mask |= (1ULL << aux_gpio_pins[i]);
    }

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = pin_bit_mask,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

/**
 * Drives one of the AUX_0-AUX_9 pins (indexed 0-9) high or low.
 * Returns false if aux_index is out of range.
 */
bool aux_set_level(int aux_index, int level)
{
    if (aux_index < 0 || aux_index >= AUX_COUNT) {
        ESP_LOGE(TAG, "Invalid AUX index: %d; should be in [0, %d)", aux_index, AUX_COUNT);
        return false;
    }

    gpio_set_level(aux_gpio_pins[aux_index], level);
    return true;
}

