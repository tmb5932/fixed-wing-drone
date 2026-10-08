#ifndef GLOBALS_H
#define GLOBALS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// I2C config
#define I2C_PORT    I2C_NUM_0
// 100kHz, not 400kHz: the v2.1 board relies on the ESP32's weak internal
// pull-ups (see i2c_bus.c), and at 400kHz the extra bus capacitance of the
// cabled airspeed sensor produced regular transaction timeouts / stuck-bus
// errors. Confirmed on the bench: 0 failures at 100kHz with IMU + airspeed
// both connected, in flight and setup mode. Bandwidth is ample for both
// sensors at their current rates. Revisit if the board gets real external
// pull-ups (~2.2-4.7k to 3.3V).
#define I2C_FREQ_HZ 100000
#define I2C_SCL_PIN GPIO_NUM_11
#define I2C_SDA_PIN GPIO_NUM_10

#endif // GLOBALS_H
