#ifndef GLOBALS_H
#define GLOBALS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// I2C config
#define I2C_PORT    I2C_NUM_0
#define I2C_FREQ_HZ 400000
#define I2C_SCL_PIN GPIO_NUM_11
#define I2C_SDA_PIN GPIO_NUM_10

#endif // GLOBALS_H
