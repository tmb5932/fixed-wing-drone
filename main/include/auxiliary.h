#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdbool.h>
#include "driver/gpio.h"

#define AUX_0 GPIO_NUM_0
#define AUX_1 GPIO_NUM_1
#define AUX_2 GPIO_NUM_2
#define AUX_3 GPIO_NUM_3
#define AUX_4 GPIO_NUM_4
#define AUX_5 GPIO_NUM_5
#define AUX_6 GPIO_NUM_6
#define AUX_7 GPIO_NUM_7
#define AUX_8 GPIO_NUM_8
#define AUX_9 GPIO_NUM_9

#define AUX_COUNT 10

#define AUX_QT_YELLOW_GPIO GPIO_NUM_12
#define AUX_QT_BLUE_GPIO GPIO_NUM_13

void init_auxiliary_gpio(void);
bool aux_set_level(int aux_index, int level);

#endif // AUXILIARY_H