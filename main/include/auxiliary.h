#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdbool.h>
#include "driver/gpio.h"

// GPIO0 is physically the board's BOOT button, read as a plain input at boot
// by setup_mode_requested() (main.c) to decide whether to enter setup mode
// -- deliberately not exposed here as AUX_0/a generic auxiliary GPIO, since
// init_auxiliary_gpio() below configures every AUX_* pin as an output, which
// would fight that input configuration. The next board rev (v3.1) removes
// this pin from the AUX header entirely, so it's dropped here now rather
// than carried as a "remember to exclude it" trap.
#define AUX_1 GPIO_NUM_1
#define AUX_2 GPIO_NUM_2
#define AUX_3 GPIO_NUM_3
#define AUX_4 GPIO_NUM_4
#define AUX_5 GPIO_NUM_5
#define AUX_6 GPIO_NUM_6
#define AUX_7 GPIO_NUM_7
#define AUX_8 GPIO_NUM_8
#define AUX_9 GPIO_NUM_9

#define AUX_COUNT 9

#define AUX_QT_YELLOW_GPIO GPIO_NUM_12
#define AUX_QT_BLUE_GPIO GPIO_NUM_13

void init_auxiliary_gpio(void);

// aux_number: 1-9, matching AUX_1-AUX_9 (there is no aux_number 0 -- see the
// GPIO0/BOOT-button note above).
bool aux_set_level(int aux_number, int level);

#endif // AUXILIARY_H