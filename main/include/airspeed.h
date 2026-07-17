#ifndef AIRSPEED_H
#define AIRSPEED_H

#include <stdint.h>
#include <stdbool.h>

#define AIRSPEED_SAMPLE_RATE_HZ  (100)
#define AIRSPEED_SAMPLE_PERIOD_MS  (1000 / AIRSPEED_SAMPLE_RATE_HZ)

#define AIRSPEED_MUTEX_WAIT_MS (50)

int16_t airspeed_get(void);
bool airspeed_reading(void);
void airspeed_enable(void);
void airspeed_disable(void);
void airspeed_init(void);

#endif // AIRSPEED_H