#ifndef GPS_H
#define GPS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "gps_math.h"

typedef enum Hemisphere {
    NORTH = 'N',
    EAST = 'E',
    SOUTH = 'S',
    WEST = 'W'
} Hemisphere_t;

typedef struct {
    bool valid;
    double latitude_deg;
    double longitude_deg;
    double speed_knots;
    double speed_mph;
    double course_deg;
    // esp_timer_get_time() when this fix was stored. gps_task only ever
    // stores valid fixes, so `valid` alone stays true forever after the
    // first fix even if the receiver later loses it -- check freshness
    // against GPS_FIX_MAX_AGE_US instead (see nav.c).
    int64_t timestamp_us;
} gps_data_t;

// A fix older than this is treated as "no fix" (the module runs at 5Hz, so
// this is ~10 missed fixes).
#define GPS_FIX_MAX_AGE_US (2000000)

#define GPS_DATA_MUTEX_WAIT_MS (500) // matches GPS_MUTEX_WAIT in gps.c

extern gps_data_t latest_gps_data;
extern SemaphoreHandle_t gps_data_mutex;
extern volatile bool gps_ready;

bool parse_nmea_rmc(const char *sentence, gps_data_t *out);

esp_err_t uart_send_bytes(const uint8_t *data, size_t len);

int uart_receive_bytes(uint8_t *buffer, size_t max_len, TickType_t timeout);

int uart_read_line(char *out, size_t max_len, TickType_t timeout);

void configure_gps(void);

void init_gps_uart(void);

void init_gps(void);

void loop_uart_gps(void);

bool read_gps(gps_data_t* gps);

void gps_task(void *pvParameters);

#endif // GPS_H
