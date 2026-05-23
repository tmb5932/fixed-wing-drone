#ifndef GPS_H
#define GPS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define UART_TX_PIN (6)
#define UART_RX_PIN (5)
#define UART_PORT_NUM (UART_NUM_1)
#define UART_BUF_SIZE (2048)
#define UART_BAUD_RATE (9600)

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
} gps_data_t;

extern gps_data_t latest_gps_data;
extern SemaphoreHandle_t gps_data_mutex;

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
