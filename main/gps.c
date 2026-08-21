#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "gps.h"

#define UART_TX_PIN (GPIO_NUM_41)
#define UART_RX_PIN (GPIO_NUM_42)
#define UART_PORT_NUM (UART_NUM_1)
#define UART_BUF_SIZE (2048)
#define UART_BAUD_RATE (9600)

#define GPS_MUTEX_WAIT (500) // in milliseconds

static const char *TAG = "GPS";

gps_data_t latest_gps_data = {0};
SemaphoreHandle_t gps_data_mutex;

static const uint8_t rate_5Hz[] = {
    0xB5,0x62,0x06,0x08,
    0x06,0x00,
    0xC8,0x00,
    0x01,0x00,
    0x00,0x00,
    0xDD,0x68
};

// Disable GGA
static const uint8_t cmd_disable_gga[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F};
// Disable GLL
static const uint8_t cmd_disable_gll[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11};
// Disable GSA
static const uint8_t cmd_disable_gsa[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13};
// Disable GSV
static const uint8_t cmd_disable_gsv[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15};
// Disable VTG
static const uint8_t cmd_disable_vtg[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19};
// Enable RMC
static const uint8_t cmd_enable_rmc[]  = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x01,0xFE,0x17};

void configure_gps(void)
{
    uart_send_bytes(cmd_disable_gga, sizeof(cmd_disable_gga));
    vTaskDelay(pdMS_TO_TICKS(250));
    uart_send_bytes(cmd_disable_gll, sizeof(cmd_disable_gll));
    vTaskDelay(pdMS_TO_TICKS(250));
    uart_send_bytes(cmd_disable_gsa, sizeof(cmd_disable_gsa));
    vTaskDelay(pdMS_TO_TICKS(250));
    uart_send_bytes(cmd_disable_gsv, sizeof(cmd_disable_gsv));
    vTaskDelay(pdMS_TO_TICKS(250));
    uart_send_bytes(cmd_disable_vtg, sizeof(cmd_disable_vtg));
    vTaskDelay(pdMS_TO_TICKS(250));

    uart_send_bytes(cmd_enable_rmc, sizeof(cmd_enable_rmc));
    vTaskDelay(pdMS_TO_TICKS(250));
    uart_send_bytes(rate_5Hz, sizeof(rate_5Hz));
    vTaskDelay(pdMS_TO_TICKS(500));
}


/*
 * Convert NMEA lat/lon format to decimal degrees.
 *
 * Latitude format:  ddmm.mmmm
 * Longitude format: dddmm.mmmm
 */
static double nmea_to_decimal_degrees(const char *nmea, Hemisphere_t hemisphere)
{
    if (nmea == NULL || *nmea == '\0') {
        return 0.0;
    }

    double raw = atof(nmea);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (hemisphere == SOUTH || hemisphere == WEST) {
        decimal = -decimal;
    }

    return decimal;
}

/*
 * Simple parser for RMC sentences.
 *
 * Example:
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 *
 * Fields used:
 *   [2] status     A = valid, V = invalid
 *   [3] latitude
 *   [4] N/S
 *   [5] longitude
 *   [6] E/W
 *   [7] speed in knots
 *   [8] course in degrees
 */
bool parse_nmea_rmc(const char *sentence, gps_data_t *out)
{
    if (sentence == NULL || out == NULL) {
        return false;
    }

    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0) {
        return false;
    }

    // Make a writable copy because strtok modifies the string
    char buf[128];
    size_t len = strlen(sentence);
    if (len >= sizeof(buf)) {
        return false;
    }
    strcpy(buf, sentence);

    char *fields[16] = {0};
    int field_count = 0;

    char *token = strtok(buf, ",");
    while (token != NULL && field_count < 16) {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }

    if (field_count < 9) {
        return false;
    }

    // Status field
    if (fields[2] == NULL || fields[2][0] != 'A') {
        out->valid = false;
        out->latitude_deg = 0.0;
        out->longitude_deg = 0.0;
        out->speed_knots = 0.0;
        out->speed_mph = 0.0;
        out->course_deg = 0.0;
        return true;
    }

    Hemisphere_t ns = (Hemisphere_t) (fields[4] && fields[4][0]) ? fields[4][0] : 'N';
    Hemisphere_t ew = (Hemisphere_t) (fields[6] && fields[6][0]) ? fields[6][0] : 'E';

    out->valid = true;
    out->latitude_deg = nmea_to_decimal_degrees(fields[3], ns);
    out->longitude_deg = nmea_to_decimal_degrees(fields[5], ew);
    out->speed_knots = (fields[7] && *fields[7]) ? atof(fields[7]) : 0.0;
    out->speed_mph = out->speed_knots * 1.15078;
    out->course_deg = (fields[8] && *fields[8]) ? atof(fields[8]) : 0.0;

    return true;
}

esp_err_t uart_send_bytes(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int written = uart_write_bytes(UART_PORT_NUM, (const char *)data, len);

    if (written < 0) {
        return ESP_FAIL;
    }

    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));

    return ESP_OK;
}

int uart_receive_bytes(uint8_t *buffer, size_t max_len, TickType_t timeout)
{
    if (buffer == NULL || max_len == 0) {
        return -1;
    }

    int len = uart_read_bytes(UART_PORT_NUM, buffer, max_len, timeout);

    return len; // returns number of bytes read
}

/**
 * Reads a single line from uart
*/
int uart_read_line(char *out, size_t max_len, TickType_t timeout)
{
    size_t idx = 0;
    uint8_t c;

    while (idx < max_len - 1) {
        int len = uart_read_bytes(UART_PORT_NUM, &c, 1, timeout);
        if (len <= 0) {
            break;
        }

        out[idx++] = (char)c;

        if (c == '\n') {
            break;
        }
    }

    out[idx] = '\0';
    return idx;
}

float degrees_to_rads(float degrees) {
    return degrees * M_PI / 180.0;
}

float rads_to_degrees(float rads) {
    return rads * 180.0 / M_PI;
}

/**
 * Calculates heading from current location to target location in degrees.
 * math gotten from here: https://www.movable-type.co.uk/scripts/latlong.html
 * 
 * Returns heading in degrees from 0 to 360, where 0 is north, 90 is east, etc.
*/
float heading_to_target(float cur_lat, float cur_long, float goal_lat, float goal_long)
{
    float lat1 = degrees_to_rads(cur_lat);
    float lat2 = degrees_to_rads(goal_lat);
    float d_long = degrees_to_rads((goal_long - cur_long));

    float y = sin(d_long) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(d_long);
    float heading_rad = atan2(y, x);
    float heading_deg = fmod((rads_to_degrees(heading_rad) + 360.0), 360.0);

    return heading_deg;
}


void init_gps_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void init_gps(void)
{
    init_gps_uart();
    vTaskDelay(pdMS_TO_TICKS(1000));
    configure_gps();

    gps_data_mutex = xSemaphoreCreateMutex();
    if (gps_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create GPS Mutex!");
        abort();
    };
}

void loop_uart_gps(void)
{
    char line[128];
    gps_data_t gps;

    while (1) 
    {
        int len = uart_read_line(line, sizeof(line), pdMS_TO_TICKS(1000));
        if (len > 0) {
            ESP_LOGD(TAG, "Received GPS line: %s", line);
            if (parse_nmea_rmc(line, &gps)) {
                if (gps.valid) {
                    ESP_LOGI(TAG, "Lat: %.6f, Lon: %.6f, Speed: %.2f mph, Course: %.2f\n",
                        gps.latitude_deg,
                        gps.longitude_deg,
                        gps.speed_mph,
                        gps.course_deg);
                } else {
                    ESP_LOGI(TAG, "RMC parsed, but no valid fix yet\n");
                }
            }
        }
    }
}

bool read_gps(gps_data_t* gps)
{
    char line[128];

    int len = uart_read_line(line, sizeof(line), pdMS_TO_TICKS(1000));
    if (len > 0) {
        ESP_LOGD(TAG, "Received GPS line: %s", line);
        if (parse_nmea_rmc(line, gps)) {
            return true;
        }
    }

    return false;
}

void gps_task(void *pvParameters)
{
    init_gps();

    while (1) {
        gps_data_t gps;
        if (!read_gps(&gps)) {
            ESP_LOGI("GPS", "Failed to get GPS fix...");
            continue;
        }
        if (gps.valid) {
            BaseType_t ret = xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(GPS_MUTEX_WAIT));
            if (ret == pdTRUE) {
                latest_gps_data = gps;
                xSemaphoreGive(gps_data_mutex);
                ESP_LOGI(TAG, "Lat: %.6f, Lon: %.6f, Speed: %.2f mph, Course: %.2f\n",
                    gps.latitude_deg, gps.longitude_deg, gps.speed_mph, gps.course_deg);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
