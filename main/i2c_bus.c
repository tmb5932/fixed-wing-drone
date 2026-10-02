#include "esp_log.h"
#include "i2c_bus.h"
#include "globals.h"

static const char *TAG = "I2C_BUS";

static i2c_master_bus_handle_t bus = NULL;

void i2c_bus_init(void) {
    if (bus != NULL) {
        return; // already created -- see i2c_bus.h's threading contract
    }
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
    ESP_LOGI(TAG, "Shared I2C bus initialized on port %d", I2C_PORT);
}

esp_err_t i2c_bus_add_device(uint8_t addr7, uint32_t clk_speed_hz, i2c_master_dev_handle_t *out_handle) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr7,
        .scl_speed_hz = clk_speed_hz,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, out_handle);
}

void i2c_bus_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C bus (0x08-0x77)...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
            ESP_LOGI(TAG, "  found device at 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "Scan complete: no devices responded anywhere on the bus.");
    } else {
        ESP_LOGI(TAG, "Scan complete: %d device(s) found.", found);
    }
}
