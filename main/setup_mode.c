#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "output_ctl.h"
#include "nav.h"
#include "gps.h"
#include "airspeed.h"
#include "imu.h"
#include "i2c_bus.h"
#include "wifi_ap.h"
#include "http_server.h"
#include "setup_mode.h"

static const char *TAG = "SETUP_MODE";

#define SETUP_MODE_PASSTHROUGH_PERIOD_MS (20)

// Continuously passes RC input straight through to the outputs, exactly like
// control_task()'s manual mode, so a servo tester or receiver connected on
// the bench actually moves and the setup UI's live pulse-width readouts are
// real -- not just for looks.
static void setup_mode_passthrough_task(void *arg) {
    uint32_t ch[NUM_RC_CHANNELS];
    while (1) {
        // A running control test (see start_direction_test()/start_level_test()) takes
        // over the outputs instead of the RC pass-through.
        if (!control_test_step(SETUP_MODE_PASSTHROUGH_PERIOD_MS / 1000.0f)) {
            for (int i = 0; i < NUM_RC_CHANNELS; i++) {
                ch[i] = get_channel_pulse_width(i);
            }
            pass_through_inputs(ch);
        }
        vTaskDelay(pdMS_TO_TICKS(SETUP_MODE_PASSTHROUGH_PERIOD_MS));
    }
}

void setup_mode_run(void) {
    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, " Entering SETUP MODE.");
    ESP_LOGI(TAG, " Reset the board (without holding BOOT) to return to flight mode.");
    ESP_LOGI(TAG, "==========================================================");

    // io_hardware_init() already ran in app_main(), before the BOOT-button
    // window (so manual pass-through works during it) -- calling it again
    // here would abort on the already-claimed MCPWM slots.

    // Must run here, in this single-threaded setup phase, before airspeed_init()
    // or imu_task (below) create any task that might call
    // i2c_bus_add_device() -- see i2c_bus.h's threading contract.
    i2c_bus_init();
    i2c_bus_scan(); // bring-up diagnostic -- see i2c_bus.h

    // Pinned to core 1, same reasoning as every other task below: WiFi's own
    // driver/interrupt handling on the ESP32-S3 is tied to core 0, and this
    // is the one place in the whole project where WiFi and I2C/UART-polling
    // tasks run at the same time -- an unpinned task landing on core 0 here
    // can starve the I2C driver's recovery path long enough to trip the
    // interrupt watchdog and crash the whole chip under real WiFi load
    // (confirmed on the bench: a phone actively loading the setup page
    // crashed imu_task mid I2C-transaction with exactly this signature).
    // 4096, not 2048: this task also runs the control-direction test
    // (control_test_step()), which does PID math and float-formatting
    // ESP_LOGs -- the same thing that overflowed a 2048 stack elsewhere.
    BaseType_t ret = xTaskCreatePinnedToCore(setup_mode_passthrough_task, "setup_passthrough", 4096, NULL, 1, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create setup-mode pass-through task! Error: %d", ret);
    }

    // step_nav() (nav_task, started below) early-returns whenever !gps_ready,
    // so nav_init() is safe to call regardless of whether a GPS fix is ever
    // acquired here -- it just needs nav_config_mutex to exist so
    // nav_set_mission()/nav_set_heading_pid_gains() work from the HTTP
    // handlers, unchanged from their normal-flight-mode callers.
    nav_init();

    // Unlike normal flight boot, setup mode has a real use for a live GPS fix
    // even though it never engages autonomous flight: the web UI's initial
    // mission-map center (GET /api/state's "gps" field, see http_server.c)
    // uses whatever fix is available here instead of defaulting to the ocean.
    ret = xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS task! Error: %d", ret);
    }

    // Same reasoning as gps_task above: setup mode has a real use for a live
    // airspeed reading (the web UI's Airspeed card, see http_server.c) even
    // though it never drives the throttle output itself. Without this,
    // airspeed_mutex stays NULL and airspeed_get()/airspeed_reading() would
    // hard-assert the first time the API handler called them.
    airspeed_init();

    // Same reasoning again for IMU (the web UI's IMU card): without this,
    // imu_data_mutex stays NULL and imu_ready stays false, so an API handler
    // reading imu_data would either hard-assert or just report stale zeros.
    ret = xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 5, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task! Error: %d", ret);
    }

    wifi_ap_init();
    http_server_start();

    ESP_LOGI(TAG, "Setup mode ready.");

    // Everything setup mode actually does runs in background tasks
    // (setup_mode_passthrough_task, the WiFi/HTTP server's own tasks) --
    // this call itself must still never return, or app_main() falls through
    // into normal flight boot (control_task etc.) alongside setup mode.
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
