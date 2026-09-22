#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "output_ctl.h"
#include "nav.h"
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
        for (int i = 0; i < NUM_RC_CHANNELS; i++) {
            ch[i] = get_channel_pulse_width(i);
        }
        pass_through_inputs(ch);
        vTaskDelay(pdMS_TO_TICKS(SETUP_MODE_PASSTHROUGH_PERIOD_MS));
    }
}

void setup_mode_run(void) {
    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, " Entering SETUP MODE.");
    ESP_LOGI(TAG, " Reset the board (without holding BOOT) to return to flight mode.");
    ESP_LOGI(TAG, "==========================================================");

    io_hardware_init();

    BaseType_t ret = xTaskCreate(setup_mode_passthrough_task, "setup_passthrough", 2048, NULL, 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create setup-mode pass-through task! Error: %d", ret);
    }

    // Safe with no GPS task running: step_nav() early-returns whenever
    // !gps_ready, which it always is here. This just needs nav_config_mutex
    // to exist so nav_set_mission()/nav_set_heading_pid_gains() work from the
    // HTTP handlers below, unchanged from their normal-flight-mode callers.
    nav_init();

    wifi_ap_init();
    http_server_start();

    ESP_LOGI(TAG, "Setup mode ready.");
}
