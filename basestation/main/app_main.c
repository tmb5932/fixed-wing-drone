#include "esp_log.h"
#include "wifi_ap.h"
#include "espnow_link.h"
#include "http_server.h"
#include "store.h"

static const char *TAG = "BASESTATION";

void app_main(void) {
    ESP_ERROR_CHECK(store_init());     // NVS + in-memory state first -- everything below reads/writes it
    wifi_ap_init();                    // AP (phone) + STA (ESP-NOW) together; also logs our MAC for bootstrap
    espnow_link_init();                // ESP-NOW to the FC; never gated, unlike the FC's own link
    http_server_start();               // local HTTP/JSON API + embedded web UI

    ESP_LOGI(TAG, "basestation up");
}
