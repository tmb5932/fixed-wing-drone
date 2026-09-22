#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "wifi_ap.h"
#include "setup_mode_config.h"

static const char *TAG = "WIFI_AP";

void wifi_ap_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    bool open_network = (strlen(SETUP_MODE_AP_PASS) == 0);
    wifi_config_t ap_cfg = {
        .ap = {
            .channel = SETUP_MODE_WIFI_CHANNEL,
            .max_connection = SETUP_MODE_AP_MAX_CONN,
            .authmode = open_network ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = false },
        },
    };
    strlcpy((char *)ap_cfg.ap.ssid, SETUP_MODE_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(SETUP_MODE_AP_SSID);
    strlcpy((char *)ap_cfg.ap.password, SETUP_MODE_AP_PASS, sizeof(ap_cfg.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP up: SSID=\"%s\" channel=%d %s -- browse to http://192.168.4.1/",
             SETUP_MODE_AP_SSID, SETUP_MODE_WIFI_CHANNEL, open_network ? "(open network)" : "(WPA2)");
}
