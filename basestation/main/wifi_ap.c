#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "wifi_ap.h"
#include "basestation_config.h"
#include "link_secrets.h"

static const char *TAG = "WIFI_AP";

void wifi_ap_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // The AP interface needs a netif (for DHCP to the phone, default
    // gateway, etc). The STA interface is ESP-NOW-only here (mirrors the
    // FC's config_link.c, which also never associates STA to anything) so
    // it doesn't need one.
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    // One-time MAC bootstrap: log this device's STA MAC so it can be copied
    // into the FC's config_link_secrets.h. Always logged (not just on a
    // special build) so a normal boot with a placeholder link_secrets.h is
    // enough to read it off the serial monitor -- see link_secrets.h.example.
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    ESP_LOGI(TAG, "==> basestation STA MAC (put this in the FC's config_link_secrets.h "
                   "CONFIG_LINK_PEER_MAC): %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    bool open_network = (strlen(BASESTATION_AP_PASS) == 0);
    wifi_config_t ap_cfg = {
        .ap = {
            .channel = CONFIG_LINK_WIFI_CHANNEL, // AP+STA share one radio/channel; must match the ESP-NOW channel
            .max_connection = BASESTATION_AP_MAX_CONN,
            .authmode = open_network ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = false },
        },
    };
    strlcpy((char *)ap_cfg.ap.ssid, BASESTATION_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(BASESTATION_AP_SSID);
    strlcpy((char *)ap_cfg.ap.password, BASESTATION_AP_PASS, sizeof(ap_cfg.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP up: SSID=\"%s\" channel=%d %s", BASESTATION_AP_SSID, CONFIG_LINK_WIFI_CHANNEL,
             open_network ? "(open network)" : "(WPA2)");
}
