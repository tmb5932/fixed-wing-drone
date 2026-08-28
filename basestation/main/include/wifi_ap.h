#ifndef WIFI_AP_H
#define WIFI_AP_H

// Brings up the basestation's own WiFi access point (for the phone) plus
// the STA interface (for ESP-NOW to the FC), both simultaneously via
// WIFI_MODE_APSTA -- see BASESTATION_SPEC.md's "WiFi: basestation hosts its
// own AP" section. Unlike the FC, there's no flight-safety reason to ever
// duty-cycle this: call once from app_main() and leave it running for the
// device's whole lifetime.
//
// Also logs this device's STA MAC address at boot (needed for the one-time
// MAC-bootstrap step -- see link_secrets.h.example and basestation/README.md).
//
// Must be called before espnow_link_init() (which assumes WiFi/ESP-NOW's
// underlying netif/event-loop bring-up already happened here) and before
// http_server_start() (which needs the AP's netif up to actually serve the
// phone).
void wifi_ap_init(void);

#endif // WIFI_AP_H
