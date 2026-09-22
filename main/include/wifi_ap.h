#ifndef WIFI_AP_H
#define WIFI_AP_H

/**
 * Brings up the FC's own WiFi access point for setup mode (for a phone or
 * laptop to connect to, then browse to http://192.168.4.1/). Only ever
 * called from setup_mode_run() -- normal flight boot never touches WiFi at
 * all, so there's no runtime gate to get wrong, unlike the old ESP-NOW
 * config link.
 */
void wifi_ap_init(void);

#endif // WIFI_AP_H
