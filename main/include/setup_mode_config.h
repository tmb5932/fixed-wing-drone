#ifndef SETUP_MODE_CONFIG_H
#define SETUP_MODE_CONFIG_H

// Non-secret, compile-time-tunable setup-mode settings. Committed (unlike
// the old ESP-NOW config-link's PMK/LMK) -- there's no pairing dance and no
// in-flight exposure risk, since this AP only ever exists in a deliberately
// -entered ground setup mode (see setup_mode_requested() in main.c). Change
// the password before fielding this for real; an empty password ("") makes
// the AP open instead of WPA2.
#define SETUP_MODE_AP_SSID "plane-fc-setup"
#define SETUP_MODE_AP_PASS "flightline123"
#define SETUP_MODE_AP_MAX_CONN (4)
#define SETUP_MODE_WIFI_CHANNEL (6)

#endif // SETUP_MODE_CONFIG_H
