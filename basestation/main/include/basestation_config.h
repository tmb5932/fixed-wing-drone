#ifndef BASESTATION_CONFIG_H
#define BASESTATION_CONFIG_H

// Non-secret, compile-time-tunable basestation settings. Unlike
// link_secrets.h this file is committed -- nothing here is
// device-identifying or security-sensitive, it's just local defaults the
// owner may want to tweak for their own field/hardware.

// The basestation's own WiFi access point. The phone connects to this
// (not to any pre-existing network -- see BASESTATION_SPEC.md's "basestation
// hosts its own AP" section). Change the password before fielding this for
// real; an empty password ("") makes the AP open instead of WPA2.
#define BASESTATION_AP_SSID "plane-basestation"
#define BASESTATION_AP_PASS "flightline123"
#define BASESTATION_AP_MAX_CONN (4)

// Default map center used when there's no better data to center on. v1 has
// no FC->basestation telemetry/readback packet (see BASESTATION_SPEC.md's
// "Explicitly out of scope" section), so "last known FC-reported GPS
// position" never actually has data to fall back from in practice today --
// this compiled-in "home field" coordinate is effectively the only map
// center source in v1. Set it to your actual flying field so a fresh/blank
// basestation still opens the map somewhere you can click accurately;
// (0, 0) is the ocean, not a useful default, but it's what ships until you
// edit this.
#define BASESTATION_HOME_LAT_DEFAULT (0.0)
#define BASESTATION_HOME_LON_DEFAULT (0.0)

// How long to wait for a CL_PKT_ACK after an esp_now_send() before giving up
// on this attempt and leaving the item pending for the next retry. Local
// ESP-NOW round-trips are normally well under this even on a busy channel;
// generous headroom costs little since most sends happen in the background.
#define ESPNOW_LINK_ACK_TIMEOUT_MS (500)

// Background retry cadence: how often the sync task wakes up on its own
// (independent of the notify-on-edit fast path and the explicit "sync now"
// HTTP endpoint) to retry anything still pending -- e.g. the FC's link gate
// was closed on the last attempt and may have opened since.
#define ESPNOW_LINK_RETRY_PERIOD_MS (8000)

#endif // BASESTATION_CONFIG_H
