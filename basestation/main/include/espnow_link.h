#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

// Basestation <-> FC ESP-NOW link. Unlike the FC's config_link.c, there is
// no safety gate here -- the basestation never flies, so this comes up once
// at boot and stays up for the device's whole lifetime (see
// BASESTATION_SPEC.md's "Explicitly out of scope" section). What this DOES
// have to handle is the FC's *own* gate: the FC's receiver isn't always
// listening, so esp_now_send() calls made through this module routinely go
// unacknowledged, and that's normal, not an error -- see store.h's
// STORE_STATUS_PENDING and the retry logic in espnow_link.c.
//
// Must be called after wifi_ap_init() (needs WiFi already in APSTA mode with
// the AP started) and after store_init() (the sync task reads/writes store
// state from the moment it starts).

void espnow_link_init(void);

// Wakes the background sync task to attempt delivery of anything currently
// pending, without blocking the caller. Call this right after an HTTP
// handler marks something pending (store_set_mission/store_set_pid_fields)
// so a reachable FC gets updated promptly, without making the HTTP response
// wait on it.
void espnow_link_kick_async(void);

// Synchronously runs one full delivery pass (mission, then each PID target)
// over everything currently pending, then returns. Used by the explicit
// "sync now" HTTP endpoint so the caller can report the outcome immediately.
// Bounded by (# pending items) * ESPNOW_LINK_ACK_TIMEOUT_MS in the worst
// case (nothing reachable) -- fine for an explicit, infrequent user action.
void espnow_link_sync_now(void);

#endif // ESPNOW_LINK_H
