#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stddef.h>
#include "esp_err.h"
#include "config_link_proto.h"
#include "nav.h"

// Stateless NVS marshalling helpers for persisted config (PID gains,
// mission). Not a state owner itself -- callers (main.c, nav.c) hold the
// live values and call these to load an override at boot / save after an
// applied change, same role pid.c plays for the PID math itself.
//
// cl_pid_gains_t (the wire-protocol struct) is reused directly as the NVS
// blob format for PID gains, rather than a separate storage-only struct --
// keeps the type/file count down. If the wire format's layout ever changes
// later, old NVS blobs could misparse; that's cheaply fixed by bumping the
// key names below if/when it actually happens, not worth designing around
// now.

// nvs_flash_init() plus the standard ESP_ERR_NVS_NO_FREE_PAGES erase-retry.
// Call once, first thing in app_main(), before anything else touches NVS.
esp_err_t config_store_init(void);

// Returns false (leaving *out untouched) if `key` isn't present in NVS --
// the caller should keep its compiled-in default in that case.
bool config_store_load_pid_gains(const char *key, cl_pid_gains_t *out);
esp_err_t config_store_save_pid_gains(const char *key, const cl_pid_gains_t *g);

// Returns false (leaving *out_count/*out_loop untouched) if no mission is persisted.
bool config_store_load_mission(waypoint_t *out, size_t max_count, size_t *out_count, bool *out_loop);
esp_err_t config_store_save_mission(const waypoint_t *wps, size_t count, bool loop);

#endif // CONFIG_STORE_H
