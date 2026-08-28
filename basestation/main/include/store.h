#ifndef STORE_H
#define STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "config_link_proto.h"

// In-memory + NVS-persisted state for everything the basestation owns: the
// intended mission (full waypoint list), all four PID targets' gains, and a
// per-field/per-mission delivery status (unset / pending / confirmed /
// failed) tracking what has and hasn't been confirmed by the FC. This is the
// basestation's own separate NVS namespace on its own device -- unrelated
// to (and not required to agree in format with) the FC's config_store.c.
//
// All accessors are internally mutex-guarded; callers on both the HTTP
// server task(s) and the ESP-NOW sync task can use these concurrently
// without any extra locking.

typedef enum {
    STORE_STATUS_UNSET = 0,     // never edited by the user since first boot
    STORE_STATUS_PENDING,       // saved locally, not yet acked by the FC
    STORE_STATUS_CONFIRMED,     // FC acked CL_ACK_OK
    STORE_STATUS_FAILED,        // FC acked a rejection status -- see fail_reason
} store_status_t;

typedef struct {
    store_status_t status;
    cl_ack_status_t fail_reason; // meaningful only when status == STORE_STATUS_FAILED
} store_field_state_t;

// Index into a PID target's per-field state/gains arrays. Matches the bit
// order of cl_pid_field_mask_t (CL_FIELD_KP=bit0 -> index 0, etc.) so a
// fields_present bitmask can be walked directly against these arrays.
typedef enum {
    STORE_PID_IDX_KP = 0,
    STORE_PID_IDX_KI = 1,
    STORE_PID_IDX_KD = 2,
    STORE_PID_IDX_ILIMIT = 3,
    STORE_PID_FIELD_COUNT = 4,
} store_pid_field_idx_t;

typedef struct {
    cl_pid_gains_t gains;
    store_field_state_t field[STORE_PID_FIELD_COUNT];
} store_pid_target_t;

typedef struct {
    cl_waypoint_t points[CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET];
    size_t count;
    bool loop; // nonzero on the wire = wrap back to the first waypoint after the last, instead of orbiting it forever
    store_field_state_t state;
} store_mission_t;

// nvs_flash_init() (+ the standard erase-retry) and loads any persisted
// state into memory, falling back to empty/zeroed defaults (status UNSET)
// for anything not yet stored. Call once, first thing in app_main().
esp_err_t store_init(void);

// ---- Reads (snapshot copies -- safe to use without further locking) ----

void store_get_mission(store_mission_t *out);
void store_get_pid(cl_pid_target_t target, store_pid_target_t *out);

// Returns true and fills *out_points/*out_count/*out_loop if the mission is
// currently PENDING (i.e. there's something the sync task should try to send).
bool store_mission_is_pending(cl_waypoint_t *out_points, size_t *out_count, bool *out_loop);

// Returns the bitmask (cl_pid_field_mask_t) of fields currently PENDING for
// `target`, and fills *out_gains with the target's current full gains
// (callers send the whole gains struct on the wire but only the pending
// bits are meaningful to the FC -- see cl_set_pid_t's fields_present).
// Returns 0 if nothing is pending for this target.
uint8_t store_pid_pending_mask(cl_pid_target_t target, cl_pid_gains_t *out_gains);

// ---- Writes (from HTTP handlers, in response to a user edit) ----

// Replaces the whole mission (0..CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET
// points) and the loop flag, persists it, and marks it PENDING. Returns
// ESP_ERR_INVALID_ARG without changing anything if count is out of range or
// any point's lat/lon is non-finite or out of the valid +/-90/+/-180 range.
// Callers that want to leave `loop` unchanged (e.g. an edit that doesn't
// touch it) should read the current value via store_get_mission() first and
// pass that back -- this function always sets it, it has no "unchanged"
// sentinel of its own.
esp_err_t store_set_mission(const cl_waypoint_t *points, size_t count, bool loop);

// Merges the fields marked in `fields_present` into `target`'s live gains,
// persists, and marks exactly those fields PENDING (clearing any stale
// fail_reason on them). Fields not present in the mask are left completely
// untouched, mirroring the FC's own cl_set_pid_t semantics. Returns
// ESP_ERR_INVALID_ARG without changing anything if any present field is
// non-finite.
esp_err_t store_set_pid_fields(cl_pid_target_t target, uint8_t fields_present, const cl_pid_gains_t *gains);

// ---- Writes (from the ESP-NOW sync task, after a send attempt resolves) ----

// Only called after a real ack was received (never on a timeout -- a
// timeout leaves the item PENDING for the next retry, untouched).
void store_mark_mission_result(store_status_t status, cl_ack_status_t fail_reason);
void store_mark_pid_result(cl_pid_target_t target, uint8_t fields_sent_mask, store_status_t status, cl_ack_status_t fail_reason);

#endif // STORE_H
