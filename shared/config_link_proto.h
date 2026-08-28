#ifndef CONFIG_LINK_PROTO_H
#define CONFIG_LINK_PROTO_H

#include <stdint.h>

// Wire protocol for the FC <-> basestation ESP-NOW config link. This header
// is shared verbatim between the two independent ESP-IDF projects (main/ and
// basestation/) -- it must not depend on anything FC-internal or
// basestation-internal, only fixed-width types.
//
// The FC's ESP-NOW receiver is not always listening: it only opens after the
// plane has been idle/grounded for a while and closes instantly the moment
// it isn't (see main/config_link.c). Anything built against this protocol
// must expect sends to sometimes go unacknowledged for long stretches (mid
// flight, or before the FC's gate has opened) -- that's normal, not an
// error, and callers must queue/retry rather than assume delivery.

#define CONFIG_LINK_PROTO_MAGIC (0xC0FFEE01u)

typedef enum {
    CL_PKT_SET_MISSION = 1,
    CL_PKT_SET_PID     = 2,
    CL_PKT_ACK         = 3,
} cl_pkt_type_t;

typedef enum {
    CL_PID_ROLL     = 0,
    CL_PID_PITCH    = 1,
    CL_PID_HEADING  = 2,
    CL_PID_AIRSPEED = 3,
} cl_pid_target_t;

typedef enum {
    CL_ACK_OK               = 0,
    CL_ACK_BAD_MAGIC        = 1,
    CL_ACK_BAD_TYPE         = 2,
    CL_ACK_BAD_LEN          = 3,
    CL_ACK_OUT_OF_RANGE     = 4,
    CL_ACK_NVS_WRITE_FAILED = 5,  // applied live, but not guaranteed to survive a reboot
} cl_ack_status_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;   // CONFIG_LINK_PROTO_MAGIC
    uint8_t  type;    // cl_pkt_type_t
    uint8_t  seq;     // echoed back in the ack, so the sender can match replies
    uint16_t count;   // waypoint count for SET_MISSION; unused (0) otherwise
} cl_hdr_t;           // 8 bytes

typedef struct __attribute__((packed)) {
    double lat_deg;
    double lon_deg;
} cl_waypoint_t;      // 16 bytes

// ESP-NOW's payload cap is ~250 bytes. (250 - 9) / sizeof(cl_waypoint_t)
// = 241 / 16 = 15.06 -> 15 waypoints max per packet (9 = sizeof(cl_hdr_t) + the loop byte below).
#define CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET (15)

typedef struct __attribute__((packed)) {
    cl_hdr_t hdr;
    uint8_t loop;  // nonzero = after reaching the last waypoint, wrap back to the
                   // first one instead of holding an orbit there indefinitely
    cl_waypoint_t waypoints[CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET];
    // Actual esp_now_send() length must be
    // offsetof(cl_set_mission_t, waypoints) + hdr.count*sizeof(cl_waypoint_t)
    // -- never send sizeof(cl_set_mission_t) as-is, most of the array is usually unused.
} cl_set_mission_t;

// This is a whole-mission REPLACE, not a per-waypoint patch: the sender is
// always expected to send its full intended mission (up to
// CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET waypoints) plus the loop flag, which
// the FC applies as a complete replacement of its current mission (also
// resetting mission progress back to waypoint 0).

typedef struct __attribute__((packed)) {
    float k_p;
    float k_i;
    float k_d;
    float i_limit;
} cl_pid_gains_t;     // 16 bytes

// Bit N set = the corresponding field in `gains` is present in this packet
// and should overwrite the FC's live value for that one field; bit clear =
// leave that field untouched. This lets a sender update only the field(s) a
// user actually edited without needing to know or guess the other three
// current values first (there is no FC->sender readback packet in v1).
typedef enum {
    CL_FIELD_KP     = 1 << 0,
    CL_FIELD_KI     = 1 << 1,
    CL_FIELD_KD     = 1 << 2,
    CL_FIELD_ILIMIT = 1 << 3,
} cl_pid_field_mask_t;

typedef struct __attribute__((packed)) {
    cl_hdr_t hdr;
    uint8_t  target;          // cl_pid_target_t
    uint8_t  fields_present;  // bitmask of cl_pid_field_mask_t
    uint8_t  _pad[2];
    cl_pid_gains_t gains;     // only the fields whose bit is set in fields_present are meaningful
} cl_set_pid_t;                // 28 bytes

typedef struct __attribute__((packed)) {
    cl_hdr_t hdr;      // type = CL_PKT_ACK, seq = echo of the request's seq
    uint8_t  status;   // cl_ack_status_t
    uint8_t  reserved[3];
} cl_ack_t;            // 12 bytes

#endif // CONFIG_LINK_PROTO_H
