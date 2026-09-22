#ifndef PID_TYPES_H
#define PID_TYPES_H

// Shared types for persisted/settable PID gains. Split out from
// config_store.h so both config_store.h (NVS blob format) and nav.h
// (nav_set_heading_pid_gains()'s signature) can include this without a
// circular dependency between the two.
//
// Formerly part of the wire protocol shared with the now-removed ESP-NOW
// basestation link (cl_pid_gains_t etc. in config_link_proto.h) -- kept
// byte-identical (same 4 floats) so existing persisted NVS blobs for
// pid_roll/pid_pitch/pid_hdg/pid_aspd still parse unchanged.

typedef struct {
    float k_p;
    float k_i;
    float k_d;
    float i_limit;
} pid_gains_t;

// Bit N set = the corresponding field in a pid_gains_t is present in this
// update and should overwrite the live value for that one field; bit clear
// = leave that field untouched.
typedef enum {
    PID_FIELD_KP     = 1 << 0,
    PID_FIELD_KI     = 1 << 1,
    PID_FIELD_KD     = 1 << 2,
    PID_FIELD_ILIMIT = 1 << 3,
} pid_field_mask_t;

typedef enum {
    PID_TARGET_ROLL     = 0,
    PID_TARGET_PITCH    = 1,
    PID_TARGET_HEADING  = 2,
    PID_TARGET_AIRSPEED = 3,
} pid_target_t;

#endif // PID_TYPES_H
