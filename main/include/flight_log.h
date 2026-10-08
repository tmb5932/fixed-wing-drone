#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Small in-memory flight event log for diagnosing in-flight incidents
// after landing (RC dropouts, failsafe entries, reboots and why). Lives in
// RTC memory that survives any reset -- including a brownout reset or the
// reset button -- but not a power cycle. So after an incident: land, DON'T
// unplug the battery, hold BOOT and press reset to enter setup mode, and
// read it from the setup page (GET /api/flight_log).
//
// No flash writes during flight on purpose: a flash erase stalls both
// cores' code execution for milliseconds at a time.

typedef enum {
    FLOG_BOOT = 0,           // value = esp_reset_reason_t
    FLOG_RC_DROPOUT,         // value = gap length in ms (short gap, link recovered)
    FLOG_RC_LOST,            // link silent for RC_SIGNAL_LOSS_US -- failsafe territory
    FLOG_RC_REGAINED,        // value = total outage in ms
    FLOG_FAILSAFE_RTH,
    FLOG_FAILSAFE_DESCEND,
    FLOG_FAILSAFE_CLEARED,
    FLOG_AUTONOMOUS_LOCKOUT, // value: 0 = no GPS, 1 = radio regained during descent
    FLOG_IMU_FAULT,
    FLOG_NO_RADIO_SAFE,      // radio lost, autonomous unavailable -> trim + motors off
    FLOG_TYPE_COUNT,
} flight_log_type_t;

typedef struct {
    uint32_t uptime_ms;  // since that boot
    uint16_t boot;       // boot counter the event happened in
    uint8_t type;        // flight_log_type_t
    uint8_t _pad;
    uint32_t value;
} flight_log_entry_t;

#define FLIGHT_LOG_CAPACITY (64)  // oldest entries are overwritten

// Call once, first thing in app_main(). Validates/initializes the RTC log
// (wiped on a power-on reset, where RTC memory holds garbage), bumps the
// boot counter and records this boot's reset reason.
void flight_log_boot(void);

// Records an event. Safe to call from any task (not from an ISR).
void flight_log_event(flight_log_type_t type, uint32_t value);

// Copies up to `max` entries, oldest first. Returns the number copied.
size_t flight_log_read(flight_log_entry_t *out, size_t max);
uint16_t flight_log_current_boot(void);
void flight_log_clear(void);

#endif // FLIGHT_LOG_H
