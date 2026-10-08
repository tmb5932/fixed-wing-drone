#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "flight_log.h"

static const char *TAG = "FLIGHT_LOG";

#define FLIGHT_LOG_MAGIC (0x464C4F47u)  // "FLOG"

typedef struct {
    uint32_t magic;
    uint16_t boot;
    uint16_t head;   // next write index
    uint16_t count;  // valid entries, <= FLIGHT_LOG_CAPACITY
    uint16_t _pad;
    flight_log_entry_t entries[FLIGHT_LOG_CAPACITY];
} flight_log_t;

// RTC_NOINIT: not zeroed or reloaded at boot, so it survives every reset
// except a power cycle.
static RTC_NOINIT_ATTR flight_log_t s_log;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static bool log_looks_valid(void) {
    return s_log.magic == FLIGHT_LOG_MAGIC &&
           s_log.count <= FLIGHT_LOG_CAPACITY &&
           s_log.head < FLIGHT_LOG_CAPACITY;
}

static void log_wipe(void) {
    memset(&s_log, 0, sizeof(s_log));
    s_log.magic = FLIGHT_LOG_MAGIC;
}

void flight_log_boot(void) {
    esp_reset_reason_t reason = esp_reset_reason();
    taskENTER_CRITICAL(&s_lock);
    // Power-on: RTC memory is garbage (or a stale-but-plausible leftover),
    // so always start clean rather than trust the magic alone.
    if (reason == ESP_RST_POWERON || !log_looks_valid()) {
        log_wipe();
    }
    s_log.boot++;
    taskEXIT_CRITICAL(&s_lock);

    flight_log_event(FLOG_BOOT, (uint32_t)reason);
    if (reason == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "Previous reset was a BROWNOUT (supply voltage sagged)");
    }
}

void flight_log_event(flight_log_type_t type, uint32_t value) {
    flight_log_entry_t e = {
        .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .type = (uint8_t)type,
        .value = value,
    };
    taskENTER_CRITICAL(&s_lock);
    e.boot = s_log.boot;
    s_log.entries[s_log.head] = e;
    s_log.head = (s_log.head + 1) % FLIGHT_LOG_CAPACITY;
    if (s_log.count < FLIGHT_LOG_CAPACITY) {
        s_log.count++;
    }
    taskEXIT_CRITICAL(&s_lock);
}

size_t flight_log_read(flight_log_entry_t *out, size_t max) {
    taskENTER_CRITICAL(&s_lock);
    size_t n = s_log.count < max ? s_log.count : max;
    size_t start = (s_log.head + FLIGHT_LOG_CAPACITY - s_log.count) % FLIGHT_LOG_CAPACITY;
    // Oldest first; if max < count, keep the newest `n`.
    start = (start + (s_log.count - n)) % FLIGHT_LOG_CAPACITY;
    for (size_t i = 0; i < n; i++) {
        out[i] = s_log.entries[(start + i) % FLIGHT_LOG_CAPACITY];
    }
    taskEXIT_CRITICAL(&s_lock);
    return n;
}

uint16_t flight_log_current_boot(void) {
    return s_log.boot;
}

void flight_log_clear(void) {
    taskENTER_CRITICAL(&s_lock);
    uint16_t boot = s_log.boot;
    log_wipe();
    s_log.boot = boot;
    taskEXIT_CRITICAL(&s_lock);
}
