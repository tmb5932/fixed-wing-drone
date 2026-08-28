#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nav.h"
#include "imu.h"
#include "gps.h"
#include "pid.h"

static const char *TAG = "NAV";

#define NAV_TASK_HZ (10)

#define WAYPOINT_ACCEPTANCE_RADIUS_M (30.0)

// Below this groundspeed, GPS course-over-ground is too noisy to trust as a
// heading fallback (a stationary/slow-moving receiver's COG can swing
// wildly), so get_current_heading_deg() refuses to use it.
#define NAV_MIN_GPS_SPEED_KTS (3.0f)

// Placeholder mission, same "hardcoded and reflash to change" convention as
// the PID gains in main.c. Replace with real coordinates before flight.
static const waypoint_t MISSION_WAYPOINTS[] = {
    {0.0, 0.0},
    {0.0, 0.0},
};
#define NUM_WAYPOINTS (sizeof(MISSION_WAYPOINTS) / sizeof(MISSION_WAYPOINTS[0]))

static size_t current_wp_idx = 0;

// Placeholder gain: k_p ~= 1.0 maps a 45deg heading error to the existing
// +/-45deg set_goal_roll_deg() clamp, so no separate error->bank lookup
// table is needed. k_d = 0 for now since the GPS-COG-derived error can be
// noisy and isn't worth differentiating yet.
static pid_cfg_t HEADING_PID_CFG = {
    .k_p = 1.0f,
    .k_i = 0.0f,
    .k_d = 0.0f,
    .i_limit = 45.0f,
    .integral = 0,
    .last_err = 0,
    .first = true
};

extern void set_goal_roll_deg(float deg);

/**
 * Shortest-turn signed error from current to desired heading, in [-180, 180].
 * Positive means "turn right (clockwise)" to reach desired.
 */
static float heading_error_deg(float current_deg, float desired_deg)
{
    float err = fmodf(desired_deg - current_deg + 540.0f, 360.0f) - 180.0f;
    return err;
}

/**
 * Resolves the best available current heading, in degrees true-north-
 * clockwise (same convention as heading_to_target()). Prefers the fused
 * compass heading when the magnetometer is trustworthy; falls back to GPS
 * course-over-ground when there's a valid fix and enough groundspeed for
 * COG to be meaningful. Returns false (leaving *out untouched) if neither
 * source is usable right now.
 */
static bool get_current_heading_deg(float *out)
{
    // imu_data_mutex/gps_data_mutex are only created partway through
    // imu_task's/gps_task's own init sequences -- NULL until then, and
    // xSemaphoreTake() on a NULL handle is a hard assert, not a graceful
    // failure. imu_ready/gps_ready are only set true after each mutex
    // actually exists, so this doubles as the "handle is safe to use" check.
    if (imu_ready) {
        BaseType_t ret = xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT));
        if (ret == pdTRUE) {
            bool mag_valid = imu_data.mag_valid;
            float yaw = imu_data.yaw;
            xSemaphoreGive(imu_data_mutex);
            if (mag_valid) {
                *out = yaw;
                return true;
            }
        }
    }

    if (gps_ready) {
        BaseType_t ret = xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(GPS_DATA_MUTEX_WAIT_MS));
        if (ret == pdTRUE) {
            bool usable = latest_gps_data.valid && latest_gps_data.speed_knots >= NAV_MIN_GPS_SPEED_KTS;
            float course = (float)latest_gps_data.course_deg;
            xSemaphoreGive(gps_data_mutex);
            if (usable) {
                *out = course;
                return true;
            }
        }
    }

    return false;
}

/**
 * Runs one nav cycle: advances the mission by waypoint-acceptance radius,
 * and steers goal_roll_deg toward the current target via HEADING_PID_CFG.
 * Falls back to a safe wings-level hold whenever there's no mission, no
 * GPS fix, or no usable heading source.
 */
static void step_nav(void)
{
    if (NUM_WAYPOINTS == 0 || !gps_ready) {
        // !gps_ready also covers the startup window before gps_data_mutex
        // exists -- see the comment in get_current_heading_deg().
        set_goal_roll_deg(0.0f);
        pid_reset(&HEADING_PID_CFG);
        return;
    }

    BaseType_t ret = xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(GPS_DATA_MUTEX_WAIT_MS));
    if (ret != pdTRUE) {
        set_goal_roll_deg(0.0f);
        pid_reset(&HEADING_PID_CFG);
        return;
    }
    bool have_fix = latest_gps_data.valid;
    double cur_lat = latest_gps_data.latitude_deg;
    double cur_lon = latest_gps_data.longitude_deg;
    xSemaphoreGive(gps_data_mutex);

    if (!have_fix) {
        set_goal_roll_deg(0.0f);
        pid_reset(&HEADING_PID_CFG);
        return;
    }

    float current_heading_deg;
    if (!get_current_heading_deg(&current_heading_deg)) {
        set_goal_roll_deg(0.0f);
        pid_reset(&HEADING_PID_CFG);
        return;
    }

    // Advance the mission if we've reached the current target. Clamped at
    // the last waypoint once the mission is exhausted -- a bank-limited
    // plane continuously steered at a fixed point naturally settles into a
    // stable orbit around it, which is simple and safe. A true tangent-
    // point loiter circle is a documented future enhancement, not this one.
    const waypoint_t *target = &MISSION_WAYPOINTS[current_wp_idx];
    double dist_m = distance_to_target(cur_lat, cur_lon, target->lat_deg, target->lon_deg);
    if (dist_m <= WAYPOINT_ACCEPTANCE_RADIUS_M && current_wp_idx + 1 < NUM_WAYPOINTS) {
        current_wp_idx++;
        target = &MISSION_WAYPOINTS[current_wp_idx];
        dist_m = distance_to_target(cur_lat, cur_lon, target->lat_deg, target->lon_deg);
    }

    float desired_heading_deg = heading_to_target(cur_lat, cur_lon, target->lat_deg, target->lon_deg);
    float wrapped_error = heading_error_deg(current_heading_deg, desired_heading_deg);

    // Wraparound trick that keeps pid.c untouched/generic (sitl/ links it
    // directly for offline tuning): pid_step() computes err = goal -
    // current internally, so passing the already-wrapped error as "goal"
    // against a fixed "current" of 0.0f reproduces the correct signed error
    // with zero changes to pid.c/pid.h.
    const float dt_s = 1.0f / NAV_TASK_HZ;
    float roll_cmd_deg = pid_step(&HEADING_PID_CFG, 0.0f, wrapped_error, dt_s);
    set_goal_roll_deg(roll_cmd_deg);
}

void nav_reset(void)
{
    pid_reset(&HEADING_PID_CFG);
}

static void nav_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / NAV_TASK_HZ);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        step_nav();
    }
}

void nav_init(void)
{
    BaseType_t ret = xTaskCreate(nav_task, "nav_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create nav task! Error: %d", ret);
        abort();
    }
}
