#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nav.h"
#include "imu.h"
#include "gps.h"
#include "gps_math.h"
#include "pid.h"
#include "config_store.h"

static const char *TAG = "NAV";

#define NAV_TASK_HZ (10)

#define WAYPOINT_ACCEPTANCE_RADIUS_M (30.0)

// Below this groundspeed, GPS course-over-ground is too noisy to trust as a
// heading fallback (a stationary/slow-moving receiver's COG can swing
// wildly), so get_current_heading_deg() refuses to use it.
#define NAV_MIN_GPS_SPEED_KTS (3.0f)

#define NAV_CONFIG_MUTEX_WAIT_MS (15)

// Mutable mission (was a compile-time const array before waypoints became
// settable at runtime) -- guarded by nav_config_mutex since it's now written
// from the setup-mode HTTP server task (a different task) in response to a
// POST /api/mission request, while nav_task reads it every cycle regardless
// of autonomous mode.
// Placeholder default, same "hardcoded until reconfigured" convention as the
// PID gains in main.c -- overridden at boot by nav_init() if NVS has a
// persisted mission.
static waypoint_t mission_waypoints[NAV_MAX_WAYPOINTS] = {
    {0.0, 0.0},
    {0.0, 0.0},
};
static size_t num_waypoints = 2;
static size_t current_wp_idx = 0;
// When true, reaching the last waypoint wraps current_wp_idx back to 0
// instead of holding an orbit there indefinitely -- off by default, same
// "hardcoded until reconfigured" convention as the rest of this state.
static bool mission_loop = false;
static SemaphoreHandle_t nav_config_mutex;

// k_p ~= 1.0 maps a 45deg heading error to the existing +/-45deg
// set_goal_roll_deg() clamp, so no separate error->bank lookup table is
// needed. k_d = 0 since the GPS-COG-derived error can be noisy and isn't
// worth differentiating. SITL-validated (sitl/nav_sim.exe): reaches all 32
// combinations of 8 approach bearings x 4 distances (50-600m) with no
// tuning changes needed, and stays stable up to 10deg of heading-sensor
// noise; k_p=1.5-2.0 converges marginally tighter on the clean-signal sweep
// but starts failing that same noise sweep, so this is deliberately left as
// the more conservative choice rather than the tightest one. Also guarded
// by nav_config_mutex now, for the same cross-task reason as the mission
// above.
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

// Takes nav_config_mutex and resets HEADING_PID_CFG; used from every
// wings-level-fallback exit in step_nav() below, and from nav_reset().
static void reset_heading_pid_locked(void)
{
    if (nav_config_mutex != NULL && xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) == pdTRUE) {
        pid_reset(&HEADING_PID_CFG);
        xSemaphoreGive(nav_config_mutex);
    }
}

/**
 * Runs one nav cycle: advances the mission by waypoint-acceptance radius,
 * and steers goal_roll_deg toward the current target via HEADING_PID_CFG.
 * Falls back to a safe wings-level hold whenever there's no mission, no
 * GPS fix, or no usable heading source.
 */
static void step_nav(void)
{
    if (!gps_ready) {
        // !gps_ready also covers the startup window before gps_data_mutex
        // exists -- see the comment in get_current_heading_deg().
        set_goal_roll_deg(0.0f);
        reset_heading_pid_locked();
        return;
    }

    BaseType_t ret = xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(GPS_DATA_MUTEX_WAIT_MS));
    if (ret != pdTRUE) {
        set_goal_roll_deg(0.0f);
        return;
    }
    bool have_fix = latest_gps_data.valid;
    double cur_lat = latest_gps_data.latitude_deg;
    double cur_lon = latest_gps_data.longitude_deg;
    xSemaphoreGive(gps_data_mutex);

    if (!have_fix) {
        set_goal_roll_deg(0.0f);
        reset_heading_pid_locked();
        return;
    }

    float current_heading_deg;
    if (!get_current_heading_deg(&current_heading_deg)) {
        set_goal_roll_deg(0.0f);
        reset_heading_pid_locked();
        return;
    }

    if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) != pdTRUE) {
        set_goal_roll_deg(0.0f);
        return;
    }

    if (num_waypoints == 0) {
        pid_reset(&HEADING_PID_CFG);
        xSemaphoreGive(nav_config_mutex);
        set_goal_roll_deg(0.0f);
        return;
    }

    // Advance the mission if we've reached the current target. At the last
    // waypoint: if mission_loop is set, wrap back to waypoint 0 and keep
    // going; otherwise clamp there -- a bank-limited plane continuously
    // steered at a fixed point naturally settles into a stable orbit around
    // it, which is simple and safe. A true tangent-point loiter circle is a
    // documented future enhancement, not this one.
    waypoint_t target = mission_waypoints[current_wp_idx];
    double dist_m = distance_to_target(cur_lat, cur_lon, target.lat_deg, target.lon_deg);
    if (dist_m <= WAYPOINT_ACCEPTANCE_RADIUS_M) {
        if (current_wp_idx + 1 < num_waypoints) {
            current_wp_idx++;
        } else if (mission_loop) {
            current_wp_idx = 0;
        }
        target = mission_waypoints[current_wp_idx];
        dist_m = distance_to_target(cur_lat, cur_lon, target.lat_deg, target.lon_deg);
    }

    float desired_heading_deg = heading_to_target(cur_lat, cur_lon, target.lat_deg, target.lon_deg);
    float wrapped_error = heading_error_deg(current_heading_deg, desired_heading_deg);

    // Wraparound trick that keeps pid.c untouched/generic (sitl/ links it
    // directly for offline tuning): pid_step() computes err = goal -
    // current internally, so passing the already-wrapped error as "goal"
    // against a fixed "current" of 0.0f reproduces the correct signed error
    // with zero changes to pid.c/pid.h.
    const float dt_s = 1.0f / NAV_TASK_HZ;
    float roll_cmd_deg = pid_step(&HEADING_PID_CFG, 0.0f, wrapped_error, dt_s);
    xSemaphoreGive(nav_config_mutex);

    set_goal_roll_deg(roll_cmd_deg);
}

void nav_reset(void)
{
    reset_heading_pid_locked();
}

bool nav_set_mission(const waypoint_t *wps, size_t count, bool loop)
{
    if (count > NAV_MAX_WAYPOINTS || nav_config_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) != pdTRUE) {
        return false;
    }
    if (count > 0) {
        memcpy(mission_waypoints, wps, count * sizeof(waypoint_t));
    }
    num_waypoints = count;
    current_wp_idx = 0;
    mission_loop = loop;
    xSemaphoreGive(nav_config_mutex);

    return config_store_save_mission(wps, count, loop) == ESP_OK;
}

bool nav_set_heading_pid_gains(uint8_t fields_present, const pid_gains_t *gains)
{
    if (nav_config_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) != pdTRUE) {
        return false;
    }
    if (fields_present & PID_FIELD_KP)     HEADING_PID_CFG.k_p = gains->k_p;
    if (fields_present & PID_FIELD_KI)     HEADING_PID_CFG.k_i = gains->k_i;
    if (fields_present & PID_FIELD_KD)     HEADING_PID_CFG.k_d = gains->k_d;
    if (fields_present & PID_FIELD_ILIMIT) HEADING_PID_CFG.i_limit = gains->i_limit;
    pid_gains_t merged = { HEADING_PID_CFG.k_p, HEADING_PID_CFG.k_i, HEADING_PID_CFG.k_d, HEADING_PID_CFG.i_limit };
    xSemaphoreGive(nav_config_mutex);

    return config_store_save_pid_gains("pid_hdg", &merged) == ESP_OK;
}

void nav_get_mission(waypoint_t *out, size_t max_count, size_t *out_count, bool *out_loop)
{
    if (nav_config_mutex == NULL || xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) != pdTRUE) {
        *out_count = 0;
        *out_loop = false;
        return;
    }
    size_t n = (num_waypoints < max_count) ? num_waypoints : max_count;
    memcpy(out, mission_waypoints, n * sizeof(waypoint_t));
    *out_count = n;
    *out_loop = mission_loop;
    xSemaphoreGive(nav_config_mutex);
}

pid_gains_t nav_get_heading_pid_gains(void)
{
    pid_gains_t g = {0};
    if (nav_config_mutex != NULL && xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) == pdTRUE) {
        g = (pid_gains_t){ HEADING_PID_CFG.k_p, HEADING_PID_CFG.k_i, HEADING_PID_CFG.k_d, HEADING_PID_CFG.i_limit };
        xSemaphoreGive(nav_config_mutex);
    }
    return g;
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
    nav_config_mutex = xSemaphoreCreateMutex();
    if (nav_config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create nav config mutex!");
        abort();
    }

    waypoint_t loaded_wps[NAV_MAX_WAYPOINTS];
    size_t loaded_count;
    bool loaded_loop;
    if (config_store_load_mission(loaded_wps, NAV_MAX_WAYPOINTS, &loaded_count, &loaded_loop)) {
        memcpy(mission_waypoints, loaded_wps, loaded_count * sizeof(waypoint_t));
        num_waypoints = loaded_count;
        mission_loop = loaded_loop;
        ESP_LOGI(TAG, "Loaded %d persisted waypoint(s) from NVS (loop=%d)", (int)loaded_count, loaded_loop);
    }

    pid_gains_t g;
    if (config_store_load_pid_gains("pid_hdg", &g)) {
        HEADING_PID_CFG.k_p = g.k_p;
        HEADING_PID_CFG.k_i = g.k_i;
        HEADING_PID_CFG.k_d = g.k_d;
        HEADING_PID_CFG.i_limit = g.i_limit;
        ESP_LOGI(TAG, "Loaded persisted heading PID gains from NVS");
    }

    BaseType_t ret = xTaskCreate(nav_task, "nav_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create nav task! Error: %d", ret);
        abort();
    }
}
