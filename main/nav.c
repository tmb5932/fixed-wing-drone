#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nav.h"
#include "imu.h"
#include "gps.h"
#include "gps_math.h"
#include "pid.h"
#include "config_store.h"
#include "flight_log.h"

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

// ---- Home position ----
//
// Captured once per boot from GPS while the plane sits still on the ground:
// the first HOME_SKIP_FIXES fixes are thrown away (a fresh receiver's first
// fixes are its least accurate, and RMC carries no satellite count/HDOP to
// judge them by), then HOME_AVG_FIXES consecutive stationary fixes are
// averaged. Moving mid-average restarts it. Never persisted: after an
// in-air reboot the plane is moving, so no home gets captured at all, and a
// signal loss then descends in place (see step_failsafe()). Only touched by
// nav_task, apart from nav_get_home()'s guarded read.
#define HOME_SKIP_FIXES (10)
#define HOME_AVG_FIXES (25)          // ~5s at the module's 5Hz
#define HOME_MAX_SPEED_KTS (2.0f)    // "stationary" -- well under any flying speed

static waypoint_t home;
static bool home_set = false;
static int home_fixes_seen = 0;
static int home_avg_count = 0;
static double home_lat_sum = 0.0, home_lon_sum = 0.0;
static int64_t home_last_fix_ts = 0;

// ---- Signal-loss failsafe ----
#define FAILSAFE_HOME_RADIUS_M (WAYPOINT_ACCEPTANCE_RADIUS_M)
#define FAILSAFE_SPIRAL_BANK_DEG (30.0f)

static volatile bool signal_lost = false;          // written by control_task
static volatile nav_failsafe_t failsafe_state = NAV_FAILSAFE_NONE;  // written by nav_task

// Persisted autonomous mode -- see nav.h. Guarded by nav_config_mutex.
static nav_mode_t nav_mode = NAV_MODE_WAYPOINT;

// Heading-hold state (NAV_MODE_HEADING_HOLD). held_valid=false means "capture
// the current heading on the next cycle"; nav_reset() clears it on every
// manual->autonomous edge, so the held heading is always the one the plane
// had when the pilot engaged, not one left over from an earlier engagement.
// held_src pins which sensor the heading was captured from: compass heading
// and GPS course over ground differ (wind crab, declination), so steering a
// compass-captured heading with a COG reading would turn the plane -- a
// source change recaptures instead. Guarded by nav_config_mutex.
typedef enum { HEADING_SRC_COMPASS, HEADING_SRC_GPS_COG } heading_source_t;
static bool held_valid = false;
static float held_heading_deg = 0.0f;
static heading_source_t held_src = HEADING_SRC_COMPASS;

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
static bool get_current_heading_deg(float *out, heading_source_t *out_src)
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
                *out_src = HEADING_SRC_COMPASS;
                return true;
            }
        }
    }

    if (gps_ready) {
        BaseType_t ret = xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(GPS_DATA_MUTEX_WAIT_MS));
        if (ret == pdTRUE) {
            bool fresh = (esp_timer_get_time() - latest_gps_data.timestamp_us) < GPS_FIX_MAX_AGE_US;
            bool usable = latest_gps_data.valid && fresh && latest_gps_data.speed_knots >= NAV_MIN_GPS_SPEED_KTS;
            float course = (float)latest_gps_data.course_deg;
            xSemaphoreGive(gps_data_mutex);
            if (usable) {
                *out = course;
                *out_src = HEADING_SRC_GPS_COG;
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
 * Snapshot of the latest GPS fix. Returns false if there's no fix, or the
 * latest one is older than GPS_FIX_MAX_AGE_US (receiver lost its fix).
 */
static bool get_fresh_fix(double *lat, double *lon, float *speed_kts, int64_t *ts)
{
    // gps_ready also covers the window before gps_data_mutex exists -- see
    // get_current_heading_deg().
    if (!gps_ready || xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(GPS_DATA_MUTEX_WAIT_MS)) != pdTRUE) {
        return false;
    }
    bool valid = latest_gps_data.valid;
    *lat = latest_gps_data.latitude_deg;
    *lon = latest_gps_data.longitude_deg;
    *speed_kts = (float)latest_gps_data.speed_knots;
    *ts = latest_gps_data.timestamp_us;
    xSemaphoreGive(gps_data_mutex);
    return valid && (esp_timer_get_time() - *ts) < GPS_FIX_MAX_AGE_US;
}

static void set_home(double lat, double lon)
{
    if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) != pdTRUE) {
        return;  // retried next cycle -- home_set is still false
    }
    home.lat_deg = lat;
    home.lon_deg = lon;
    home_set = true;
    xSemaphoreGive(nav_config_mutex);
    ESP_LOGI(TAG, "Home set (averaged GPS): %.6f, %.6f", lat, lon);
}

// Runs every nav cycle until home is set -- see the HOME_* constants.
static void capture_home_step(void)
{
    if (home_set) {
        return;
    }
    double lat, lon;
    float speed_kts;
    int64_t ts;
    if (!get_fresh_fix(&lat, &lon, &speed_kts, &ts) || ts == home_last_fix_ts) {
        return;  // no fix, or no new fix since last cycle (nav runs 10Hz, GPS 5Hz)
    }
    home_last_fix_ts = ts;

    if (speed_kts > HOME_MAX_SPEED_KTS) {
        // Moving: restart the average.
        home_avg_count = 0;
        home_lat_sum = home_lon_sum = 0.0;
        return;
    }

    if (home_fixes_seen < HOME_SKIP_FIXES) {
        home_fixes_seen++;
        return;
    }
    home_lat_sum += lat;
    home_lon_sum += lon;
    home_avg_count++;
    if (home_avg_count >= HOME_AVG_FIXES) {
        set_home(home_lat_sum / home_avg_count, home_lon_sum / home_avg_count);
    }
}

static void enter_failsafe_state(nav_failsafe_t s, const char *why)
{
    if (failsafe_state == s) {
        return;
    }
    failsafe_state = s;
    reset_heading_pid_locked();
    switch (s) {
        case NAV_FAILSAFE_RTH:
            flight_log_event(FLOG_FAILSAFE_RTH, 0);
            ESP_LOGW(TAG, "FAILSAFE: signal lost -- returning home");
            break;
        case NAV_FAILSAFE_DESCEND:
            flight_log_event(FLOG_FAILSAFE_DESCEND, 0);
            ESP_LOGW(TAG, "FAILSAFE: %s -- motors off, spiral descent", why);
            break;
        default:
            flight_log_event(FLOG_FAILSAFE_CLEARED, 0);
            ESP_LOGW(TAG, "FAILSAFE cleared: signal regained");
            break;
    }
}

/**
 * Signal-loss failsafe nav cycle. Returns true if the failsafe is active and
 * has set goal_roll_deg this cycle (step_nav() then skips the normal mode).
 */
static bool step_failsafe(void)
{
    if (!signal_lost) {
        if (failsafe_state != NAV_FAILSAFE_NONE) {
            enter_failsafe_state(NAV_FAILSAFE_NONE, NULL);
            // Heading hold resumes with a freshly captured heading, not the
            // one from before the loss -- the plane may be far from it now.
            if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) == pdTRUE) {
                held_valid = false;
                xSemaphoreGive(nav_config_mutex);
            }
        }
        return false;
    }

    double lat, lon;
    float speed_kts;
    int64_t ts;
    bool have_fix = get_fresh_fix(&lat, &lon, &speed_kts, &ts);

    waypoint_t h;
    bool have_home = nav_get_home(&h);

    if (failsafe_state == NAV_FAILSAFE_NONE) {
        if (!have_home || !have_fix) {
            enter_failsafe_state(NAV_FAILSAFE_DESCEND, have_home ? "signal lost, no GPS fix" : "signal lost, no home set");
        } else {
            enter_failsafe_state(NAV_FAILSAFE_RTH, NULL);
        }
    }

    if (failsafe_state == NAV_FAILSAFE_RTH) {
        if (!have_fix) {
            // No radio and no GPS: nothing left to navigate by, so come down
            // where we are rather than fly on blind. (have_fix already
            // tolerates GPS_FIX_MAX_AGE_US of dropout.)
            enter_failsafe_state(NAV_FAILSAFE_DESCEND, "GPS lost during return");
        } else {
            double dist_m = distance_to_target(lat, lon, h.lat_deg, h.lon_deg);
            float heading_deg;
            heading_source_t src;
            if (dist_m <= FAILSAFE_HOME_RADIUS_M) {
                enter_failsafe_state(NAV_FAILSAFE_DESCEND, "reached home, still no signal");
            } else if (!get_current_heading_deg(&heading_deg, &src)) {
                set_goal_roll_deg(0.0f);
                return true;
            } else {
                float err = heading_error_deg(heading_deg, heading_to_target(lat, lon, h.lat_deg, h.lon_deg));
                float roll_cmd = 0.0f;
                if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) == pdTRUE) {
                    roll_cmd = pid_step(&HEADING_PID_CFG, 0.0f, err, 1.0f / NAV_TASK_HZ);
                    xSemaphoreGive(nav_config_mutex);
                }
                set_goal_roll_deg(roll_cmd);
                return true;
            }
        }
    }

    // NAV_FAILSAFE_DESCEND: constant-bank spiral (motor cut / nose-down
    // pitch are applied in main.c's update_autonomous_outputs()).
    set_goal_roll_deg(FAILSAFE_SPIRAL_BANK_DEG);
    return true;
}

/**
 * NAV_MODE_HEADING_HOLD's nav cycle: holds whatever heading the plane had
 * when autonomous engaged (see held_valid), steering goal_roll_deg through
 * the same HEADING_PID_CFG waypoint mode uses. Wings-level whenever there's
 * no usable heading source. Doesn't need a GPS fix if the compass is valid.
 */
static void step_heading_hold(void)
{
    float current_heading_deg;
    heading_source_t src;
    bool have_heading = get_current_heading_deg(&current_heading_deg, &src);

    if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) != pdTRUE) {
        set_goal_roll_deg(0.0f);
        return;
    }

    if (!have_heading) {
        held_valid = false;
        pid_reset(&HEADING_PID_CFG);
        xSemaphoreGive(nav_config_mutex);
        set_goal_roll_deg(0.0f);
        return;
    }

    bool captured = false;
    if (!held_valid || src != held_src) {
        held_heading_deg = current_heading_deg;
        held_src = src;
        held_valid = true;
        pid_reset(&HEADING_PID_CFG);
        captured = true;
    }

    // Same wraparound trick as step_nav() below.
    const float dt_s = 1.0f / NAV_TASK_HZ;
    float wrapped_error = heading_error_deg(current_heading_deg, held_heading_deg);
    float roll_cmd_deg = pid_step(&HEADING_PID_CFG, 0.0f, wrapped_error, dt_s);
    xSemaphoreGive(nav_config_mutex);

    if (captured) {
        ESP_LOGI(TAG, "Heading hold: holding %.1f deg (%s)", current_heading_deg,
                 src == HEADING_SRC_COMPASS ? "compass" : "GPS course");
    }
    set_goal_roll_deg(roll_cmd_deg);
}

/**
 * Runs one nav cycle: advances the mission by waypoint-acceptance radius,
 * and steers goal_roll_deg toward the current target via HEADING_PID_CFG.
 * Falls back to a safe wings-level hold whenever there's no mission, no
 * GPS fix, or no usable heading source.
 */
static void step_nav(void)
{
    capture_home_step();

    if (step_failsafe()) {
        return;
    }

    if (nav_get_mode() == NAV_MODE_HEADING_HOLD) {
        step_heading_hold();
        return;
    }

    if (!gps_ready) {
        // !gps_ready also covers the startup window before gps_data_mutex
        // exists -- see the comment in get_current_heading_deg().
        set_goal_roll_deg(0.0f);
        reset_heading_pid_locked();
        return;
    }

    double cur_lat, cur_lon;
    float speed_kts;
    int64_t fix_ts;
    if (!get_fresh_fix(&cur_lat, &cur_lon, &speed_kts, &fix_ts)) {
        set_goal_roll_deg(0.0f);
        reset_heading_pid_locked();
        return;
    }

    float current_heading_deg;
    heading_source_t src;
    if (!get_current_heading_deg(&current_heading_deg, &src)) {
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
    if (nav_config_mutex != NULL && xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) == pdTRUE) {
        pid_reset(&HEADING_PID_CFG);
        held_valid = false;
        xSemaphoreGive(nav_config_mutex);
    }
    // nav_task keeps running in manual mode too, so goal_roll_deg can still
    // hold a bank command toward a stale heading-hold target from before
    // this engagement. Zero it so the up-to-one-nav-cycle (100ms) gap before
    // nav_task recaptures is flown wings-level instead.
    set_goal_roll_deg(0.0f);
}

void nav_set_signal_lost(bool lost)
{
    signal_lost = lost;
}

nav_failsafe_t nav_get_failsafe(void)
{
    return failsafe_state;
}

bool nav_gps_fix_ok(void)
{
    double lat, lon;
    float speed_kts;
    int64_t ts;
    return get_fresh_fix(&lat, &lon, &speed_kts, &ts);
}

bool nav_get_home(waypoint_t *out)
{
    bool ok = false;
    if (nav_config_mutex != NULL && xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) == pdTRUE) {
        ok = home_set;
        if (ok) *out = home;
        xSemaphoreGive(nav_config_mutex);
    }
    return ok;
}

nav_mode_t nav_get_mode(void)
{
    nav_mode_t m = NAV_MODE_WAYPOINT;
    if (nav_config_mutex != NULL && xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) == pdTRUE) {
        m = nav_mode;
        xSemaphoreGive(nav_config_mutex);
    }
    return m;
}

bool nav_set_mode(nav_mode_t mode)
{
    if ((int)mode < 0 || (int)mode >= NAV_MODE_COUNT || nav_config_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(nav_config_mutex, pdMS_TO_TICKS(NAV_CONFIG_MUTEX_WAIT_MS)) != pdTRUE) {
        return false;
    }
    nav_mode = mode;
    held_valid = false;
    pid_reset(&HEADING_PID_CFG);
    xSemaphoreGive(nav_config_mutex);

    return config_store_save_nav_mode(mode) == ESP_OK;
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

    nav_mode_t loaded_mode;
    if (config_store_load_nav_mode(&loaded_mode)) {
        nav_mode = loaded_mode;
        ESP_LOGI(TAG, "Loaded persisted nav mode from NVS: %s",
                 nav_mode == NAV_MODE_HEADING_HOLD ? "heading hold" : "waypoint");
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
