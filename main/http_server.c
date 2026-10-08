#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "http_server.h"
#include "output_ctl.h"
#include "nav.h"
#include "flight_log.h"
#include "esp_system.h"
#include "gps.h"
#include "airspeed.h"
#include "imu.h"
#include "pwm_output.h"
#include "pid_types.h"

static const char *TAG = "HTTP_SERVER";

// Setters main.c/nav.c already expose for the (now-removed) ESP-NOW config
// link -- reused here verbatim, same pattern config_link.c used.
extern bool set_roll_pid_gains(uint8_t fields_present, const pid_gains_t *g);
extern bool set_pitch_pid_gains(uint8_t fields_present, const pid_gains_t *g);
extern bool set_airspeed_pid_gains(uint8_t fields_present, const pid_gains_t *g);

// ---- Embedded web UI assets (see main/CMakeLists.txt's EMBED_TXTFILES /
// EMBED_FILES). ESP-IDF's embed machinery names the generated symbols after
// just the file's basename (not its path). ----

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t app_js_start[] asm("_binary_app_js_start");
extern const uint8_t app_js_end[]   asm("_binary_app_js_end");
extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[]   asm("_binary_style_css_end");
extern const uint8_t leaflet_js_start[] asm("_binary_leaflet_js_start");
extern const uint8_t leaflet_js_end[]   asm("_binary_leaflet_js_end");
extern const uint8_t leaflet_css_start[] asm("_binary_leaflet_css_start");
extern const uint8_t leaflet_css_end[]   asm("_binary_leaflet_css_end");
extern const uint8_t marker_icon_png_start[] asm("_binary_marker_icon_png_start");
extern const uint8_t marker_icon_png_end[]   asm("_binary_marker_icon_png_end");
extern const uint8_t marker_icon_2x_png_start[] asm("_binary_marker_icon_2x_png_start");
extern const uint8_t marker_icon_2x_png_end[]   asm("_binary_marker_icon_2x_png_end");
extern const uint8_t marker_shadow_png_start[] asm("_binary_marker_shadow_png_start");
extern const uint8_t marker_shadow_png_end[]   asm("_binary_marker_shadow_png_end");

#define MAX_JSON_BODY_LEN (2048)
#define MAX_MISSION_POINTS_PER_REQUEST (NAV_MAX_WAYPOINTS)

#define STR_(x) #x
#define STR(x) STR_(x)

// ---- string <-> enum helpers ----

static bool parse_pid_target(const char *s, pid_target_t *out) {
    if (strcmp(s, "roll") == 0)     { *out = PID_TARGET_ROLL; return true; }
    if (strcmp(s, "pitch") == 0)    { *out = PID_TARGET_PITCH; return true; }
    if (strcmp(s, "heading") == 0)  { *out = PID_TARGET_HEADING; return true; }
    if (strcmp(s, "airspeed") == 0) { *out = PID_TARGET_AIRSPEED; return true; }
    return false;
}

static const char *pid_target_name(pid_target_t t) {
    switch (t) {
        case PID_TARGET_ROLL:     return "roll";
        case PID_TARGET_PITCH:    return "pitch";
        case PID_TARGET_HEADING:  return "heading";
        case PID_TARGET_AIRSPEED: return "airspeed";
        default:                  return "?";
    }
}

static pid_gains_t get_pid_gains(pid_target_t t) {
    switch (t) {
        case PID_TARGET_ROLL:     return get_roll_pid_gains();
        case PID_TARGET_PITCH:    return get_pitch_pid_gains();
        case PID_TARGET_HEADING:  return nav_get_heading_pid_gains();
        case PID_TARGET_AIRSPEED: return get_airspeed_pid_gains();
        default:                  return (pid_gains_t){0};
    }
}

static bool set_pid_gains(pid_target_t t, uint8_t fields_present, const pid_gains_t *g) {
    switch (t) {
        case PID_TARGET_ROLL:     return set_roll_pid_gains(fields_present, g);
        case PID_TARGET_PITCH:    return set_pitch_pid_gains(fields_present, g);
        case PID_TARGET_HEADING:  return nav_set_heading_pid_gains(fields_present, g);
        case PID_TARGET_AIRSPEED: return set_airspeed_pid_gains(fields_present, g);
        default:                  return false;
    }
}

// Loose sanity bound on an individual PID gain field -- rejects obviously
// corrupted/garbage values (NaN, Inf, wild magnitudes from a bad request),
// not a judgment about what's a "reasonable" tuning value; that's left to
// whoever's editing the field. Same bound the old config-link used.
#define PID_GAIN_ABS_MAX (10000.0f)

static bool gain_ok(uint8_t fields_present, uint8_t bit, float v) {
    if (!(fields_present & bit)) {
        return true;
    }
    return isfinite(v) && fabsf(v) <= PID_GAIN_ABS_MAX;
}

static const char *nav_mode_name(nav_mode_t m) {
    return (m == NAV_MODE_HEADING_HOLD) ? "heading_hold" : "waypoint";
}

static bool parse_nav_mode(const char *s, nav_mode_t *out) {
    if (strcmp(s, "waypoint") == 0)     { *out = NAV_MODE_WAYPOINT; return true; }
    if (strcmp(s, "heading_hold") == 0) { *out = NAV_MODE_HEADING_HOLD; return true; }
    return false;
}

static const char *airframe_mode_name(airframe_mode_t m) {
    return (m == AIRFRAME_AILEVON_MODE) ? "ailevon" : "conventional";
}

static bool parse_airframe_mode(const char *s, airframe_mode_t *out) {
    if (strcmp(s, "conventional") == 0) { *out = AIRFRAME_CONVENTIONAL; return true; }
    if (strcmp(s, "ailevon") == 0)      { *out = AIRFRAME_AILEVON_MODE; return true; }
    return false;
}

// ---- JSON building ----

static cJSON *pid_gains_json(pid_gains_t g) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "k_p", g.k_p);
    cJSON_AddNumberToObject(o, "k_i", g.k_i);
    cJSON_AddNumberToObject(o, "k_d", g.k_d);
    cJSON_AddNumberToObject(o, "i_limit", g.i_limit);
    return o;
}

static cJSON *mission_json(void) {
    waypoint_t wps[NAV_MAX_WAYPOINTS];
    size_t count;
    bool loop;
    nav_get_mission(wps, NAV_MAX_WAYPOINTS, &count, &loop);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "loop", loop);
    cJSON *points = cJSON_CreateArray();
    for (size_t i = 0; i < count; i++) {
        cJSON *p = cJSON_CreateObject();
        cJSON_AddNumberToObject(p, "lat", wps[i].lat_deg);
        cJSON_AddNumberToObject(p, "lon", wps[i].lon_deg);
        cJSON_AddItemToArray(points, p);
    }
    cJSON_AddItemToObject(o, "points", points);
    return o;
}

// Best-effort current fix, for the web UI's initial mission-map center and
// its live "plane is here" marker (polled repeatedly -- see api_gps_get()).
// "valid": false whenever gps_task hasn't produced a fix yet (e.g. cold
// start, no sky view on the bench) -- the caller falls back to browser
// geolocation or (0,0) for centering in that case, and just doesn't draw the
// live marker, same as if this endpoint didn't exist.
static cJSON *gps_json(void) {
    bool valid = false;
    double lat = 0.0, lon = 0.0, course_deg = 0.0;
    if (gps_ready && xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(GPS_DATA_MUTEX_WAIT_MS)) == pdTRUE) {
        valid = latest_gps_data.valid;
        lat = latest_gps_data.latitude_deg;
        lon = latest_gps_data.longitude_deg;
        course_deg = latest_gps_data.course_deg;
        xSemaphoreGive(gps_data_mutex);
    }
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "valid", valid);
    cJSON_AddNumberToObject(o, "lat", lat);
    cJSON_AddNumberToObject(o, "lon", lon);
    cJSON_AddNumberToObject(o, "course_deg", course_deg);

    waypoint_t home;
    if (nav_get_home(&home)) {
        cJSON *h = cJSON_CreateObject();
        cJSON_AddNumberToObject(h, "lat", home.lat_deg);
        cJSON_AddNumberToObject(h, "lon", home.lon_deg);
        cJSON_AddItemToObject(o, "home", h);
    } else {
        cJSON_AddNullToObject(o, "home");
    }
    return o;
}

// Bench-verification readout for the setup-mode IMU card. mag_valid mirrors
// imu_data_t's own meaning: true only when yaw was just fused with a
// trustworthy, non-stale magnetometer sample (see imu.h) -- false means
// yaw is gyro-only drift, not a real compass heading, which matters when
// judging whether the yaw number on screen should be trusted.
static cJSON *imu_json(void) {
    bool ready = imu_ready;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    bool mag_valid = false;
    if (ready && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT)) == pdTRUE) {
        roll = imu_data.roll;
        pitch = imu_data.pitch;
        yaw = imu_data.yaw;
        mag_valid = imu_data.mag_valid;
        xSemaphoreGive(imu_data_mutex);
    }
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ready", ready);
    cJSON_AddNumberToObject(o, "roll", roll);
    cJSON_AddNumberToObject(o, "pitch", pitch);
    cJSON_AddNumberToObject(o, "yaw", yaw);
    cJSON_AddBoolToObject(o, "mag_valid", mag_valid);
    return o;
}

static cJSON *output_channel_json(int ch, const char *name) {
    output_cfg_t cfg = get_channel_output_cfg(ch);
    item_type_t type = channel_output_type(ch);
    uint16_t abs_min = PULSEWIDTH_ABS_MIN_US;
    uint16_t abs_max = PULSEWIDTH_ABS_MAX_US;

    // get_channel_pulse_width() is only valid for the 6 RC-mirrored channels
    // -- ESC1/ESC2 have no capture input of their own (see output_ctl.h's
    // ESC1_CH/ESC2_CH comment). They always mirror RC_THROTTLE's live input
    // (see apply_throttle_to_escs() in main.c), so that's the truthful
    // "live_us" to report for them too.
    uint32_t live_us = (ch < NUM_RC_CHANNELS) ? get_channel_pulse_width(ch) : get_channel_pulse_width(RC_THROTTLE);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "channel", ch + 1);
    cJSON_AddStringToObject(o, "name", name);
    cJSON_AddStringToObject(o, "type", (type == MOTOR_TYPE) ? "motor" : "servo");
    cJSON_AddNumberToObject(o, "live_us", live_us);
    cJSON_AddNumberToObject(o, "out_us", get_channel_output_us(ch));
    cJSON_AddNumberToObject(o, "min_us", cfg.min_us);
    cJSON_AddNumberToObject(o, "max_us", cfg.max_us);
    cJSON_AddBoolToObject(o, "reversed", cfg.reversed);
    // Physical servo_out pin (1-based) this function currently drives, or
    // null for ESC1/ESC2, which have their own dedicated connectors.
    if (ch < NUM_RC_CHANNELS) {
        cJSON_AddNumberToObject(o, "output_pin", get_servo_output_map().phys_out[ch] + 1);
    } else {
        cJSON_AddNullToObject(o, "output_pin");
    }
    cJSON_AddNumberToObject(o, "abs_min_us", abs_min);
    cJSON_AddNumberToObject(o, "abs_max_us", abs_max);
    return o;
}

static const char *const CHANNEL_NAMES[NUM_OUTPUT_CHANNELS] = {
    [RC_AILERON]  = "Aileron",
    [RC_ELEVATOR] = "Elevator",
    [RC_THROTTLE] = "Spare",
    [RC_DIAL]     = "Aux (dial)",
    [RC_RUDDER]   = "Rudder",
    [RC_SWITCH]   = "Aux (switch)",
    [ESC1_CH]     = "ESC1",
    [ESC2_CH]     = "ESC2",
};

// Separate from CHANNEL_NAMES above: that array names each *output* slot
// (where RC_THROTTLE's own output is "Spare", see main.c), but these name
// each logical RC *input* function -- RC_THROTTLE is very much still "the
// throttle stick" on the input side, just no longer mirrored to its own
// output pin.
static const char *const RC_INPUT_NAMES[NUM_RC_CHANNELS] = {
    [RC_AILERON]  = "Aileron",
    [RC_ELEVATOR] = "Elevator",
    [RC_THROTTLE] = "Throttle",
    [RC_DIAL]     = "Aux (dial)",
    [RC_RUDDER]   = "Rudder",
    [RC_SWITCH]   = "Aux (switch)",
};

static cJSON *rc_input_map_json(void) {
    rc_input_map_cfg_t map = get_rc_input_map();

    cJSON *o = cJSON_CreateObject();
    cJSON *entries = cJSON_CreateArray();
    for (int logical = 0; logical < NUM_RC_CHANNELS; logical++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddNumberToObject(e, "logical", logical + 1);
        cJSON_AddStringToObject(e, "name", RC_INPUT_NAMES[logical]);
        cJSON_AddNumberToObject(e, "physical_pin", map.phys_ch[logical] + 1);
        cJSON_AddItemToArray(entries, e);
    }
    cJSON_AddItemToObject(o, "map", entries);

    // Raw per-physical-pin live pulse widths, bypassing the map entirely --
    // lets the setup-mode UI show "wiggle a stick, watch which pin number
    // moves" regardless of how the logical functions are currently mapped.
    cJSON *live = cJSON_CreateArray();
    for (int phys = 0; phys < NUM_RC_CHANNELS; phys++) {
        cJSON_AddItemToArray(live, cJSON_CreateNumber(get_physical_pulse_width(phys)));
    }
    cJSON_AddItemToObject(o, "physical_live_us", live);
    return o;
}

static cJSON *servo_output_map_json(void) {
    servo_output_map_cfg_t map = get_servo_output_map();

    cJSON *o = cJSON_CreateObject();
    cJSON *entries = cJSON_CreateArray();
    for (int logical = 0; logical < NUM_RC_CHANNELS; logical++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddNumberToObject(e, "logical", logical + 1);
        cJSON_AddStringToObject(e, "name", CHANNEL_NAMES[logical]);
        cJSON_AddNumberToObject(e, "output_pin", map.phys_out[logical] + 1);
        cJSON_AddItemToArray(entries, e);
    }
    cJSON_AddItemToObject(o, "map", entries);
    return o;
}

static cJSON *rc_mode_json(void) {
    rc_mode_status_t status = get_rc_mode_status();
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "autonomous", status.autonomous);
    cJSON_AddBoolToObject(o, "radio_connected", status.radio_connected);
    cJSON_AddBoolToObject(o, "signal_stale", status.signal_stale);
    return o;
}

static cJSON *motor_cfg_json(motor_cfg_t cfg) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "motor_count", cfg.motor_count);
    cJSON_AddBoolToObject(o, "esc1_is_left", cfg.esc1_is_left);
    return o;
}

// Bench-verification readout for the setup-mode Airspeed card: reading=false
// means the sensor never self-validated (see airspeed.c's WHO_AM_I-style
// gate) and live_cms is meaningless in that case, same convention
// update_autonomous_outputs() already uses in main.c.
static cJSON *airspeed_live_json(airspeed_cfg_t cfg) {
    bool reading = airspeed_reading();
    int16_t raw_cms = airspeed_get();
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "target_cms", cfg.target_cms);
    cJSON_AddNumberToObject(o, "fallback_pct", cfg.fallback_pct);
    cJSON_AddBoolToObject(o, "reading", reading);
    cJSON_AddNumberToObject(o, "live_cms", (reading && raw_cms != INT16_MIN) ? raw_cms : 0);
    return o;
}

static cJSON *full_state_json(void) {
    cJSON *root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "mission", mission_json());
    cJSON_AddItemToObject(root, "gps", gps_json());

    cJSON *pid = cJSON_CreateObject();
    for (pid_target_t t = PID_TARGET_ROLL; t <= PID_TARGET_AIRSPEED; t++) {
        cJSON_AddItemToObject(pid, pid_target_name(t), pid_gains_json(get_pid_gains(t)));
    }
    cJSON_AddItemToObject(root, "pid", pid);

    cJSON *outputs = cJSON_CreateArray();
    for (int ch = 0; ch < NUM_OUTPUT_CHANNELS; ch++) {
        cJSON_AddItemToArray(outputs, output_channel_json(ch, CHANNEL_NAMES[ch]));
    }
    cJSON_AddItemToObject(root, "outputs", outputs);

    cJSON_AddStringToObject(root, "airframe", airframe_mode_name(get_airframe_mode()));
    cJSON_AddStringToObject(root, "nav_mode", nav_mode_name(nav_get_mode()));
    cJSON_AddItemToObject(root, "motor_cfg", motor_cfg_json(get_motor_cfg()));

    cJSON_AddItemToObject(root, "airspeed_cfg", airspeed_live_json(get_airspeed_cfg()));

    return root;
}

// status_code: only 200 ("OK") and 400 ("400 Bad Request") are ever passed
// in this file -- internal JSON-encode failures go through httpd_resp_send_500
// directly instead of through here.
static esp_err_t send_json(httpd_req_t *req, cJSON *obj, int status_code) {
    char *text = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (text == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    if (status_code == 400) {
        httpd_resp_set_status(req, "400 Bad Request");
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, text, strlen(text));
    free(text);
    return ret;
}

static esp_err_t send_error(httpd_req_t *req, int status_code, const char *msg) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "error", msg);
    return send_json(req, o, status_code);
}

static esp_err_t recv_json_body(httpd_req_t *req, cJSON **out) {
    if (req->content_len == 0 || req->content_len > MAX_JSON_BODY_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    char *buf = malloc(req->content_len + 1);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    size_t received = 0;
    while (received < req->content_len) {
        int r = httpd_req_recv(req, buf + received, req->content_len - received);
        if (r <= 0) {
            free(buf);
            return ESP_FAIL;
        }
        received += (size_t)r;
    }
    buf[received] = '\0';
    *out = cJSON_Parse(buf);
    free(buf);
    return (*out != NULL) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

// ---- API handlers ----

static esp_err_t api_state_get(httpd_req_t *req) {
    return send_json(req, full_state_json(), 200);
}

static esp_err_t api_gps_get(httpd_req_t *req) {
    return send_json(req, gps_json(), 200);
}

static esp_err_t api_imu_get(httpd_req_t *req) {
    return send_json(req, imu_json(), 200);
}

static esp_err_t api_mission_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    cJSON *points_arr = cJSON_GetObjectItemCaseSensitive(body, "points");
    if (!cJSON_IsArray(points_arr)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"points\" must be an array");
    }
    int n = cJSON_GetArraySize(points_arr);
    if (n > MAX_MISSION_POINTS_PER_REQUEST) {
        cJSON_Delete(body);
        return send_error(req, 400, "point count must be 0.." STR(MAX_MISSION_POINTS_PER_REQUEST));
    }

    waypoint_t wps[MAX_MISSION_POINTS_PER_REQUEST];
    for (int i = 0; i < n; i++) {
        cJSON *p = cJSON_GetArrayItem(points_arr, i);
        cJSON *lat = cJSON_GetObjectItemCaseSensitive(p, "lat");
        cJSON *lon = cJSON_GetObjectItemCaseSensitive(p, "lon");
        if (!cJSON_IsNumber(lat) || !cJSON_IsNumber(lon)) {
            cJSON_Delete(body);
            return send_error(req, 400, "each point needs numeric \"lat\" and \"lon\"");
        }
        wps[i].lat_deg = lat->valuedouble;
        wps[i].lon_deg = lon->valuedouble;
    }

    // "loop" is optional -- an edit that doesn't mention it (e.g. just
    // moving a point) must leave the previously-stored value alone.
    size_t cur_count;
    bool loop;
    waypoint_t cur_wps[NAV_MAX_WAYPOINTS];
    nav_get_mission(cur_wps, NAV_MAX_WAYPOINTS, &cur_count, &loop);
    cJSON *loop_j = cJSON_GetObjectItemCaseSensitive(body, "loop");
    if (loop_j != NULL) {
        if (!cJSON_IsBool(loop_j)) {
            cJSON_Delete(body);
            return send_error(req, 400, "\"loop\" must be a boolean");
        }
        loop = cJSON_IsTrue(loop_j);
    }
    cJSON_Delete(body);

    if (!nav_set_mission(wps, (size_t)n, loop)) {
        ESP_LOGW(TAG, "mission applied live but failed to persist to NVS");
    }
    return send_json(req, mission_json(), 200);
}

static esp_err_t api_pid_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    cJSON *target_j = cJSON_GetObjectItemCaseSensitive(body, "target");
    pid_target_t target;
    if (!cJSON_IsString(target_j) || !parse_pid_target(target_j->valuestring, &target)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"target\" must be one of roll/pitch/heading/airspeed");
    }

    static const struct { const char *key; uint8_t bit; } fields[] = {
        { "k_p", PID_FIELD_KP }, { "k_i", PID_FIELD_KI }, { "k_d", PID_FIELD_KD }, { "i_limit", PID_FIELD_ILIMIT },
    };
    uint8_t fields_present = 0;
    pid_gains_t gains = {0};
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        cJSON *v = cJSON_GetObjectItemCaseSensitive(body, fields[i].key);
        if (v == NULL) {
            continue;
        }
        if (!cJSON_IsNumber(v)) {
            cJSON_Delete(body);
            return send_error(req, 400, "PID fields must be numbers");
        }
        fields_present |= fields[i].bit;
        float fv = (float)v->valuedouble;
        switch (fields[i].bit) {
            case PID_FIELD_KP:     gains.k_p = fv; break;
            case PID_FIELD_KI:     gains.k_i = fv; break;
            case PID_FIELD_KD:     gains.k_d = fv; break;
            case PID_FIELD_ILIMIT: gains.i_limit = fv; break;
        }
    }
    cJSON_Delete(body);

    if (fields_present == 0) {
        return send_error(req, 400, "no PID fields present -- include at least one of k_p/k_i/k_d/i_limit");
    }
    bool ok = gain_ok(fields_present, PID_FIELD_KP, gains.k_p)
        && gain_ok(fields_present, PID_FIELD_KI, gains.k_i)
        && gain_ok(fields_present, PID_FIELD_KD, gains.k_d)
        && gain_ok(fields_present, PID_FIELD_ILIMIT, gains.i_limit);
    if (!ok) {
        return send_error(req, 400, "gain value(s) out of range (must be finite, magnitude <= " STR(PID_GAIN_ABS_MAX) ")");
    }

    if (!set_pid_gains(target, fields_present, &gains)) {
        ESP_LOGW(TAG, "pid gains applied live but failed to persist to NVS");
    }
    return send_json(req, pid_gains_json(get_pid_gains(target)), 200);
}

static esp_err_t api_outputs_get(httpd_req_t *req) {
    cJSON *arr = cJSON_CreateArray();
    for (int ch = 0; ch < NUM_OUTPUT_CHANNELS; ch++) {
        cJSON_AddItemToArray(arr, output_channel_json(ch, CHANNEL_NAMES[ch]));
    }
    return send_json(req, arr, 200);
}

static esp_err_t api_outputs_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    cJSON *channel_j = cJSON_GetObjectItemCaseSensitive(body, "channel");
    if (!cJSON_IsNumber(channel_j)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"channel\" must be a number (1.." STR(NUM_OUTPUT_CHANNELS) ")");
    }
    int ch = channel_j->valueint - 1;
    if (ch < 0 || ch >= NUM_OUTPUT_CHANNELS) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"channel\" out of range (1.." STR(NUM_OUTPUT_CHANNELS) ")");
    }

    output_cfg_t cfg = get_channel_output_cfg(ch);
    cJSON *min_j = cJSON_GetObjectItemCaseSensitive(body, "min_us");
    cJSON *max_j = cJSON_GetObjectItemCaseSensitive(body, "max_us");
    cJSON *rev_j = cJSON_GetObjectItemCaseSensitive(body, "reversed");
    if (min_j != NULL) {
        if (!cJSON_IsNumber(min_j)) { cJSON_Delete(body); return send_error(req, 400, "\"min_us\" must be a number"); }
        cfg.min_us = (uint16_t)min_j->valueint;
    }
    if (max_j != NULL) {
        if (!cJSON_IsNumber(max_j)) { cJSON_Delete(body); return send_error(req, 400, "\"max_us\" must be a number"); }
        cfg.max_us = (uint16_t)max_j->valueint;
    }
    if (rev_j != NULL) {
        if (!cJSON_IsBool(rev_j)) { cJSON_Delete(body); return send_error(req, 400, "\"reversed\" must be a boolean"); }
        cfg.reversed = cJSON_IsTrue(rev_j);
    }
    cJSON_Delete(body);

    esp_err_t err = set_channel_output_cfg(ch, &cfg);
    if (err == ESP_ERR_INVALID_ARG) {
        return send_error(req, 400, "invalid range (min_us must be < max_us, within the channel's absolute bounds)");
    }
    if (err != ESP_OK) {
        return send_error(req, 500, "applied live, but failed to save to flash -- will be lost on reboot");
    }
    return send_json(req, output_channel_json(ch, CHANNEL_NAMES[ch]), 200);
}

static esp_err_t api_nav_mode_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }
    cJSON *mode_j = cJSON_GetObjectItemCaseSensitive(body, "mode");
    nav_mode_t mode;
    if (!cJSON_IsString(mode_j) || !parse_nav_mode(mode_j->valuestring, &mode)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"mode\" must be \"waypoint\" or \"heading_hold\"");
    }
    cJSON_Delete(body);

    if (!nav_set_mode(mode)) {
        ESP_LOGW(TAG, "nav mode change failed to apply or persist");
    }
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "nav_mode", nav_mode_name(nav_get_mode()));
    return send_json(req, o, 200);
}

static esp_err_t api_airframe_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }
    cJSON *mode_j = cJSON_GetObjectItemCaseSensitive(body, "mode");
    airframe_mode_t mode;
    if (!cJSON_IsString(mode_j) || !parse_airframe_mode(mode_j->valuestring, &mode)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"mode\" must be \"conventional\" or \"ailevon\"");
    }
    cJSON_Delete(body);

    if (!set_airframe_mode(mode)) {
        ESP_LOGW(TAG, "airframe mode applied live but failed to persist to NVS");
    }
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "airframe", airframe_mode_name(get_airframe_mode()));
    return send_json(req, o, 200);
}

static esp_err_t api_airspeed_get(httpd_req_t *req) {
    return send_json(req, airspeed_live_json(get_airspeed_cfg()), 200);
}

static esp_err_t api_airspeed_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }
    airspeed_cfg_t cfg = get_airspeed_cfg();
    cJSON *target_j = cJSON_GetObjectItemCaseSensitive(body, "target_cms");
    cJSON *fallback_j = cJSON_GetObjectItemCaseSensitive(body, "fallback_pct");
    if (target_j != NULL) {
        if (!cJSON_IsNumber(target_j)) { cJSON_Delete(body); return send_error(req, 400, "\"target_cms\" must be a number"); }
        cfg.target_cms = (float)target_j->valuedouble;
    }
    if (fallback_j != NULL) {
        if (!cJSON_IsNumber(fallback_j)) { cJSON_Delete(body); return send_error(req, 400, "\"fallback_pct\" must be a number"); }
        cfg.fallback_pct = (float)fallback_j->valuedouble;
    }
    cJSON_Delete(body);

    if (!set_airspeed_cfg(&cfg)) {
        return send_error(req, 400, "invalid airspeed cfg (fallback_pct must be 0.0-1.0, values must be finite)");
    }
    return send_json(req, airspeed_live_json(cfg), 200);
}

static esp_err_t api_motor_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }
    motor_cfg_t cfg = get_motor_cfg();
    cJSON *count_j = cJSON_GetObjectItemCaseSensitive(body, "motor_count");
    cJSON *left_j = cJSON_GetObjectItemCaseSensitive(body, "esc1_is_left");
    if (count_j != NULL) {
        if (!cJSON_IsNumber(count_j)) { cJSON_Delete(body); return send_error(req, 400, "\"motor_count\" must be a number"); }
        cfg.motor_count = (uint8_t)count_j->valueint;
    }
    if (left_j != NULL) {
        if (!cJSON_IsBool(left_j)) { cJSON_Delete(body); return send_error(req, 400, "\"esc1_is_left\" must be a boolean"); }
        cfg.esc1_is_left = cJSON_IsTrue(left_j);
    }
    cJSON_Delete(body);

    if (!set_motor_cfg(&cfg)) {
        return send_error(req, 400, "\"motor_count\" must be 1 or 2");
    }
    return send_json(req, motor_cfg_json(get_motor_cfg()), 200);
}

// Bench-spins one ESC briefly so the operator can see which physical motor
// it is -- see start_esc_test() in main.c. Body: {"esc": 1|2, "duration_ms"?}.
#define ESC_TEST_DEFAULT_MS (1000)
static esp_err_t api_motor_test_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }
    cJSON *esc_j = cJSON_GetObjectItemCaseSensitive(body, "esc");
    cJSON *dur_j = cJSON_GetObjectItemCaseSensitive(body, "duration_ms");
    if (!cJSON_IsNumber(esc_j) || (dur_j != NULL && !cJSON_IsNumber(dur_j))) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"esc\" must be 1 or 2, \"duration_ms\" (optional) a number");
    }
    int esc = esc_j->valueint;
    int duration_ms = dur_j ? dur_j->valueint : ESC_TEST_DEFAULT_MS;
    cJSON_Delete(body);

    int esc_ch = (esc == 1) ? ESC1_CH : (esc == 2) ? ESC2_CH : -1;
    if (!start_esc_test(esc_ch, duration_ms)) {
        return send_error(req, 400, "\"esc\" must be 1 or 2, \"duration_ms\" in 1.." STR(ESC_TEST_MAX_MS));
    }
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "esc", esc);
    cJSON_AddNumberToObject(o, "duration_ms", duration_ms);
    return send_json(req, o, 200);
}

// Setup-mode control tests -- see start_direction_test()/start_level_test()
// in main.c.
#define DIRECTION_TEST_MS (4000)
#define LEVEL_TEST_MS (15000)

static cJSON *control_test_json(void) {
    control_test_status_t s = get_control_test_status();
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "active", s.active);
    cJSON_AddNumberToObject(o, "remaining_ms", s.remaining_ms);
    cJSON_AddStringToObject(o, "kind", s.kind == CONTROL_TEST_LEVEL ? "level" : "direction");
    cJSON_AddNumberToObject(o, "roll_deg", s.roll_deg);
    cJSON_AddNumberToObject(o, "pitch_deg", s.pitch_deg);
    cJSON_AddNumberToObject(o, "roll_cmd_us", s.roll_cmd_us);
    cJSON_AddNumberToObject(o, "pitch_cmd_us", s.pitch_cmd_us);
    return o;
}

static esp_err_t api_control_test_get(httpd_req_t *req) {
    return send_json(req, control_test_json(), 200);
}

// Body: {"test": "pitch_up" | "pitch_down" | "roll_left" | "roll_right" | "level" | "stop"}
static esp_err_t api_control_test_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }
    cJSON *test_j = cJSON_GetObjectItemCaseSensitive(body, "test");
    if (!cJSON_IsString(test_j)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"test\" must be a string");
    }
    char t[16];
    strlcpy(t, test_j->valuestring, sizeof(t));
    cJSON_Delete(body);

    const float full = CONTROL_TEST_FULL_CMD_US;
    bool ok = true;
    if (strcmp(t, "pitch_up") == 0)        ok = start_direction_test(0.0f, full, DIRECTION_TEST_MS);
    else if (strcmp(t, "pitch_down") == 0) ok = start_direction_test(0.0f, -full, DIRECTION_TEST_MS);
    else if (strcmp(t, "roll_right") == 0) ok = start_direction_test(full, 0.0f, DIRECTION_TEST_MS);
    else if (strcmp(t, "roll_left") == 0)  ok = start_direction_test(-full, 0.0f, DIRECTION_TEST_MS);
    else if (strcmp(t, "level") == 0) {
        if (!start_level_test(LEVEL_TEST_MS)) {
            return send_error(req, 409, "IMU isn't ready yet -- wait for gyro calibration to finish (keep the board still)");
        }
    }
    else if (strcmp(t, "stop") == 0)       stop_control_test();
    else {
        return send_error(req, 400, "\"test\" must be pitch_up, pitch_down, roll_left, roll_right, level or stop");
    }
    if (!ok) {
        return send_error(req, 500, "couldn't start test");
    }
    return send_json(req, control_test_json(), 200);
}

static esp_err_t api_rc_map_get(httpd_req_t *req) {
    return send_json(req, rc_input_map_json(), 200);
}

static esp_err_t api_rc_map_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    cJSON *logical_j = cJSON_GetObjectItemCaseSensitive(body, "logical");
    cJSON *phys_j = cJSON_GetObjectItemCaseSensitive(body, "physical_pin");
    if (!cJSON_IsNumber(logical_j) || !cJSON_IsNumber(phys_j)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"logical\" and \"physical_pin\" must both be numbers (1.." STR(NUM_RC_CHANNELS) ")");
    }
    int logical = logical_j->valueint - 1;
    int phys = phys_j->valueint - 1;
    cJSON_Delete(body);

    if (logical < 0 || logical >= NUM_RC_CHANNELS) {
        return send_error(req, 400, "\"logical\" out of range (1.." STR(NUM_RC_CHANNELS) ")");
    }

    rc_input_map_cfg_t map = get_rc_input_map();
    map.phys_ch[logical] = (uint8_t)phys;
    if (!set_rc_input_map(&map)) {
        return send_error(req, 400, "\"physical_pin\" out of range (1.." STR(NUM_RC_CHANNELS) ")");
    }
    return send_json(req, rc_input_map_json(), 200);
}

static esp_err_t api_servo_map_get(httpd_req_t *req) {
    return send_json(req, servo_output_map_json(), 200);
}

// Assigns one logical function to a physical servo_out pin. Whichever
// function previously held that pin takes over this function's old pin, so
// the map stays a permutation (see config_store.h's servo_output_map_cfg_t)
// and no pin is ever driven by two functions at once.
static esp_err_t api_servo_map_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    cJSON *logical_j = cJSON_GetObjectItemCaseSensitive(body, "logical");
    cJSON *pin_j = cJSON_GetObjectItemCaseSensitive(body, "output_pin");
    if (!cJSON_IsNumber(logical_j) || !cJSON_IsNumber(pin_j)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"logical\" and \"output_pin\" must both be numbers (1.." STR(NUM_RC_CHANNELS) ")");
    }
    int logical = logical_j->valueint - 1;
    int pin = pin_j->valueint - 1;
    cJSON_Delete(body);

    if (logical < 0 || logical >= NUM_RC_CHANNELS) {
        return send_error(req, 400, "\"logical\" out of range (1.." STR(NUM_RC_CHANNELS) ")");
    }
    if (pin < 0 || pin >= NUM_RC_CHANNELS) {
        return send_error(req, 400, "\"output_pin\" out of range (1.." STR(NUM_RC_CHANNELS) ")");
    }

    servo_output_map_cfg_t map = get_servo_output_map();
    for (int other = 0; other < NUM_RC_CHANNELS; other++) {
        if (other != logical && map.phys_out[other] == pin) {
            map.phys_out[other] = map.phys_out[logical];
        }
    }
    map.phys_out[logical] = (uint8_t)pin;
    if (!set_servo_output_map(&map)) {
        ESP_LOGW(TAG, "servo output map applied live but failed to persist to NVS");
    }
    return send_json(req, servo_output_map_json(), 200);
}

static cJSON *trim_json(void) {
    trim_cfg_t trim = get_trim_cfg();
    cJSON *o = cJSON_CreateObject();
    cJSON *arr = cJSON_CreateArray();
    for (int ch = 0; ch < NUM_RC_CHANNELS; ch++) {
        if (!trim_channel_allowed(ch)) continue;
        cJSON *e = cJSON_CreateObject();
        cJSON_AddNumberToObject(e, "logical", ch + 1);
        cJSON_AddStringToObject(e, "name", CHANNEL_NAMES[ch]);
        cJSON_AddNumberToObject(e, "center_us", trim.center_us[ch]);
        cJSON_AddNumberToObject(e, "live_us", get_channel_pulse_width(ch));
        cJSON_AddItemToArray(arr, e);
    }
    cJSON_AddItemToObject(o, "channels", arr);
    cJSON_AddNumberToObject(o, "neutral_us", TRIM_NEUTRAL_US);
    cJSON_AddNumberToObject(o, "max_offset_us", TRIM_MAX_OFFSET_US);
    return o;
}

static esp_err_t api_trim_get(httpd_req_t *req) {
    return send_json(req, trim_json(), 200);
}

// Body is one of:
//   {"capture": true}                    -- capture all trimmable channels
//   {"logical": N, "center_us": US}      -- set one channel by hand
static esp_err_t api_trim_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    if (cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(body, "capture"))) {
        cJSON_Delete(body);
        char err[128];
        if (!capture_trim_from_inputs(err, sizeof(err))) {
            return send_error(req, 400, err);
        }
        return send_json(req, trim_json(), 200);
    }

    cJSON *logical_j = cJSON_GetObjectItemCaseSensitive(body, "logical");
    cJSON *center_j = cJSON_GetObjectItemCaseSensitive(body, "center_us");
    if (!cJSON_IsNumber(logical_j) || !cJSON_IsNumber(center_j)) {
        cJSON_Delete(body);
        return send_error(req, 400, "expected {\"capture\": true} or numeric \"logical\" and \"center_us\"");
    }
    int ch = logical_j->valueint - 1;
    int center = center_j->valueint;
    cJSON_Delete(body);

    if (ch < 0 || ch >= NUM_RC_CHANNELS || !trim_channel_allowed(ch)) {
        return send_error(req, 400, "\"logical\" must be a trimmable channel (aileron, elevator, rudder)");
    }
    if (center < TRIM_NEUTRAL_US - TRIM_MAX_OFFSET_US || center > TRIM_NEUTRAL_US + TRIM_MAX_OFFSET_US) {
        return send_error(req, 400, "\"center_us\" must be within " STR(TRIM_MAX_OFFSET_US) " us of " STR(TRIM_NEUTRAL_US));
    }
    if (!set_trim_center(ch, (uint16_t)center)) {
        ESP_LOGW(TAG, "trim applied live but failed to persist to NVS");
    }
    return send_json(req, trim_json(), 200);
}

static const char *reset_reason_name(uint32_t r) {
    switch ((esp_reset_reason_t)r) {
        case ESP_RST_POWERON:   return "power on";
        case ESP_RST_EXT:       return "external pin reset";
        case ESP_RST_SW:        return "software reset";
        case ESP_RST_PANIC:     return "CRASH (panic)";
        case ESP_RST_INT_WDT:   return "CRASH (interrupt watchdog)";
        case ESP_RST_TASK_WDT:  return "CRASH (task watchdog)";
        case ESP_RST_WDT:       return "CRASH (other watchdog)";
        case ESP_RST_BROWNOUT:  return "BROWNOUT (supply voltage sagged)";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        case ESP_RST_SDIO:      return "SDIO reset";
        default:                return "unknown";
    }
}

static const char *const FLOG_TYPE_NAMES[FLOG_TYPE_COUNT] = {
    [FLOG_BOOT]               = "boot",
    [FLOG_RC_DROPOUT]         = "rc_dropout",
    [FLOG_RC_LOST]            = "rc_lost",
    [FLOG_RC_REGAINED]        = "rc_regained",
    [FLOG_FAILSAFE_RTH]       = "failsafe_rth",
    [FLOG_FAILSAFE_DESCEND]   = "failsafe_descend",
    [FLOG_FAILSAFE_CLEARED]   = "failsafe_cleared",
    [FLOG_AUTONOMOUS_LOCKOUT] = "autonomous_lockout",
    [FLOG_IMU_FAULT]          = "imu_fault",
    [FLOG_NO_RADIO_SAFE]      = "no_radio_safe",
};

static cJSON *flight_log_json(void) {
    static flight_log_entry_t entries[FLIGHT_LOG_CAPACITY];  // static: keep it off the httpd stack
    size_t n = flight_log_read(entries, FLIGHT_LOG_CAPACITY);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "current_boot", flight_log_current_boot());
    cJSON *arr = cJSON_CreateArray();
    for (size_t i = 0; i < n; i++) {
        const flight_log_entry_t *e = &entries[i];
        cJSON *j = cJSON_CreateObject();
        cJSON_AddNumberToObject(j, "boot", e->boot);
        cJSON_AddNumberToObject(j, "t_ms", e->uptime_ms);
        cJSON_AddStringToObject(j, "type", e->type < FLOG_TYPE_COUNT ? FLOG_TYPE_NAMES[e->type] : "unknown");
        cJSON_AddNumberToObject(j, "value", e->value);
        if (e->type == FLOG_BOOT) {
            cJSON_AddStringToObject(j, "reset_reason", reset_reason_name(e->value));
        }
        cJSON_AddItemToArray(arr, j);
    }
    cJSON_AddItemToObject(o, "entries", arr);
    return o;
}

static esp_err_t api_flight_log_get(httpd_req_t *req) {
    return send_json(req, flight_log_json(), 200);
}

// Body: {"clear": true}
static esp_err_t api_flight_log_post(httpd_req_t *req) {
    cJSON *body;
    if (recv_json_body(req, &body) != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }
    bool clear = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(body, "clear"));
    cJSON_Delete(body);
    if (!clear) {
        return send_error(req, 400, "expected {\"clear\": true}");
    }
    flight_log_clear();
    return send_json(req, flight_log_json(), 200);
}

static esp_err_t api_mode_get(httpd_req_t *req) {
    return send_json(req, rc_mode_json(), 200);
}

// ---- Static asset handlers ----

static esp_err_t send_embedded_text(httpd_req_t *req, const char *content_type, const uint8_t *start, const uint8_t *end) {
    httpd_resp_set_type(req, content_type);
    // EMBED_TXTFILES appends a trailing null terminator; exclude it from Content-Length.
    return httpd_resp_send(req, (const char *)start, (end - start) - 1);
}

static esp_err_t send_embedded_binary(httpd_req_t *req, const char *content_type, const uint8_t *start, const uint8_t *end) {
    httpd_resp_set_type(req, content_type);
    return httpd_resp_send(req, (const char *)start, end - start);
}

static esp_err_t root_get(httpd_req_t *req)          { return send_embedded_text(req, "text/html", index_html_start, index_html_end); }
static esp_err_t app_js_get(httpd_req_t *req)         { return send_embedded_text(req, "application/javascript", app_js_start, app_js_end); }
static esp_err_t style_css_get(httpd_req_t *req)      { return send_embedded_text(req, "text/css", style_css_start, style_css_end); }
static esp_err_t leaflet_js_get(httpd_req_t *req)     { return send_embedded_text(req, "application/javascript", leaflet_js_start, leaflet_js_end); }
static esp_err_t leaflet_css_get(httpd_req_t *req)    { return send_embedded_text(req, "text/css", leaflet_css_start, leaflet_css_end); }
static esp_err_t marker_icon_get(httpd_req_t *req)    { return send_embedded_binary(req, "image/png", marker_icon_png_start, marker_icon_png_end); }
static esp_err_t marker_icon_2x_get(httpd_req_t *req) { return send_embedded_binary(req, "image/png", marker_icon_2x_png_start, marker_icon_2x_png_end); }
static esp_err_t marker_shadow_get(httpd_req_t *req)  { return send_embedded_binary(req, "image/png", marker_shadow_png_start, marker_shadow_png_end); }

void http_server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 40;
    config.stack_size = 8192;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    static const httpd_uri_t routes[] = {
        { .uri = "/",                      .method = HTTP_GET,  .handler = root_get },
        { .uri = "/app.js",                .method = HTTP_GET,  .handler = app_js_get },
        { .uri = "/style.css",             .method = HTTP_GET,  .handler = style_css_get },
        { .uri = "/leaflet.js",            .method = HTTP_GET,  .handler = leaflet_js_get },
        { .uri = "/leaflet.css",           .method = HTTP_GET,  .handler = leaflet_css_get },
        { .uri = "/images/marker-icon.png",    .method = HTTP_GET, .handler = marker_icon_get },
        { .uri = "/images/marker-icon-2x.png", .method = HTTP_GET, .handler = marker_icon_2x_get },
        { .uri = "/images/marker-shadow.png",  .method = HTTP_GET, .handler = marker_shadow_get },
        { .uri = "/api/state",             .method = HTTP_GET,  .handler = api_state_get },
        { .uri = "/api/gps",               .method = HTTP_GET,  .handler = api_gps_get },
        { .uri = "/api/imu",               .method = HTTP_GET,  .handler = api_imu_get },
        { .uri = "/api/mission",           .method = HTTP_POST, .handler = api_mission_post },
        { .uri = "/api/pid",               .method = HTTP_POST, .handler = api_pid_post },
        { .uri = "/api/outputs",           .method = HTTP_GET,  .handler = api_outputs_get },
        { .uri = "/api/outputs",           .method = HTTP_POST, .handler = api_outputs_post },
        { .uri = "/api/nav_mode",          .method = HTTP_POST, .handler = api_nav_mode_post },
        { .uri = "/api/airframe",          .method = HTTP_POST, .handler = api_airframe_post },
        { .uri = "/api/airspeed",          .method = HTTP_GET,  .handler = api_airspeed_get },
        { .uri = "/api/airspeed",          .method = HTTP_POST, .handler = api_airspeed_post },
        { .uri = "/api/motor",             .method = HTTP_POST, .handler = api_motor_post },
        { .uri = "/api/control_test",      .method = HTTP_GET,  .handler = api_control_test_get },
        { .uri = "/api/control_test",      .method = HTTP_POST, .handler = api_control_test_post },
        { .uri = "/api/motor_test",        .method = HTTP_POST, .handler = api_motor_test_post },
        { .uri = "/api/rc_map",            .method = HTTP_GET,  .handler = api_rc_map_get },
        { .uri = "/api/rc_map",            .method = HTTP_POST, .handler = api_rc_map_post },
        { .uri = "/api/servo_map",         .method = HTTP_GET,  .handler = api_servo_map_get },
        { .uri = "/api/servo_map",         .method = HTTP_POST, .handler = api_servo_map_post },
        { .uri = "/api/trim",              .method = HTTP_GET,  .handler = api_trim_get },
        { .uri = "/api/trim",              .method = HTTP_POST, .handler = api_trim_post },
        { .uri = "/api/flight_log",        .method = HTTP_GET,  .handler = api_flight_log_get },
        { .uri = "/api/flight_log",        .method = HTTP_POST, .handler = api_flight_log_post },
        { .uri = "/api/mode",              .method = HTTP_GET,  .handler = api_mode_get },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI(TAG, "HTTP server listening on http://192.168.4.1/");
}
