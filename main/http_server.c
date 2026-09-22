#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "http_server.h"
#include "output_ctl.h"
#include "nav.h"
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

static cJSON *output_channel_json(int ch, const char *name) {
    output_cfg_t cfg = get_channel_output_cfg(ch);
    item_type_t type = channel_output_type(ch);
    uint16_t abs_min = (type == MOTOR_TYPE) ? MOTOR_MIN_PULSEWIDTH_US : SERVO_MIN_PULSEWIDTH_US;
    uint16_t abs_max = (type == MOTOR_TYPE) ? MOTOR_MAX_PULSEWIDTH_US : SERVO_MAX_PULSEWIDTH_US;

    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "channel", ch + 1);
    cJSON_AddStringToObject(o, "name", name);
    cJSON_AddStringToObject(o, "type", (type == MOTOR_TYPE) ? "motor" : "servo");
    cJSON_AddNumberToObject(o, "live_us", get_channel_pulse_width(ch));
    cJSON_AddNumberToObject(o, "min_us", cfg.min_us);
    cJSON_AddNumberToObject(o, "max_us", cfg.max_us);
    cJSON_AddBoolToObject(o, "reversed", cfg.reversed);
    cJSON_AddNumberToObject(o, "abs_min_us", abs_min);
    cJSON_AddNumberToObject(o, "abs_max_us", abs_max);
    return o;
}

static const char *const CHANNEL_NAMES[NUM_OUTPUT_CHANNELS] = {
    [RC_AILERON]  = "Aileron",
    [RC_ELEVATOR] = "Elevator",
    [RC_THROTTLE] = "Throttle (legacy)",
    [RC_DIAL]     = "Aux (dial)",
    [RC_RUDDER]   = "Rudder",
    [RC_SWITCH]   = "Aux (switch)",
    [ESC1_CH]     = "ESC1",
    [ESC2_CH]     = "ESC2",
};

static cJSON *motor_cfg_json(motor_cfg_t cfg) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "motor_count", cfg.motor_count);
    cJSON_AddBoolToObject(o, "esc1_is_left", cfg.esc1_is_left);
    return o;
}

static cJSON *full_state_json(void) {
    cJSON *root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "mission", mission_json());

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
    cJSON_AddItemToObject(root, "motor_cfg", motor_cfg_json(get_motor_cfg()));

    airspeed_cfg_t aspd = get_airspeed_cfg();
    cJSON *aspd_j = cJSON_CreateObject();
    cJSON_AddNumberToObject(aspd_j, "target_cms", aspd.target_cms);
    cJSON_AddNumberToObject(aspd_j, "fallback_pct", aspd.fallback_pct);
    cJSON_AddItemToObject(root, "airspeed_cfg", aspd_j);

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

    if (!set_channel_output_cfg(ch, &cfg)) {
        return send_error(req, 400, "invalid range (min_us must be < max_us, within the channel's absolute bounds)");
    }
    return send_json(req, output_channel_json(ch, CHANNEL_NAMES[ch]), 200);
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
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "target_cms", cfg.target_cms);
    cJSON_AddNumberToObject(o, "fallback_pct", cfg.fallback_pct);
    return send_json(req, o, 200);
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
    config.max_uri_handlers = 20;
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
        { .uri = "/api/mission",           .method = HTTP_POST, .handler = api_mission_post },
        { .uri = "/api/pid",               .method = HTTP_POST, .handler = api_pid_post },
        { .uri = "/api/outputs",           .method = HTTP_GET,  .handler = api_outputs_get },
        { .uri = "/api/outputs",           .method = HTTP_POST, .handler = api_outputs_post },
        { .uri = "/api/airframe",          .method = HTTP_POST, .handler = api_airframe_post },
        { .uri = "/api/airspeed",          .method = HTTP_POST, .handler = api_airspeed_post },
        { .uri = "/api/motor",             .method = HTTP_POST, .handler = api_motor_post },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI(TAG, "HTTP server listening on http://192.168.4.1/");
}
