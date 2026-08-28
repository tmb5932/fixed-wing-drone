#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "http_server.h"
#include "store.h"
#include "espnow_link.h"
#include "basestation_config.h"
#include "config_link_proto.h"

static const char *TAG = "HTTP_SERVER";

// ---- Embedded web UI assets (see main/CMakeLists.txt's EMBED_TXTFILES /
// EMBED_FILES). ESP-IDF's embed machinery names the generated symbols after
// just the file's basename (not its path), so these names are derived from
// main/web/{index.html,app.js,style.css,lib/leaflet.js,lib/leaflet.css,
// lib/images/marker-icon*.png,marker-shadow.png} accordingly. ----

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

#define STR_(x) #x
#define STR(x) STR_(x)

// ---- string <-> enum helpers ----

static const char *status_str(store_status_t s) {
    switch (s) {
        case STORE_STATUS_UNSET:     return "unset";
        case STORE_STATUS_PENDING:   return "pending";
        case STORE_STATUS_CONFIRMED: return "confirmed";
        case STORE_STATUS_FAILED:    return "failed";
        default:                     return "unset";
    }
}

static const char *ack_reason_str(cl_ack_status_t r) {
    switch (r) {
        case CL_ACK_OK:               return "ok";
        case CL_ACK_BAD_MAGIC:        return "bad_magic";
        case CL_ACK_BAD_TYPE:         return "bad_type";
        case CL_ACK_BAD_LEN:          return "bad_len";
        case CL_ACK_OUT_OF_RANGE:     return "out_of_range";
        case CL_ACK_NVS_WRITE_FAILED: return "nvs_write_failed";
        default:                      return "unknown";
    }
}

static bool parse_target(const char *s, cl_pid_target_t *out) {
    if (strcmp(s, "roll") == 0)          { *out = CL_PID_ROLL; return true; }
    if (strcmp(s, "pitch") == 0)         { *out = CL_PID_PITCH; return true; }
    if (strcmp(s, "heading") == 0)       { *out = CL_PID_HEADING; return true; }
    if (strcmp(s, "airspeed") == 0)      { *out = CL_PID_AIRSPEED; return true; }
    return false;
}

static const char *target_name(cl_pid_target_t t) {
    switch (t) {
        case CL_PID_ROLL:     return "roll";
        case CL_PID_PITCH:    return "pitch";
        case CL_PID_HEADING:  return "heading";
        case CL_PID_AIRSPEED: return "airspeed";
        default:              return "?";
    }
}

// ---- JSON building ----

static cJSON *field_json(float value, store_field_state_t st) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "value", value);
    cJSON_AddStringToObject(o, "status", status_str(st.status));
    cJSON_AddStringToObject(o, "fail_reason", st.status == STORE_STATUS_FAILED ? ack_reason_str(st.fail_reason) : "none");
    return o;
}

static cJSON *pid_target_json(const store_pid_target_t *t) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddItemToObject(o, "k_p", field_json(t->gains.k_p, t->field[STORE_PID_IDX_KP]));
    cJSON_AddItemToObject(o, "k_i", field_json(t->gains.k_i, t->field[STORE_PID_IDX_KI]));
    cJSON_AddItemToObject(o, "k_d", field_json(t->gains.k_d, t->field[STORE_PID_IDX_KD]));
    cJSON_AddItemToObject(o, "i_limit", field_json(t->gains.i_limit, t->field[STORE_PID_IDX_ILIMIT]));
    return o;
}

static cJSON *mission_json(const store_mission_t *m) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "status", status_str(m->state.status));
    cJSON_AddStringToObject(o, "fail_reason", m->state.status == STORE_STATUS_FAILED ? ack_reason_str(m->state.fail_reason) : "none");
    cJSON_AddBoolToObject(o, "loop", m->loop);
    cJSON *points = cJSON_CreateArray();
    for (size_t i = 0; i < m->count; i++) {
        cJSON *p = cJSON_CreateObject();
        cJSON_AddNumberToObject(p, "lat", m->points[i].lat_deg);
        cJSON_AddNumberToObject(p, "lon", m->points[i].lon_deg);
        cJSON_AddItemToArray(points, p);
    }
    cJSON_AddItemToObject(o, "points", points);
    return o;
}

static cJSON *full_state_json(void) {
    cJSON *root = cJSON_CreateObject();

    cJSON *center = cJSON_CreateObject();
    cJSON_AddNumberToObject(center, "lat", BASESTATION_HOME_LAT_DEFAULT);
    cJSON_AddNumberToObject(center, "lon", BASESTATION_HOME_LON_DEFAULT);
    cJSON_AddItemToObject(root, "map_center", center);

    store_mission_t m;
    store_get_mission(&m);
    cJSON_AddItemToObject(root, "mission", mission_json(&m));

    cJSON *pid = cJSON_CreateObject();
    for (cl_pid_target_t t = CL_PID_ROLL; t <= CL_PID_AIRSPEED; t++) {
        store_pid_target_t pt;
        store_get_pid(t, &pt);
        cJSON_AddItemToObject(pid, target_name(t), pid_target_json(&pt));
    }
    cJSON_AddItemToObject(root, "pid", pid);

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
    esp_err_t ret = recv_json_body(req, &body);
    if (ret != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    cJSON *points_arr = cJSON_GetObjectItemCaseSensitive(body, "points");
    if (!cJSON_IsArray(points_arr)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"points\" must be an array");
    }
    int n = cJSON_GetArraySize(points_arr);
    if (n > CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET) {
        cJSON_Delete(body);
        return send_error(req, 400, "too many points (max " STR(CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET) ")");
    }

    cl_waypoint_t pts[CONFIG_LINK_MAX_WAYPOINTS_PER_PACKET];
    for (int i = 0; i < n; i++) {
        cJSON *p = cJSON_GetArrayItem(points_arr, i);
        cJSON *lat = cJSON_GetObjectItemCaseSensitive(p, "lat");
        cJSON *lon = cJSON_GetObjectItemCaseSensitive(p, "lon");
        if (!cJSON_IsNumber(lat) || !cJSON_IsNumber(lon)) {
            cJSON_Delete(body);
            return send_error(req, 400, "each point needs numeric \"lat\" and \"lon\"");
        }
        pts[i].lat_deg = lat->valuedouble;
        pts[i].lon_deg = lon->valuedouble;
    }

    // "loop" is optional -- an edit that doesn't mention it (e.g. just
    // moving a point) must leave the previously-stored value alone rather
    // than silently resetting it to false, so default to whatever's
    // currently stored and only override if the request actually sent it.
    store_mission_t current;
    store_get_mission(&current);
    bool loop = current.loop;
    cJSON *loop_j = cJSON_GetObjectItemCaseSensitive(body, "loop");
    if (loop_j != NULL) {
        if (!cJSON_IsBool(loop_j)) {
            cJSON_Delete(body);
            return send_error(req, 400, "\"loop\" must be a boolean");
        }
        loop = cJSON_IsTrue(loop_j);
    }
    cJSON_Delete(body);

    if (store_set_mission(pts, (size_t)n, loop) != ESP_OK) {
        return send_error(req, 400, "invalid point(s) -- lat must be -90..90, lon -180..180");
    }
    espnow_link_kick_async();

    store_mission_t m;
    store_get_mission(&m);
    return send_json(req, mission_json(&m), 200);
}

static esp_err_t api_pid_post(httpd_req_t *req) {
    cJSON *body;
    esp_err_t ret = recv_json_body(req, &body);
    if (ret != ESP_OK) {
        return send_error(req, 400, "invalid or missing JSON body");
    }

    cJSON *target_j = cJSON_GetObjectItemCaseSensitive(body, "target");
    cl_pid_target_t target;
    if (!cJSON_IsString(target_j) || !parse_target(target_j->valuestring, &target)) {
        cJSON_Delete(body);
        return send_error(req, 400, "\"target\" must be one of roll/pitch/heading/airspeed");
    }

    static const struct { const char *key; uint8_t bit; } fields[] = {
        { "k_p", CL_FIELD_KP }, { "k_i", CL_FIELD_KI }, { "k_d", CL_FIELD_KD }, { "i_limit", CL_FIELD_ILIMIT },
    };
    uint8_t fields_present = 0;
    cl_pid_gains_t gains = {0};
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        cJSON *v = cJSON_GetObjectItemCaseSensitive(body, fields[i].key);
        if (v == NULL) {
            continue; // not being edited -- leave absent from fields_present
        }
        if (!cJSON_IsNumber(v)) {
            cJSON_Delete(body);
            return send_error(req, 400, "PID fields must be numbers");
        }
        fields_present |= fields[i].bit;
        float fv = (float)v->valuedouble;
        switch (fields[i].bit) {
            case CL_FIELD_KP:     gains.k_p = fv; break;
            case CL_FIELD_KI:     gains.k_i = fv; break;
            case CL_FIELD_KD:     gains.k_d = fv; break;
            case CL_FIELD_ILIMIT: gains.i_limit = fv; break;
        }
    }
    cJSON_Delete(body);

    if (fields_present == 0) {
        return send_error(req, 400, "no PID fields present -- include at least one of k_p/k_i/k_d/i_limit");
    }
    if (store_set_pid_fields(target, fields_present, &gains) != ESP_OK) {
        return send_error(req, 400, "invalid gain value(s) (must be finite numbers)");
    }
    espnow_link_kick_async();

    store_pid_target_t pt;
    store_get_pid(target, &pt);
    return send_json(req, pid_target_json(&pt), 200);
}

static esp_err_t api_sync_post(httpd_req_t *req) {
    espnow_link_sync_now();
    return send_json(req, full_state_json(), 200);
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
    config.max_uri_handlers = 16;
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
        { .uri = "/api/sync",              .method = HTTP_POST, .handler = api_sync_post },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI(TAG, "HTTP server listening (connect to the \"%s\" AP, then browse to http://192.168.4.1/)",
             BASESTATION_AP_SSID);
}
