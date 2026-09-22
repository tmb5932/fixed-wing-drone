idf_component_register(SRCS "http_server.c" "setup_mode.c" "wifi_ap.c" "nav.c" "auxiliary.c" "main.c" "pwm_output.c" "rc_capture.c" "imu.c" "madgwick_wrapper.cpp" "MadgwickAHRS.cpp" "gps.c" "gps_math.c" "airspeed.c" "pid.c" "config_store.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash esp_wifi driver esp_timer esp_http_server json
                    EMBED_TXTFILES "web/index.html" "web/app.js" "web/style.css"
                                   "web/lib/leaflet.js" "web/lib/leaflet.css"
                    EMBED_FILES "web/lib/images/marker-icon.png" "web/lib/images/marker-icon-2x.png"
                                "web/lib/images/marker-shadow.png")
