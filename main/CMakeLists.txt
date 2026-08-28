idf_component_register(SRCS "nav.c" "auxiliary.c" "main.c" "pwm_output.c" "rc_capture.c" "imu.c" "madgwick_wrapper.cpp" "MadgwickAHRS.cpp" "gps.c" "airspeed.c" "pid.c" "config_link.c" "config_store.c"
                    INCLUDE_DIRS "include" "../shared"
                    REQUIRES nvs_flash esp_wifi driver esp_timer)
