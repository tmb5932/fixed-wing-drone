idf_component_register(SRCS "app_main.c" "wifi_ap.c" "espnow_link.c" "http_server.c" "store.c"
                    INCLUDE_DIRS "include" "../../shared"
                    REQUIRES nvs_flash esp_wifi driver esp_timer esp_http_server json
                    EMBED_TXTFILES "web/index.html" "web/app.js" "web/style.css"
                                   "web/lib/leaflet.js" "web/lib/leaflet.css"
                    EMBED_FILES "web/lib/images/marker-icon.png" "web/lib/images/marker-icon-2x.png"
                                "web/lib/images/marker-shadow.png")
