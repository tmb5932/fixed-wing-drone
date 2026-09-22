#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

// Starts the local setup-mode HTTP/JSON API + embedded web UI. Requires
// wifi_ap_init() to have already brought up the AP's netif. Only ever called
// from setup_mode_run().
void http_server_start(void);

#endif // HTTP_SERVER_H
