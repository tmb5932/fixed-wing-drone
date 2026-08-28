#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

// Local HTTP/JSON API + the embedded web UI, for the phone browser on the
// basestation's own AP. See basestation/README.md for the full endpoint/
// JSON documentation. No flight-safety gating here at all (unlike
// espnow_link.c) -- bring this up once and leave it running.
//
// Must be called after store_init() (handlers read/write store state
// immediately) and after espnow_link_init() (handlers kick/wait on it).
void http_server_start(void);

#endif // HTTP_SERVER_H
