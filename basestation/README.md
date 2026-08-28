# Basestation

A standalone ESP-IDF v5.5 project (`esp32s3`) for the ground-based basestation
described in `../BASESTATION_SPEC.md`. This device never flies: it hosts its
own WiFi access point + a local web page (for a phone) to place mission
waypoints and tune PID gains, and relays those settings to the flight
controller (FC, the `../main/` project) over ESP-NOW using the shared wire
protocol in `../shared/config_link_proto.h`.

## One-time MAC bootstrap

ESP-NOW's encrypted peer entries must be unicast, so both devices' real MAC
addresses need to be hardcoded as each other's peer *before* the encrypted
link will do anything. This is a manual, one-time step:

1. Build and flash this project as-is (the placeholder `link_secrets.h`
   below is enough for it to boot and run). On boot it logs its own STA MAC
   address at `ESP_LOGI` level, e.g.:
   ```
   I (421) WIFI_AP: ==> basestation STA MAC (put this in the FC's config_link_secrets.h CONFIG_LINK_PEER_MAC): 24:6f:28:aa:bb:cc
   ```
   Watch this over `idf.py -p <port> monitor` (or `idf.py flash monitor`).
2. Do the same for the FC (`../main/`) -- it also logs its own STA MAC at
   boot (see `../main/config_link_secrets.h.example`).
3. Edit `basestation/main/link_secrets.h` (this project, gitignored): set
   `CONFIG_LINK_PEER_MAC` to the **FC's** MAC from step 2.
4. Edit `main/config_link_secrets.h` (the FC project, gitignored): set
   `CONFIG_LINK_PEER_MAC` to the **basestation's** MAC from step 1.
5. Pick real, matching 16-byte `CONFIG_LINK_PMK` / `CONFIG_LINK_LMK` values
   (any 16 bytes, just must be identical byte-for-byte on both sides) and a
   matching `CONFIG_LINK_WIFI_CHANNEL`, and put them in both files.
6. Reflash both boards with their real `link_secrets.h` /
   `config_link_secrets.h` in place.

Until this is done, the two devices' ESP-NOW peers won't match and every
send will silently go unanswered -- indistinguishable, from the basestation
UI's point of view, from "FC currently unreachable" (see below). That's
expected and not a bug in the app logic; it just means step 1-6 haven't
happened yet.

`basestation/main/link_secrets.h.example` is the committed template (copy it
to `link_secrets.h`, which is gitignored, matching the FC project's existing
`config_link_secrets.h` / `config_link_secrets.h.example` pattern). A real
`link_secrets.h` seeded with placeholder values is already checked into the
working tree of this repo's clone so the project builds out of the box; it
must still be edited with real values per the steps above before the link
will actually work.

## Building

This machine (per the parent project's notes) has two ESP-IDF v5.5 installs;
the one that works for this repo is
`/Users/travis/.espressif/v5.5.3/esp-idf`. One-time setup:

```sh
source /Users/travis/.espressif/v5.5.3/esp-idf/export.sh
cd /Users/travis/projects/autonomous-rc-plane/basestation
idf.py set-target esp32s3
```

After that, iterate directly with ninja:

```sh
cd /Users/travis/projects/autonomous-rc-plane/basestation/build
ninja
```

Re-run `idf.py set-target esp32s3` (sourced) only if you need a from-scratch
reconfigure (e.g. after editing `main/CMakeLists.txt`).

`sdkconfig.defaults` sets a 4MB flash size and points at a custom
single-factory-app partition table (`partitions.csv`, 1920K factory
partition) rather than either stock single-app Kconfig choice -- the regular
one (1M factory) leaves the embedded web UI assets very little headroom,
and the "large" one's bootloader was found to overflow its fixed IRAM
region on this toolchain (v5.5.3, gcc 14.2.0) for reasons unrelated to app
partition size. If your basestation board has a different (e.g. smaller)
flash chip, adjust `CONFIG_ESPTOOLPY_FLASHSIZE_*` and `partitions.csv`
accordingly.

Flashing/monitoring needs the sourced environment too:
`idf.py -p <port> flash monitor`.

## Architecture

- `main/wifi_ap.c` -- brings up WiFi in `WIFI_MODE_APSTA`: an AP for the
  phone (SSID/password in `main/include/basestation_config.h`) and a
  never-associating STA interface for ESP-NOW, both on
  `CONFIG_LINK_WIFI_CHANNEL` (AP+STA share one radio/channel on ESP32).
  Logs the STA MAC at boot for the bootstrap step above. Comes up once at
  boot and stays up for the device's whole life -- no RF-safety gating,
  unlike the FC (this device never flies).
- `main/espnow_link.c` -- the ESP-NOW link to the FC. A background task
  drains anything the store (below) has marked `pending`: for each item, it
  sends the packet and waits up to `ESPNOW_LINK_ACK_TIMEOUT_MS` for a
  matching-seq `cl_ack_t`. No ack in time is treated as "FC unreachable
  right now" (its receiver is gated by a safety state machine and is
  routinely closed -- see `../shared/config_link_proto.h` and
  `../main/config_link.c`), not an error: the item just stays `pending` for
  the next retry. Retries happen on its own cadence
  (`ESPNOW_LINK_RETRY_PERIOD_MS`), immediately after any local edit (a
  "kick"), and on-demand via `POST /api/sync`.
- `main/store.c` -- the basestation's own NVS-backed state: the full
  mission, all four PID targets' gains, and a `pending`/`confirmed`/`failed`
  status (+ failure reason where applicable) per PID field and for the
  mission as a whole. This is the durable source of truth for the user's
  *intended* config, independent of whether the FC has ever seen it.
- `main/http_server.c` -- `esp_http_server`-based local API + serves the
  embedded web UI (`main/web/`, embedded into the firmware image via
  `EMBED_TXTFILES`/`EMBED_FILES` in `main/CMakeLists.txt` -- no filesystem
  partition needed for content this small).
- `main/web/` -- the page itself: `index.html` + `app.js` + `style.css`,
  plus a locally-vendored copy of Leaflet (`web/lib/leaflet.{js,css}` +
  marker icons) so the app loads and the map is clickable with zero
  internet dependency. Real satellite/street tile imagery layers on top when
  the phone's *other* data path (cellular) has signal -- see "Map tiles"
  below.

## HTTP API

All endpoints are served from the basestation's own AP (default
`http://192.168.4.1/`, ESP-IDF's default AP gateway address). JSON both ways.

### `GET /api/state`

Full current state -- called on page load and by the periodic poll/refresh.

```jsonc
{
  "map_center": { "lat": 0.0, "lon": 0.0 },
  "mission": {
    "status": "unset|pending|confirmed|failed",
    "fail_reason": "none|bad_magic|bad_type|bad_len|out_of_range|nvs_write_failed",
    "loop": false,
    "points": [ { "lat": 47.6, "lon": -122.3 }, ... ]
  },
  "pid": {
    "roll": {
      "k_p":     { "value": 0.0, "status": "...", "fail_reason": "..." },
      "k_i":     { "value": 0.0, "status": "...", "fail_reason": "..." },
      "k_d":     { "value": 0.0, "status": "...", "fail_reason": "..." },
      "i_limit": { "value": 0.0, "status": "...", "fail_reason": "..." }
    },
    "pitch": { /* same shape */ },
    "heading": { /* same shape */ },
    "airspeed": { /* same shape */ }
  }
}
```

`fail_reason` is `"none"` whenever `status` isn't `"failed"`. `status` values
mirror `store_status_t` in `main/include/store.h`: `unset` means never
touched since the basestation's own storage was last blank (e.g. first
boot); `pending` means saved locally and waiting on the FC; `confirmed`
means the FC acked `CL_ACK_OK`; `failed` means the FC rejected it (see
`fail_reason` for the specific `cl_ack_status_t` reason -- notably
`nvs_write_failed` means the FC applied it live but couldn't persist it, so
it won't survive the FC's next reboot even though it "worked" right now).

`map_center` is a fixed fallback ("home field") coordinate,
`BASESTATION_HOME_LAT_DEFAULT`/`BASESTATION_HOME_LON_DEFAULT` in
`main/include/basestation_config.h` -- see "v1 limitations" below for why
this is always what's used in v1.

### `POST /api/mission`

Body: `{ "points": [ { "lat": <num>, "lon": <num> }, ... ], "loop"?: <bool> }`
(0 to 15 points). This always replaces the *entire* mission -- there's no
partial/patch op, matching `cl_set_mission_t`'s whole-mission-replace wire
semantics (`../shared/config_link_proto.h`). The web UI computes the full
intended list client-side (add/move/delete all just re-derive the array and
resubmit it); the firmware treats whatever's posted as the new authoritative
mission, persists it immediately, marks it `pending`, and kicks the
background sync task.

`loop` is optional: nonzero/`true` on the wire (`cl_set_mission_t.loop`)
tells the FC to wrap back to the first waypoint after reaching the last one,
instead of orbiting it forever. If the request omits `loop` entirely, the
previously-stored value is kept as-is (an edit that only moves/adds/removes
a point doesn't silently reset looping to off) -- to actually change it,
include it explicitly. The web UI's "Loop mission" checkbox always sends its
current state on every mission edit, so this mainly matters for direct API
callers.

Response: `200` with the same shape as `state.mission` above, or `400`
`{ "error": "..." }` if a point's lat/lon is out of range, there are more
than 15 points, or `loop` is present but not a boolean.

### `POST /api/pid`

Body: `{ "target": "roll"|"pitch"|"heading"|"airspeed", "k_p"?: <num>,
"k_i"?: <num>, "k_d"?: <num>, "i_limit"?: <num> }` -- include only the
field(s) actually being changed; **omitted keys are left completely
untouched**, both in basestation storage and (via `fields_present` in
`cl_set_pid_t`) on the wire to the FC. This is what lets the UI send just
the one field a user edited without needing to know or resend the other
three current values.

Response: `200` with the same shape as `state.pid.<target>` above, or `400`
if `target` is missing/invalid, no PID fields are present, or a present
field isn't a finite number.

### `POST /api/sync`

No body needed (any body is ignored). Synchronously runs one full delivery
pass over everything currently `pending` (mission, then each PID target),
each attempt bounded by `ESPNOW_LINK_ACK_TIMEOUT_MS`, then returns `200`
with the same shape as `GET /api/state` reflecting the outcome. This is the
explicit "sync now" action -- for forcing an immediate retry rather than
waiting for the background task's own cadence.

### Static assets

`GET /`, `/app.js`, `/style.css`, `/leaflet.js`, `/leaflet.css`,
`/images/marker-icon.png`, `/images/marker-icon-2x.png`,
`/images/marker-shadow.png` -- the web UI itself, all served from firmware
flash (embedded at build time), no dependency on any external network.

## Map tiles

The basestation's AP has no internet (it's not connected to anything
upstream) -- only the app/page itself is guaranteed available offline. The
map requests real tile imagery from a public OSM tile server
(`https://tile.openstreetmap.org/...`) over whatever *other* data path the
phone has (typically cellular, since most phones keep using cellular data
for internet-bound requests while separately associated to a WiFi AP with no
upstream route). If that's unavailable (no signal, airplane mode, etc.),
failed tile fetches render as a plain blank background (`errorTileUrl` in
`app.js` points at a 1x1 transparent PNG) instead of broken-image icons, and
the map stays **fully clickable** regardless -- Leaflet's click handler
reports `e.latlng` from its own projection math, independent of whether any
tile image ever rendered. No offline tile caching is implemented (out of
scope per the spec).

## v1 limitations

- **No FC state readback.** There's no wire-protocol packet for the FC to
  report its actual live config back to the basestation (see
  `../shared/config_link_proto.h`'s comments). The basestation only ever
  knows what it last locally set and, separately, what it last got acked --
  it can't detect config the FC has from some other source. In particular
  this means `map_center` can never actually be a "last known FC-reported
  GPS position" in v1 (the spec calls this out as the aspirational future
  behavior once a v2 readback packet exists) -- it's always the compiled-in
  `BASESTATION_HOME_LAT_DEFAULT`/`_LON_DEFAULT` fallback today. Set those to
  your actual flying field's coordinates in
  `main/include/basestation_config.h` before fielding this.
- **Single client, no multi-tab/multi-device merge.** The mission point
  list shown in the browser is treated as locally authoritative once
  loaded; background polling refreshes status badges and PID gain values
  but deliberately does not re-pull the mission point list over an
  already-loaded page (see the comment in `app.js`'s `applyState()`). Two
  phones editing at once, or a page left open across a basestation reboot
  that changed state some other way, can show a stale point list until
  reloaded. Fine for the intended one-person-in-a-field use case; a real
  multi-client sync model is future scope if it turns out to matter.
- **No offline tile cache.** See "Map tiles" above -- deliberate, per spec.
- **AP credentials are compiled-in placeholders.** `BASESTATION_AP_SSID` /
  `BASESTATION_AP_PASS` in `main/include/basestation_config.h` ship with
  placeholder values (an empty password means an open network); change them
  before relying on this for anything you care about being private.
- **No HTTPS.** The local API/UI is plain HTTP -- reasonable for a
  device-hosted AP with no upstream network, but worth knowing.
