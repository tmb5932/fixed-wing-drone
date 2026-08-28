// Basestation web UI. No build step, no framework -- plain ES6 talking to
// the local /api/* endpoints documented in basestation/README.md.
//
// Key design point (see BASESTATION_SPEC.md): the FC link can go quiet for
// long stretches and that's normal. Every edit here is already saved on the
// basestation (its own NVS) the moment the server responds 200, regardless
// of whether the FC ever acks it -- so this UI never blocks on, or makes
// the user redo, anything because of a slow/absent FC. Status dots just
// reflect best-known delivery state and get refreshed by polling.

const PID_TARGETS = ["roll", "pitch", "heading", "airspeed"];
const PID_FIELDS = ["k_p", "k_i", "k_d", "i_limit"];
const POLL_INTERVAL_MS = 5000;

// A 1x1 transparent PNG, used as Leaflet's errorTileUrl so a failed tile
// fetch (no internet on this map's data connection) renders as blank
// instead of a broken-image icon -- the map stays a plain background and
// stays fully clickable either way.
const BLANK_TILE =
  "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=";

let map;
let mapMarkers = []; // Leaflet marker objects, in mission order (parallel to `mission`)
let mission = [];    // [{lat, lon}, ...] -- the locally-authoritative point list
let loopMission = false; // wrap back to the first waypoint after the last, instead of orbiting it forever
let missionDirty = false; // true while a mission edit is in flight, to avoid a stale poll clobbering it

// dirty[target] is a Set of field names the user has touched since the last
// successful save for that target -- only these get sent on "Save changes",
// per the spec's "only send fields the user actually edited" requirement.
const dirty = Object.fromEntries(PID_TARGETS.map((t) => [t, new Set()]));

function el(html) {
  const t = document.createElement("template");
  t.innerHTML = html.trim();
  return t.content.firstElementChild;
}

function showToast(msg) {
  const toast = document.getElementById("toast");
  toast.textContent = msg;
  toast.classList.add("show");
  clearTimeout(showToast._t);
  showToast._t = setTimeout(() => toast.classList.remove("show"), 2600);
}

async function apiGet(path) {
  const res = await fetch(path);
  if (!res.ok) throw new Error((await res.json().catch(() => ({}))).error || res.statusText);
  return res.json();
}

async function apiPost(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || res.statusText);
  return data;
}

// ---------------- Map / mission ----------------

function initMap(centerLat, centerLon) {
  map = L.map("map", { zoomControl: true }).setView([centerLat, centerLon], 15);

  L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors',
    errorTileUrl: BLANK_TILE,
  }).addTo(map);

  L.Icon.Default.mergeOptions({
    iconUrl: "/images/marker-icon.png",
    iconRetinaUrl: "/images/marker-icon-2x.png",
    shadowUrl: "/images/marker-shadow.png",
  });

  // Clicking always yields valid lat/lon from Leaflet's own projection math,
  // independent of whether any tile imagery actually rendered.
  map.on("click", (e) => {
    mission.push({ lat: e.latlng.lat, lon: e.latlng.lng });
    renderMission();
    submitMission();
  });
}

function renderMission() {
  mapMarkers.forEach((m) => map.removeLayer(m));
  mapMarkers = mission.map((pt, i) => {
    const marker = L.marker([pt.lat, pt.lon], { draggable: true }).addTo(map);
    marker.bindTooltip(String(i + 1), { permanent: true, direction: "top", offset: [0, -28] });
    marker.on("dragend", () => {
      const ll = marker.getLatLng();
      mission[i] = { lat: ll.lat, lon: ll.lng };
      submitMission();
    });
    return marker;
  });

  const list = document.getElementById("waypoint-list");
  list.innerHTML = "";
  mission.forEach((pt, i) => {
    const row = el(`
      <li class="waypoint-row">
        <span class="idx">${i + 1}</span>
        <span class="coords">${pt.lat.toFixed(6)}, ${pt.lon.toFixed(6)}</span>
        <button class="icon-btn danger" type="button" aria-label="Remove waypoint ${i + 1}">&times;</button>
      </li>
    `);
    row.querySelector("button").addEventListener("click", () => {
      mission.splice(i, 1);
      renderMission();
      submitMission();
    });
    list.appendChild(row);
  });
}

function setMissionBadge(status, failReason) {
  const badge = document.getElementById("mission-status");
  badge.className = "badge " + status;
  badge.textContent =
    status === "failed" ? `failed: ${failReason}` :
    status === "unset" ? "no mission sent yet" :
    status;
}

async function submitMission() {
  missionDirty = true;
  try {
    const result = await apiPost("/api/mission", {
      points: mission.map((p) => ({ lat: p.lat, lon: p.lon })),
      loop: loopMission,
    });
    setMissionBadge(result.status, result.fail_reason);
  } catch (err) {
    showToast("Couldn't save mission: " + err.message);
  } finally {
    missionDirty = false;
  }
}

function onLoopToggleChanged() {
  loopMission = document.getElementById("loop-toggle").checked;
  submitMission();
}

async function clearMission() {
  mission = [];
  renderMission();
  await submitMission();
}

// ---------------- PID cards ----------------

function pidCardFor(target) {
  return document.querySelector(`.pid-card[data-target="${target}"]`);
}

function buildPidCards() {
  const container = document.getElementById("pid-targets");
  const template = document.getElementById("pid-card-template");
  PID_TARGETS.forEach((target) => {
    const card = template.content.firstElementChild.cloneNode(true);
    card.dataset.target = target;
    card.querySelector(".pid-title").textContent = target;

    card.querySelectorAll("input[data-field]").forEach((input) => {
      input.addEventListener("input", () => {
        dirty[target].add(input.dataset.field);
        input.classList.add("dirty");
        card.querySelector(".save-btn").disabled = false;
      });
    });

    card.querySelector(".save-btn").addEventListener("click", () => savePid(target));
    container.appendChild(card);
  });
}

// Renders server state into a PID card, but never overwrites a field the
// user currently has unsaved edits in (dirty) -- a background poll or a
// sync-now refresh must not yank the input out from under someone typing.
function renderPidTarget(target, targetState) {
  const card = pidCardFor(target);
  let anyFailed = false;
  let failMsgs = [];

  PID_FIELDS.forEach((field) => {
    const f = targetState[field];
    const input = card.querySelector(`input[data-field="${field}"]`);
    const dot = card.querySelector(`.status-dot[data-field="${field}"]`);

    if (!dirty[target].has(field) && document.activeElement !== input) {
      input.value = f.value;
    }
    dot.className = "status-dot " + f.status;
    dot.title = f.status;
    if (f.status === "failed") {
      anyFailed = true;
      failMsgs.push(`${field}: ${f.fail_reason}`);
    }
  });

  card.querySelector(".fail-reason").textContent = anyFailed ? failMsgs.join(", ") : "";
}

async function savePid(target) {
  const fields = dirty[target];
  if (fields.size === 0) return;
  const card = pidCardFor(target);
  const payload = { target };
  fields.forEach((field) => {
    const input = card.querySelector(`input[data-field="${field}"]`);
    const v = parseFloat(input.value);
    if (Number.isNaN(v)) return;
    payload[field] = v;
  });

  try {
    const result = await apiPost("/api/pid", payload);
    dirty[target].clear();
    card.querySelectorAll("input[data-field]").forEach((i) => i.classList.remove("dirty"));
    card.querySelector(".save-btn").disabled = true;
    renderPidTarget(target, result);
    showToast(`${target} PID saved locally, syncing to FC`);
  } catch (err) {
    showToast(`Couldn't save ${target}: ` + err.message);
  }
}

// ---------------- Full-state load / refresh ----------------

function applyState(state, { isInitialLoad } = {}) {
  if (isInitialLoad) {
    initMap(state.map_center.lat, state.map_center.lon);
    mission = state.mission.points.map((p) => ({ lat: p.lat, lon: p.lon }));
    renderMission();
  }
  // On non-initial refreshes the point list itself is left alone -- it's
  // locally authoritative and only ever changes via direct user action (see
  // missionDirty's usage in submitMission()) -- only the status badge below
  // and the PID fields/dots are safe to refresh passively from a poll. The
  // loop checkbox rides along with the point list for the same reason: skip
  // it while a mission submit is in flight so a poll landing mid-toggle
  // doesn't flicker it back.
  if (!missionDirty) {
    loopMission = state.mission.loop;
    document.getElementById("loop-toggle").checked = loopMission;
  }
  setMissionBadge(state.mission.status, state.mission.fail_reason);

  PID_TARGETS.forEach((target) => renderPidTarget(target, state.pid[target]));
}

async function loadInitialState() {
  try {
    const state = await apiGet("/api/state");
    applyState(state, { isInitialLoad: true });
  } catch (err) {
    showToast("Couldn't reach basestation API: " + err.message);
    // Fall back to a blank, still-clickable map so the page isn't dead.
    initMap(0, 0);
  }
}

async function pollState() {
  try {
    const state = await apiGet("/api/state");
    applyState(state, { isInitialLoad: false });
  } catch (err) {
    // Transient network hiccup on the phone<->basestation link -- ignore
    // silently, the next poll will retry. This is a much shorter/friendlier
    // link than the basestation<->FC one, but treat it the same way: don't
    // nag the user over something that'll very likely resolve itself.
  }
}

async function syncNow() {
  const btn = document.getElementById("sync-btn");
  btn.disabled = true;
  const original = btn.textContent;
  btn.textContent = "Syncing...";
  try {
    const res = await fetch("/api/sync", { method: "POST" });
    const state = await res.json();
    applyState(state, { isInitialLoad: false });
    showToast("Sync attempted");
  } catch (err) {
    showToast("Sync failed: " + err.message);
  } finally {
    btn.disabled = false;
    btn.textContent = original;
  }
}

document.addEventListener("DOMContentLoaded", () => {
  buildPidCards();
  document.getElementById("clear-mission-btn").addEventListener("click", clearMission);
  document.getElementById("loop-toggle").addEventListener("change", onLoopToggleChanged);
  document.getElementById("sync-btn").addEventListener("click", syncNow);
  loadInitialState();
  setInterval(pollState, POLL_INTERVAL_MS);
});
