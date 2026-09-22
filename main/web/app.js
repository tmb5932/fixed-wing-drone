// FC setup-mode web UI. No build step, no framework -- plain ES6 talking to
// the local /api/* endpoints (see main/http_server.c). Unlike the old
// basestation (a relay to a separate FC that could go quiet for a long
// time), every edit here applies directly and synchronously on this same
// device -- a 200 response means it's already live and persisted, so there's
// no pending/confirmed state to track or poll for. The only thing worth
// polling is /api/outputs, for the live RC pulse-width readout.

const PID_TARGETS = ["roll", "pitch", "heading", "airspeed"];
const PID_FIELDS = ["k_p", "k_i", "k_d", "i_limit"];
const OUTPUT_POLL_INTERVAL_MS = 250;

// A 1x1 transparent PNG, used as Leaflet's errorTileUrl so a failed tile
// fetch (no internet on this AP's connection) renders as blank instead of a
// broken-image icon -- the map stays a plain background and stays fully
// clickable either way.
const BLANK_TILE =
  "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=";

let map;
let mapMarkers = []; // Leaflet marker objects, in mission order (parallel to `mission`)
let mission = [];    // [{lat, lon}, ...] -- the locally-authoritative point list
let loopMission = false; // wrap back to the first waypoint after the last, instead of orbiting it forever
let missionDirty = false; // true while a mission edit is in flight

// dirty[target] is a Set of field names the user has touched since the last
// successful save for that target -- only these get sent on "Save changes".
const dirty = Object.fromEntries(PID_TARGETS.map((t) => [t, new Set()]));
const outputDirty = []; // outputDirty[ch] = Set of field names touched since last save

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

async function submitMission() {
  missionDirty = true;
  try {
    await apiPost("/api/mission", {
      points: mission.map((p) => ({ lat: p.lat, lon: p.lon })),
      loop: loopMission,
    });
  } catch (err) {
    showToast("Couldn't save mission: " + err.message);
  } finally {
    missionDirty = false;
  }
}

function onLoopToggleChanged() {
  loopMission = document.getElementById("loop-toggle").checked;
  if (mission.length > 0) submitMission();
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

function renderPidTarget(target, gains) {
  const card = pidCardFor(target);
  PID_FIELDS.forEach((field) => {
    const input = card.querySelector(`input[data-field="${field}"]`);
    if (!dirty[target].has(field) && document.activeElement !== input) {
      input.value = gains[field];
    }
  });
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
    card.querySelector(".fail-reason").textContent = "";
    renderPidTarget(target, result);
    showToast(`${target} PID saved`);
  } catch (err) {
    card.querySelector(".fail-reason").textContent = err.message;
    showToast(`Couldn't save ${target}: ` + err.message);
  }
}

// ---------------- Output channels ----------------

function outputRowFor(ch) {
  return document.querySelector(`.output-row[data-channel="${ch}"]`);
}

function buildOutputRows(outputs) {
  const container = document.getElementById("output-channels");
  const template = document.getElementById("output-row-template");
  outputs.forEach((o) => {
    const row = template.content.firstElementChild.cloneNode(true);
    row.dataset.channel = o.channel;
    row.querySelector(".output-title").textContent = `${o.channel}. ${o.name} (${o.type})`;
    outputDirty[o.channel] = new Set();

    row.querySelectorAll("input[data-field]").forEach((input) => {
      const evt = input.type === "checkbox" ? "change" : "input";
      input.addEventListener(evt, () => {
        outputDirty[o.channel].add(input.dataset.field);
        if (input.type !== "checkbox") input.classList.add("dirty");
        row.querySelector(".save-btn").disabled = false;
      });
    });

    row.querySelector(".set-min-btn").addEventListener("click", () => {
      const liveUs = outputRowFor(o.channel).dataset.liveUs;
      row.querySelector('input[data-field="min_us"]').value = liveUs;
      outputDirty[o.channel].add("min_us");
      row.querySelector('input[data-field="min_us"]').classList.add("dirty");
      row.querySelector(".save-btn").disabled = false;
    });
    row.querySelector(".set-max-btn").addEventListener("click", () => {
      const liveUs = outputRowFor(o.channel).dataset.liveUs;
      row.querySelector('input[data-field="max_us"]').value = liveUs;
      outputDirty[o.channel].add("max_us");
      row.querySelector('input[data-field="max_us"]').classList.add("dirty");
      row.querySelector(".save-btn").disabled = false;
    });

    row.querySelector(".save-btn").addEventListener("click", () => saveOutput(o.channel));
    container.appendChild(row);
    renderOutputChannel(o);
  });
}

function renderOutputChannel(o) {
  const row = outputRowFor(o.channel);
  row.dataset.liveUs = o.live_us;
  row.querySelector(".live-us").textContent = `${o.live_us} µs`;

  const dirtyFields = outputDirty[o.channel];
  const minInput = row.querySelector('input[data-field="min_us"]');
  const maxInput = row.querySelector('input[data-field="max_us"]');
  const revInput = row.querySelector('input[data-field="reversed"]');
  minInput.min = o.abs_min_us; minInput.max = o.abs_max_us;
  maxInput.min = o.abs_min_us; maxInput.max = o.abs_max_us;
  if (!dirtyFields.has("min_us") && document.activeElement !== minInput) minInput.value = o.min_us;
  if (!dirtyFields.has("max_us") && document.activeElement !== maxInput) maxInput.value = o.max_us;
  if (!dirtyFields.has("reversed")) revInput.checked = o.reversed;
}

async function saveOutput(ch) {
  const fields = outputDirty[ch];
  if (fields.size === 0) return;
  const row = outputRowFor(ch);
  const payload = { channel: ch };
  if (fields.has("min_us")) payload.min_us = parseInt(row.querySelector('input[data-field="min_us"]').value, 10);
  if (fields.has("max_us")) payload.max_us = parseInt(row.querySelector('input[data-field="max_us"]').value, 10);
  if (fields.has("reversed")) payload.reversed = row.querySelector('input[data-field="reversed"]').checked;

  try {
    const result = await apiPost("/api/outputs", payload);
    fields.clear();
    row.querySelectorAll("input[data-field]").forEach((i) => i.classList.remove("dirty"));
    row.querySelector(".save-btn").disabled = true;
    row.querySelector(".fail-reason").textContent = "";
    renderOutputChannel(result);
    showToast(`Channel ${ch} saved`);
  } catch (err) {
    row.querySelector(".fail-reason").textContent = err.message;
    showToast(`Couldn't save channel ${ch}: ` + err.message);
  }
}

async function pollOutputs() {
  try {
    const outputs = await apiGet("/api/outputs");
    outputs.forEach(renderOutputChannel);
  } catch (err) {
    // Transient hiccup on the local AP link -- ignore, next poll retries.
  }
}

// ---------------- Motors ----------------

function initMotorCfg(cfg) {
  const leftCheckbox = document.getElementById("esc1-is-left");
  const sideRow = document.getElementById("motor-side-row");

  function updateSideRowVisibility(count) {
    sideRow.style.display = count === 2 ? "" : "none";
  }

  document.querySelectorAll('input[name="motor-count"]').forEach((radio) => {
    radio.checked = Number(radio.value) === cfg.motor_count;
    radio.addEventListener("change", async () => {
      const count = Number(radio.value);
      updateSideRowVisibility(count);
      try {
        await apiPost("/api/motor", { motor_count: count });
        showToast(`Motor count set to ${count}`);
      } catch (err) {
        showToast("Couldn't save motor count: " + err.message);
      }
    });
  });
  updateSideRowVisibility(cfg.motor_count);

  leftCheckbox.checked = cfg.esc1_is_left;
  leftCheckbox.addEventListener("change", async () => {
    try {
      await apiPost("/api/motor", { esc1_is_left: leftCheckbox.checked });
      showToast("Motor side assignment saved");
    } catch (err) {
      showToast("Couldn't save motor side assignment: " + err.message);
    }
  });
}

// ---------------- Airframe ----------------

function initAirframe(mode) {
  document.querySelectorAll('input[name="airframe"]').forEach((radio) => {
    radio.checked = radio.value === mode;
    radio.addEventListener("change", async () => {
      try {
        await apiPost("/api/airframe", { mode: radio.value });
        showToast(`Airframe set to ${radio.value}`);
      } catch (err) {
        showToast("Couldn't save airframe mode: " + err.message);
      }
    });
  });
}

// ---------------- Airspeed ----------------

function initAirspeed(cfg) {
  const targetInput = document.getElementById("airspeed-target");
  const fallbackInput = document.getElementById("airspeed-fallback");
  const saveBtn = document.getElementById("airspeed-save-btn");
  targetInput.value = cfg.target_cms;
  fallbackInput.value = (cfg.fallback_pct * 100).toFixed(0);

  [targetInput, fallbackInput].forEach((input) => {
    input.addEventListener("input", () => { saveBtn.disabled = false; });
  });

  saveBtn.addEventListener("click", async () => {
    try {
      const result = await apiPost("/api/airspeed", {
        target_cms: parseFloat(targetInput.value),
        fallback_pct: parseFloat(fallbackInput.value) / 100,
      });
      targetInput.value = result.target_cms;
      fallbackInput.value = (result.fallback_pct * 100).toFixed(0);
      saveBtn.disabled = true;
      showToast("Airspeed cfg saved");
    } catch (err) {
      showToast("Couldn't save airspeed cfg: " + err.message);
    }
  });
}

// ---------------- Initial load ----------------

async function loadInitialState() {
  try {
    const state = await apiGet("/api/state");

    const center = state.mission.points[0] || { lat: 0, lon: 0 };
    initMap(center.lat, center.lon);
    mission = state.mission.points.map((p) => ({ lat: p.lat, lon: p.lon }));
    loopMission = state.mission.loop;
    document.getElementById("loop-toggle").checked = loopMission;
    renderMission();

    PID_TARGETS.forEach((target) => renderPidTarget(target, state.pid[target]));
    buildOutputRows(state.outputs);
    initMotorCfg(state.motor_cfg);
    initAirframe(state.airframe);
    initAirspeed(state.airspeed_cfg);
  } catch (err) {
    showToast("Couldn't reach setup API: " + err.message);
    initMap(0, 0);
  }
}

document.addEventListener("DOMContentLoaded", () => {
  buildPidCards();
  document.getElementById("clear-mission-btn").addEventListener("click", clearMission);
  document.getElementById("loop-toggle").addEventListener("change", onLoopToggleChanged);
  loadInitialState();
  setInterval(pollOutputs, OUTPUT_POLL_INTERVAL_MS);
});
