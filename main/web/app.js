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
const GPS_POLL_INTERVAL_MS = 1000;

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

  // L.Icon.Default always prepends its own auto-detected imagePath in front
  // of iconUrl/iconRetinaUrl/shadowUrl, even when those are already absolute
  // -- clearing it to "" is the standard workaround, otherwise these 404 as
  // "/images//images/...".
  L.Icon.Default.imagePath = "";
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
  const pinLabel = o.output_pin == null ? "" : ` → servo_out_${o.output_pin}`;
  row.querySelector(".output-title").textContent = `${o.name} (${o.type})${pinLabel}`;
  row.querySelector(".live-us").textContent = `in ${o.live_us} µs → out ${o.out_us} µs`;

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

// ---------------- RC channel mapping ----------------

function buildRcMapRows(entries) {
  const container = document.getElementById("rc-map-rows");
  const template = document.getElementById("rc-map-row-template");
  entries.forEach((entry) => {
    const row = template.content.firstElementChild.cloneNode(true);
    row.dataset.logical = entry.logical;
    row.querySelector(".rc-map-name").textContent = entry.name;

    const select = row.querySelector(".rc-map-select");
    entries.forEach((_, i) => {
      const opt = document.createElement("option");
      opt.value = i + 1;
      opt.textContent = `Pin ${i + 1}`;
      select.appendChild(opt);
    });
    select.value = entry.physical_pin;
    select.addEventListener("change", async () => {
      try {
        await apiPost("/api/rc_map", { logical: entry.logical, physical_pin: Number(select.value) });
        showToast(`${entry.name} set to pin ${select.value}`);
      } catch (err) {
        showToast(`Couldn't save ${entry.name} mapping: ` + err.message);
      }
    });

    container.appendChild(row);
  });
}

function renderRcMap(data) {
  const grid = document.getElementById("rc-map-live-grid");
  grid.innerHTML = "";
  data.physical_live_us.forEach((us, i) => {
    const span = document.createElement("span");
    span.className = "rc-map-live-pin";
    span.textContent = `Pin ${i + 1}: ${us} µs`;
    grid.appendChild(span);
  });

  // Keeps each row's dropdown in sync with the live mapping, but never
  // clobbers one the user is actively changing.
  data.map.forEach((entry) => {
    const row = document.querySelector(`#rc-map-rows .rc-map-row[data-logical="${entry.logical}"]`);
    if (!row) return;
    const select = row.querySelector(".rc-map-select");
    if (document.activeElement !== select) select.value = entry.physical_pin;
  });
}

async function pollRcMap() {
  try {
    renderRcMap(await apiGet("/api/rc_map"));
  } catch (err) {
    // Transient hiccup on the local AP link -- ignore, next poll retries.
  }
}

// ---------------- Servo output mapping ----------------

function buildServoMapRows(entries) {
  const container = document.getElementById("servo-map-rows");
  const template = document.getElementById("rc-map-row-template");
  entries.forEach((entry) => {
    const row = template.content.firstElementChild.cloneNode(true);
    row.classList.add("servo-map-row");
    row.dataset.logical = entry.logical;
    row.querySelector(".rc-map-name").textContent = entry.name;

    const select = row.querySelector(".rc-map-select");
    entries.forEach((_, i) => {
      const opt = document.createElement("option");
      opt.value = i + 1;
      opt.textContent = `servo_out_${i + 1}`;
      select.appendChild(opt);
    });
    select.value = entry.output_pin;
    select.addEventListener("change", async () => {
      try {
        // A swap moves another function too, so re-render every row from
        // the response rather than just trusting this one select.
        renderServoMap(await apiPost("/api/servo_map", { logical: entry.logical, output_pin: Number(select.value) }));
        pollOutputs();
        showToast(`${entry.name} now drives servo_out_${select.value}`);
      } catch (err) {
        showToast(`Couldn't save ${entry.name} output pin: ` + err.message);
        renderServoMap(await apiGet("/api/servo_map"));
      }
    });

    container.appendChild(row);
  });
}

function renderServoMap(data) {
  data.map.forEach((entry) => {
    const row = document.querySelector(`.servo-map-row[data-logical="${entry.logical}"]`);
    if (row) row.querySelector(".rc-map-select").value = entry.output_pin;
  });
}

// ---------------- Autonomous trim ----------------

let trimNeutralUs = 1500;

function buildTrimRows(data) {
  trimNeutralUs = data.neutral_us;
  document.getElementById("trim-max-offset").textContent = data.max_offset_us;
  document.getElementById("trim-neutral").textContent = data.neutral_us;

  const container = document.getElementById("trim-rows");
  data.channels.forEach((c) => {
    const row = document.createElement("div");
    row.className = "rc-map-row trim-row";
    row.dataset.logical = c.logical;
    row.innerHTML = `
      <span class="rc-map-name"></span>
      <span class="live-us trim-live"></span>
      <input class="trim-input" type="number" step="1" inputmode="numeric">
      <button class="secondary-btn trim-save-btn" type="button" disabled>Save</button>`;
    row.querySelector(".rc-map-name").textContent = c.name;

    const input = row.querySelector(".trim-input");
    const saveBtn = row.querySelector(".trim-save-btn");
    input.min = data.neutral_us - data.max_offset_us;
    input.max = data.neutral_us + data.max_offset_us;
    input.addEventListener("input", () => {
      input.classList.add("dirty");
      saveBtn.disabled = false;
    });
    saveBtn.addEventListener("click", () => saveTrim([{ logical: c.logical, center_us: parseInt(input.value, 10) }]));

    container.appendChild(row);
  });
  renderTrim(data, true);
}

function renderTrim(data, force = false) {
  data.channels.forEach((c) => {
    const row = document.querySelector(`.trim-row[data-logical="${c.logical}"]`);
    if (!row) return;
    const offset = c.center_us - trimNeutralUs;
    row.querySelector(".trim-live").textContent = `stick ${c.live_us} µs · trim ${offset >= 0 ? "+" : ""}${offset}`;
    const input = row.querySelector(".trim-input");
    if (force || (!input.classList.contains("dirty") && document.activeElement !== input)) {
      input.value = c.center_us;
      input.classList.remove("dirty");
      row.querySelector(".trim-save-btn").disabled = true;
    }
  });
}

async function saveTrim(entries) {
  const failEl = document.getElementById("trim-fail-reason");
  try {
    let result;
    for (const e of entries) result = await apiPost("/api/trim", e);
    failEl.textContent = "";
    renderTrim(result, true);
    showToast("Trim saved");
  } catch (err) {
    failEl.textContent = err.message;
    showToast("Couldn't save trim: " + err.message);
  }
}

function initTrimButtons() {
  const captureBtn = document.getElementById("trim-capture-btn");
  captureBtn.addEventListener("click", async () => {
    const failEl = document.getElementById("trim-fail-reason");
    captureBtn.disabled = true;
    try {
      renderTrim(await apiPost("/api/trim", { capture: true }), true);
      failEl.textContent = "";
      showToast("Trim captured from sticks");
    } catch (err) {
      failEl.textContent = err.message;
      showToast("Trim capture failed: " + err.message);
    } finally {
      captureBtn.disabled = false;
    }
  });
  document.getElementById("trim-reset-btn").addEventListener("click", () => {
    const rows = document.querySelectorAll(".trim-row");
    saveTrim([...rows].map((r) => ({ logical: Number(r.dataset.logical), center_us: trimNeutralUs })));
  });
}

async function pollTrim() {
  try {
    renderTrim(await apiGet("/api/trim"));
  } catch (err) {
    // Transient hiccup on the local AP link -- ignore, next poll retries.
  }
}

// ---------------- Control direction test ----------------

function fmtSigned(n, digits = 0) {
  const v = Number(n).toFixed(digits);
  return n >= 0 ? `+${v}` : v;
}

function describeCmd(us, pos, neg) {
  if (Math.abs(us) < 1) return "neutral";
  return `${fmtSigned(us)} µs (${us > 0 ? pos : neg})`;
}

function renderControlTest(s) {
  const el = document.getElementById("ctrl-test-status");
  if (!s.active) {
    el.textContent = "Idle";
    return;
  }
  const left = `${(s.remaining_ms / 1000).toFixed(1)}s left`;
  const roll = `roll ${describeCmd(s.roll_cmd_us, "right", "left")}`;
  const pitch = `pitch ${describeCmd(s.pitch_cmd_us, "up", "down")}`;
  if (s.kind === "level") {
    el.textContent = `Level test, ${left} — IMU roll ${fmtSigned(s.roll_deg, 1)}°, pitch ${fmtSigned(s.pitch_deg, 1)}° — autopilot wants ${roll}, ${pitch}`;
  } else {
    el.textContent = `Direction test, ${left} — commanding ${roll}, ${pitch}`;
  }
}

function initControlTest() {
  document.querySelectorAll(".ctrl-test-btn").forEach((btn) => {
    btn.addEventListener("click", async () => {
      try {
        renderControlTest(await apiPost("/api/control_test", { test: btn.dataset.test }));
        showToast(`Control test: ${btn.textContent}`);
      } catch (err) {
        showToast("Couldn't start control test: " + err.message);
      }
    });
  });
  document.getElementById("ctrl-test-stop").addEventListener("click", async () => {
    try {
      renderControlTest(await apiPost("/api/control_test", { test: "stop" }));
    } catch (err) {
      showToast("Couldn't stop control test: " + err.message);
    }
  });
}

async function pollControlTest() {
  try {
    renderControlTest(await apiGet("/api/control_test"));
  } catch (err) {
    // Transient hiccup on the local AP link -- ignore, next poll retries.
  }
}

// ---------------- Flight log ----------------

function describeFlightLogEntry(e) {
  switch (e.type) {
    case "boot": return `Boot #${e.boot} — reset reason: ${e.reset_reason}`;
    case "rc_dropout": return `Radio dropout ${e.value} ms (recovered)`;
    case "rc_lost": return "Radio LOST (silent > 1 s)";
    case "rc_regained": return `Radio regained after ${(e.value / 1000).toFixed(1)} s`;
    case "failsafe_rth": return "Failsafe: returning home";
    case "failsafe_descend": return "Failsafe: motors off, spiral descent";
    case "failsafe_cleared": return "Failsafe cleared";
    case "autonomous_lockout": return e.value === 1 ? "Autonomous locked out: radio regained during descent" : "Autonomous locked out: no GPS fix";
    case "imu_fault": return "IMU FAULT — autonomous disabled until reboot";
    case "no_radio_safe": return "No radio, autonomous unavailable — trim + motors off";
    default: return `${e.type} (${e.value})`;
  }
}

function isBadFlightLogEntry(e) {
  if (e.type === "boot") return /BROWNOUT|CRASH/.test(e.reset_reason);
  return ["rc_lost", "failsafe_descend", "imu_fault", "no_radio_safe"].includes(e.type);
}

function renderFlightLog(data) {
  const list = document.getElementById("flight-log");
  list.innerHTML = "";
  if (data.entries.length === 0) {
    const li = document.createElement("li");
    li.textContent = "No events.";
    list.appendChild(li);
    return;
  }
  data.entries.forEach((e) => {
    const li = document.createElement("li");
    const t = e.type === "boot" ? "" : `[boot #${e.boot} +${(e.t_ms / 1000).toFixed(1)}s] `;
    li.textContent = t + describeFlightLogEntry(e) + (e.boot === data.current_boot && e.type === "boot" ? " (this setup session)" : "");
    if (e.type === "boot") li.classList.add("flog-boot");
    if (isBadFlightLogEntry(e)) li.classList.add("flog-bad");
    list.appendChild(li);
  });
}

async function loadFlightLog() {
  try {
    renderFlightLog(await apiGet("/api/flight_log"));
  } catch (err) {
    showToast("Couldn't load flight log: " + err.message);
  }
}

function initFlightLog() {
  document.getElementById("flight-log-refresh").addEventListener("click", loadFlightLog);
  document.getElementById("flight-log-clear").addEventListener("click", async () => {
    try {
      renderFlightLog(await apiPost("/api/flight_log", { clear: true }));
      showToast("Flight log cleared");
    } catch (err) {
      showToast("Couldn't clear flight log: " + err.message);
    }
  });
  loadFlightLog();
}

// ---------------- Mode status (manual vs. autonomous) ----------------

function renderMode(data) {
  const badge = document.getElementById("mode-badge");
  if (!data.radio_connected) {
    badge.textContent = "NO RADIO";
    badge.className = "mode-badge mode-none";
  } else if (data.signal_stale) {
    badge.textContent = "AUTONOMOUS (failsafe: signal lost)";
    badge.className = "mode-badge mode-auto";
  } else if (data.autonomous) {
    badge.textContent = "AUTONOMOUS";
    badge.className = "mode-badge mode-auto";
  } else {
    badge.textContent = "MANUAL";
    badge.className = "mode-badge mode-manual";
  }
}

async function pollMode() {
  try {
    renderMode(await apiGet("/api/mode"));
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

  document.querySelectorAll(".esc-test-btn").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const esc = Number(btn.dataset.esc);
      try {
        const result = await apiPost("/api/motor_test", { esc });
        showToast(`Spinning ESC${esc}...`);
        // Mirrors the firmware's own timeout -- purely to stop double-taps
        // from re-triggering mid-test; the board ends the test on its own.
        document.querySelectorAll(".esc-test-btn").forEach((b) => (b.disabled = true));
        setTimeout(() => {
          document.querySelectorAll(".esc-test-btn").forEach((b) => (b.disabled = false));
        }, result.duration_ms);
      } catch (err) {
        showToast(`Couldn't test ESC${esc}: ` + err.message);
      }
    });
  });

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

function initNavMode(mode) {
  document.querySelectorAll('input[name="nav-mode"]').forEach((radio) => {
    radio.checked = radio.value === mode;
    radio.addEventListener("change", async () => {
      try {
        const result = await apiPost("/api/nav_mode", { mode: radio.value });
        showToast(result.nav_mode === "heading_hold" ? "Autonomous: straight & level" : "Autonomous: follow waypoints");
      } catch (err) {
        showToast("Couldn't save autonomous mode: " + err.message);
      }
    });
  });
}

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

let airspeedDirty = false;

function renderAirspeedLive(data) {
  document.getElementById("airspeed-live").textContent = data.reading ? `${data.live_cms} cm/s` : "no reading";
}

function initAirspeed(cfg) {
  const targetInput = document.getElementById("airspeed-target");
  const fallbackInput = document.getElementById("airspeed-fallback");
  const saveBtn = document.getElementById("airspeed-save-btn");
  targetInput.value = cfg.target_cms;
  fallbackInput.value = (cfg.fallback_pct * 100).toFixed(0);
  renderAirspeedLive(cfg);

  [targetInput, fallbackInput].forEach((input) => {
    input.addEventListener("input", () => { airspeedDirty = true; saveBtn.disabled = false; });
  });

  saveBtn.addEventListener("click", async () => {
    try {
      const result = await apiPost("/api/airspeed", {
        target_cms: parseFloat(targetInput.value),
        fallback_pct: parseFloat(fallbackInput.value) / 100,
      });
      airspeedDirty = false;
      targetInput.value = result.target_cms;
      fallbackInput.value = (result.fallback_pct * 100).toFixed(0);
      saveBtn.disabled = true;
      renderAirspeedLive(result);
      showToast("Airspeed cfg saved");
    } catch (err) {
      showToast("Couldn't save airspeed cfg: " + err.message);
    }
  });
}

// Polled independently of loadInitialState() so the live reading (and the
// target/fallback fields, unless you're mid-edit) stay current without a
// page refresh -- same "don't clobber an in-progress edit" rule the output
// rows already follow.
async function pollAirspeed() {
  try {
    const data = await apiGet("/api/airspeed");
    renderAirspeedLive(data);
    if (!airspeedDirty) {
      document.getElementById("airspeed-target").value = data.target_cms;
      document.getElementById("airspeed-fallback").value = (data.fallback_pct * 100).toFixed(0);
    }
  } catch (err) {
    // Transient hiccup on the local AP link -- ignore, next poll retries.
  }
}

// ---------------- IMU ----------------

function renderImu(data) {
  document.getElementById("imu-ready-badge").textContent = data.ready ? "ready" : "not ready";
  document.getElementById("imu-roll").textContent = `${data.roll.toFixed(1)}°`;
  document.getElementById("imu-pitch").textContent = `${data.pitch.toFixed(1)}°`;
  document.getElementById("imu-yaw").textContent = `${data.yaw.toFixed(1)}°`;
  document.getElementById("imu-mag-valid").textContent = data.mag_valid ? "yes" : "no";
}

async function pollImu() {
  try {
    renderImu(await apiGet("/api/imu"));
  } catch (err) {
    // Transient hiccup on the local AP link -- ignore, next poll retries.
  }
}

// ---------------- Live "plane is here" marker ----------------

let planeMarker = null;

function updatePlaneMarker(lat, lon) {
  if (!map) return; // map isn't created yet -- see loadInitialState()
  if (!planeMarker) {
    const icon = L.divIcon({
      className: "plane-marker-icon",
      html: '<div class="plane-dot"></div>',
      iconSize: [16, 16],
      iconAnchor: [8, 8],
    });
    planeMarker = L.marker([lat, lon], { icon, zIndexOffset: 1000, interactive: false }).addTo(map);
    planeMarker.bindTooltip("Plane", { permanent: false, direction: "top", offset: [0, -10] });
  } else {
    planeMarker.setLatLng([lat, lon]);
  }
}

let homeMarker = null;

function updateHomeMarker(home) {
  if (!map || !home) return;
  if (!homeMarker) {
    homeMarker = L.circleMarker([home.lat, home.lon], {
      radius: 8, color: "#16a34a", weight: 3, fillColor: "#16a34a", fillOpacity: 0.35, interactive: false,
    }).addTo(map);
    homeMarker.bindTooltip("Home (return-to-home point)", { direction: "top" });
  } else {
    homeMarker.setLatLng([home.lat, home.lon]);
  }
}

async function pollGps() {
  try {
    const data = await apiGet("/api/gps");
    if (data.valid) updatePlaneMarker(data.lat, data.lon);
    updateHomeMarker(data.home);
  } catch (err) {
    // Transient hiccup on the local AP link -- ignore, next poll retries.
  }
}

// ---------------- Initial load ----------------

// Best-effort browser geolocation, for the rare case the plane's onboard GPS
// hasn't gotten a fix yet (or this page is ever served over something other
// than the setup AP's plain HTTP). Note: most browsers block
// navigator.geolocation entirely on an insecure origin like
// http://192.168.4.1/, so this will typically just time out and fall
// through -- it's here in case that's ever not true (HTTPS added later, a
// browser/OS that's more lenient, etc.), not because it's expected to work
// today.
function getBrowserLocation() {
  return new Promise((resolve) => {
    if (!navigator.geolocation) {
      resolve(null);
      return;
    }
    navigator.geolocation.getCurrentPosition(
      (pos) => resolve({ lat: pos.coords.latitude, lon: pos.coords.longitude }),
      () => resolve(null),
      { timeout: 4000, maximumAge: 60000 }
    );
  });
}

async function loadInitialState() {
  try {
    const state = await apiGet("/api/state");

    // Prefer, in order: an existing mission's first waypoint (so you see
    // your actual plan, not just where you happen to be standing), the
    // plane's own onboard GPS fix (the most reliable source here -- see
    // getBrowserLocation()'s note on why the browser one usually can't run
    // at all), browser geolocation as a last-ditch best effort, then the
    // ocean.
    let center = state.mission.points[0];
    if (!center && state.gps.valid) center = { lat: state.gps.lat, lon: state.gps.lon };
    if (!center) center = await getBrowserLocation();
    if (!center) center = { lat: 0, lon: 0 };
    initMap(center.lat, center.lon);
    mission = state.mission.points.map((p) => ({ lat: p.lat, lon: p.lon }));
    loopMission = state.mission.loop;
    document.getElementById("loop-toggle").checked = loopMission;
    renderMission();

    PID_TARGETS.forEach((target) => renderPidTarget(target, state.pid[target]));
    buildOutputRows(state.outputs);
    initMotorCfg(state.motor_cfg);
    initAirframe(state.airframe);
    initNavMode(state.nav_mode);
    initAirspeed(state.airspeed_cfg);

    const rcMap = await apiGet("/api/rc_map");
    buildRcMapRows(rcMap.map);

    const servoMap = await apiGet("/api/servo_map");
    buildServoMapRows(servoMap.map);

    buildTrimRows(await apiGet("/api/trim"));
    initTrimButtons();
  } catch (err) {
    showToast("Couldn't reach setup API: " + err.message);
    initMap(0, 0);
  }
}

document.addEventListener("DOMContentLoaded", () => {
  buildPidCards();
  initControlTest();
  initFlightLog();
  document.getElementById("clear-mission-btn").addEventListener("click", clearMission);
  document.getElementById("loop-toggle").addEventListener("change", onLoopToggleChanged);
  loadInitialState();
  setInterval(pollOutputs, OUTPUT_POLL_INTERVAL_MS);
  setInterval(pollAirspeed, OUTPUT_POLL_INTERVAL_MS);
  setInterval(pollImu, OUTPUT_POLL_INTERVAL_MS);
  setInterval(pollRcMap, OUTPUT_POLL_INTERVAL_MS);
  setInterval(pollTrim, OUTPUT_POLL_INTERVAL_MS);
  setInterval(pollControlTest, OUTPUT_POLL_INTERVAL_MS);
  setInterval(pollMode, OUTPUT_POLL_INTERVAL_MS);
  setInterval(pollGps, GPS_POLL_INTERVAL_MS);
});
