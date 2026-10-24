"use strict";

/* ACMFDS live dashboard frontend.
 * All displayed values come from the per-step rows streamed over the
 * WebSocket — the same rows that are written to the CSV flight log.
 * The browser never steps the simulation: it only reads snapshots. */

const $ = (id) => document.getElementById(id);

const COL = {
  cyan: "#38c6ff", green: "#3ddc84", amber: "#ffb347", red: "#ff5c57",
  muted: "#7e8ea4", grid: "#1d2937", text: "#c7d3e2",
};

const state = {
  since: 0,
  rows: [],
  status: "idle",
  outcome: "",
  reason: "",
  summary: null,
  config: null,
  moving: true,
  gps: null,
  world: null,
  device: null,      // operator device fix — browser-only, never sent anywhere
  watchId: null,     // geolocation.watchPosition handle (null = not tracking)
  originSource: "",  // "device" | "manual" — only affects the honesty note
  connected: false,
};

const fmt = (v, d = 2) =>
  (v === null || v === undefined || Number.isNaN(Number(v))) ? "--" : Number(v).toFixed(d);

function ctxColor(ctx) {
  if (ctx === "EMERGENCY" || ctx === "OBSTACLE_CRITICAL") return COL.red;
  if (ctx === "OBSTACLE_NEAR" || ctx === "LOW_BATTERY" || ctx === "STRONG_WIND") return COL.amber;
  if (ctx === "NORMAL") return COL.green;
  return COL.cyan;  // LANDING / MISSION_COMPLETE / unknown
}

/* ------------------------------------------------------------------ API */
async function api(path, body, method = "POST") {
  const res = await fetch(path, {
    method,
    headers: { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  let data = null;
  try { data = await res.json(); } catch (e) { /* no body */ }
  if (!res.ok) {
    let msg = `HTTP ${res.status}`;
    if (data && Array.isArray(data.detail)) {
      msg = data.detail.map((d) => `${(d.loc || []).slice(-1)[0]}: ${d.msg}`).join("; ");
    } else if (data && data.detail) { msg = data.detail; }
    throw new Error(msg);
  }
  if (data && data.ok === false) throw new Error(data.error || "rejected");
  return data;
}

function collectConfig() {
  return {
    scenario: $("cfgScenario").value,
    target_altitude: parseFloat($("cfgAlt").value),
    goal_x: parseFloat($("cfgGoalX").value),
    goal_y: parseFloat($("cfgGoalY").value),
    cruise_speed: parseFloat($("cfgSpeed").value),
    initial_battery: parseFloat($("cfgBattery").value),
    wind_enabled: $("cfgWindOn").checked,
    wind_speed: parseFloat($("cfgWind").value),
    wind_direction_deg: parseFloat($("cfgWindDir").value),
    dynamic_obstacles_enabled: $("cfgMoving").checked,
    native_window: $("cfgNativeWindow").checked,
  };
}

/* ------------------------------------------------------------- WebSocket */
function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { state.connected = true; state.since = 0; state.rows = []; renderStatus(); };
  ws.onclose = () => { state.connected = false; renderStatus(); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => { try { applySnapshot(JSON.parse(ev.data)); } catch (e) { /* ignore */ } };
}

function applySnapshot(m) {
  const prevStatus = state.status;
  const prevTotal = state.since;
  const prevGps = JSON.stringify(state.gps);
  state.status = m.status;
  state.outcome = m.outcome;
  state.reason = m.reason;
  state.summary = m.summary;
  state.config = m.config;
  state.moving = m.moving_enabled;
  state.gps = m.gps || null;
  if (m.world) state.world = m.world;
  if (m.total < state.since) { state.rows = []; state.since = 0; }  // server reset
  if (m.rows.length) state.rows = state.rows.concat(m.rows);
  state.since = m.total;
  // Repaint only when something actually changed: an idle-streaming
  // snapshot (10 Hz, no new rows) must not redraw the full history.
  if (m.rows.length || m.status !== prevStatus || m.total !== prevTotal ||
      JSON.stringify(state.gps) !== prevGps) render();
}

/* ---------------------------------------------------------- canvas util */
function ctx2d(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  const g = canvas.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  return [g, w, h];
}

/* Axes with full titles, units, and tick labels on both dimensions. */
function axes(g, w, h, x0, x1, y0, y1, xTitle, yTitle) {
  const padL = 48, padR = 12, padT = 12, padB = 32;
  const X = (v) => padL + (w - padL - padR) * ((v - x0) / (x1 - x0 || 1));
  const Y = (v) => h - padB - (h - padT - padB) * ((v - y0) / (y1 - y0 || 1));
  const font = "11px ui-monospace, Consolas, monospace";
  const dec = (range) => (range >= 20 ? 0 : 1);
  g.strokeStyle = COL.grid; g.fillStyle = COL.muted; g.lineWidth = 1;
  // Y gridlines + tick labels (right-aligned against the axis)
  g.font = font; g.textAlign = "right";
  for (let i = 0; i <= 3; i++) {
    const yv = y0 + (y1 - y0) * i / 3;
    const y = Y(yv);
    g.beginPath(); g.moveTo(padL, y); g.lineTo(w - padR, y); g.stroke();
    g.fillText(yv.toFixed(dec(y1 - y0)), padL - 5, y + 3.5);
  }
  // X ticks + labels along the bottom
  g.textAlign = "center";
  for (let i = 0; i <= 4; i++) {
    const xv = x0 + (x1 - x0) * i / 4;
    const x = X(xv);
    g.beginPath(); g.moveTo(x, h - padB); g.lineTo(x, h - padB + 4); g.stroke();
    g.fillText(xv.toFixed(dec(x1 - x0)), x, h - padB + 15);
  }
  // axis titles with units: X centered below the ticks, Y rotated at left
  g.font = "bold " + font;
  g.fillText(xTitle, padL + (w - padL - padR) / 2, h - 3);
  g.save();
  g.translate(11, padT + (h - padT - padB) / 2);
  g.rotate(-Math.PI / 2);
  g.fillText(yTitle, 0, 0);
  g.restore();
  g.textAlign = "left";
  return { X, Y, padL, padR, padT, padB };
}

function lineChart(canvasId, series, yRange, hline, xTitle, yTitle) {
  const [g, w, h] = ctx2d($(canvasId));
  const rows = state.rows;
  if (!rows.length) return;
  const t0 = rows[0].timestamp;
  const t1 = Math.max(rows[rows.length - 1].timestamp, t0 + 0.1);
  const [y0, y1] = yRange(rows);
  const a = axes(g, w, h, t0, t1, y0, y1, xTitle, yTitle);
  for (const s of series) {
    g.strokeStyle = s.color; g.lineWidth = 1.4;
    if (s.dash) g.setLineDash([4, 4]); else g.setLineDash([]);
    g.beginPath();
    let first = true;
    for (const r of rows) {
      const v = r[s.key];
      if (v === null || v === undefined) continue;   // genuinely missing field -> gap, never fabricated
      const x = a.X(r.timestamp), y = a.Y(v);
      if (first) { g.moveTo(x, y); first = false; } else g.lineTo(x, y);
    }
    g.stroke();
  }
  g.setLineDash([]);
  if (hline) {
    const y = a.Y(hline.y);
    g.strokeStyle = hline.color || COL.muted; g.lineWidth = 1; g.setLineDash([2, 3]);
    g.beginPath(); g.moveTo(a.padL, y); g.lineTo(w - a.padR, y); g.stroke();
    g.setLineDash([]);
    g.fillStyle = COL.muted; g.font = "9.5px ui-monospace, Consolas, monospace";
    g.fillText(hline.label, a.padL + 4, y - 3);
  }
}

function pathChart() {
  const [g, w, h] = ctx2d($("chPath"));
  const rows = state.rows;
  $("pathLegend").innerHTML =
    `<span class="lg"><i style="background:${COL.cyan}"></i>flown path (logged rows)</span>` +
    `<span class="lg"><i style="background:#e9ecf5"></i>current drone</span>` +
    `<span class="lg"><i style="background:${COL.green}"></i>goal</span>` +
    `<span class="lg"><i style="background:${COL.muted}"></i>start</span>` +
    `<span class="lg"><i style="background:#b22228"></i>static obstacles</span>` +
    `<span class="lg"><i style="background:#d68a1a"></i>moving obstacles</span>`;
  if (!rows.length) return;
  const x0 = -8, x1 = 8, y0 = -5.5, y1 = 5.5;
  const padL = 34, padR = 14, padT = 14, padB = 30;
  const X = (v) => padL + (w - padL - padR) * (v - x0) / (x1 - x0);
  const Y = (v) => h - padB - (h - padT - padB) * (v - y0) / (y1 - y0);
  // gridlines + tick labels (every 2 m), then axis titles with units
  const font = "11px ui-monospace, Consolas, monospace";
  g.strokeStyle = COL.grid; g.lineWidth = 1;
  g.fillStyle = COL.muted; g.font = font;
  g.textAlign = "center";
  for (let gx = -8; gx <= 8; gx += 2) {
    g.beginPath(); g.moveTo(X(gx), Y(y1)); g.lineTo(X(gx), Y(y0)); g.stroke();
    g.fillText(String(gx), X(gx), h - padB + 15);
  }
  g.textAlign = "right";
  for (let gy = -4; gy <= 4; gy += 2) {
    g.beginPath(); g.moveTo(X(x0), Y(gy)); g.lineTo(X(x1), Y(gy)); g.stroke();
    g.fillText(String(gy), padL - 5, Y(gy) + 3.5);
  }
  g.font = "bold " + font; g.textAlign = "center";
  g.fillText("X position (m)", padL + (w - padL - padR) / 2, h - 3);
  g.save();
  g.translate(11, padT + (h - padT - padB) / 2);
  g.rotate(-Math.PI / 2);
  g.fillText("Y position (m)", 0, 0);
  g.restore();
  g.textAlign = "left";
  // static obstacles (actual spawned geometry)
  const world = state.world;
  if (world) {
    g.fillStyle = "rgba(178, 34, 40, 0.55)"; g.strokeStyle = "#b22228";
    for (const o of (world.obstacles.static || [])) {
      const [cx, cy] = o.position, [hx, hy] = o.half_extents;
      g.fillRect(X(cx - hx), Y(cy + hy), X(cx + hx) - X(cx - hx), Y(cy - hy) - Y(cy + hy));
      g.strokeRect(X(cx - hx), Y(cy + hy), X(cx + hx) - X(cx - hx), Y(cy - hy) - Y(cy + hy));
    }
  }
  // flown trail
  g.strokeStyle = COL.cyan; g.lineWidth = 1.6; g.beginPath();
  rows.forEach((r, i) => {
    const x = X(r.position_x), y = Y(r.position_y);
    if (i === 0) g.moveTo(x, y); else g.lineTo(x, y);
  });
  g.stroke();
  // start position (first logged row of this flight)
  g.fillStyle = COL.muted;
  g.beginPath(); g.arc(X(rows[0].position_x), Y(rows[0].position_y), 3, 0, Math.PI * 2); g.fill();
  g.fillText("start", X(rows[0].position_x) + 5, Y(rows[0].position_y) - 4);
  // goal marker (from the validated config actually used by this run)
  const cfg = state.config || {};
  if (cfg.goal_x !== undefined) {
    g.strokeStyle = COL.green; g.lineWidth = 1.6;
    g.beginPath(); g.arc(X(cfg.goal_x), Y(cfg.goal_y), 5, 0, Math.PI * 2); g.stroke();
    g.fillStyle = COL.green;
    g.fillText("goal", X(cfg.goal_x) + 7, Y(cfg.goal_y) + 3);
  }
  // current moving-obstacle positions (from the logged row — real values)
  const last = rows[rows.length - 1];
  if (last.moving_obstacles) {
    g.fillStyle = COL.amber;
    for (const part of last.moving_obstacles.split(";")) {
      const p = part.split(",").map(Number);
      if (p.length >= 2 && p.every((n) => Number.isFinite(n))) {
        g.fillRect(X(p[0]) - 3, Y(p[1]) - 3, 6, 6);
      }
    }
  }
  // current drone position
  g.fillStyle = "#eef7ff";
  g.beginPath(); g.arc(X(last.position_x), Y(last.position_y), 3.4, 0, Math.PI * 2); g.fill();
  g.strokeStyle = COL.cyan; g.lineWidth = 1;
  g.beginPath(); g.arc(X(last.position_x), Y(last.position_y), 5.5, 0, Math.PI * 2); g.stroke();
}

/* ------------------------------------------- GPS-referenced obstacle map */
/* Top-down schematic in GEOGRAPHIC coordinates, referenced to the configured
 * origin. Every marker is converted from the same real PyBullet world X/Y
 * values used by the local trajectory plot (toGeo == telemetry/gps.py), so
 * relative geometry and distances are identical between the two views.
 * Requires an origin; without one it says so instead of inventing one. */
function renderGpsMap() {
  const [g2, w, h] = ctx2d($("chGpsMap"));
  const g = state.gps;
  const info = $("gpsMapInfo");
  const rows = state.rows;
  const legend = $("gpsMapLegend");
  legend.innerHTML =
    `<span class="lg"><i style="background:${COL.cyan}"></i>flown route (sim)</span>` +
    `<span class="lg"><i style="background:#e9ecf5"></i>drone (sim position)</span>` +
    `<span class="lg"><i style="background:${COL.green}"></i>destination (sim)</span>` +
    `<span class="lg"><i style="background:${COL.muted}"></i>start (sim)</span>` +
    `<span class="lg"><i style="background:#b22228"></i>simulated static obstacle</span>` +
    `<span class="lg"><i style="background:#d68a1a"></i>simulated moving obstacle</span>` +
    `<span class="lg"><i style="background:#c05ae0"></i>origin (world 0,0)</span>`;
  const noOrigin = !g || !g.origin_set;
  if (noOrigin) {
    g2.fillStyle = COL.muted; g2.font = "12px ui-monospace, Consolas, monospace";
    g2.fillText("Origin not set — set a geographic origin (device fix or manual) to enable the geographic map.", 10, 22);
    info.textContent = "Origin not set — the map needs a geographic origin before positions can be geo-referenced.";
    return;
  }
  const lat0 = g.ref_lat, lon0 = g.ref_lon;
  const F = (s) => s.toFixed(6);
  // World frame (metres) — same extents as the local top-down plot so the two
  // views are directly comparable. Aspect is 1:1 (no distortion of geometry).
  const x0 = -8, x1 = 8, y0 = -5.5, y1 = 5.5, pad = 30;
  const spanX = x1 - x0, spanY = y1 - y0;
  const scale = Math.min((w - 2 * pad) / spanX, (h - 2 * pad) / spanY);
  const X = (v) => pad + (w - 2 * pad) / 2 + (v - (x0 + x1) / 2) * scale;
  const Y = (v) => h - pad - (h - 2 * pad) / 2 + ((v - (y0 + y1) / 2)) * scale;
  // Graticule: 2 m grid labelled in degrees (the same conversion as the drone)
  g2.strokeStyle = COL.grid; g2.lineWidth = 1;
  g2.fillStyle = COL.muted; g2.font = "8.5px ui-monospace, Consolas, monospace";
  for (let gx = -8; gx <= 8; gx += 2) {
    g2.beginPath(); g2.moveTo(X(gx), Y(y0)); g2.lineTo(X(gx), Y(y1)); g2.stroke();
    const [, glon] = toGeo(gx, 0, lat0, lon0);
    g2.fillText(glon.toFixed(6) + "°", X(gx) - 18, h - 6);
  }
  for (let gy = -4; gy <= 4; gy += 2) {
    g2.beginPath(); g2.moveTo(X(x0), Y(gy)); g2.lineTo(X(x1), Y(gy)); g2.stroke();
    const [glat] = toGeo(0, gy, lat0, lon0);
    g2.fillText(glat.toFixed(6) + "°", 2, Y(gy) - 2);
  }
  // origin crosshair at world (0,0)
  g2.strokeStyle = "#c05ae0"; g2.lineWidth = 1.4;
  g2.beginPath(); g2.moveTo(X(0) - 7, Y(0)); g2.lineTo(X(0) + 7, Y(0));
  g2.moveTo(X(0), Y(0) - 7); g2.lineTo(X(0), Y(0) + 7); g2.stroke();
  g2.fillStyle = "#c05ae0"; g2.font = "9.5px ui-monospace, Consolas, monospace";
  g2.fillText("origin", X(0) + 9, Y(0) + 3);
  // simulated static obstacles (actual spawned geometry)
  const world = state.world;
  if (world) {
    g2.fillStyle = "rgba(178, 34, 40, 0.55)"; g2.strokeStyle = "#b22228";
    for (const o of (world.obstacles.static || [])) {
      const [cx, cy] = o.position, [hx, hy] = o.half_extents;
      g2.fillRect(X(cx - hx), Y(cy + hy), X(cx + hx) - X(cx - hx), Y(cy - hy) - Y(cy + hy));
      g2.strokeRect(X(cx - hx), Y(cy + hy), X(cx + hx) - X(cx - hx), Y(cy - hy) - Y(cy + hy));
    }
  }
  if (!rows.length) {
    // Origin is set but nothing flown yet: still show destination + start.
    const ds = g.drone_start_world || [-4, 0];
    g2.fillStyle = COL.muted;
    g2.beginPath(); g2.arc(X(ds[0]), Y(ds[1]), 3, 0, Math.PI * 2); g2.fill();
    g2.fillText("start", X(ds[0]) + 5, Y(ds[1]) - 4);
    const cfg = state.config || {};
    if (cfg.goal_x !== undefined) {
      g2.strokeStyle = COL.green; g2.lineWidth = 1.6;
      g2.beginPath(); g2.arc(X(cfg.goal_x), Y(cfg.goal_y), 5, 0, Math.PI * 2); g2.stroke();
      g2.fillStyle = COL.green; g2.fillText("destination", X(cfg.goal_x) + 7, Y(cfg.goal_y) + 3);
    }
    info.textContent = "Origin set (" + F(lat0) + ", " + F(lon0) + ") — start a flight to see the live route.";
    return;
  }
  // flown route
  g2.strokeStyle = COL.cyan; g2.lineWidth = 1.6; g2.beginPath();
  rows.forEach((r, i) => {
    const px = X(r.position_x), py = Y(r.position_y);
    if (i === 0) g2.moveTo(px, py); else g2.lineTo(px, py);
  });
  g2.stroke();
  // start
  const s = rows[0];
  g2.fillStyle = COL.muted;
  g2.beginPath(); g2.arc(X(s.position_x), Y(s.position_y), 3, 0, Math.PI * 2); g2.fill();
  g2.fillText("start", X(s.position_x) + 5, Y(s.position_y) - 4);
  // destination
  const cfg = state.config || {};
  if (cfg.goal_x !== undefined) {
    g2.strokeStyle = COL.green; g2.lineWidth = 1.6;
    g2.beginPath(); g2.arc(X(cfg.goal_x), Y(cfg.goal_y), 5, 0, Math.PI * 2); g2.stroke();
    g2.fillStyle = COL.green; g2.fillText("destination", X(cfg.goal_x) + 7, Y(cfg.goal_y) + 3);
  }
  const last = rows[rows.length - 1];
  // moving obstacles — actual simulated positions from the last logged row
  if (last.moving_obstacles) {
    g2.fillStyle = COL.amber;
    for (const part of last.moving_obstacles.split(";")) {
      const p = part.split(",").map(Number);
      if (p.length >= 2 && p.every((n) => Number.isFinite(n)))
        g2.fillRect(X(p[0]) - 3, Y(p[1]) - 3, 6, 6);
    }
  }
  // live drone + heading arrow from the row's true yaw
  const dx = X(last.position_x), dy = Y(last.position_y);
  const yaw = Number(last.yaw) || 0;
  const hx2 = dx + Math.cos(yaw) * 12, hy2 = dy - Math.sin(yaw) * 12;
  g2.strokeStyle = "#eef7ff"; g2.lineWidth = 1.6;
  g2.beginPath(); g2.moveTo(dx, dy); g2.lineTo(hx2, hy2); g2.stroke();
  g2.fillStyle = "#eef7ff";
  g2.beginPath(); g2.arc(dx, dy, 3.4, 0, Math.PI * 2); g2.fill();
  g2.strokeStyle = COL.cyan; g2.lineWidth = 1;
  g2.beginPath(); g2.arc(dx, dy, 5.5, 0, Math.PI * 2); g2.stroke();
  const [dlat, dlon] = toGeo(last.position_x, last.position_y, lat0, lon0);
  // Readout: current simulated coordinates + origin, honestly labeled.
  let txt =
    "SIMULATED drone: " + F(dlat) + ", " + F(dlon) + "   (world X=" +
    fmt(last.position_x, 2) + ", Y=" + fmt(last.position_y, 2) + " m)\n" +
    "origin: " + F(lat0) + ", " + F(lon0) + " ↔ world (0, 0)   heading " +
    (yaw * 180 / Math.PI).toFixed(0) + "°   updated t=" + fmt(last.timestamp, 1) + " s (sim)";
  if (state.originSource === "device" && state.device && Number.isFinite(state.device.accuracy)) {
    txt += "\norigin from device fix (±" + fmt(state.device.accuracy, 0) +
      " m): that uncertainty shifts the ABSOLUTE placement only — relative distances come from the simulation.";
  }
  txt += "\nAll markers are SIMULATED positions referenced to the origin — not a real map, and the simulated obstacles are not real-world objects.";
  info.textContent = txt;
}

/* ------------------------------------------------------ decision timeline */
/* Real state names from the FlightContext enum (decision/acmfds.py), in
 * severity order for the categorical Y axis. */
const DECISION_STATES = [
  "EMERGENCY", "OBSTACLE_CRITICAL", "OBSTACLE_NEAR", "LOW_BATTERY",
  "STRONG_WIND", "LANDING", "MISSION_COMPLETE", "NORMAL",
];

let timelineHover = null;                        // {x} while pointer is on the canvas
const timelineGeom = { t0: 0, t1: 1, padL: 0, padR: 0, w: 0 };

function timelineRuns(rows) {
  /* Group CONSECUTIVE rows sharing one acmfds_context into runs. Only
   * consecutive samples merge: every genuine state transition starts a new
   * run, so distinct emergency episodes can never fuse into one bar. */
  const runs = [];
  for (let i = 0; i < rows.length; i++) {
    const ctx = rows[i].acmfds_context;
    if (runs.length && runs[runs.length - 1].context === ctx) {
      runs[runs.length - 1].end = i;
    } else {
      runs.push({ context: ctx, start: i, end: i });
    }
  }
  return runs;
}

function timelineChart() {
  const [g, w, h] = ctx2d($("chTimeline"));
  const rows = state.rows;
  const legend = $("timelineLegend");
  if (!rows.length) {
    legend.innerHTML = "";
    $("tlTip").style.display = "none";
    timelineHover = null;
    return;
  }
  const t0 = rows[0].timestamp;
  const t1 = Math.max(rows[rows.length - 1].timestamp, t0 + 0.1);
  const padL = 130, padR = 14, padT = 12, padB = 34;
  const X = (t) => padL + (w - padL - padR) * (t - t0) / (t1 - t0);
  Object.assign(timelineGeom, { t0, t1, padL, padR, w });

  const nStates = DECISION_STATES.length;
  const rowH = (h - padT - padB) / nStates;
  const rowY = (s) => padT + rowH * (DECISION_STATES.indexOf(s) + 0.5);
  const font = "10.5px ui-monospace, Consolas, monospace";

  // row separators + Y labels (real FlightContext state names)
  g.strokeStyle = COL.grid; g.lineWidth = 1;
  for (let i = 0; i <= nStates; i++) {
    const y = padT + rowH * i;
    g.beginPath(); g.moveTo(padL, y); g.lineTo(w - padR, y); g.stroke();
  }
  g.fillStyle = COL.muted; g.font = "9.5px ui-monospace, Consolas, monospace";
  g.textAlign = "right";
  for (const s of DECISION_STATES) g.fillText(s, padL - 10, rowY(s) + 3.5);

  // X ticks + axis titles with units
  g.font = font; g.textAlign = "center";
  for (let i = 0; i <= 4; i++) {
    const tv = t0 + (t1 - t0) * i / 4;
    const x = X(tv);
    g.beginPath(); g.moveTo(x, h - padB); g.lineTo(x, h - padB + 4); g.stroke();
    g.fillText(tv.toFixed(1), x, h - padB + 15);
  }
  g.font = "bold " + font;
  g.fillText("Simulation time (s)", padL + (w - padL - padR) / 2, h - 4);
  g.save();
  g.translate(11, padT + (h - padT - padB) / 2);
  g.rotate(-Math.PI / 2);
  g.fillText("Decision state", 0, 0);
  g.restore();

  // one horizontal segment per run of consecutive same-state rows; segment
  // bounds are the first/last row timestamps of that run (real durations).
  for (const run of timelineRuns(rows)) {
    const y = rowY(run.context);
    const c = ctxColor(run.context);
    const x1 = X(rows[run.start].timestamp);
    const x2 = X(rows[run.end].timestamp);
    if (x2 - x1 < 2.5) {
      // single-sample run: a marker, not a stretched bar
      g.fillStyle = c;
      g.beginPath(); g.arc(x1, y, 3, 0, Math.PI * 2); g.fill();
    } else {
      const bh = Math.max(4, Math.min(12, rowH - 8));
      g.globalAlpha = 0.85;
      g.fillStyle = c;
      g.beginPath();
      g.roundRect ? g.roundRect(x1, y - bh / 2, x2 - x1, bh, 3)
                  : g.rect(x1, y - bh / 2, x2 - x1, bh);
      g.fill();
      g.globalAlpha = 1;
    }
  }
  // hover cursor line (tooltip content is filled by the mousemove handler)
  if (timelineHover !== null) {
    const x = Math.min(Math.max(timelineHover.x, padL), w - padR);
    g.strokeStyle = "#eef7ff"; g.lineWidth = 1;
    g.setLineDash([3, 3]);
    g.beginPath(); g.moveTo(x, padT); g.lineTo(x, h - padB); g.stroke();
    g.setLineDash([]);
  }
  // legend: contexts actually present in this flight (existing colours)
  const seen = [...new Set(rows.map((r) => r.acmfds_context))];
  legend.innerHTML = seen.sort().map(
    (c) => `<span class="lg"><i style="background:${ctxColor(c)}"></i>${c}</span>`
  ).join("");
}

/* ------------------------------------------------------- 3D scene view */
/* Orthographic projection of the REAL streamed state — the drone pose is
 * the last logged row, obstacles are the actual spawned geometry from the
 * server's world block plus the row's moving-obstacle positions. This is
 * a viewer, not a second simulation: nothing is integrated here. */
const SCENE = { yaw: -0.62, pitch: 0.40, center: [0.0, 0.0, 1.3] };

function sceneBasis(w, h) {
  const a = SCENE.yaw, e = SCENE.pitch;
  const f = [-Math.cos(e) * Math.cos(a), -Math.cos(e) * Math.sin(a), -Math.sin(e)];
  const r = [-Math.sin(a), Math.cos(a), 0.0];
  const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
  const span = 17.5;                                     // world x-extent to fit
  const scale = Math.min(w * 0.92 / span, h * 0.92 / (span * 0.52));
  const P = (pt) => {
    const v = [pt[0] - SCENE.center[0], pt[1] - SCENE.center[1], pt[2] - SCENE.center[2]];
    return [w / 2 + (v[0] * r[0] + v[1] * r[1] + v[2] * r[2]) * scale,
            h / 2 - (v[0] * u[0] + v[1] * u[1] + v[2] * u[2]) * scale,
            v[0] * f[0] + v[1] * f[1] + v[2] * f[2]];
  };
  return { P, cam: [-f[0], -f[1], -f[2]], scale };
}

function boxFaces(center, half) {
  const [cx, cy, cz] = center, [hx, hy, hz] = half;
  const c = (i, j, k) => [cx + i * hx, cy + j * hy, cz + k * hz];
  return [
    { pts: [c(-1, -1, 1), c(1, -1, 1), c(1, 1, 1), c(-1, 1, 1)], n: [0, 0, 1] },
    { pts: [c(1, -1, -1), c(1, -1, 1), c(1, 1, 1), c(1, 1, -1)], n: [1, 0, 0] },
    { pts: [c(-1, 1, -1), c(-1, 1, 1), c(1, 1, 1), c(1, 1, -1)], n: [0, 1, 0] },
    { pts: [c(-1, -1, -1), c(-1, -1, 1), c(1, -1, 1), c(1, -1, -1)], n: [0, -1, 0] },
    { pts: [c(-1, -1, -1), c(-1, -1, 1), c(-1, 1, 1), c(-1, 1, -1)], n: [-1, 0, 0] },
  ];
}

function rotZyx(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function shade(rgb, n, cam) {
  const d = Math.abs(n[0] * cam[0] + n[1] * cam[1] + n[2] * cam[2]);
  const k = 0.45 + 0.55 * d;
  return `rgba(${Math.round(rgb[0] * k)},${Math.round(rgb[1] * k)},${Math.round(rgb[2] * k)},0.92)`;
}

function drawScene() {
  const [g, w, h] = ctx2d($("scene3d"));
  const { P, cam, scale } = sceneBasis(w, h);
  // ground grid
  g.lineWidth = 1; g.strokeStyle = COL.grid;
  for (let gx = -8; gx <= 8; gx += 2) {
    const a = P([gx, -5, 0]), b = P([gx, 5, 0]);
    g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
  }
  for (let gy = -4; gy <= 4; gy += 2) {
    const a = P([-8, gy, 0]), b = P([8, gy, 0]);
    g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
  }
  const drawables = [];   // {depth, draw} — painter's algorithm, far first
  const face = (pts, n, rgb) => {
    const pp = pts.map(P);
    if ((n[0] * cam[0] + n[1] * cam[1] + n[2] * cam[2]) <= 0) return;
    const depth = pp.reduce((s, p) => s + p[2], 0) / pp.length;
    drawables.push({ depth, draw: () => {
      g.fillStyle = shade(rgb, n, cam); g.strokeStyle = COL.grid; g.lineWidth = 1;
      g.beginPath(); g.moveTo(pp[0][0], pp[0][1]);
      for (let i = 1; i < pp.length; i++) g.lineTo(pp[i][0], pp[i][1]);
      g.closePath(); g.fill(); g.stroke();
    }});
  };
  const world = state.world;
  if (world) {
    // flown trail (subsampled, real logged positions)
    const rows = state.rows;
    if (rows.length > 1) {
      drawables.push({ depth: 1e9, draw: () => {
        g.strokeStyle = COL.cyan; g.lineWidth = 1.3; g.beginPath();
        let first = true;
        for (let i = 0; i < rows.length; i += Math.ceil(rows.length / 400)) {
          const r = rows[i], p = P([r.position_x, r.position_y, r.altitude]);
          if (first) { g.moveTo(p[0], p[1]); first = false; } else g.lineTo(p[0], p[1]);
        }
        const last = rows[rows.length - 1], p = P([last.position_x, last.position_y, last.altitude]);
        g.lineTo(p[0], p[1]); g.stroke();
      }});
    }
    // goal post + ring
    const [gx, gy, gz] = world.goal;
    drawables.push({ depth: 1e9 - 1, draw: () => {
      const a = P([gx, gy, 0]), b = P([gx, gy, gz]);
      g.strokeStyle = COL.green; g.lineWidth = 2;
      g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
      g.beginPath(); g.arc(b[0], b[1], 4, 0, Math.PI * 2); g.stroke();
      g.fillStyle = COL.green; g.font = "10px ui-monospace, Consolas, monospace";
      g.fillText("goal", b[0] + 6, b[1] - 4);
    }});
    for (const o of (world.obstacles.static || []))
      for (const f of boxFaces(o.position, o.half_extents)) face(f.pts, f.n, [178, 34, 40]);
    const last = state.rows.length ? state.rows[state.rows.length - 1] : null;
    if (last && last.moving_obstacles) {
      const halves = (world.obstacles.moving || []).map((m) => m.half_extents);
      last.moving_obstacles.split(";").forEach((part, i) => {
        const c = part.split(",").map(Number);
        if (c.length >= 3 && c.every(Number.isFinite) && halves[i % halves.length])
          for (const f of boxFaces(c, halves[i % halves.length])) face(f.pts, f.n, [214, 138, 26]);
      });
    }
    // drone: body + arms + rotors from the row's real pose
    if (last) {
      const R = rotZyx(last.roll, last.pitch, last.yaw);
      const at = (dx, dy, dz) => P([
        last.position_x + R[0][0] * dx + R[0][1] * dy + R[0][2] * dz,
        last.position_y + R[1][0] * dx + R[1][1] * dy + R[1][2] * dz,
        last.altitude + R[2][0] * dx + R[2][1] * dy + R[2][2] * dz,
      ]);
      for (const f of boxFaces([last.position_x, last.position_y, last.altitude], [0.25, 0.25, 0.08]))
        face(f.pts, f.n, [225, 236, 248]);
      for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
        const tip = at(0.30 * sx, 0.30 * sy, 0), base = at(0.12 * sx, 0.12 * sy, 0);
        drawables.push({ depth: (tip[2] + base[2]) / 2, draw: () => {
          g.strokeStyle = COL.muted; g.lineWidth = 2;
          g.beginPath(); g.moveTo(base[0], base[1]); g.lineTo(tip[0], tip[1]); g.stroke();
          g.fillStyle = COL.cyan;
          g.beginPath(); g.arc(tip[0], tip[1], 2.6, 0, Math.PI * 2); g.fill();
        }});
      }
    }
    drawables.sort((a, b) => b.depth - a.depth);
    for (const d of drawables) d.draw();
    const phase = last ? String(last.mission_phase || "") : "";
    const ctxN = last ? String(last.acmfds_context || "") : "";
    // Live XYZ chip anchored to the SAME projected drone pose used above —
    // it is re-derived from the row every frame, so it can never detach.
    if (last) {
      const anchor = P([last.position_x, last.position_y, last.altitude + 0.45]);
      const text = `XYZ ${fmt(last.position_x, 2)}, ${fmt(last.position_y, 2)}, ${fmt(last.altitude, 2)} m`;
      g.font = "bold 11px ui-monospace, Consolas, monospace";
      const tw = g.measureText(text).width;
      const bx = Math.min(Math.max(anchor[0] + 8, 4), w - tw - 12);
      const by = Math.min(Math.max(anchor[1] - 10, 4), h - 20);
      g.fillStyle = "rgba(5, 12, 22, 0.82)";
      g.strokeStyle = COL.cyan; g.lineWidth = 1;
      g.beginPath();
      g.roundRect ? g.roundRect(bx, by, tw + 10, 17, 4) : g.rect(bx, by, tw + 10, 17);
      g.fill(); g.stroke();
      g.fillStyle = "#eaf6ff";
      g.fillText(text, bx + 5, by + 12.5);
    }
    g.font = "bold 12px ui-monospace, Consolas, monospace";
    g.fillStyle = ctxColor(ctxN);
    g.fillText(`${phase || "no telemetry yet"}${ctxN ? "  |  " + ctxN : ""}`, 10, 16);
  } else {
    g.fillStyle = COL.muted; g.font = "12px ui-monospace, Consolas, monospace";
    g.fillText("start a flight — the view draws the actual spawned world and live drone pose", 10, 20);
  }
  $("scene3dLegend").innerHTML =
    `<span class="lg"><i style="background:#b22228"></i>static obstacle</span>` +
    `<span class="lg"><i style="background:#d68a1a"></i>moving obstacle</span>` +
    `<span class="lg"><i style="background:${COL.green}"></i>goal</span>` +
    `<span class="lg"><i style="background:#e9ecf5"></i>drone (true pose)</span>` +
    `<span class="lg"><i style="background:${COL.cyan}"></i>flown path</span>`;
}

/* ------------------------------------------------------------- rendering */
function renderStatus() {
  const pill = $("statusPill");
  pill.textContent = state.status.toUpperCase();
  pill.className = `pill ${state.status}`;
  $("conn").className = `dot ${state.connected ? "online" : "offline"}`;

  const s = state.status;
  $("btnStart").disabled = !(s === "idle" || s === "finished");
  $("btnPause").disabled = s !== "running";
  $("btnResume").disabled = s !== "paused";
  $("btnStop").disabled = !(s === "running" || s === "paused");
  $("btnReset").disabled = !(s === "idle" || s === "finished");
  const lock = (s === "running" || s === "paused");
  for (const id of ["cfgScenario", "cfgAlt", "cfgSpeed", "cfgGoalX", "cfgGoalY",
                    "cfgBattery", "cfgWind", "cfgWindDir", "cfgWindOn",
                    "cfgNativeWindow"]) {
    $(id).disabled = lock;
  }
  $("outcome").textContent = state.outcome
    ? `${state.outcome.toUpperCase()}${state.reason ? " — " + state.reason : ""}` : "";
  $("outcome").style.color =
    state.outcome === "landed" ? COL.green : state.outcome ? COL.red : COL.muted;
}

function renderTiles() {
  const r = state.rows.length ? state.rows[state.rows.length - 1] : null;
  $("tPhase").textContent = r ? r.mission_phase : "--";
  $("tAlt").textContent = r ? `${fmt(r.altitude, 2)} m` : "--";
  $("tTgt").textContent = r ? `${fmt(r.target_altitude, 2)} m` : "--";
  $("tSpeed").textContent = r ? `${fmt(r.horizontal_speed, 2)} m/s` : "--";
  $("tPos").textContent = r ? `(${fmt(r.position_x, 1)}, ${fmt(r.position_y, 1)})` : "--";
  $("tObst").textContent = r ? `${fmt(r.obstacle_distance, 2)} m` : "--";
  $("tBatt").textContent = r ? `${fmt(r.battery_percentage, 1)} %` : "--";
  $("tWind").textContent = r ? `${fmt(r.wind_speed, 2)} m/s` : "--";
  $("tCtx").textContent = r ? r.acmfds_context : "--";
  $("tDec").textContent = r ? r.acmfds_decision : "--";
  $("tDir").textContent = r ? (r.movement_action || r.selected_direction || "--") : "--";
  $("tTime").textContent = r ? `${fmt(r.timestamp, 1)} s` : "--";

  const sev = (el, ctx) => {
    el.classList.remove("green", "amber", "red");
    if (!ctx) return;
    const c = ctxColor(ctx);
    if (c === COL.red) el.classList.add("red");
    else if (c === COL.amber) el.classList.add("amber");
    else if (c === COL.green) el.classList.add("green");
  };
  sev($("tCtx"), r && r.acmfds_context);
  sev($("tDec"), r && r.acmfds_context);
  if (r) {
    const d = r.obstacle_distance;
    $("tObst").classList.remove("green", "amber", "red");
    if (d <= 0.8) $("tObst").classList.add("red");
    else if (d <= 2.0) $("tObst").classList.add("amber");
  }
}

/* Battery telemetry line: initial / current / consumed, from the same
 * logged rows the CSV writes (nothing synthesized). "pp" = percentage
 * points. While paused the numbers freeze with the physics. */
function renderBatteryDetail() {
  const el = $("battDetail");
  const rows = state.rows;
  if (!rows.length) {
    el.textContent = "battery: no flight yet — initial / current / consumed appear here";
    return;
  }
  const b0 = rows[0].battery_percentage;
  const b = rows[rows.length - 1].battery_percentage;
  el.textContent =
    `initial ${fmt(b0, 1)} %   current ${fmt(b, 1)} %   consumed ${(b0 - b).toFixed(2)} pp`
    + (state.status === "paused" ? "   [PAUSED — battery frozen with physics]" : "");
}

function renderDecision() {
  const r = state.rows.length ? state.rows[state.rows.length - 1] : null;
  const ctx = $("decCtx"), dec = $("decDec"), dir = $("decDir");
  if (!r) {
    ctx.textContent = "--"; dec.textContent = "--"; dir.textContent = "--";
    ctx.className = "badge"; dec.className = "badge"; dir.className = "badge dim";
    $("decReason").textContent = "Waiting for first decision";
    $("decCands").innerHTML = "";
    $("cmdLine").textContent = "";
    return;
  }
  // Actual movement action from the logged row: the directional pick when
  // ACMFDS selects one (LEFT/RIGHT/CLIMB), otherwise the decision in force
  // (MOVE_FORWARD / HOLD / SAFE_LANDING / ...). Never synthesized here.
  const action = r.movement_action || r.selected_direction || r.acmfds_decision;
  ctx.textContent = r.acmfds_context;
  ctx.className = "badge " + (ctxColor(r.acmfds_context) === COL.red ? "red"
    : ctxColor(r.acmfds_context) === COL.amber ? "amber"
    : ctxColor(r.acmfds_context) === COL.green ? "green" : "");
  // The ACMFDS decision is always shown separately in this badge; the
  // direction/action badge names the movement actually in force.
  dec.textContent = r.acmfds_decision;
  const dirTxt = r.selected_direction
    ? "direction: " + r.selected_direction + (r.selected_direction !== action
        ? " (action " + action + ")" : "")
    : "action: " + action;
  dir.textContent = state.status === "finished"
    ? "final action: " + action + "  [flight " + (state.outcome || "ended") + "]"
    : dirTxt;

  $("decReason").textContent = r.decision_reason;
  const parts = (r.candidates || "").split(" | ").filter(Boolean);
  $("decCands").innerHTML = parts.map((p) => {
    const [d, c, verdict] = p.split(":");
    const ok = verdict === "safe";
    return `<div class="cand ${ok ? "ok" : "rej"}"><span>${d}</span><span>${c}</span>` +
           `<span>${ok ? "accepted" : "rejected"}</span></div>`;
  }).join("");
  $("cmdLine").textContent =
    `executed command: fwd ${fmt(r.command_forward_speed)} m/s | lat ${fmt(r.command_lateral_speed)} m/s | ` +
    `alt ${fmt(r.command_target_altitude)} m | thrust ${fmt(r.thrust_n, 1)} N`;
}

function renderSummary() {
  const el = $("summary");
  const s = state.summary;
  if (!s) { el.textContent = "no completed run yet"; return; }
  const dec = Object.entries(s.decisions || {}).map(([k, v]) => `${k}=${v}`).join(", ");
  el.innerHTML =
    `<span class="k">outcome        </span> ${state.outcome} — ${state.reason}\n` +
    `<span class="k">mission done   </span> ${s.mission_completed}\n` +
    `<span class="k">landed         </span> ${s.landed ? "YES (touchdown confirmed)" : "NO"}\n` +
    `<span class="k">duration/steps </span> ${fmt(s.duration_s, 1)} s / ${s.steps}\n` +
    `<span class="k">min obst dist  </span> ${fmt(s.min_obstacle_distance, 3)} m\n` +
    `<span class="k">alt err mean/mx</span> ${fmt(s.mean_altitude_error, 3)} / ${fmt(s.max_altitude_error, 3)} m\n` +
    `<span class="k">battery end    </span> ${fmt(s.battery_end_percent, 1)} %\n` +
    `<span class="k">collisions     </span> ${s.collision_count}\n` +
    `<span class="k">emergency e/st </span> ${s.emergency_events} / ${s.emergency_steps} (entries/steps)\n` +
    `<span class="k">safe-land e/st </span> ${s.safe_landing_events} / ${s.safe_landing_steps} (entries/steps)\n` +
    `<span class="k">wind mean/max  </span> ${fmt(s.wind_mean)} / ${fmt(s.wind_max)} m/s\n` +
    `<span class="k">decisions      </span> ${dec || "--"}`;
}

function renderDeviceLoc() {
  const el = $("deviceLoc");
  const d = state.device;
  if (!d) {
    el.textContent = "DEVICE LOCATION: not requested (click the button; the browser will ask permission)";
    return;
  }
  if (d.error) {
    el.textContent = "DEVICE LOCATION: unavailable — " + d.error;
    return;
  }
  const live = state.watchId !== null ? "LIVE — " : "";
  const lowAcc = Number.isFinite(d.accuracy) && d.accuracy > 50
    ? "\nLOW ACCURACY FIX — treat these coordinates as approximate."
    : "";
  el.textContent =
    "DEVICE LOCATION (" + live + "A — operator device, browser geolocation; stays in this browser):\n" +
    `lat ${d.lat.toFixed(6)}, lon ${d.lon.toFixed(6)}, accuracy ±${fmt(d.accuracy, 0)} m\n` +
    `fix time ${d.time}   source: ${d.source}` + lowAcc;
}

function renderWorldPanel() {
  const r = state.rows.length ? state.rows[state.rows.length - 1] : null;
  if (!r) {
    for (const id of ["wX", "wY", "wZ", "wYaw", "wAtt", "wGoal"]) $(id).textContent = id.toUpperCase().slice(0, 1) + " --";
    $("wYaw").textContent = "heading --";
    $("wAtt").textContent = "attitude --";
    $("wGoal").textContent = "goal dist --";
    return;
  }
  const yawDeg = (r.yaw * 180 / Math.PI).toFixed(1);
  $("wX").textContent = `X ${fmt(r.position_x, 3)} m`;
  $("wY").textContent = `Y ${fmt(r.position_y, 3)} m`;
  $("wZ").textContent = `Z ${fmt(r.altitude, 3)} m`;
  $("wYaw").textContent = `heading ${yawDeg}\u00B0 (${fmt(r.yaw, 2)} rad)`;
  $("wAtt").textContent = `roll ${fmt(r.roll, 3)} rad, pitch ${fmt(r.pitch, 3)} rad`;
  $("wGoal").textContent = (r.goal_distance === undefined || r.goal_distance === null)
    ? `goal dist (x,y to ${fmt(state.config && state.config.goal_x, 1)},${fmt(state.config && state.config.goal_y, 1)}) --`
    : `goal dist ${fmt(r.goal_distance, 3)} m`;
}

/* Geographic conversion — the exact same flat-earth small-area transform the
 * server uses (telemetry/gps.py): X east, Y north, origin (ref) at world (0,0).
 * lat = lat0 + y/111320 ; lon = lon0 + x/(111320*cos(lat0)) */
const M_PER_DEG_LAT = 111320.0;
function toGeo(x, y, lat0, lon0) {
  return [lat0 + y / M_PER_DEG_LAT, lon0 + x / (M_PER_DEG_LAT * Math.cos(lat0 * Math.PI / 180))];
}

function renderGps() {
  const g = state.gps;
  const el = $("gpsValue");
  const on = $("gpsOn");
  if (document.activeElement !== on) on.checked = !!(g && g.enabled);
  const rl = $("gpsRefLat"), ro = $("gpsRefLon");
  if (document.activeElement !== rl) rl.value = g && g.ref_lat != null ? g.ref_lat : "";
  if (document.activeElement !== ro) ro.value = g && g.ref_lon != null ? g.ref_lon : "";
  renderOriginInfo();
  if (!g || !g.enabled) { el.textContent = "SIM GPS: Disabled"; return; }
  if (!g.origin_set) {
    el.textContent = "SIM GPS: Origin not set — set a geographic origin to see simulated coordinates";
    return;
  }
  if (g.lat === null || g.lat === undefined) {
    el.textContent = "SIM GPS: enabled — waiting for telemetry (start a flight)";
    return;
  }
  el.textContent =
    `SIM GPS (simulated): ${g.lat.toFixed(6)}, ${g.lon.toFixed(6)}\n` +
    `origin ${g.ref_lat.toFixed(6)}, ${g.ref_lon.toFixed(6)}  (world X=0, Y=0)\n` +
    `updated at t=${fmt(g.t, 1)} s (sim) — computed from the last logged physics row`;
}

function renderOriginInfo() {
  const g = state.gps;
  const el = $("originInfo");
  if (!g || !g.origin_set) {
    el.textContent = "GEOGRAPHIC ORIGIN: not set — simulated GPS and the geographic map stay off until an origin is configured.";
    return;
  }
  const ds = g.drone_start_world || [-4, 0];
  const [dslat, dslon] = toGeo(ds[0], ds[1], g.ref_lat, g.ref_lon);
  el.textContent =
    "GEOGRAPHIC ORIGIN: " + g.ref_lat.toFixed(6) + ", " + g.ref_lon.toFixed(6) +
    "  \u2194  world X=0.000, Y=0.000 m (X=east, Y=north)\n" +
    "drone start: world X=" + ds[0].toFixed(3) + ", Y=" + ds[1].toFixed(3) + " m  \u2194  " +
    dslat.toFixed(6) + ", " + dslon.toFixed(6) + "\n" +
    "Setting the origin never enables GPS by itself and never moves the drone.";
}

function render() {
  renderStatus();
  renderTiles();
  renderWorldPanel();
  renderDecision();
  renderSummary();
  renderGps();
  renderDeviceLoc();
  drawScene();
  renderGpsMap();
  renderBatteryDetail();
  // Battery: a fixed 0-100 axis makes a real ~8% draw look flat. Use an
  // honest auto-ranged axis (labeled with true values, min span 10%) plus a
  // dashed marker at the starting charge so consumption is visible without
  // exaggeration.
  lineChart("chBattery", [{ key: "battery_percentage", color: COL.green }],
            (rows) => {
              const vals = rows.map((r) => r.battery_percentage).filter((v) => v !== null && v !== undefined);
              if (!vals.length) return [0, 100];
              let lo = Math.max(0, Math.floor(Math.min(...vals) - 2));
              let hi = Math.min(100, Math.ceil(Math.max(...vals) + 2));
              if (hi - lo < 10) { hi = Math.min(100, lo + 10); lo = Math.max(0, hi - 10); }
              return [lo, hi];
            },
            state.rows.length
              ? { y: state.rows[0].battery_percentage,
                  label: "start " + fmt(state.rows[0].battery_percentage, 1) + "%",
                  color: COL.muted }
              : null,
            "Simulation time (s)", "Battery (%)");
  lineChart("chAlt", [
    { key: "altitude", color: COL.cyan },
    { key: "target_altitude", color: COL.muted, dash: true },
  ], (rows) => {
    const peak = Math.max(...rows.map((r) => r.altitude || 0), 4.6);
    return [0, Math.ceil(peak * 2) / 2 + 0.2];
  }, null, "Simulation time (s)", "Altitude (m)");
  $("altLegend").innerHTML =
    `<span class="lg"><i style="background:${COL.cyan}"></i>altitude (actual, logged)</span>` +
    `<span class="lg"><i style="background:transparent;border-top:2px dashed ${COL.muted};height:0"></i>target altitude (commanded)</span>`;
  pathChart();
  lineChart("chVel", [
    { key: "horizontal_speed", color: COL.cyan },
    { key: "velocity_z", color: COL.amber },
  ], (rows) => {
    const peak = Math.max(0.4, ...rows.map((r) => Math.abs(r.horizontal_speed || 0)));
    return [-peak - 0.1, peak + 0.1];
  }, null, "Simulation time (s)", "Velocity (m/s)");
  $("velLegend").innerHTML =
    `<span class="lg"><i style="background:${COL.cyan}"></i>horizontal speed</span>` +
    `<span class="lg"><i style="background:${COL.amber}"></i>vertical speed (velocity_z)</span>`;
  $("obstLegend").innerHTML =
    `<span class="lg"><i style="background:${COL.cyan}"></i>nearest ray (FWD/LEFT/RIGHT min)</span>` +
    `<span class="lg"><i style="background:transparent;border-top:2px dashed ${COL.muted};height:0"></i>forward ray</span>` +
    `<span class="lg">flat 5.00 m = no return within the 5.0 m sensor range (cap, valid data)</span>` +
    `<span class="lg">jumps = ray entering/leaving range or sweeping off a body mid-turn</span>`;
  lineChart("chObst", [
    { key: "obstacle_distance", color: COL.cyan },
    { key: "clearance_front", color: COL.muted, dash: true },
  ], () => [0, 5.2], { y: 5.0, label: "5.0 m sensor range cap (no return)" },
     "Simulation time (s)", "Distance (m)");
  lineChart("chWind", [{ key: "wind_speed", color: COL.amber }], (rows) => {
    const peak = Math.max(4, ...rows.map((r) => r.wind_speed || 0));
    return [0, Math.ceil(peak) + 0.5];
  }, null, "Simulation time (s)", "Wind speed (m/s)");
  timelineChart();
}

/* -------------------------------------------------------------- controls */
function wireControls() {
  $("btnStart").onclick = async () => {
    $("cfgError").textContent = "";
    try { await api("/api/start", collectConfig()); }
    catch (e) { $("cfgError").textContent = e.message; }
  };
  $("btnPause").onclick = () => api("/api/pause").catch((e) => ($("cfgError").textContent = e.message));
  $("btnResume").onclick = () => api("/api/resume").catch((e) => ($("cfgError").textContent = e.message));
  $("btnStop").onclick = () => api("/api/stop").catch((e) => ($("cfgError").textContent = e.message));
  $("btnReset").onclick = () => api("/api/reset").catch((e) => ($("cfgError").textContent = e.message));
  $("cfgMoving").onchange = () =>
    api("/api/moving", { enabled: $("cfgMoving").checked })
      .catch((e) => ($("cfgError").textContent = e.message));

  // Decision timeline hover: cursor line + tooltip, both driven by the SAME
  // logged rows the chart and the CSV use (no separate data source).
  const tlCanvas = $("chTimeline");
  const tlTip = $("tlTip");
  tlCanvas.addEventListener("mousemove", (ev) => {
    const rows = state.rows;
    if (!rows.length) return;
    const { t0, t1, padL, padR, w } = timelineGeom;
    const gx = ev.offsetX;
    if (!w || gx < padL || gx > w - padR) {
      timelineHover = null;
      tlTip.style.display = "none";
      timelineChart();
      return;
    }
    timelineHover = { x: gx };
    const t = t0 + (gx - padL) / (w - padL - padR) * (t1 - t0);
    // nearest logged row (rows are near-uniform at the control timestep)
    let idx = Math.round((t - t0) / ((t1 - t0) / (rows.length - 1 || 1)));
    idx = Math.max(0, Math.min(rows.length - 1, idx));
    for (const j of [idx - 1, idx + 1]) {
      if (j >= 0 && j < rows.length &&
          Math.abs(rows[j].timestamp - t) < Math.abs(rows[idx].timestamp - t)) idx = j;
    }
    const r = rows[idx];
    tlTip.innerHTML =
      `<b>t = ${fmt(r.timestamp, 3)} s</b>\n` +
      `state: <span style="color:${ctxColor(r.acmfds_context)}">${r.acmfds_context}</span>\n` +
      `decision: ${r.acmfds_decision}\n` +
      `direction: ${r.selected_direction || "—"}\n` +
      `reason: ${r.decision_reason}`;
    tlTip.style.display = "block";
    const card = tlCanvas.parentElement;
    const left = tlCanvas.offsetLeft + gx + 16;
    tlTip.style.left = Math.max(4, Math.min(left, card.clientWidth - tlTip.offsetWidth - 4)) + "px";
    tlTip.style.top = Math.max(4, tlCanvas.offsetTop + ev.offsetY - tlTip.offsetHeight - 8) + "px";
    timelineChart();
  });
  tlCanvas.addEventListener("mouseleave", () => {
    timelineHover = null;
    tlTip.style.display = "none";
    timelineChart();
  });

  // Simulated-GPS origin + enable are INDEPENDENT: posting settings never
  // starts or touches a flight, and setting the origin never enables GPS.
  const postGps = () => {
    const latRaw = $("gpsRefLat").value.trim();
    const lonRaw = $("gpsRefLon").value.trim();
    if ((latRaw === "") !== (lonRaw === "")) {
      $("gpsNote").textContent = "Origin needs BOTH latitude and longitude (or clear both to unset the origin).";
      return;
    }
    const body = {
      enabled: $("gpsOn").checked,
      ref_lat: latRaw === "" ? null : parseFloat(latRaw),
      ref_lon: lonRaw === "" ? null : parseFloat(lonRaw),
    };
    if (body.ref_lat !== null && !Number.isFinite(body.ref_lat)) {
      $("gpsNote").textContent = "Origin latitude is not a valid number.";
      return;
    }
    if (body.ref_lon !== null && !Number.isFinite(body.ref_lon)) {
      $("gpsNote").textContent = "Origin longitude is not a valid number.";
      return;
    }
    return api("/api/gps", body)
      .then(() => {
        $("gpsNote").textContent = body.ref_lat === null
          ? "Origin unset. Simulated GPS and the geographic map are off."
          : "Origin set from the fields. Enable simulated GPS separately with the checkbox if desired.";
      })
      .catch((e) => ($("gpsNote").textContent = "GPS setup rejected: " + e.message));
  };
  $("gpsOn").onchange = postGps;
  $("btnGpsApply").onclick = () => { state.originSource = "manual"; postGps(); };
  // Operator device location (concept A): requested only by an explicit
  // click, displayed only in the browser, never sent to the server or any
  // external service, and never used by the simulation.
  const geoErrorText = (err) =>
    err.code === 1 ? "permission denied — allow location for this site (address bar) and click again"
    : err.code === 2 ? "position unavailable (no location provider on this device/network)"
    : err.code === 3 ? "timed out before a fix was found"
    : err.message;
  const showFix = (pos, source) => {
    state.device = {
      lat: pos.coords.latitude,
      lon: pos.coords.longitude,
      accuracy: pos.coords.accuracy,
      time: new Date(pos.timestamp).toLocaleTimeString(),
      source,
    };
    $("btnDeviceAsRef").disabled = false;
    $("gpsNote").textContent =
      "Device fix shown above (concept A — operator device). It is NOT the drone's position " +
      "and did not touch the simulation. 'Use device fix as origin' copies it into the " +
      "geographic origin (concept B) only if you click that explicitly.";
    renderDeviceLoc();
  };
  $("btnMyLocation").onclick = () => {
    if (!navigator.geolocation) {
      state.device = { error: "this browser does not support geolocation" };
      renderDeviceLoc();
      return;
    }
    $("gpsNote").textContent =
      "Requesting device location — approve or deny in the browser prompt. " +
      "The fix is displayed only; it is not sent anywhere and does not affect the drone.";
    navigator.geolocation.getCurrentPosition(
      (pos) => showFix(pos, "operator device (browser geolocation, one-shot fix)"),
      (err) => {
        state.device = { error: geoErrorText(err) };
        $("gpsNote").textContent = "Device location request failed — see the DEVICE LOCATION box.";
        renderDeviceLoc();
      },
      { enableHighAccuracy: true, timeout: 10000, maximumAge: 30000 }
    );
  };
  // Continuous device tracking (concept A, optional): explicit start, clear
  // stop, positions stay in the browser exactly like the one-shot fix.
  $("btnTrack").onclick = () => {
    if (!navigator.geolocation) {
      state.device = { error: "this browser does not support geolocation" };
      renderDeviceLoc();
      return;
    }
    if (state.watchId !== null) return;
    state.watchId = navigator.geolocation.watchPosition(
      (pos) => showFix(pos, "operator device (browser geolocation, continuous watch)"),
      (err) => {
        state.device = { error: geoErrorText(err) };
        stopTracking();
        $("gpsNote").textContent = "Continuous tracking failed — see the DEVICE LOCATION box.";
        renderDeviceLoc();
      },
      { enableHighAccuracy: true, timeout: 15000, maximumAge: 10000 }
    );
    $("btnTrack").style.display = "none";
    $("btnStopTrack").style.display = "";
    $("gpsNote").textContent = "Tracking device continuously — the fixes stay in this browser. Click Stop tracking to end.";
    renderDeviceLoc();
  };
  const stopTracking = () => {
    if (state.watchId !== null && navigator.geolocation) {
      navigator.geolocation.clearWatch(state.watchId);
    }
    state.watchId = null;
    $("btnTrack").style.display = "";
    $("btnStopTrack").style.display = "none";
    renderDeviceLoc();
  };
  $("btnStopTrack").onclick = stopTracking;
  // Concept B: use the device fix as the geographic ORIGIN. This only fills
  // the origin fields — it does NOT enable simulated GPS (the checkbox stays
  // as the user left it) and never touches the running simulation.
  $("btnDeviceAsRef").onclick = () => {
    const d = state.device;
    if (!d || d.error) return;
    $("gpsRefLat").value = d.lat.toFixed(6);
    $("gpsRefLon").value = d.lon.toFixed(6);
    state.originSource = "device";
    postGps();
  };
}

async function loadScenarios() {
  try {
    const res = await fetch("/api/scenarios");
    const list = await res.json();
    const sel = $("cfgScenario");
    sel.innerHTML = list.map((s) => `<option value="${s.name}">${s.name}</option>`).join("");
    const showDesc = () => {
      const sc = list.find((s) => s.name === sel.value);
      $("scenarioDesc").textContent = sc ? sc.description : "";
    };
    sel.onchange = showDesc;
    sel.value = "normal";
    showDesc();
  } catch (e) { /* server not reachable yet; WS retry will surface it */ }
}

wireControls();
loadScenarios();
connect();
