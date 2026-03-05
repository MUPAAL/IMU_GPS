/**
 * nav_visualizer.js
 * Navigation console frontend for 03_Nav module.
 *
 * Connects to nav_bridge WebSocket, renders:
 *   - Leaflet map: robot position, heading arrow, waypoints, track history
 *   - Command output panel (V command, nav state)
 *   - RTK, IMU, Robot, Fused state panels
 */

'use strict';

// ── Config ────────────────────────────────────────────────
const DEFAULT_POS = [38.9412928598587, -92.31884600793728];
const WS_URL = `ws://${window.location.hostname}:${Number(window.location.port || 8785) + 1}`;
const TRACK_MAX = 2000;      // max track history points
const RECONNECT_MS = 3000;

// ── Map setup ─────────────────────────────────────────────
const map = L.map('map', { zoomControl: true });

// Tile layers
const offlineTile = L.tileLayer('./assets/tiles/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: 'Offline tiles',
  errorTileUrl: '',
});

const esriSat = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 19, attribution: 'Esri Satellite' }
);

const osmLayer = L.tileLayer(
  'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  { maxZoom: 19, attribution: '© OpenStreetMap contributors' }
);

// Default to satellite
esriSat.addTo(map);

L.control.layers(
  { 'Offline (LAN)': offlineTile, 'Satellite (Esri)': esriSat, 'Street (OSM)': osmLayer },
  {},
  { position: 'bottomleft' }
).addTo(map);

map.setView(DEFAULT_POS, 18);

// ── Map layers ────────────────────────────────────────────
const trackPoints = [];
const trackLine = L.polyline([], { color: '#2ecc71', weight: 3, opacity: 0.75 }).addTo(map);

// Robot position marker (blue circle + heading arrow)
const robotIcon = L.divIcon({
  className: '',
  iconSize: [30, 30],
  iconAnchor: [15, 15],
  html: `<div id="robotMarkerDiv" style="width:30px;height:30px;position:relative;">
    <div style="
      position:absolute;top:5px;left:5px;
      width:20px;height:20px;border-radius:50%;
      background:#1b6c8d;border:3px solid #fff;
      box-shadow:0 0 6px rgba(0,0,0,0.4);
    "></div>
    <div id="headingArrow" style="
      position:absolute;top:-4px;left:13px;
      width:0;height:0;
      border-left:4px solid transparent;
      border-right:4px solid transparent;
      border-bottom:14px solid #e74c3c;
      transform-origin:4px 18px;
      transform:rotate(0deg);
    "></div>
  </div>`,
});

const robotMarker = L.marker(DEFAULT_POS, { icon: robotIcon, zIndexOffset: 1000 }).addTo(map);
let mapCentered = false;

// Waypoints layer
const waypointMarkers = [];
let waypointPolyline = L.polyline([], {
  color: '#3498db', weight: 2, dashArray: '6 6', opacity: 0.8
}).addTo(map);

// ── State ─────────────────────────────────────────────────
let ws = null;
let reconnectTimer = null;
let editMode = false;
let waypoints = [];         // [{lat, lon}, ...]
let currentWpIndex = 0;

// ── WebSocket ─────────────────────────────────────────────
function connect() {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;

  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    clearTimeout(reconnectTimer);
    setStatus(true);
    logEvent(`WebSocket connected: ${WS_URL}`);
  };

  ws.onmessage = (ev) => {
    try {
      const data = JSON.parse(ev.data);
      handleFrame(data);
    } catch (e) {
      console.warn('WS parse error:', e);
    }
  };

  ws.onclose = () => {
    setStatus(false);
    logEvent(`Disconnected, reconnecting in ${RECONNECT_MS / 1000}s…`);
    reconnectTimer = setTimeout(connect, RECONNECT_MS);
  };

  ws.onerror = (e) => {
    console.error('WS error:', e);
  };
}

function sendMsg(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// ── Frame handler ─────────────────────────────────────────
function handleFrame(d) {
  const state = d.state || {};
  const nav   = d.nav   || {};
  const cmd   = d.cmd   || {};
  const robot = d.robot || {};
  const imu   = d.imu_raw || {};
  const rtk   = d.rtk_raw || {};
  const src   = d.source || {};

  // Update source badges
  updateBadge('badgeImu',   src.imu,   'IMU');
  updateBadge('badgeRtk',   src.rtk,   'RTK');
  updateBadge('badgeRobot', src.robot, 'Robot');

  // Update map
  if (state.lat != null && state.lon != null) {
    const pos = [state.lat, state.lon];
    robotMarker.setLatLng(pos);

    trackPoints.push(pos);
    if (trackPoints.length > TRACK_MAX) trackPoints.shift();
    trackLine.setLatLngs(trackPoints);

    if (!mapCentered) {
      map.setView(pos, 18);
      mapCentered = true;
    }
  }

  // Update heading arrow
  if (state.heading_deg != null) {
    const arrow = document.getElementById('headingArrow');
    if (arrow) arrow.style.transform = `rotate(${state.heading_deg}deg)`;
  }

  // Update waypoint highlight
  currentWpIndex = nav.waypoint_index || 0;
  updateWaypointStyles();

  // Server-side waypoints (keep in sync if changed)
  if (nav.waypoints && nav.waypoints.length !== waypoints.length) {
    waypoints = nav.waypoints;
    renderWaypoints();
  }

  // ── Command panel ──────────────────────────────────────
  setText('vCmdDisplay', (cmd.v_cmd || 'V0.00,0.000').replace('\n', ''));
  setNavStateBadge(cmd.nav_state || 'IDLE');
  setText('cmdSpeed',    fmt(cmd.speed_ms, 3) + ' m/s');
  setText('cmdAngRate',  fmt(cmd.ang_rate_rads, 4) + ' rad/s');
  setText('cmdDist',     cmd.distance_m != null ? fmt(cmd.distance_m, 1) + ' m' : '—');
  setText('cmdHErr',     cmd.heading_error_deg != null ? fmt(cmd.heading_error_deg, 1) + '°' : '—');
  setText('cmdBearing',  nav.bearing_deg != null ? fmt(nav.bearing_deg, 1) + '°' : '—');
  setText('cmdWp',
    (cmd.total_waypoints > 0)
      ? `${cmd.target_waypoint + 1} / ${cmd.total_waypoints}`
      : '— / —'
  );

  // ── RTK panel ──────────────────────────────────────────
  setText('rtkLat',    rtk.lat != null ? fmt(rtk.lat, 8) + '°' : '—');
  setText('rtkLon',    rtk.lon != null ? fmt(rtk.lon, 8) + '°' : '—');
  setText('rtkAlt',    rtk.alt != null ? fmt(rtk.alt, 2) + ' m' : '—');
  setText('rtkFix',    fixLabel(rtk.fix_quality));
  setText('rtkSats',   rtk.num_sats != null ? rtk.num_sats : '—');
  setText('rtkHdop',   rtk.hdop != null ? fmt(rtk.hdop, 2) : '—');
  setText('rtkSpeed',  rtk.speed_ms != null
    ? `${fmt(rtk.speed_ms, 2)} m/s (${fmt(rtk.speed_knots, 2)} kt)` : '—');
  setText('rtkTrack',  rtk.track_deg != null ? fmt(rtk.track_deg, 1) + '°' : '—');
  setText('rtkSource', rtk.source || '—');

  // ── IMU panel ──────────────────────────────────────────
  const euler = imu.euler || {};
  const accel = imu.lin_accel || {};
  const gyro  = imu.gyro || {};
  const mag   = imu.mag || {};

  updateCalBadge(imu.cal);
  setText('imuYaw',   euler.yaw   != null ? fmt(euler.yaw, 1)   + '°' : '—');
  setText('imuPitch', euler.pitch != null ? fmt(euler.pitch, 1) + '°' : '—');
  setText('imuRoll',  euler.roll  != null ? fmt(euler.roll, 1)  + '°' : '—');
  setText('imuAccel', accelStr(accel));
  setText('imuGyro',  gyroStr(gyro));
  setText('imuMag',   magStr(mag));
  setText('imuHz',    imu.hz != null ? imu.hz + ' Hz' : '—');

  // ── Robot panel ────────────────────────────────────────
  updateRobotState(robot.state_name || 'UNKNOWN');
  setText('robotSpeed',   robot.meas_speed_ms   != null ? fmt(robot.meas_speed_ms, 3)   + ' m/s'   : '—');
  setText('robotAngRate', robot.meas_ang_rate_rads != null ? fmt(robot.meas_ang_rate_rads, 4) + ' rad/s' : '—');

  const soc = robot.soc != null ? robot.soc : null;
  setText('robotSoc',  soc != null ? soc + '%' : '—');
  const bar = document.getElementById('robotSocBar');
  if (bar && soc != null) {
    bar.style.width = Math.max(0, Math.min(100, soc)) + '%';
    bar.style.background = soc < 20 ? '#c23027' : soc < 50 ? '#b07010' : '#1b8c42';
  }

  const conn = robot.connected;
  setText('robotConn', conn ? '● Connected' : '○ Disconnected');
  el('robotConn').style.color = conn ? '#1b8c42' : '#c23027';

  // ── Fused state ────────────────────────────────────────
  setText('fusedHeading', state.heading_deg != null ? fmt(state.heading_deg, 1) + '°' : '—');
  setText('fusedSpeed',   state.speed_ms   != null ? fmt(state.speed_ms, 2) + ' m/s' : '—');
  setText('fusedMotion',  state.motion || '—');
}

// ── Map interaction ───────────────────────────────────────

map.on('click', (e) => {
  if (!editMode) return;
  const wp = { lat: e.latlng.lat, lon: e.latlng.lng };
  waypoints.push(wp);
  renderWaypoints();
  sendMsg({ type: 'set_waypoints', waypoints });
  logEvent(`Added waypoint ${waypoints.length}: ${fmt(wp.lat, 6)}, ${fmt(wp.lon, 6)}`);
});

function renderWaypoints() {
  // Remove old markers
  waypointMarkers.forEach(m => map.removeLayer(m));
  waypointMarkers.length = 0;

  waypoints.forEach((wp, i) => {
    const isTarget = i === currentWpIndex;
    const icon = L.divIcon({
      className: '',
      iconSize: [24, 24],
      iconAnchor: [12, 12],
      html: `<div style="
        width:24px;height:24px;border-radius:50%;
        background:${isTarget ? '#e74c3c' : '#3498db'};
        border:2px solid #fff;
        display:flex;align-items:center;justify-content:center;
        color:#fff;font-size:10px;font-weight:700;
        box-shadow:0 0 4px rgba(0,0,0,0.4);
      ">${i + 1}</div>`,
    });
    const m = L.marker([wp.lat, wp.lon], { icon, draggable: editMode })
      .addTo(map)
      .bindPopup(`WP ${i + 1}: ${fmt(wp.lat, 6)}, ${fmt(wp.lon, 6)}`);
    m.on('dragend', () => {
      const ll = m.getLatLng();
      waypoints[i] = { lat: ll.lat, lon: ll.lng };
      renderWaypoints();
      sendMsg({ type: 'set_waypoints', waypoints });
    });
    waypointMarkers.push(m);
  });

  // Update path line
  const lls = waypoints.map(w => [w.lat, w.lon]);
  waypointPolyline.setLatLngs(lls);
}

function updateWaypointStyles() {
  waypointMarkers.forEach((m, i) => {
    const isTarget = i === currentWpIndex;
    const div = m.getElement()?.querySelector('div');
    if (div) div.style.background = isTarget ? '#e74c3c' : '#3498db';
  });
}

// ── Controls ──────────────────────────────────────────────

el('btnStartNav').addEventListener('click', () => {
  sendMsg({ type: 'start_nav' });
  logEvent('Navigation started');
});

el('btnStopNav').addEventListener('click', () => {
  sendMsg({ type: 'stop_nav' });
  logEvent('Navigation stopped');
});

el('btnClearWp').addEventListener('click', () => {
  waypoints = [];
  renderWaypoints();
  sendMsg({ type: 'clear_waypoints' });
  logEvent('Waypoints cleared');
});

el('btnEditRoute').addEventListener('click', () => {
  editMode = !editMode;
  el('btnEditRoute').classList.toggle('active', editMode);
  el('btnEditRoute').textContent = editMode ? '✓ Done Editing' : 'Edit Route';
  map.getContainer().style.cursor = editMode ? 'crosshair' : '';
  // Re-render with drag enabled/disabled
  renderWaypoints();
  logEvent(editMode ? 'Edit mode on: click map to add waypoints' : 'Edit mode off');
});

el('btnFindMe').addEventListener('click', () => {
  if (!navigator.geolocation) {
    logEvent('Geolocation not supported');
    return;
  }
  navigator.geolocation.getCurrentPosition(
    (pos) => {
      const ll = [pos.coords.latitude, pos.coords.longitude];
      map.setView(ll, 18);
      logEvent(`Located: ${fmt(ll[0], 6)}, ${fmt(ll[1], 6)}`);
    },
    (err) => logEvent(`Locate failed: ${err.message}`)
  );
});

el('btnCenterCurrent').addEventListener('click', () => {
  const ll = robotMarker.getLatLng();
  map.setView(ll, map.getZoom());
  logEvent(`Centered to robot: ${fmt(ll.lat, 6)}, ${fmt(ll.lng, 6)}`);
});

el('btnExportRoute').addEventListener('click', () => {
  if (waypoints.length === 0) { logEvent('No waypoints to export'); return; }
  const csv = 'lat,lon\n' + waypoints.map(w => `${w.lat},${w.lon}`).join('\n');
  downloadText('route.csv', csv);
  logEvent(`Exported ${waypoints.length} waypoints`);
});

el('btnClearTrack').addEventListener('click', () => {
  trackPoints.length = 0;
  trackLine.setLatLngs([]);
  logEvent('Track cleared');
});

el('csvFile').addEventListener('change', (ev) => {
  const file = ev.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (e) => {
    try {
      const loaded = parseCsv(e.target.result);
      if (loaded.length === 0) { logEvent('CSV: no valid points found'); return; }
      waypoints = loaded;
      renderWaypoints();
      sendMsg({ type: 'set_waypoints', waypoints });
      logEvent(`Loaded ${waypoints.length} waypoints from CSV`);
      if (waypoints.length > 0) map.setView([waypoints[0].lat, waypoints[0].lon], 18);
    } catch (err) {
      logEvent(`CSV load failed: ${err.message}`);
    }
  };
  reader.readAsText(file);
  ev.target.value = '';
});

// ── Helpers ───────────────────────────────────────────────

function el(id) { return document.getElementById(id); }
function setText(id, val) {
  const e = document.getElementById(id);
  if (e) e.textContent = val;
}

function fmt(v, decimals) {
  if (v == null || isNaN(v)) return '—';
  return Number(v).toFixed(decimals);
}

function accelStr(a) {
  if (!a || a.x == null) return '—';
  return `${fmt(a.x, 2)} / ${fmt(a.y, 2)} / ${fmt(a.z, 2)} m/s²`;
}

function gyroStr(g) {
  if (!g || g.x == null) return '—';
  return `${fmt(g.x, 3)} / ${fmt(g.y, 3)} / ${fmt(g.z, 3)} rad/s`;
}

function magStr(m) {
  if (!m || m.x == null) return '—';
  return `${fmt(m.x, 1)} / ${fmt(m.y, 1)} / ${fmt(m.z, 1)} µT`;
}

function fixLabel(q) {
  const labels = { 0: 'No Fix', 1: 'GPS', 2: 'DGPS', 4: 'RTK Fixed', 5: 'RTK Float' };
  return labels[q] || `Fix(${q})`;
}

function updateBadge(id, ok, label) {
  const e = document.getElementById(id);
  if (!e) return;
  e.textContent = `${label} ${ok ? '✓' : '✗'}`;
  e.className = 'badge ' + (ok ? 'ok' : 'bad');
}

function setStatus(connected) {
  const dot = document.getElementById('statusDot');
  const txt = document.getElementById('statusText');
  if (dot) dot.className = 'dot' + (connected ? ' ok' : '');
  if (txt) txt.textContent = connected ? 'Connected' : 'Disconnected';
}

function setNavStateBadge(state) {
  const e = document.getElementById('navStateBadge');
  if (!e) return;
  e.textContent = state;
  e.className = 'nav-state-badge ' + state;
}

function updateCalBadge(cal) {
  const e = document.getElementById('imuCal');
  if (!e) return;
  const n = cal != null ? cal : '—';
  e.textContent = cal != null ? `${cal} / 3` : '—';
  e.className = 'cal-val ' + (cal != null ? `cal-${Math.min(3, Math.max(0, cal))}` : '');
}

function updateRobotState(stateName) {
  const e = document.getElementById('robotState');
  if (!e) return;
  e.textContent = stateName;
  if (stateName === 'AUTO_ACTIVE') {
    e.className = 'state-badge state-active';
  } else if (stateName === 'AUTO_READY') {
    e.className = 'state-badge state-ready';
  } else {
    e.className = 'state-badge state-unknown';
  }
}

function logEvent(msg) {
  const list = document.getElementById('eventList');
  if (!list) return;
  const ts = new Date().toLocaleTimeString();
  const li = document.createElement('li');
  li.textContent = `[${ts}] ${msg}`;
  list.prepend(li);
  // Keep max 50 events
  while (list.children.length > 50) list.removeChild(list.lastChild);
}

function parseCsv(text) {
  const lines = text.split('\n').map(l => l.trim()).filter(Boolean);
  if (lines.length < 2) return [];
  const header = lines[0].toLowerCase().split(',').map(h => h.trim());
  const latIdx = header.indexOf('lat');
  const lonIdx = header.indexOf('lon');
  if (latIdx === -1 || lonIdx === -1) throw new Error('CSV missing lat/lon columns');
  const points = [];
  for (let i = 1; i < lines.length; i++) {
    const cols = lines[i].split(',');
    const lat = parseFloat(cols[latIdx]);
    const lon = parseFloat(cols[lonIdx]);
    if (!isNaN(lat) && !isNaN(lon)) points.push({ lat, lon });
  }
  return points;
}

function downloadText(filename, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' }));
  a.download = filename;
  a.click();
  URL.revokeObjectURL(a.href);
}

// ── Boot ──────────────────────────────────────────────────
logEvent('Nav Console started');
setStatus(false);
connect();
