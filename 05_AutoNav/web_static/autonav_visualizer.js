/**
 * autonav_visualizer.js — Leaflet map + WebSocket client for the
 * AutoNav Controller dashboard.
 *
 * Features:
 *   - Live robot position on map
 *   - Waypoint markers with planned path polyline
 *   - Click-to-add boundary points for coverage planner
 *   - Navigation state panel updates
 *   - CSV waypoint upload (drag-drop or file picker)
 */

// ── Config ───────────────────────────────────────────────────────────────────

const DEFAULT_LAT = 38.9413;
const DEFAULT_LON = -92.3188;
const DEFAULT_ZOOM = 18;

// ── DOM ──────────────────────────────────────────────────────────────────────

const statusDot     = document.getElementById("statusDot");
const statusText    = document.getElementById("statusText");
const navState      = document.getElementById("navState");
const navProgress   = document.getElementById("navProgress");
const navDistance    = document.getElementById("navDistance");
const navBearingErr = document.getElementById("navBearingErr");
const navFixQuality = document.getElementById("navFixQuality");
const selNavMode    = document.getElementById("selNavMode");
const selFilterMode = document.getElementById("selFilterMode");
const ctrlLinear    = document.getElementById("ctrlLinear");
const ctrlAngular   = document.getElementById("ctrlAngular");
const csvUpload     = document.getElementById("csvUpload");
const btnFindMe     = document.getElementById("btnFindMe");
const btnStartNav   = document.getElementById("btnStartNav");
const btnStopNav    = document.getElementById("btnStopNav");
const btnGenCoverage = document.getElementById("btnGenCoverage");

// ── Leaflet Map ──────────────────────────────────────────────────────────────

const map = L.map("map").setView([DEFAULT_LAT, DEFAULT_LON], DEFAULT_ZOOM);

L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 22,
  attribution: "&copy; OpenStreetMap",
}).addTo(map);

// Robot marker (blue circle)
const robotMarker = L.circleMarker([DEFAULT_LAT, DEFAULT_LON], {
  radius: 8, fillColor: "#2563eb", fillOpacity: 0.9,
  color: "#fff", weight: 2,
}).addTo(map);

// Waypoint markers + path
let waypointMarkers = [];
let pathLine = null;

// Boundary points for coverage planner
let boundaryMarkers = [];
let boundaryLine = null;
let boundaryMode = false;

function clearWaypoints() {
  waypointMarkers.forEach(m => map.removeLayer(m));
  waypointMarkers = [];
  if (pathLine) { map.removeLayer(pathLine); pathLine = null; }
}

function drawWaypoints(waypoints) {
  // waypoints not directly available from status; drawn when CSV uploaded
}

function clearBoundary() {
  boundaryMarkers.forEach(m => map.removeLayer(m));
  boundaryMarkers = [];
  if (boundaryLine) { map.removeLayer(boundaryLine); boundaryLine = null; }
}

// Click map to add boundary points
map.on("click", (e) => {
  const marker = L.circleMarker(e.latlng, {
    radius: 6, fillColor: "#e53935", fillOpacity: 0.8,
    color: "#fff", weight: 1,
  }).addTo(map);
  boundaryMarkers.push(marker);

  // Redraw boundary polygon
  if (boundaryLine) map.removeLayer(boundaryLine);
  if (boundaryMarkers.length >= 2) {
    const pts = boundaryMarkers.map(m => m.getLatLng());
    pts.push(pts[0]); // close polygon
    boundaryLine = L.polyline(pts, { color: "#e53935", weight: 2, dashArray: "5,5" }).addTo(map);
  }
});

// ── WebSocket ────────────────────────────────────────────────────────────────

const WS_PORT = parseInt(window.location.port, 10) + 1;
const WS_URL  = `ws://${window.location.hostname}:${WS_PORT}`;

let ws = null;
let reconnectTimer = null;
let lastRobotPos = null;

function connect() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    statusDot.classList.add("connected");
    statusText.textContent = "Connected";
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  };

  ws.onclose = () => {
    statusDot.classList.remove("connected");
    statusText.textContent = "Disconnected";
    scheduleReconnect();
  };

  ws.onerror = () => { ws.close(); };

  ws.onmessage = (evt) => {
    try {
      updateUI(JSON.parse(evt.data));
    } catch (e) {
      console.warn("Parse error:", e);
    }
  };
}

function scheduleReconnect() {
  if (!reconnectTimer) {
    reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, 2000);
  }
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(msg));
  }
}

// ── UI Update ────────────────────────────────────────────────────────────────

function updateUI(data) {
  // State
  const state = data.state || "idle";
  navState.textContent = state;
  navState.className = `state-${state}`;

  // Progress
  const prog = data.progress || [0, 0];
  navProgress.textContent = `${prog[0]}/${prog[1]}`;

  // Distance
  navDistance.textContent = data.distance_m != null
    ? `${data.distance_m.toFixed(1)} m` : "—";

  // Bearing error
  navBearingErr.textContent = data.bearing_error != null
    ? `${data.bearing_error > 0 ? "+" : ""}${data.bearing_error.toFixed(1)}°` : "—";

  // GPS quality
  navFixQuality.textContent = data.fix_quality;

  // Control output
  ctrlLinear.textContent = data.linear_cmd.toFixed(3);
  ctrlAngular.textContent = data.angular_cmd.toFixed(3);

  // Mode selects (reflect server state)
  if (selNavMode.value !== data.nav_mode) selNavMode.value = data.nav_mode;
  if (selFilterMode.value !== data.filter_mode) selFilterMode.value = data.filter_mode;

  // Robot position on map
  if (data.position) {
    const lat = data.position.lat;
    const lon = data.position.lon;
    robotMarker.setLatLng([lat, lon]);
    lastRobotPos = [lat, lon];
  }
}

// ── Controls ─────────────────────────────────────────────────────────────────

btnStartNav.addEventListener("click", () => {
  send({ type: "start_nav" });
});

btnStopNav.addEventListener("click", () => {
  send({ type: "stop_nav" });
});

btnFindMe.addEventListener("click", () => {
  if (lastRobotPos) {
    map.setView(lastRobotPos, 18);
  }
});

selNavMode.addEventListener("change", () => {
  send({ type: "set_nav_mode", mode: selNavMode.value });
});

selFilterMode.addEventListener("change", () => {
  send({ type: "set_filter_mode", mode: selFilterMode.value });
});

// CSV upload
csvUpload.addEventListener("change", (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (evt) => {
    const csvText = evt.target.result;
    send({ type: "load_waypoints", csv: csvText });
    displayWaypointsFromCSV(csvText);
  };
  reader.readAsText(file);
  e.target.value = ""; // reset for re-upload
});

function displayWaypointsFromCSV(csvText) {
  clearWaypoints();
  const lines = csvText.trim().split("\n");
  const coords = [];
  for (let i = 1; i < lines.length; i++) {
    const parts = lines[i].split(",");
    if (parts.length >= 3) {
      const lat = parseFloat(parts[1]);
      const lon = parseFloat(parts[2]);
      if (!isNaN(lat) && !isNaN(lon)) {
        coords.push([lat, lon]);
        const m = L.circleMarker([lat, lon], {
          radius: 5, fillColor: "#f59e0b", fillOpacity: 0.9,
          color: "#fff", weight: 1,
        }).bindTooltip(`WP ${parts[0]}`).addTo(map);
        waypointMarkers.push(m);
      }
    }
  }
  if (coords.length > 0) {
    pathLine = L.polyline(coords, { color: "#f59e0b", weight: 2, opacity: 0.7 }).addTo(map);
    map.fitBounds(pathLine.getBounds().pad(0.1));
  }
}

// Coverage planner
btnGenCoverage.addEventListener("click", () => {
  if (boundaryMarkers.length < 3) {
    alert("Click map to add at least 3 boundary points first.");
    return;
  }
  const boundary = boundaryMarkers.map(m => {
    const ll = m.getLatLng();
    return { lat: ll.lat, lon: ll.lng };
  });

  send({
    type: "generate_coverage",
    boundary,
    row_spacing: parseFloat(document.getElementById("cpSpacing").value) || 1.0,
    direction_deg: parseFloat(document.getElementById("cpDirection").value) || 0.0,
    overlap: parseFloat(document.getElementById("cpOverlap").value) || 0.0,
    tolerance_m: parseFloat(document.getElementById("cpTolerance").value) || 1.0,
    max_speed: parseFloat(document.getElementById("cpMaxSpeed").value) || 0.5,
  });

  clearBoundary();
});

// ── Init ─────────────────────────────────────────────────────────────────────
connect();
