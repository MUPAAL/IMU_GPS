const DEFAULT_POS = [38.9412928598587, -92.31884600793728];
const WS_URL = `ws://${window.location.hostname}:${Number(window.location.port || 8775) + 1}`;
const STRINGS = {
  title: 'RTK Path Visualizer',
  disconnected: 'Disconnected',
  connected: 'Connected',
  reconnecting: 'Disconnected, reconnecting in 3s',
  csvLabel: 'CSV Route File',
findMe: 'Find Me',
  centerCurrent: 'Center Current',
  editRoute: 'Edit Route',
  doneEdit: 'Finish Edit',
  undoNode: 'Undo Node',
  startSim: 'Start Simulation',
  stopSim: 'Stop Simulation',
  exportRoute: 'Export Route CSV',
  clearTrack: 'Clear Track',
  exportLog: 'Export Log',
  cardCurrent: 'Current Position',
  cardDual: 'Dual RTK Status',
  latitude: 'Latitude',
  longitude: 'Longitude',
  source: 'Source',
  heading: 'Heading',
  baseline: 'Baseline',
  fix: 'Fix',
  satellites: 'Satellites',
  speed: 'Speed',
  online: 'Online',
  offline: 'Offline',
  noFix: 'No Fix',
  headingInvalid: 'invalid',
  cardMission: 'Mission Progress',
  reached: 'Reached Waypoints',
  target: 'Current Target',
  distance: 'Target Distance',
  cardEvents: 'Events',
  allReached: 'All reached',
  missingTiles: 'Satellite unavailable, switched to offline map',
  layerOffline: 'Offline Map (LAN/local)',
  layerSat: 'Satellite (Esri)',
  layerOsm: 'Street (OSM)',
  csvMissingLatLon: 'CSV missing lat/lon columns',
  cleared: 'Track and logs cleared',
  noLogs: 'No logs to export',
  noRoute: 'No route to export',
  routeExported: 'Exported {count} route points',
  reachedWp: 'Waypoint {id} reached ({dist} m)',
  wsConnected: 'WebSocket connected {url}',
  simStopped: 'Simulation stopped',
  simEmpty: 'No simulation path. Import CSV or edit route first',
  simStarted: 'Simulation started, {count} points',
  simDone: 'Simulation completed',
  editOn: 'Edit mode on: click map to add points (tol={tolerance}m, speed={speed}m/s)',
  editOff: 'Edit finished, generated {count} points',
  undoDone: 'Removed latest node #{id}',
  undoEmpty: 'No node to undo',
  geoUnsupported: 'Geolocation is not supported by this browser',
  geoOk: 'Located {lat}, {lon}',
  geoFail: 'Locate failed: {message}',
  centered: 'Centered to current position {lat}, {lon}',
  loadedPoints: 'Loaded {count} route points',
  csvLoadFail: 'CSV load failed: {message}',
  boot1: 'System started, default location loaded',
  boot2: 'Default base map is satellite map',
};

function t(key, vars = {}) {
  const raw = STRINGS[key] ?? key;
  return raw.replace(/\{(\w+)\}/g, (_, k) => vars[k] ?? `{${k}}`);
}

const map = L.map('map').setView(DEFAULT_POS, 19);

const offlineLayer = L.tileLayer('./assets/tiles/{z}/{x}/{y}.png', {
  maxZoom: 20,
  attribution: 'Offline tiles (LAN/local)',
});

const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 20,
  attribution: '&copy; OpenStreetMap contributors'
});

const esriSatLayer = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {
    maxZoom: 20,
    attribution: 'Tiles &copy; Esri'
  }
);

let activeBaseLayer = null;
function switchBaseLayer(nextLayer) {
  if (activeBaseLayer === nextLayer) return;
  [offlineLayer, esriSatLayer, osmLayer].forEach((layer) => {
    if (map.hasLayer(layer)) map.removeLayer(layer);
  });
  nextLayer.addTo(map);
  activeBaseLayer = nextLayer;
}

switchBaseLayer(esriSatLayer);

let hasSwitchedToOffline = false;
esriSatLayer.on('tileerror', () => {
  if (hasSwitchedToOffline) return;
  hasSwitchedToOffline = true;
  switchBaseLayer(offlineLayer);
  addEvent(t('missingTiles'), '#b57812');
});

if (!navigator.onLine) {
  switchBaseLayer(offlineLayer);
}

window.addEventListener('offline', () => {
  switchBaseLayer(offlineLayer);
});

window.addEventListener('online', () => {
  if (activeBaseLayer === offlineLayer) {
    switchBaseLayer(esriSatLayer);
  }
});

let baseLayerControl = null;
function renderLayerControl() {
  if (baseLayerControl) map.removeControl(baseLayerControl);
  baseLayerControl = L.control.layers({
    [t('layerOffline')]: offlineLayer,
    [t('layerSat')]: esriSatLayer,
    [t('layerOsm')]: osmLayer,
  }).addTo(map);
}
renderLayerControl();
map.on('baselayerchange', (e) => {
  activeBaseLayer = e.layer;
});

const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const pageTitle = document.getElementById('pageTitle');
const csvLabel = document.getElementById('csvLabel');
const csvFile = document.getElementById('csvFile');
const btnFindMe = document.getElementById('btnFindMe');
const btnCenterCurrent = document.getElementById('btnCenterCurrent');
const btnEditRoute = document.getElementById('btnEditRoute');
const btnUndoNode = document.getElementById('btnUndoNode');
const editRouteOptions = document.getElementById('editRouteOptions');
const editTolerance = document.getElementById('editTolerance');
const editMaxSpeed = document.getElementById('editMaxSpeed');
const btnStartSim = document.getElementById('btnStartSim');
const btnExportRoute = document.getElementById('btnExportRoute');
const btnClearTrack = document.getElementById('btnClearTrack');
const btnExportLog = document.getElementById('btnExportLog');
const cardCurrentTitle = document.getElementById('cardCurrentTitle');
const cardDualTitle = document.getElementById('cardDualTitle');
const labelLat = document.getElementById('labelLat');
const labelLon = document.getElementById('labelLon');
const labelSource = document.getElementById('labelSource');
const labelHeading = document.getElementById('labelHeading');
const labelBaseline = document.getElementById('labelBaseline');
const labelSats = document.getElementById('labelSats');
const labelSpeed = document.getElementById('labelSpeed');
const cardMissionTitle = document.getElementById('cardMissionTitle');
const labelReached = document.getElementById('labelReached');
const labelTarget = document.getElementById('labelTarget');
const labelDistance = document.getElementById('labelDistance');
const cardEventTitle = document.getElementById('cardEventTitle');
const headingVal = document.getElementById('headingVal');
const baselineVal = document.getElementById('baselineVal');
const dualRtkInfo = document.getElementById('dualRtkInfo');

let liveSocket = null;
let currentMarker = L.circleMarker(DEFAULT_POS, {
  radius: 8,
  color: '#073b4c',
  weight: 2,
  fillColor: '#118ab2',
  fillOpacity: 0.9,
}).addTo(map);
const SOURCE_COLORS = ['#118ab2', '#ef476f', '#06d6a0', '#ffd166'];
const sourceMarkers = new Map();
let headingLine = null;
let headingArrowMarker = null;

let waypoints = [];
let waypointMarkers = [];
let plannedPath = null;
let targetCircle = null;
let trackPoints = [];
let trackSegments = [];
let logs = [];
let isEditMode = false;
let simPath = [];
let simTimer = null;
let hasFirstFix = false;
let connectionState = 'disconnected';

function sourceColor(sourceId, index = 0) {
  const seed = Array.from(sourceId || '').reduce((acc, ch) => acc + ch.charCodeAt(0), 0);
  return SOURCE_COLORS[(seed + index) % SOURCE_COLORS.length];
}

function formatFixQuality(q) {
  const v = Number(q);
  if (!Number.isFinite(v)) return '-';
  if (v === 4) return 'RTK FIX';
  if (v === 5) return 'RTK FLOAT';
  if (v === 2) return 'DGPS';
  if (v === 1) return 'GPS';
  return t('noFix');
}

function renderDualRtkInfo(frame, sourceFrames) {
  if (!dualRtkInfo) return;
  if (headingVal) {
    if (frame.heading_valid && Number.isFinite(Number(frame.heading_deg))) {
      const deg = Number(frame.heading_deg).toFixed(2);
      headingVal.textContent = `${deg}\u00b0 ${frame.heading_dir || ''}`.trim();
    } else {
      headingVal.textContent = t('headingInvalid');
    }
  }
  if (baselineVal) {
    baselineVal.textContent = Number.isFinite(Number(frame.heading_baseline_m))
      ? `${Number(frame.heading_baseline_m).toFixed(2)} m`
      : '-';
  }

  dualRtkInfo.innerHTML = '';
  sourceFrames.forEach((src) => {
    const card = document.createElement('div');
    card.className = `rtk-source-card${src.connected ? '' : ' offline'}`;
    const latText = Number.isFinite(Number(src.lat)) ? Number(src.lat).toFixed(8) : '-';
    const lonText = Number.isFinite(Number(src.lon)) ? Number(src.lon).toFixed(8) : '-';
    const satText = src.num_sats == null ? '-' : `${src.num_sats}`;
    const hdopText = Number.isFinite(Number(src.hdop)) ? Number(src.hdop).toFixed(2) : '-';
    card.innerHTML = `
      <div class="rtk-source-head">
        <strong>${src.label || src.source_id}</strong>
        <span class="rtk-source-tag">${src.connected ? t('online') : t('offline')}</span>
      </div>
      <div class="rtk-source-grid">
        <span>${t('latitude')}</span><strong>${latText}</strong>
        <span>${t('longitude')}</span><strong>${lonText}</strong>
        <span>${t('fix')}</span><strong>${formatFixQuality(src.fix_quality)}</strong>
        <span>${t('satellites')}</span><strong>${satText}</strong>
        <span>HDOP</span><strong>${hdopText}</strong>
      </div>
    `;
    dualRtkInfo.appendChild(card);
  });
}

function syncDualRtkMap(frame, sourceFrames) {
  const activeIds = new Set();
  sourceFrames.forEach((src, idx) => {
    const sourceId = src.source_id;
    if (!sourceId) return;
    const lat = Number(src.lat);
    const lon = Number(src.lon);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      return;
    }
    activeIds.add(sourceId);
    let marker = sourceMarkers.get(sourceId);
    if (!marker) {
      const color = sourceColor(sourceId, idx);
      marker = L.circleMarker([lat, lon], {
        radius: 7,
        color,
        weight: 2,
        fillColor: color,
        fillOpacity: 0.55,
      }).addTo(map);
      sourceMarkers.set(sourceId, marker);
    } else {
      marker.setLatLng([lat, lon]);
    }
    marker.bindTooltip(`${src.label || sourceId}<br/>${lat.toFixed(8)}, ${lon.toFixed(8)}`);
  });

  for (const [sourceId, marker] of sourceMarkers.entries()) {
    if (!activeIds.has(sourceId)) {
      marker.remove();
      sourceMarkers.delete(sourceId);
    }
  }

  if (headingLine) {
    headingLine.remove();
    headingLine = null;
  }
  if (headingArrowMarker) {
    headingArrowMarker.remove();
    headingArrowMarker = null;
  }

  let sourceA = sourceFrames.find((s) => s.source_id === frame.heading_source_a);
  let sourceB = sourceFrames.find((s) => s.source_id === frame.heading_source_b);
  if (!sourceA || !sourceB) {
    const points = sourceFrames.filter((s) => Number.isFinite(Number(s.lat)) && Number.isFinite(Number(s.lon)));
    if (points.length >= 2) {
      sourceA = points[0];
      sourceB = points[1];
    }
  }
  if (!sourceA || !sourceB) return;

  const aLat = Number(sourceA?.lat);
  const aLon = Number(sourceA?.lon);
  const bLat = Number(sourceB?.lat);
  const bLon = Number(sourceB?.lon);
  if (![aLat, aLon, bLat, bLon].every((v) => Number.isFinite(v))) return;

  headingLine = L.polyline([[aLat, aLon], [bLat, bLon]], {
    color: '#e66b10',
    weight: 4,
    opacity: 0.9,
    lineCap: 'round',
  }).addTo(map);

  let headingDeg = Number(frame.heading_deg);
  if (!Number.isFinite(headingDeg)) {
    const dLat = bLat - aLat;
    const dLon = bLon - aLon;
    headingDeg = (Math.atan2(dLon, dLat) * 180 / Math.PI + 360) % 360;
  }

  const icon = L.divIcon({
    className: '',
    html: `<div class="heading-arrow" style="transform: rotate(${headingDeg - 90}deg)"></div>`,
    iconSize: [18, 12],
    iconAnchor: [18, 6],
  });
  headingArrowMarker = L.marker([bLat, bLon], { icon, interactive: false }).addTo(map);
}

function waypointTooltipHtml(wp, idx) {
  const reachedText = wp.reached ? 'yes' : 'no';
  const reachedAt = wp.reached_at || '-';
  const maxSpeed = wp.max_speed == null ? '-' : wp.max_speed;
  return [
    `ID: ${wp.id} (#${idx + 1})`,
    `lat: ${Number(wp.lat).toFixed(8)}`,
    `lon: ${Number(wp.lon).toFixed(8)}`,
    `tolerance_m: ${wp.tolerance_m}`,
    `max_speed: ${maxSpeed}`,
    `reached: ${reachedText}`,
    `reached_at: ${reachedAt}`,
  ].join('<br/>');
}

function setField(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function setEditOptionsVisible(visible) {
  if (!editRouteOptions) return;
  editRouteOptions.classList.toggle('show', visible);
  editRouteOptions.setAttribute('aria-hidden', visible ? 'false' : 'true');
}

function getEditParams() {
  const tolRaw = Number(editTolerance?.value);
  const speedRaw = Number(editMaxSpeed?.value);
  const tolerance = Number.isFinite(tolRaw) && tolRaw > 0 ? tolRaw : 0.5;
  const maxSpeed = Number.isFinite(speedRaw) && speedRaw >= 0 ? speedRaw : 1;
  if (editTolerance) editTolerance.value = tolerance.toString();
  if (editMaxSpeed) editMaxSpeed.value = maxSpeed.toString();
  return { tolerance, maxSpeed };
}

function updateUndoButtonState() {
  if (!btnUndoNode) return;
  const canUndo = isEditMode && waypoints.length > 0;
  btnUndoNode.disabled = !canUndo;
}

function addEvent(text, color = '#1b6c8d') {
  const ul = document.getElementById('eventList');
  const li = document.createElement('li');
  li.textContent = text;
  li.style.borderLeftColor = color;
  ul.prepend(li);
  while (ul.children.length > 80) ul.removeChild(ul.lastChild);
}

function distanceMeters(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const toRad = (deg) => deg * Math.PI / 180;
  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);
  const a = Math.sin(dLat / 2) ** 2 +
            Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(a));
}

function interpolatePath(points, stepMeters = 0.5) {
  if (points.length < 2) return [...points];
  const result = [points[0]];
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    const d = distanceMeters(a[0], a[1], b[0], b[1]);
    const steps = Math.max(1, Math.ceil(d / stepMeters));
    for (let k = 1; k <= steps; k += 1) {
      const t = k / steps;
      result.push([
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
      ]);
    }
  }
  return result;
}

function parseCsvRows(text) {
  const lines = text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);
  if (!lines.length) return [];

  const splitLine = (line) => line.includes('\t') ? line.split('\t') : line.split(',');
  const headers = splitLine(lines[0]).map((h) => h.trim().toLowerCase());

  const col = {
    id: headers.indexOf('id'),
    lat: headers.indexOf('lat'),
    lon: headers.indexOf('lon'),
    tolerance_m: headers.indexOf('tolerance_m'),
    max_speed: headers.indexOf('max_speed'),
  };

  if (col.lat < 0 || col.lon < 0) throw new Error(t('csvMissingLatLon'));

  return lines.slice(1).map((line, idx) => {
    const cells = splitLine(line).map((v) => v.trim());
    const id = col.id >= 0 ? (cells[col.id] || `${idx}`) : `${idx}`;
    const lat = Number(cells[col.lat]);
    const lon = Number(cells[col.lon]);
    const tolerance = col.tolerance_m >= 0 ? Number(cells[col.tolerance_m] || 0.5) : 0.5;
    const maxSpeed = col.max_speed >= 0 ? Number(cells[col.max_speed] || 0) : null;
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
    return {
      id,
      lat,
      lon,
      tolerance_m: Number.isFinite(tolerance) && tolerance > 0 ? tolerance : 0.5,
      max_speed: Number.isFinite(maxSpeed) ? maxSpeed : null,
      reached: false,
      reached_at: null,
    };
  }).filter(Boolean);
}

function redrawWaypoints(options = {}) {
  const { fitView = true } = options;
  waypointMarkers.forEach((m) => m.remove());
  waypointMarkers = [];

  if (plannedPath) {
    plannedPath.remove();
    plannedPath = null;
  }

  if (!waypoints.length) {
    setField('reachVal', '0 / 0');
    setField('targetVal', '-');
    setField('distVal', '-');
    simPath = [];
    updateUndoButtonState();
    return;
  }

  plannedPath = L.polyline(waypoints.map((w) => [w.lat, w.lon]), {
    color: '#2b6cb0',
    weight: 3,
    opacity: 0.85,
    dashArray: '8, 6',
  }).addTo(map);

  waypoints.forEach((w, idx) => {
    const m = L.circleMarker([w.lat, w.lon], {
      radius: 6,
      color: '#4a5568',
      weight: 2,
      fillColor: '#cbd5e0',
      fillOpacity: 0.9,
    }).addTo(map);
    m.bindTooltip(waypointTooltipHtml(w, idx), { permanent: false });
    waypointMarkers[idx] = m;
  });

  simPath = interpolatePath(waypoints.map((w) => [w.lat, w.lon]), 0.4);
  if (fitView) {
    const bounds = L.latLngBounds(waypoints.map((w) => [w.lat, w.lon]));
    bounds.extend(currentMarker.getLatLng());
    map.fitBounds(bounds.pad(0.2));
  }
  updateUndoButtonState();
}

function findCurrentTarget() {
  for (let i = 0; i < waypoints.length; i += 1) {
    if (!waypoints[i].reached) return { wp: waypoints[i], idx: i };
  }
  return null;
}

function updateWaypointStyles(activeIdx) {
  waypointMarkers.forEach((m, idx) => {
    const wp = waypoints[idx];
    let color = '#4a5568';
    let fill = '#cbd5e0';
    if (wp.reached) {
      color = '#1f8f46';
      fill = '#6ee7b7';
    } else if (idx === activeIdx) {
      color = '#b57812';
      fill = '#f6ad55';
    }
    m.setStyle({ color, fillColor: fill });
    m.setTooltipContent(waypointTooltipHtml(wp, idx));
  });
}

function addTrackSegment(a, b, color) {
  const seg = L.polyline([a, b], {
    color,
    weight: 5,
    opacity: 0.85,
    lineCap: 'round',
  }).addTo(map);
  trackSegments.push(seg);
}

function clearTrack() {
  trackSegments.forEach((s) => s.remove());
  trackSegments = [];
  trackPoints = [];
  logs = [];
  addEvent(t('cleared'), '#6b7280');
}

function exportLogs() {
  if (!logs.length) {
    addEvent(t('noLogs'), '#6b7280');
    return;
  }
  const header = [
    'timestamp','lat','lon','source','rtk_source','rtk_source_label','fix_quality','num_sats','hdop','speed_mps','target_id','target_distance_m','status'
  ];
  const rows = logs.map((r) => [
    r.timestamp, r.lat, r.lon, r.source, r.rtk_source, r.rtk_source_label, r.fix_quality, r.num_sats,
    r.hdop, r.speed_mps, r.target_id, r.target_distance_m, r.status
  ]);
  const csv = [header, ...rows].map((line) => line.join(',')).join('\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `rtk_log_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click();
  URL.revokeObjectURL(url);
}

function exportRouteCsv() {
  if (!waypoints.length) {
    addEvent(t('noRoute'), '#6b7280');
    return;
  }
  const header = ['id', 'lat', 'lon', 'tolerance_m', 'max_speed'];
  const rows = waypoints.map((w, idx) => [
    w.id ?? `${idx}`,
    Number(w.lat).toFixed(8),
    Number(w.lon).toFixed(8),
    w.tolerance_m ?? 0.5,
    w.max_speed ?? '',
  ]);
  const csv = [header, ...rows].map((line) => line.join(',')).join('\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `route_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click();
  URL.revokeObjectURL(url);
  addEvent(t('routeExported', { count: waypoints.length }), '#1f8f46');
}

function pushFrame(frame) {
  updateByFrame(frame);
}

function updateByFrame(frame) {
  if (simTimer && frame.source !== 'sim') return;

  const lat = Number(frame.lat);
  const lon = Number(frame.lon);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;

  const point = [lat, lon];
  currentMarker.setLatLng(point);

  if (!hasFirstFix && frame.source === 'rtk') {
    hasFirstFix = true;
    console.log('First RTK fix, moving map to', point);
    map.setView(point, map.getZoom());
  }

  const sourceFrames = Array.isArray(frame.rtk_source_frames) ? frame.rtk_source_frames : [];
  renderDualRtkInfo(frame, sourceFrames);
  syncDualRtkMap(frame, sourceFrames);

  setField('latVal', lat.toFixed(8));
  setField('lonVal', lon.toFixed(8));
  const sourceParts = [];
  if (frame.rtk_source_label) sourceParts.push(frame.rtk_source_label);
  else if (frame.rtk_source) sourceParts.push(frame.rtk_source);
  if (frame.source) sourceParts.push(frame.source);
  setField('srcVal', sourceParts.length ? sourceParts.join(' · ') : 'unknown');
  setField('fixVal', `${frame.fix_quality ?? '-'} `);
  setField('satVal', `${frame.num_sats ?? '-'} `);
  setField('hdopVal', frame.hdop == null ? '-' : Number(frame.hdop).toFixed(2));

  const speedMps = frame.speed_knots == null ? null : Number(frame.speed_knots) * 0.514444;
  setField('spdVal', speedMps == null ? '-' : `${speedMps.toFixed(2)} m/s`);

  if (!trackPoints.length || distanceMeters(trackPoints.at(-1)[0], trackPoints.at(-1)[1], lat, lon) >= 0.2) {
    trackPoints.push(point);

    let status = 'no_target';
    let color = '#c23a27';
    let targetId = '';
    let targetDist = '';
    let activeIdx = -1;

    const targetInfo = findCurrentTarget();
    if (targetInfo) {
      const { wp, idx } = targetInfo;
      activeIdx = idx;
      const d = distanceMeters(lat, lon, wp.lat, wp.lon);
      targetId = wp.id;
      targetDist = d.toFixed(2);

      if (targetCircle) targetCircle.remove();
      targetCircle = L.circle([wp.lat, wp.lon], {
        radius: wp.tolerance_m,
        color: '#b57812',
        fillColor: '#fbd38d',
        fillOpacity: 0.18,
        weight: 2,
      }).addTo(map);

      if (d <= wp.tolerance_m) {
        color = '#1f8f46';
        status = 'reached';
        wp.reached = true;
        wp.reached_at = new Date().toISOString();
        addEvent(t('reachedWp', { id: wp.id, dist: d.toFixed(2) }), '#1f8f46');
      } else if (d <= wp.tolerance_m * 2) {
        color = '#b57812';
        status = 'approaching';
      } else {
        color = '#c23a27';
        status = 'off_path';
      }

      setField('targetVal', wp.id);
      setField('distVal', `${d.toFixed(2)} m`);
    } else {
      setField('targetVal', t('allReached'));
      setField('distVal', '-');
      if (targetCircle) {
        targetCircle.remove();
        targetCircle = null;
      }
      status = 'completed';
      color = '#1f8f46';
    }

    if (trackPoints.length >= 2) {
      addTrackSegment(trackPoints.at(-2), trackPoints.at(-1), color);
    }

    updateWaypointStyles(activeIdx);
    const reachedCount = waypoints.filter((w) => w.reached).length;
    setField('reachVal', `${reachedCount} / ${waypoints.length}`);

    logs.push({
      timestamp: new Date().toISOString(),
      lat: lat.toFixed(8),
      lon: lon.toFixed(8),
      source: frame.source || '',
      rtk_source: frame.rtk_source || '',
      rtk_source_label: frame.rtk_source_label || '',
      fix_quality: frame.fix_quality ?? '',
      num_sats: frame.num_sats ?? '',
      hdop: frame.hdop ?? '',
      speed_mps: speedMps == null ? '' : speedMps.toFixed(3),
      target_id: targetId,
      target_distance_m: targetDist,
      status,
    });
  }
}


function connectWebSocket() {
  const ws = new WebSocket(WS_URL);
  liveSocket = ws;

  ws.onopen = () => {
    connectionState = 'connected';
    statusDot.style.background = '#1f8f46';
    statusText.textContent = t('connected');
    addEvent(t('wsConnected', { url: WS_URL }), '#1f8f46');
  };

  ws.onmessage = (event) => {
    try {
      if (simTimer) return;
      const frame = JSON.parse(event.data);
pushFrame(frame);
    } catch (err) {
      console.error(err);
    }
  };

  ws.onclose = () => {
    if (liveSocket === ws) liveSocket = null;
    connectionState = 'reconnecting';
    statusDot.style.background = '#c23a27';
    statusText.textContent = t('reconnecting');
    setTimeout(connectWebSocket, 3000);
  };

  ws.onerror = () => ws.close();
}

function resetReachState() {
  waypoints.forEach((w) => {
    w.reached = false;
    w.reached_at = null;
  });
  updateWaypointStyles(-1);
  setField('reachVal', `0 / ${waypoints.length}`);
}

function stopSimulation() {
  if (simTimer) {
    clearInterval(simTimer);
    simTimer = null;
  }
  btnStartSim.textContent = t('startSim');
}

function startSimulation() {
  if (simTimer) {
    stopSimulation();
    addEvent(t('simStopped'), '#6b7280');
    return;
  }
  if (simPath.length < 2) {
    addEvent(t('simEmpty'), '#c23a27');
    return;
  }

  clearTrack();
  resetReachState();

  let i = 0;
  btnStartSim.textContent = t('stopSim');
  addEvent(t('simStarted', { count: simPath.length }), '#1b6c8d');

  simTimer = setInterval(() => {
    if (i >= simPath.length) {
      stopSimulation();
      addEvent(t('simDone'), '#1f8f46');
      return;
    }

    const p = simPath[i];
    const prev = i > 0 ? simPath[i - 1] : p;
    const d = distanceMeters(prev[0], prev[1], p[0], p[1]);
    const speedKnots = (d / 0.25) / 0.514444;

    pushFrame({
      lat: p[0],
      lon: p[1],
      source: 'sim',
      fix_quality: 4,
      num_sats: 18,
      hdop: 0.6,
      speed_knots: speedKnots,
    });

    i += 1;
  }, 250);
}

function addWaypointByMapClick(e) {
  if (!isEditMode) return;
  const lat = Number(e.latlng.lat);
  const lon = Number(e.latlng.lng);
  const { tolerance, maxSpeed } = getEditParams();
  waypoints.push({
    id: `${waypoints.length}`,
    lat,
    lon,
    tolerance_m: tolerance,
    max_speed: maxSpeed,
    reached: false,
    reached_at: null,
  });
  redrawWaypoints({ fitView: false });
  setField('reachVal', `0 / ${waypoints.length}`);
}

function undoLastWaypoint() {
  if (!isEditMode || !waypoints.length) {
    addEvent(t('undoEmpty'), '#6b7280');
    return;
  }
  const removed = waypoints.pop();
  redrawWaypoints({ fitView: false });
  setField('reachVal', `0 / ${waypoints.length}`);
  addEvent(t('undoDone', { id: removed.id }), '#6b7280');
}

function toggleEditRoute() {
  isEditMode = !isEditMode;
  setEditOptionsVisible(isEditMode);
  btnEditRoute.textContent = isEditMode ? t('doneEdit') : t('editRoute');
  btnEditRoute.classList.toggle('active', isEditMode);
  updateUndoButtonState();
  if (isEditMode) {
    const { tolerance, maxSpeed } = getEditParams();
    stopSimulation();
    waypoints = [];
    redrawWaypoints();
    addEvent(t('editOn', { tolerance, speed: maxSpeed }), '#b57812');
  } else {
    redrawWaypoints();
    addEvent(t('editOff', { count: waypoints.length }), '#1b6c8d');
  }
}

function findMe() {
  if (!navigator.geolocation) {
    addEvent(t('geoUnsupported'), '#c23a27');
    return;
  }

  navigator.geolocation.getCurrentPosition(
    (pos) => {
      const lat = pos.coords.latitude;
      const lon = pos.coords.longitude;
      const speedMps = Number.isFinite(pos.coords.speed) ? pos.coords.speed : null;
      const speedKnots = speedMps == null ? null : speedMps / 0.514444;
      map.setView([lat, lon], 19);
      pushFrame({
        lat,
        lon,
        source: 'find_me',
        fix_quality: 1,
        num_sats: '',
        hdop: Number.isFinite(pos.coords.accuracy) ? pos.coords.accuracy : '',
        speed_knots: speedKnots,
      });
      addEvent(t('geoOk', { lat: lat.toFixed(7), lon: lon.toFixed(7) }), '#1f8f46');
    },
    (err) => addEvent(t('geoFail', { message: err.message }), '#c23a27'),
    { enableHighAccuracy: true, timeout: 10000 }
  );
}

function centerToCurrent() {
  const p = currentMarker.getLatLng();
  map.setView([p.lat, p.lng], Math.max(map.getZoom(), 19));
  addEvent(t('centered', { lat: p.lat.toFixed(7), lon: p.lng.toFixed(7) }), '#1b6c8d');
}


csvFile.addEventListener('change', async (ev) => {
  const file = ev.target.files?.[0];
  if (!file) return;

  try {
    const text = await file.text();
    waypoints = parseCsvRows(text);
    resetReachState();
    redrawWaypoints();
    updateWaypointStyles(0);
    addEvent(t('loadedPoints', { count: waypoints.length }), '#1b6c8d');
  } catch (err) {
    addEvent(t('csvLoadFail', { message: err.message }), '#c23a27');
  }
});

btnFindMe.addEventListener('click', findMe);
btnCenterCurrent.addEventListener('click', centerToCurrent);
btnEditRoute.addEventListener('click', toggleEditRoute);
btnUndoNode.addEventListener('click', undoLastWaypoint);
btnStartSim.addEventListener('click', startSimulation);
btnExportRoute.addEventListener('click', exportRouteCsv);
btnClearTrack.addEventListener('click', clearTrack);
btnExportLog.addEventListener('click', exportLogs);

map.on('click', addWaypointByMapClick);


document.title = STRINGS.title;
pageTitle.textContent = STRINGS.title;
statusText.textContent = STRINGS.disconnected;
connectionState = 'disconnected';
statusText.textContent = t('disconnected');
addEvent(t('boot1'));
addEvent(t('boot2'));
connectWebSocket();
