const DEFAULT_POS = [38.9412928598587, -92.31884600793728];
const WS_URL = `ws://${window.location.hostname}:${Number(window.location.port || 8775) + 1}`;

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

offlineLayer.addTo(map);

let hasSwitchedFromOffline = false;
offlineLayer.on('tileerror', () => {
  if (hasSwitchedFromOffline) return;
  hasSwitchedFromOffline = true;
  if (map.hasLayer(offlineLayer)) {
    map.removeLayer(offlineLayer);
  }
  esriSatLayer.addTo(map);
  addEvent('离线瓦片缺失，已自动切换到在线卫星图', '#b57812');
});

L.control.layers({
  '离线地图 (LAN/本地)': offlineLayer,
  '卫星图 (Esri)': esriSatLayer,
  '普通地图 (OSM)': osmLayer,
}).addTo(map);

const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const csvFile = document.getElementById('csvFile');
const btnFindMe = document.getElementById('btnFindMe');
const btnCenterCurrent = document.getElementById('btnCenterCurrent');
const btnEditRoute = document.getElementById('btnEditRoute');
const editRouteOptions = document.getElementById('editRouteOptions');
const editTolerance = document.getElementById('editTolerance');
const editMaxSpeed = document.getElementById('editMaxSpeed');
const btnStartSim = document.getElementById('btnStartSim');
const btnExportRoute = document.getElementById('btnExportRoute');
const btnClearTrack = document.getElementById('btnClearTrack');
const btnExportLog = document.getElementById('btnExportLog');

let currentMarker = L.circleMarker(DEFAULT_POS, {
  radius: 8,
  color: '#073b4c',
  weight: 2,
  fillColor: '#118ab2',
  fillOpacity: 0.9,
}).addTo(map);

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

  if (col.lat < 0 || col.lon < 0) throw new Error('CSV 缺少 lat/lon 列');

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

function redrawWaypoints() {
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

  const bounds = L.latLngBounds(waypoints.map((w) => [w.lat, w.lon]));
  bounds.extend(currentMarker.getLatLng());
  map.fitBounds(bounds.pad(0.2));
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
  addEvent('已清空轨迹与日志', '#6b7280');
}

function exportLogs() {
  if (!logs.length) {
    addEvent('暂无日志可导出', '#6b7280');
    return;
  }
  const header = [
    'timestamp','lat','lon','source','fix_quality','num_sats','hdop','speed_mps','target_id','target_distance_m','status'
  ];
  const rows = logs.map((r) => [
    r.timestamp, r.lat, r.lon, r.source, r.fix_quality, r.num_sats,
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
    addEvent('暂无路径可导出', '#6b7280');
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
  addEvent(`已导出路径点 ${waypoints.length} 个`, '#1f8f46');
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

  setField('latVal', lat.toFixed(8));
  setField('lonVal', lon.toFixed(8));
  setField('srcVal', frame.source || 'unknown');
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
        addEvent(`点位 ${wp.id} 已满足 (${d.toFixed(2)} m)`, '#1f8f46');
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
      setField('targetVal', '全部满足');
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

  ws.onopen = () => {
    statusDot.style.background = '#1f8f46';
    statusText.textContent = '已连接';
    addEvent(`WebSocket 已连接 ${WS_URL}`, '#1f8f46');
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
    statusDot.style.background = '#c23a27';
    statusText.textContent = '已断开，3 秒后重连';
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
  btnStartSim.textContent = '开始模拟';
}

function startSimulation() {
  if (simTimer) {
    stopSimulation();
    addEvent('已停止模拟', '#6b7280');
    return;
  }
  if (simPath.length < 2) {
    addEvent('没有可模拟路径，请先导入 CSV 或编辑路径', '#c23a27');
    return;
  }

  clearTrack();
  resetReachState();

  let i = 0;
  btnStartSim.textContent = '停止模拟';
  addEvent(`开始模拟，共 ${simPath.length} 个轨迹点`, '#1b6c8d');

  simTimer = setInterval(() => {
    if (i >= simPath.length) {
      stopSimulation();
      addEvent('模拟完成', '#1f8f46');
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
  redrawWaypoints();
  setField('reachVal', `0 / ${waypoints.length}`);
}

function toggleEditRoute() {
  isEditMode = !isEditMode;
  setEditOptionsVisible(isEditMode);
  btnEditRoute.textContent = isEditMode ? '结束编辑' : '编辑路径';
  btnEditRoute.classList.toggle('active', isEditMode);
  if (isEditMode) {
    const { tolerance, maxSpeed } = getEditParams();
    stopSimulation();
    waypoints = [];
    redrawWaypoints();
    addEvent(`编辑模式开启：点击地图添加点位 (tol=${tolerance}m, speed=${maxSpeed}m/s)`, '#b57812');
  } else {
    redrawWaypoints();
    addEvent(`编辑完成，已生成 ${waypoints.length} 个点位`, '#1b6c8d');
  }
}

function findMe() {
  if (!navigator.geolocation) {
    addEvent('浏览器不支持定位', '#c23a27');
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
      addEvent(`定位成功 ${lat.toFixed(7)}, ${lon.toFixed(7)}`, '#1f8f46');
    },
    (err) => addEvent(`定位失败: ${err.message}`, '#c23a27'),
    { enableHighAccuracy: true, timeout: 10000 }
  );
}

function centerToCurrent() {
  const p = currentMarker.getLatLng();
  map.setView([p.lat, p.lng], Math.max(map.getZoom(), 19));
  addEvent(`已回到当前位置 ${p.lat.toFixed(7)}, ${p.lng.toFixed(7)}`, '#1b6c8d');
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
    addEvent(`已加载路径点 ${waypoints.length} 个`, '#1b6c8d');
  } catch (err) {
    addEvent(`CSV 加载失败: ${err.message}`, '#c23a27');
  }
});

btnFindMe.addEventListener('click', findMe);
btnCenterCurrent.addEventListener('click', centerToCurrent);
btnEditRoute.addEventListener('click', toggleEditRoute);
btnStartSim.addEventListener('click', startSimulation);
btnExportRoute.addEventListener('click', exportRouteCsv);
btnClearTrack.addEventListener('click', clearTrack);
btnExportLog.addEventListener('click', exportLogs);

map.on('click', addWaypointByMapClick);

addEvent('系统启动，默认位置已设定');
addEvent('当前默认底图为离线地图');
connectWebSocket();
