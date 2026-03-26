/**
 * nav_visualizer.js
 * Integrated Three.js 3D IMU + Leaflet Map visualization.
 *
 * Single WebSocket connection to nav_bridge (port+1).
 * Data paths: data.imu.* for IMU, data.rtk.* for RTK, data.nav.* for navigation.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ========== Config ==========
const WS_URL = `ws://${window.location.hostname}:${Number(window.location.port || 8785) + 1}`;

const DEFAULT_POS = [38.9412928598587, -92.31884600793728];

// ========== I18N ==========
const I18N = {
  en: {
    title: 'Nav Dashboard',
    disconnected: 'Disconnected',
    connected: 'Connected',
    reconnecting: 'Disconnected, reconnecting in 3s',
    csvLabel: 'CSV Route',
    centerCurrent: 'Center Current',
    editRoute: 'Edit Route',
    doneEdit: 'Finish Edit',
    undoNode: 'Undo Node',
    startSim: 'Start Sim',
    stopSim: 'Stop Sim',
    exportRoute: 'Export CSV',
    topNorth: 'Top North',
    cardCurrent: 'Current Position',
    latitude: 'Latitude',
    longitude: 'Longitude',
    source: 'Source',
    satellites: 'Satellites',
    speed: 'Speed',
    cardMission: 'Mission Progress',
    reached: 'Reached',
    target: 'Target',
    distance: 'Distance',
    cardEvents: 'Events',
    allReached: 'All reached',
    missingTiles: 'Satellite unavailable, switched to offline map',
    layerOffline: 'Offline Map (LAN/local)',
    layerSat: 'Satellite (Esri)',
    layerOsm: 'Street (OSM)',
    csvMissingLatLon: 'CSV missing lat/lon columns',
    cleared: 'Track and logs cleared',
    noRoute: 'No route to export',
    routeExported: 'Exported {count} route points',
    reachedWp: 'Waypoint {id} reached ({dist} m)',
    wsConnected: 'WebSocket connected',
    simStopped: 'Simulation stopped',
    simEmpty: 'No simulation path. Import CSV or edit route first',
    simStarted: 'Simulation started, {count} points',
    simDone: 'Simulation completed',
    editOn: 'Edit mode on: click map to add points (tol={tolerance}m, speed={speed}m/s)',
    editOff: 'Edit finished, generated {count} points',
    undoDone: 'Removed latest node #{id}',
    undoEmpty: 'No node to undo',
    centered: 'Centered to current position',
    loadedPoints: 'Loaded {count} route points',
    csvLoadFail: 'CSV load failed: {message}',
    boot1: 'System started',
    langTitleZh: 'Switch to Chinese',
    langTitleEn: 'Switch to English',
  },
  zh: {
    title: 'Nav 导航面板',
    disconnected: '未连接',
    connected: '已连接',
    reconnecting: '已断开，3 秒后重连',
    csvLabel: 'CSV 路径',
    centerCurrent: '回到当前位置',
    editRoute: '编辑路径',
    doneEdit: '结束编辑',
    undoNode: '撤销节点',
    startSim: '开始模拟',
    stopSim: '停止模拟',
    exportRoute: '导出CSV',
    topNorth: '顶视北向',
    cardCurrent: '当前位置',
    latitude: '纬度',
    longitude: '经度',
    source: '来源',
    satellites: '卫星',
    speed: '速度',
    cardMission: '任务进度',
    reached: '已达',
    target: '目标',
    distance: '距离',
    cardEvents: '事件',
    allReached: '全部达到',
    missingTiles: '卫星图不可用，已切换到离线地图',
    layerOffline: '离线地图',
    layerSat: '卫星图 (Esri)',
    layerOsm: '普通地图 (OSM)',
    csvMissingLatLon: 'CSV 缺少 lat/lon 列',
    cleared: '已清空轨迹与日志',
    noRoute: '暂无路径可导出',
    routeExported: '已导出 {count} 个路径点',
    reachedWp: '点位 {id} 已达 ({dist} m)',
    wsConnected: 'WebSocket 已连接',
    simStopped: '已停止模拟',
    simEmpty: '没有可模拟路径',
    simStarted: '开始模拟，共 {count} 个点',
    simDone: '模拟完成',
    editOn: '编辑模式：点击地图添加点位 (tol={tolerance}m, speed={speed}m/s)',
    editOff: '编辑完成，已生成 {count} 个点位',
    undoDone: '已撤销节点 #{id}',
    undoEmpty: '无可撤销节点',
    centered: '已回到当前位置',
    loadedPoints: '已加载 {count} 个路径点',
    csvLoadFail: 'CSV 加载失败: {message}',
    boot1: '系统启动',
    langTitleZh: '切换到中文',
    langTitleEn: 'Switch to English',
  }
};
let currentLang = 'en';

function t(key, vars = {}) {
  const raw = I18N[currentLang][key] ?? I18N.en[key] ?? key;
  return raw.replace(/\{(\w+)\}/g, (_, k) => `${vars[k] ?? ''}`);
}

// ========== DOM refs ==========
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const pageTitle = document.getElementById('pageTitle');
const langToggle = document.getElementById('langToggle');
const csvLabel = document.getElementById('csvLabel');
const csvFile = document.getElementById('csvFile');
const btnCenterCurrent = document.getElementById('btnCenterCurrent');
const btnEditRoute = document.getElementById('btnEditRoute');
const btnUndoNode = document.getElementById('btnUndoNode');
const editRouteOptions = document.getElementById('editRouteOptions');
const editTolerance = document.getElementById('editTolerance');
const editMaxSpeed = document.getElementById('editMaxSpeed');
const btnStartSim = document.getElementById('btnStartSim');
const btnExportRoute = document.getElementById('btnExportRoute');
const btnTopNorth = document.getElementById('btn-top-north');
const cardCurrentTitle = document.getElementById('cardCurrentTitle');
const labelLat = document.getElementById('labelLat');
const labelLon = document.getElementById('labelLon');
const labelSource = document.getElementById('labelSource');
const labelSats = document.getElementById('labelSats');
const labelSpeed = document.getElementById('labelSpeed');
const cardMissionTitle = document.getElementById('cardMissionTitle');
const labelReached = document.getElementById('labelReached');
const labelTarget = document.getElementById('labelTarget');
const labelDistance = document.getElementById('labelDistance');
const cardEventTitle = document.getElementById('cardEventTitle');

// ==========================================
// SECTION 1: Three.js 3D Scene (from IMU)
// ==========================================

const container = document.getElementById('canvas-container');

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x0d1117, 1);
container.insertBefore(renderer.domElement, container.firstChild);

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.position.set(1.5, 1.5, 2.5);
camera.lookAt(0, 0, 0);
const DEFAULT_CAMERA_POS = new THREE.Vector3(1.5, 1.5, 2.5);
const TOP_VIEW_HEIGHT = 4.0;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.5;
controls.maxDistance = 10;

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
dirLight.position.set(5, 8, 5);
scene.add(dirLight);
const fillLight = new THREE.DirectionalLight(0x8888ff, 0.3);
fillLight.position.set(-3, -2, -3);
scene.add(fillLight);

// Grid & world axes
scene.add(new THREE.GridHelper(3, 6, 0x303040, 0x20202c));

function worldAxis(dir, color, length = 1.0) {
  return new THREE.ArrowHelper(
    new THREE.Vector3(...dir).normalize(),
    new THREE.Vector3(0, 0, 0),
    length, color, 0.08, 0.06
  );
}
scene.add(worldAxis([1, 0, 0], 0xff4444));
scene.add(worldAxis([0, 1, 0], 0x44ff44));
scene.add(worldAxis([0, 0, 1], 0x4444ff));

// Heading arrow
const headingArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0.01, 0),
  1.3, 0xff8800, 0.18, 0.10
);
scene.add(headingArrow);

// World axis labels
function makeLabel(text, position, color = '#888888') {
  const canvas = document.createElement('canvas');
  canvas.width = 64; canvas.height = 32;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = color;
  ctx.font = 'bold 22px monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 32, 16);
  const tex = new THREE.CanvasTexture(canvas);
  const mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false });
  const sprite = new THREE.Sprite(mat);
  sprite.position.set(...position);
  sprite.scale.set(0.25, 0.12, 1);
  return sprite;
}
scene.add(makeLabel('X', [1.15, 0, 0], '#ff6666'));
scene.add(makeLabel('Y', [0, 1.15, 0], '#66ff66'));
scene.add(makeLabel('Z', [0, 0, 1.15], '#6666ff'));
const northLabel = makeLabel('N', [1.45, 0.02, 0], '#f85149');
const southLabel = makeLabel('S', [-1.45, 0.02, 0], '#7d8590');
const eastLabel = makeLabel('E', [0, 0.02, 1.45], '#7d8590');
const westLabel = makeLabel('W', [0, 0.02, -1.45], '#7d8590');
[northLabel, southLabel, eastLabel, westLabel].forEach((label) => label.scale.set(0.4, 0.2, 1));
scene.add(northLabel, southLabel, eastLabel, westLabel);

// IMU box group
const imuGroup = new THREE.Group();
scene.add(imuGroup);

const boxGeo = new THREE.BoxGeometry(1.2, 0.2, 0.8);
const boxMat = new THREE.MeshPhongMaterial({
  color: 0x1a4a6e, specular: 0x4488cc, shininess: 60, emissive: 0x0a1a2e,
});
imuGroup.add(new THREE.Mesh(boxGeo, boxMat));

const edgesGeo = new THREE.EdgesGeometry(boxGeo);
imuGroup.add(new THREE.LineSegments(edgesGeo, new THREE.LineBasicMaterial({ color: 0x39d0d8, linewidth: 1 })));

const chipGeo = new THREE.BoxGeometry(0.3, 0.05, 0.3);
const chipMesh = new THREE.Mesh(chipGeo, new THREE.MeshPhongMaterial({ color: 0x222222, specular: 0x555555, shininess: 40 }));
chipMesh.position.y = 0.125;
imuGroup.add(chipMesh);

// Body-frame axes
const ARROW_LEN = 0.8;
const bodyXArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), ARROW_LEN, 0xff2222, 0.10, 0.07);
const bodyYArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), ARROW_LEN, 0x22ff22, 0.10, 0.07);
const bodyZArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), ARROW_LEN, 0x2222ff, 0.10, 0.07);
imuGroup.add(bodyXArrow, bodyYArrow, bodyZArrow);

imuGroup.add(makeLabel('X', [ARROW_LEN + 0.12, 0, 0], '#ff4444'));
imuGroup.add(makeLabel('Y', [0, ARROW_LEN + 0.12, 0], '#44ff44'));
imuGroup.add(makeLabel('Z', [0, 0, ARROW_LEN + 0.12], '#4444ff'));

// Frame correction: BNO085 Z-up → Three.js Y-up
const FRAME_CORRECTION = new THREE.Quaternion(-Math.SQRT1_2, 0, 0, Math.SQRT1_2);

// Quaternion state
const currentQuat = new THREE.Quaternion();
const targetQuat = new THREE.Quaternion();

// Control state
let northOffsetDeg = 0;
let rawHeadingDeg = 0;
let wsHeadingDeg = null;
let wsHeadingDir = null;
let wsHeadingRaw = null;
let isPaused = false;
let lockYawOnly = false;
let topNorthView = false;
let lockYawBeforeTop = false;

// North offset helpers
function sendNorthOffset(frontendOffsetDeg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    const serverOffset = (360 - frontendOffsetDeg) % 360;
    ws.send(JSON.stringify({ set_north_offset: serverOffset }));
  }
}

// Resize handler
function onResize() {
  const w = container.clientWidth;
  const h = container.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
const resizeObserver = new ResizeObserver(onResize);
resizeObserver.observe(container);
onResize();

// Compass HUD
const compassCanvas = document.getElementById('compassCanvas');
const compassCtx = compassCanvas.getContext('2d');
const CW = 110, CH = 110, CR = 44, CX = 55, CY = 55;
compassCanvas.width = CW; compassCanvas.height = CH;

function drawCompass(deg) {
  compassCtx.clearRect(0, 0, CW, CH);
  compassCtx.strokeStyle = '#30363d'; compassCtx.lineWidth = 1.5;
  compassCtx.beginPath(); compassCtx.arc(CX, CY, CR, 0, Math.PI * 2); compassCtx.stroke();
  const cardinals = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
  cardinals.forEach(([label, a]) => {
    const r = a * Math.PI / 180;
    compassCtx.fillStyle = label === 'N' ? '#f85149' : '#7d8590';
    compassCtx.font = 'bold 10px monospace';
    compassCtx.textAlign = 'center'; compassCtx.textBaseline = 'middle';
    compassCtx.fillText(label, CX + (CR - 9) * Math.sin(r), CY - (CR - 9) * Math.cos(r));
  });
  const rad = (deg - 90) * Math.PI / 180;
  compassCtx.strokeStyle = '#ff8800'; compassCtx.lineWidth = 2.5;
  compassCtx.lineCap = 'round';
  compassCtx.beginPath();
  compassCtx.moveTo(CX, CY);
  compassCtx.lineTo(CX + CR * 0.75 * Math.cos(rad), CY + CR * 0.75 * Math.sin(rad));
  compassCtx.stroke();
  compassCtx.fillStyle = '#ff8800';
  compassCtx.beginPath(); compassCtx.arc(CX, CY, 3, 0, Math.PI * 2); compassCtx.fill();
}

// Heading display
const COMPASS_DIRS = ['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW'];
function updateHeadingDisplay(deg, dirOverride = null) {
  const idx = Math.round(deg / 22.5) % 16;
  setText('heading_deg', deg.toFixed(3) + '\u00b0');
  setText('heading_dir', dirOverride || COMPASS_DIRS[idx]);
}

function getNorthVector() {
  // World convention: +X points to North.
  return new THREE.Vector3(1, 0, 0);
}

function getCalibratedQuat(baseQuat) {
  const yawOffsetRad = THREE.MathUtils.degToRad(northOffsetDeg);
  const yawOffsetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, yawOffsetRad, 0, 'YXZ'));
  return yawOffsetQuat.multiply(baseQuat.clone());
}

function setYawLock(enabled) {
  lockYawOnly = enabled;
  btnLockYaw.classList.toggle('active', lockYawOnly);
}

function setTopNorthView(enabled) {
  topNorthView = enabled;
  btnTopNorth.classList.toggle('active', topNorthView);
  if (topNorthView) {
    lockYawBeforeTop = lockYawOnly;
    setYawLock(true);
    controls.enableRotate = false;
    controls.enablePan = false;
    controls.target.set(0, 0, 0);
    camera.position.set(0, TOP_VIEW_HEIGHT, 0.001);
    const north = getNorthVector();
    camera.up.set(north.x, north.y, north.z);
    camera.lookAt(0, 0, 0);
  } else {
    controls.enableRotate = true;
    controls.enablePan = true;
    setYawLock(lockYawBeforeTop);
    camera.up.set(0, 1, 0);
    camera.position.copy(DEFAULT_CAMERA_POS);
    camera.lookAt(0, 0, 0);
  }
  controls.update();
}

// Animation loop
const SLERP_SPEED = 0.2;

function animate() {
  requestAnimationFrame(animate);
  currentQuat.slerp(targetQuat, SLERP_SPEED);
  const calibratedQuat = getCalibratedQuat(currentQuat);

  if (lockYawOnly) {
    const euler = new THREE.Euler().setFromQuaternion(calibratedQuat, 'YXZ');
    imuGroup.quaternion.copy(new THREE.Quaternion().setFromEuler(new THREE.Euler(0, euler.y, 0, 'YXZ')));
  } else {
    imuGroup.quaternion.copy(calibratedQuat);
  }

  const chipXRaw = new THREE.Vector3(1, 0, 0).applyQuaternion(currentQuat);
  chipXRaw.y = 0;
  const chipXCal = new THREE.Vector3(1, 0, 0).applyQuaternion(calibratedQuat);
  chipXCal.y = 0;
  if (chipXCal.lengthSq() > 1e-4) {
    chipXCal.normalize();
    headingArrow.setDirection(chipXCal.clone());
  }
  rawHeadingDeg = ((Math.atan2(chipXRaw.z, chipXRaw.x) * 180 / Math.PI) + 360) % 360;
  const displayHeadingDeg = (typeof wsHeadingDeg === 'number')
    ? wsHeadingDeg
    : (rawHeadingDeg - northOffsetDeg + 360) % 360;
  updateHeadingDisplay(displayHeadingDeg, wsHeadingDir);
  drawCompass(displayHeadingDeg);

  if (topNorthView) {
    const north = getNorthVector();
    camera.up.set(north.x, north.y, north.z);
    camera.lookAt(0, 0, 0);
  }

  controls.update();
  renderer.render(scene, camera);
}
animate();

// ==========================================
// SECTION 2: IMU Data Panel (from IMU)
// ==========================================

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function fmt(v, decimals = 3) {
  if (v === undefined || v === null) return '\u2013';
  return Number(v).toFixed(decimals);
}

const CAL_LABELS = ['Unknown', 'Low', 'Medium', 'High'];
const CAL_COLORS = ['cal-0', 'cal-1', 'cal-2', 'cal-3'];

function updateImuPanel(imu) {
  if (!imu || !Object.keys(imu).length) return;

  const rot = imu.rot || {};
  setText('rot_qi', fmt(rot.qi));
  setText('rot_qj', fmt(rot.qj));
  setText('rot_qk', fmt(rot.qk));
  setText('rot_qr', fmt(rot.qr));

  const euler = imu.euler || {};
  setText('euler_roll',  euler.roll  !== undefined ? fmt(euler.roll) + '\u00b0' : '\u2013');
  setText('euler_pitch', euler.pitch !== undefined ? fmt(euler.pitch) + '\u00b0' : '\u2013');
  setText('euler_yaw',   euler.yaw   !== undefined ? fmt(euler.yaw) + '\u00b0' : '\u2013');

  const accel = imu.accel || {};
  setText('accel_x', fmt(accel.x));
  setText('accel_y', fmt(accel.y));
  setText('accel_z', fmt(accel.z));

  const lin = imu.lin_accel || {};
  setText('lin_x', fmt(lin.x));
  setText('lin_y', fmt(lin.y));
  setText('lin_z', fmt(lin.z));

  const grav = imu.gravity || {};
  setText('grav_x', fmt(grav.x));
  setText('grav_y', fmt(grav.y));
  setText('grav_z', fmt(grav.z));

  const gyro = imu.gyro || {};
  setText('gyro_x', fmt(gyro.x));
  setText('gyro_y', fmt(gyro.y));
  setText('gyro_z', fmt(gyro.z));

  const game = imu.game_rot || {};
  setText('game_qi', fmt(game.qi));
  setText('game_qj', fmt(game.qj));
  setText('game_qk', fmt(game.qk));
  setText('game_qr', fmt(game.qr));

  const mag = imu.mag || {};
  setText('mag_x', fmt(mag.x));
  setText('mag_y', fmt(mag.y));
  setText('mag_z', fmt(mag.z));

  setText('steps', imu.steps !== undefined ? imu.steps : '\u2013');

  const cal = imu.cal !== undefined ? imu.cal : -1;
  const calEl = document.getElementById('calBadge');
  if (calEl) {
    calEl.className = 'cal-badge ' + (cal >= 0 && cal <= 3 ? CAL_COLORS[cal] : 'cal-0');
  }
  setText('calText', `Cal: ${cal >= 0 && cal <= 3 ? CAL_LABELS[cal] : '\u2013'}`);

  setText('hzDisplay', imu.hz !== undefined ? `${imu.hz} Hz` : '\u2013 Hz');
}

// ==========================================
// SECTION 3: Leaflet Map (from RTK)
// ==========================================

let map = null;
let currentMarker = null;
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
let connectionState = 'disconnected';

// Map layers
let offlineLayer = null;
let esriSatLayer = null;
let osmLayer = null;
let activeBaseLayer = null;
let baseLayerControl = null;

function initMap() {
  if (!window.L) {
    console.warn('Leaflet not loaded, map disabled');
    return;
  }

  map = L.map('map').setView(DEFAULT_POS, 19);

  offlineLayer = L.tileLayer('./assets/tiles/{z}/{x}/{y}.png', {
    maxZoom: 20,
    attribution: 'Offline tiles',
  });

  osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 20,
    attribution: '&copy; OpenStreetMap contributors',
  });

  esriSatLayer = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    { maxZoom: 20, attribution: 'Tiles &copy; Esri' }
  );

  switchBaseLayer(esriSatLayer);

  let hasSwitchedToOffline = false;
  esriSatLayer.on('tileerror', () => {
    if (hasSwitchedToOffline) return;
    hasSwitchedToOffline = true;
    switchBaseLayer(offlineLayer);
    addEvent(t('missingTiles'), '#b57812');
  });

  if (!navigator.onLine) switchBaseLayer(offlineLayer);
  window.addEventListener('offline', () => switchBaseLayer(offlineLayer));
  window.addEventListener('online', () => {
    if (activeBaseLayer === offlineLayer) switchBaseLayer(esriSatLayer);
  });

  renderLayerControl();
  map.on('baselayerchange', (e) => { activeBaseLayer = e.layer; });

  currentMarker = L.circleMarker(DEFAULT_POS, {
    radius: 8, color: '#073b4c', weight: 2, fillColor: '#118ab2', fillOpacity: 0.9,
  }).addTo(map);

  map.on('click', addWaypointByMapClick);
}

function switchBaseLayer(nextLayer) {
  if (!map || activeBaseLayer === nextLayer) return;
  [offlineLayer, esriSatLayer, osmLayer].forEach((layer) => {
    if (layer && map.hasLayer(layer)) map.removeLayer(layer);
  });
  nextLayer.addTo(map);
  activeBaseLayer = nextLayer;
}

function renderLayerControl() {
  if (!map) return;
  if (baseLayerControl) map.removeControl(baseLayerControl);
  baseLayerControl = L.control.layers({
    [t('layerOffline')]: offlineLayer,
    [t('layerSat')]: esriSatLayer,
    [t('layerOsm')]: osmLayer,
  }).addTo(map);
}

// Wait for Leaflet then init map
if (window.L) {
  initMap();
} else {
  window.addEventListener('leaflet-ready', () => initMap());
}

// ==========================================
// SECTION 4: Map helpers (from RTK)
// ==========================================

function setField(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function addEvent(text, color = '#1b6c8d') {
  const ul = document.getElementById('eventList');
  if (!ul) return;
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
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1];
    const b = points[i];
    const d = distanceMeters(a[0], a[1], b[0], b[1]);
    const steps = Math.max(1, Math.ceil(d / stepMeters));
    for (let k = 1; k <= steps; k++) {
      const frac = k / steps;
      result.push([a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac]);
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
      id, lat, lon,
      tolerance_m: Number.isFinite(tolerance) && tolerance > 0 ? tolerance : 0.5,
      max_speed: Number.isFinite(maxSpeed) ? maxSpeed : null,
      reached: false, reached_at: null,
    };
  }).filter(Boolean);
}

function waypointTooltipHtml(wp, idx) {
  return [
    `ID: ${wp.id} (#${idx + 1})`,
    `lat: ${Number(wp.lat).toFixed(8)}`,
    `lon: ${Number(wp.lon).toFixed(8)}`,
    `tolerance_m: ${wp.tolerance_m}`,
    `max_speed: ${wp.max_speed == null ? '-' : wp.max_speed}`,
    `reached: ${wp.reached ? 'yes' : 'no'}`,
  ].join('<br/>');
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
  if (btnUndoNode) btnUndoNode.disabled = !(isEditMode && waypoints.length > 0);
}

function redrawWaypoints(options = {}) {
  if (!map) return;
  const { fitView = true } = options;
  waypointMarkers.forEach((m) => m.remove());
  waypointMarkers = [];
  if (plannedPath) { plannedPath.remove(); plannedPath = null; }

  if (!waypoints.length) {
    setField('reachVal', '0 / 0');
    setField('targetVal', '-');
    setField('distVal', '-');
    simPath = [];
    updateUndoButtonState();
    return;
  }

  plannedPath = L.polyline(waypoints.map((w) => [w.lat, w.lon]), {
    color: '#2b6cb0', weight: 3, opacity: 0.85, dashArray: '8, 6',
  }).addTo(map);

  waypoints.forEach((w, idx) => {
    const m = L.circleMarker([w.lat, w.lon], {
      radius: 6, color: '#4a5568', weight: 2, fillColor: '#cbd5e0', fillOpacity: 0.9,
    }).addTo(map);
    m.bindTooltip(waypointTooltipHtml(w, idx), { permanent: false });
    waypointMarkers[idx] = m;
  });

  simPath = interpolatePath(waypoints.map((w) => [w.lat, w.lon]), 0.4);
  if (fitView) {
    const bounds = L.latLngBounds(waypoints.map((w) => [w.lat, w.lon]));
    if (currentMarker) bounds.extend(currentMarker.getLatLng());
    map.fitBounds(bounds.pad(0.2));
  }
  updateUndoButtonState();
}

function findCurrentTarget() {
  for (let i = 0; i < waypoints.length; i++) {
    if (!waypoints[i].reached) return { wp: waypoints[i], idx: i };
  }
  return null;
}

function updateWaypointStyles(activeIdx) {
  waypointMarkers.forEach((m, idx) => {
    const wp = waypoints[idx];
    let color = '#4a5568', fill = '#cbd5e0';
    if (wp.reached) { color = '#1f8f46'; fill = '#6ee7b7'; }
    else if (idx === activeIdx) { color = '#b57812'; fill = '#f6ad55'; }
    m.setStyle({ color, fillColor: fill });
    m.setTooltipContent(waypointTooltipHtml(wp, idx));
  });
}

function addTrackSegment(a, b, color) {
  if (!map) return;
  const seg = L.polyline([a, b], {
    color, weight: 5, opacity: 0.85, lineCap: 'round',
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

function exportRouteCsv() {
  if (!waypoints.length) { addEvent(t('noRoute'), '#6b7280'); return; }
  const header = ['id', 'lat', 'lon', 'tolerance_m', 'max_speed'];
  const rows = waypoints.map((w, idx) => [
    w.id ?? `${idx}`, Number(w.lat).toFixed(8), Number(w.lon).toFixed(8),
    w.tolerance_m ?? 0.5, w.max_speed ?? '',
  ]);
  const csv = [header, ...rows].map((line) => line.join(',')).join('\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = `route_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click(); URL.revokeObjectURL(url);
  addEvent(t('routeExported', { count: waypoints.length }), '#1f8f46');
}

// ==========================================
// SECTION 5: RTK frame update (from RTK)
// ==========================================

function updateRtkPanel(rtk) {
  if (!rtk || !Object.keys(rtk).length) return;

  const lat = Number(rtk.lat);
  const lon = Number(rtk.lon);

  setField('latVal', Number.isFinite(lat) ? lat.toFixed(8) : '-');
  setField('lonVal', Number.isFinite(lon) ? lon.toFixed(8) : '-');
  setField('srcVal', rtk.source || 'unknown');
  setField('fixVal', `${rtk.fix_quality ?? '-'}`);
  setField('satVal', `${rtk.num_sats ?? '-'}`);
  setField('hdopVal', rtk.hdop == null ? '-' : Number(rtk.hdop).toFixed(2));

  const speedMps = rtk.speed_knots == null ? null : Number(rtk.speed_knots) * 0.514444;
  setField('spdVal', speedMps == null ? '-' : `${speedMps.toFixed(2)} m/s`);

  if (!map || !currentMarker) return;
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;

  const point = [lat, lon];
  currentMarker.setLatLng(point);

  // Track & waypoint logic
  if (!trackPoints.length || distanceMeters(trackPoints.at(-1)[0], trackPoints.at(-1)[1], lat, lon) >= 0.2) {
    trackPoints.push(point);

    let status = 'no_target';
    let color = '#c23a27';
    let activeIdx = -1;

    const targetInfo = findCurrentTarget();
    if (targetInfo) {
      const { wp, idx } = targetInfo;
      activeIdx = idx;
      const d = distanceMeters(lat, lon, wp.lat, wp.lon);

      if (targetCircle) targetCircle.remove();
      targetCircle = L.circle([wp.lat, wp.lon], {
        radius: wp.tolerance_m, color: '#b57812', fillColor: '#fbd38d',
        fillOpacity: 0.18, weight: 2,
      }).addTo(map);

      if (d <= wp.tolerance_m) {
        color = '#1f8f46'; status = 'reached';
        wp.reached = true; wp.reached_at = new Date().toISOString();
        addEvent(t('reachedWp', { id: wp.id, dist: d.toFixed(2) }), '#1f8f46');
      } else if (d <= wp.tolerance_m * 2) {
        color = '#b57812'; status = 'approaching';
      } else {
        color = '#c23a27'; status = 'off_path';
      }

      setField('targetVal', wp.id);
      setField('distVal', `${d.toFixed(2)} m`);
    } else {
      setField('targetVal', waypoints.length ? t('allReached') : '-');
      setField('distVal', '-');
      if (targetCircle) { targetCircle.remove(); targetCircle = null; }
      status = waypoints.length ? 'completed' : 'no_target';
      color = waypoints.length ? '#1f8f46' : '#c23a27';
    }

    if (trackPoints.length >= 2) {
      addTrackSegment(trackPoints.at(-2), trackPoints.at(-1), color);
    }

    updateWaypointStyles(activeIdx);
    const reachedCount = waypoints.filter((w) => w.reached).length;
    setField('reachVal', `${reachedCount} / ${waypoints.length}`);

    logs.push({
      timestamp: new Date().toISOString(),
      lat: lat.toFixed(8), lon: lon.toFixed(8),
      source: rtk.source || '', fix_quality: rtk.fix_quality ?? '',
      num_sats: rtk.num_sats ?? '', hdop: rtk.hdop ?? '',
      speed_mps: speedMps == null ? '' : speedMps.toFixed(3),
      status,
    });
  }
}

// ==========================================
// SECTION 6: Simulation (from RTK)
// ==========================================

function resetReachState() {
  waypoints.forEach((w) => { w.reached = false; w.reached_at = null; });
  updateWaypointStyles(-1);
  setField('reachVal', `0 / ${waypoints.length}`);
}

function stopSimulation() {
  if (simTimer) { clearInterval(simTimer); simTimer = null; }
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

    // Feed simulated RTK frame directly
    updateRtkPanel({
      lat: p[0], lon: p[1], source: 'sim',
      fix_quality: 4, num_sats: 18, hdop: 0.6, speed_knots: speedKnots,
    });
    i++;
  }, 250);
}

function addWaypointByMapClick(e) {
  if (!isEditMode) return;
  const { tolerance, maxSpeed } = getEditParams();
  waypoints.push({
    id: `${waypoints.length}`,
    lat: Number(e.latlng.lat), lon: Number(e.latlng.lng),
    tolerance_m: tolerance, max_speed: maxSpeed,
    reached: false, reached_at: null,
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

function centerToCurrent() {
  if (!map || !currentMarker) return;
  const p = currentMarker.getLatLng();
  map.setView([p.lat, p.lng], Math.max(map.getZoom(), 19));
  addEvent(t('centered'), '#1b6c8d');
}

// ==========================================
// SECTION 7: WebSocket connection
// ==========================================

let ws = null;
let reconnectTimer = null;

function connect() {
  if (ws) return;
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    connectionState = 'connected';
    statusDot.classList.add('connected');
    statusText.textContent = t('connected');
    addEvent(t('wsConnected'), '#1f8f46');
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  };

  ws.onmessage = (event) => {
    let data;
    try { data = JSON.parse(event.data); } catch (e) { return; }

    if (isPaused) return;

    // If simulation is running, skip live RTK updates
    const imu = data.imu || {};
    const rtk = data.rtk || {};

    // 3D rotation
    const src = imu.game_rot || imu.rot;
    if (src) {
      targetQuat.set(src.qi, src.qj, src.qk, src.qr).normalize();
      targetQuat.premultiply(FRAME_CORRECTION);
    }

    // North offset sync
    if (imu.euler?.north_offset_deg !== undefined) {
      northOffsetDeg = (360 - imu.euler.north_offset_deg) % 360;
    }

    if (imu.heading?.deg !== undefined) {
      wsHeadingDeg = Number(imu.heading.deg);
    }
    if (imu.heading?.dir !== undefined) {
      wsHeadingDir = String(imu.heading.dir);
    }
    if (imu.heading?.raw !== undefined) {
      wsHeadingRaw = Number(imu.heading.raw);
      rawHeadingDeg = wsHeadingRaw;
    }

    // Update panels
    updateImuPanel(imu);
    if (!simTimer) {
      updateRtkPanel(rtk);
    }
  };

  ws.onerror = () => {};

  ws.onclose = () => {
    ws = null;
    connectionState = 'reconnecting';
    statusDot.classList.remove('connected');
    statusText.textContent = t('reconnecting');
    reconnectTimer = setTimeout(connect, 3000);
  };
}

connect();

// ==========================================
// SECTION 8: Toolbar event handlers
// ==========================================

// IMU controls
document.getElementById('btn-set-heading').addEventListener('click', () => {
  const userDeg = parseFloat(document.getElementById('manual-heading').value) || 0;
  northOffsetDeg = (rawHeadingDeg - userDeg + 360) % 360;
  sendNorthOffset(northOffsetDeg);
});

document.getElementById('btn-clear-north').addEventListener('click', () => {
  northOffsetDeg = 0;
  document.getElementById('manual-heading').value = 0;
  sendNorthOffset(0);
});

const btnLockYaw = document.getElementById('btn-lock-yaw');
btnLockYaw.addEventListener('click', () => {
  if (topNorthView) return;
  setYawLock(!lockYawOnly);
});

btnTopNorth.addEventListener('click', () => {
  setTopNorthView(!topNorthView);
});
setTopNorthView(true);

const btnPause = document.getElementById('btn-pause');
btnPause.addEventListener('click', () => {
  isPaused = !isPaused;
  btnPause.textContent = isPaused ? 'Resume' : 'Pause';
  btnPause.classList.toggle('active', isPaused);
});

// RTK / map controls
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

btnCenterCurrent.addEventListener('click', centerToCurrent);
btnEditRoute.addEventListener('click', toggleEditRoute);
btnUndoNode.addEventListener('click', undoLastWaypoint);
btnStartSim.addEventListener('click', startSimulation);
btnExportRoute.addEventListener('click', exportRouteCsv);

// Language toggle
function applyLanguage() {
  document.documentElement.lang = currentLang === 'zh' ? 'zh-CN' : 'en';
  document.title = t('title');
  pageTitle.textContent = t('title');
  csvLabel.childNodes[0].nodeValue = `${t('csvLabel')} `;
  btnCenterCurrent.textContent = t('centerCurrent');
  btnEditRoute.textContent = isEditMode ? t('doneEdit') : t('editRoute');
  btnUndoNode.textContent = t('undoNode');
  btnStartSim.textContent = simTimer ? t('stopSim') : t('startSim');
  btnExportRoute.textContent = t('exportRoute');
  btnTopNorth.textContent = t('topNorth');
  cardCurrentTitle.textContent = t('cardCurrent');
  labelLat.textContent = t('latitude');
  labelLon.textContent = t('longitude');
  labelSource.textContent = t('source');
  labelSats.textContent = t('satellites');
  labelSpeed.textContent = t('speed');
  cardMissionTitle.textContent = t('cardMission');
  labelReached.textContent = t('reached');
  labelTarget.textContent = t('target');
  labelDistance.textContent = t('distance');
  cardEventTitle.textContent = t('cardEvents');
  if (connectionState === 'connected') statusText.textContent = t('connected');
  else if (connectionState === 'reconnecting') statusText.textContent = t('reconnecting');
  else statusText.textContent = t('disconnected');
  langToggle.title = currentLang === 'zh' ? t('langTitleEn') : t('langTitleZh');
  renderLayerControl();
}

langToggle.addEventListener('click', () => {
  currentLang = currentLang === 'zh' ? 'en' : 'zh';
  applyLanguage();
});

// Boot
applyLanguage();
addEvent(t('boot1'));
