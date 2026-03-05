/**
 * imu_visualizer.js
 * Three.js 3D visualization for BNO085 IMU data.
 *
 * Connects to WebSocket server (ws://localhost:8766 by default),
 * receives JSON frames, and animates an IMU box with body-frame axes.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ========== Config ==========
// HTTP server is on :8765, WebSocket is on :8766 (ws_port + 1)
const WS_URL = `ws://${window.location.hostname}:${Number(window.location.port || 8765) + 1}`;

// ========== Scene setup ==========
const container = document.getElementById('canvas-container');

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x0d1117, 1);
container.insertBefore(renderer.domElement, container.firstChild);

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.position.set(1.5, 1.5, 2.5);
camera.lookAt(0, 0, 0);

// Orbit controls
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.5;
controls.maxDistance = 10;

// ========== Lighting ==========
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
dirLight.position.set(5, 8, 5);
scene.add(dirLight);

const fillLight = new THREE.DirectionalLight(0x8888ff, 0.3);
fillLight.position.set(-3, -2, -3);
scene.add(fillLight);

// ========== World reference grid & axes ==========
const gridHelper = new THREE.GridHelper(3, 6, 0x303040, 0x20202c);
scene.add(gridHelper);

// World coordinate axes (thin ArrowHelper, fixed)
function worldAxis(dir, color, length = 1.2) {
  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(...dir).normalize(),
    new THREE.Vector3(0, 0, 0),
    length, color, 0.08, 0.06
  );
  return arrow;
}
scene.add(worldAxis([1, 0, 0], 0xff4444, 1.0)); // X red
scene.add(worldAxis([0, 1, 0], 0x44ff44, 1.0)); // Y green
scene.add(worldAxis([0, 0, 1], 0x4444ff, 1.0)); // Z blue

// Ground-projected heading arrow (chip X-axis projected to horizontal plane)
// Orange, longer than body axes, added to scene (not imuGroup) so it stays on ground
const headingArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0.01, 0),  // slight Y offset to avoid z-fighting with grid
  1.6, 0xff8800, 0.18, 0.10
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

// ========== IMU box (pivot group) ==========
const imuGroup = new THREE.Group();
scene.add(imuGroup);

// Main box geometry (represents the PCB / sensor chip)
const boxGeo = new THREE.BoxGeometry(1.2, 0.2, 0.8);
const boxMat = new THREE.MeshPhongMaterial({
  color: 0x1a4a6e,
  specular: 0x4488cc,
  shininess: 60,
  emissive: 0x0a1a2e,
});
const boxMesh = new THREE.Mesh(boxGeo, boxMat);
imuGroup.add(boxMesh);

// PCB edge highlight
const edgesGeo = new THREE.EdgesGeometry(boxGeo);
const edgesMat = new THREE.LineBasicMaterial({ color: 0x39d0d8, linewidth: 1 });
imuGroup.add(new THREE.LineSegments(edgesGeo, edgesMat));

// "Chip" on top
const chipGeo = new THREE.BoxGeometry(0.3, 0.05, 0.3);
const chipMat = new THREE.MeshPhongMaterial({ color: 0x222222, specular: 0x555555, shininess: 40 });
const chipMesh = new THREE.Mesh(chipGeo, chipMat);
chipMesh.position.y = 0.125;
imuGroup.add(chipMesh);

// Body-frame axes (rotate with the box)
const ARROW_LEN = 0.7;
const bodyXArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0),
  ARROW_LEN, 0xff2222, 0.10, 0.07
);
const bodyYArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0),
  ARROW_LEN, 0x22ff22, 0.10, 0.07
);
const bodyZArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0),
  ARROW_LEN, 0x2222ff, 0.10, 0.07
);
imuGroup.add(bodyXArrow, bodyYArrow, bodyZArrow);

// Body axis labels
const labelX = makeLabel('X', [ARROW_LEN + 0.12, 0, 0], '#ff4444');
const labelY = makeLabel('Y', [0, ARROW_LEN + 0.12, 0], '#44ff44');
const labelZ = makeLabel('Z', [0, 0, ARROW_LEN + 0.12], '#4444ff');
imuGroup.add(labelX, labelY, labelZ);

// ========== Coordinate frame correction ==========
// BNO085 uses Z-up (right-hand), Three.js uses Y-up (right-hand).
// Correction: rotate -90° around X  →  BNO085-Z maps to Three.js-Y
// q = (sin(-45°), 0, 0, cos(-45°)) = (-√2/2, 0, 0, √2/2)
const FRAME_CORRECTION = new THREE.Quaternion(-Math.SQRT1_2, 0, 0, Math.SQRT1_2);

// ========== Current quaternion (smoothed) ==========
const currentQuat = new THREE.Quaternion();  // starts at identity
const targetQuat  = new THREE.Quaternion();

// ========== Control state ==========
let northOffsetDeg = 0;    // heading offset applied for true-north calibration
let rawHeadingDeg  = 0;    // latest raw (uncalibrated) heading from IMU
let isPaused       = false; // whether to freeze live data updates
let lockYawOnly    = false; // whether to show yaw only (strip pitch/roll)

// ========== Resize handler ==========
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

// ========== Compass HUD ==========
const compassCanvas = document.getElementById('compassCanvas');
const compassCtx = compassCanvas.getContext('2d');
const CW = 110, CH = 110, CR = 44, CX = 55, CY = 55;
compassCanvas.width = CW; compassCanvas.height = CH;

function drawCompass(deg) {
  compassCtx.clearRect(0, 0, CW, CH);
  // Outer ring
  compassCtx.strokeStyle = '#30363d'; compassCtx.lineWidth = 1.5;
  compassCtx.beginPath(); compassCtx.arc(CX, CY, CR, 0, Math.PI * 2); compassCtx.stroke();
  // Cardinal letters (N/E/S/W)
  const cardinals = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
  cardinals.forEach(([label, a]) => {
    const r = a * Math.PI / 180;
    compassCtx.fillStyle = label === 'N' ? '#f85149' : '#7d8590';
    compassCtx.font = 'bold 10px monospace';
    compassCtx.textAlign = 'center'; compassCtx.textBaseline = 'middle';
    compassCtx.fillText(label, CX + (CR - 9) * Math.sin(r), CY - (CR - 9) * Math.cos(r));
  });
  // Heading needle (orange)
  const rad = (deg - 90) * Math.PI / 180; // -90: canvas 0°=right, we want 0°=up
  compassCtx.strokeStyle = '#ff8800'; compassCtx.lineWidth = 2.5;
  compassCtx.lineCap = 'round';
  compassCtx.beginPath();
  compassCtx.moveTo(CX, CY);
  compassCtx.lineTo(CX + CR * 0.75 * Math.cos(rad), CY + CR * 0.75 * Math.sin(rad));
  compassCtx.stroke();
  // Center dot
  compassCtx.fillStyle = '#ff8800';
  compassCtx.beginPath(); compassCtx.arc(CX, CY, 3, 0, Math.PI * 2); compassCtx.fill();
}

// ========== Heading display ==========
const COMPASS_DIRS = ['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW'];
function updateHeadingDisplay(deg) {
  const idx = Math.round(deg / 22.5) % 16;
  setText('heading_deg', deg.toFixed(1) + '°');
  setText('heading_dir', COMPASS_DIRS[idx]);
}

// ========== Animation loop ==========
const SLERP_SPEED = 0.2;  // smoothing factor per frame

function animate() {
  requestAnimationFrame(animate);
  // Smooth rotation
  currentQuat.slerp(targetQuat, SLERP_SPEED);

  // Lock Yaw: strip pitch & roll from imuGroup, keep only yaw
  if (lockYawOnly) {
    const euler = new THREE.Euler().setFromQuaternion(currentQuat, 'YXZ');
    const yawOnlyQuat = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(0, euler.y, 0, 'YXZ')
    );
    imuGroup.quaternion.copy(yawOnlyQuat);
  } else {
    imuGroup.quaternion.copy(currentQuat);
  }

  // Project chip X-axis onto horizontal plane and update heading arrow
  const chipX = new THREE.Vector3(1, 0, 0).applyQuaternion(currentQuat);
  chipX.y = 0;
  if (chipX.lengthSq() > 1e-4) {
    chipX.normalize();
    headingArrow.setDirection(chipX.clone());
  }
  // Raw heading: 0° = +X world axis, increases clockwise from above
  rawHeadingDeg = ((Math.atan2(chipX.z, chipX.x) * 180 / Math.PI) + 360) % 360;
  const displayHeadingDeg = (rawHeadingDeg - northOffsetDeg + 360) % 360;
  updateHeadingDisplay(displayHeadingDeg);
  drawCompass(displayHeadingDeg);

  controls.update();
  renderer.render(scene, camera);
}
animate();

// ========== Data panel helpers ==========
function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function fmt(v, decimals = 3) {
  if (v === undefined || v === null) return '–';
  return Number(v).toFixed(decimals);
}

const CAL_LABELS = ['Unknown', 'Low', 'Medium', 'High'];
const CAL_COLORS = ['cal-0', 'cal-1', 'cal-2', 'cal-3'];

function updatePanel(data) {
  // Rotation Vector
  const rot = data.rot || {};
  setText('rot_qi', fmt(rot.qi, 4));
  setText('rot_qj', fmt(rot.qj, 4));
  setText('rot_qk', fmt(rot.qk, 4));
  setText('rot_qr', fmt(rot.qr, 4));
  setText('rot_acc', rot.acc !== undefined ? fmt(rot.acc, 4) + ' rad' : '–');

  // Euler angles (computed by Python)
  const euler = data.euler || {};
  setText('euler_roll',  euler.roll  !== undefined ? fmt(euler.roll,  1) + '°' : '–');
  setText('euler_pitch', euler.pitch !== undefined ? fmt(euler.pitch, 1) + '°' : '–');
  setText('euler_yaw',   euler.yaw   !== undefined ? fmt(euler.yaw,   1) + '°' : '–');

  // Accelerometer
  const accel = data.accel || {};
  setText('accel_x', fmt(accel.x));
  setText('accel_y', fmt(accel.y));
  setText('accel_z', fmt(accel.z));

  // Linear acceleration
  const lin = data.lin_accel || {};
  setText('lin_x', fmt(lin.x));
  setText('lin_y', fmt(lin.y));
  setText('lin_z', fmt(lin.z));

  // Gravity
  const grav = data.gravity || {};
  setText('grav_x', fmt(grav.x));
  setText('grav_y', fmt(grav.y));
  setText('grav_z', fmt(grav.z));

  // Gyroscope
  const gyro = data.gyro || {};
  setText('gyro_x', fmt(gyro.x, 4));
  setText('gyro_y', fmt(gyro.y, 4));
  setText('gyro_z', fmt(gyro.z, 4));

  // Game rotation
  const game = data.game_rot || {};
  setText('game_qi', fmt(game.qi, 4));
  setText('game_qj', fmt(game.qj, 4));
  setText('game_qk', fmt(game.qk, 4));
  setText('game_qr', fmt(game.qr, 4));

  // Magnetometer
  const mag = data.mag || {};
  setText('mag_x', fmt(mag.x, 2));
  setText('mag_y', fmt(mag.y, 2));
  setText('mag_z', fmt(mag.z, 2));

  // Steps
  setText('steps', data.steps !== undefined ? data.steps : '–');

  // Calibration badge
  const cal = data.cal !== undefined ? data.cal : -1;
  const calEl = document.getElementById('calBadge');
  if (calEl) {
    calEl.className = 'cal-badge ' + (cal >= 0 && cal <= 3 ? CAL_COLORS[cal] : 'cal-0');
  }
  const calLabel = cal >= 0 && cal <= 3 ? CAL_LABELS[cal] : '–';
  setText('calText', `Cal: ${calLabel}`);

  // Hz
  setText('hzDisplay', data.hz !== undefined ? `${data.hz} Hz` : '– Hz');
}

// ========== WebSocket connection ==========
const statusDot  = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');

let ws = null;
let reconnectTimer = null;

function connect() {
  if (ws) return;
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    console.log('[WS] Connected to', WS_URL);
    statusDot.classList.add('connected');
    statusText.textContent = 'Connected';
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  };

  ws.onmessage = (event) => {
    let data;
    try {
      data = JSON.parse(event.data);
    } catch (e) {
      console.warn('[WS] Failed to parse JSON:', e);
      return;
    }

    if (!isPaused) {
      // Update 3D rotation using Game Rotation Vector (no magnetometer required,
      // stable indoors). Falls back to Rotation Vector if game_rot is absent.
      const src = data.game_rot || data.rot;
      if (src) {
        // BNO085 quaternion (i,j,k,real) → THREE.js (x,y,z,w)
        // Then premultiply by frame correction (BNO085 Z-up → Three.js Y-up)
        targetQuat.set(src.qi, src.qj, src.qk, src.qr).normalize();
        targetQuat.premultiply(FRAME_CORRECTION);
      }

      // Update data panel
      updatePanel(data);
    }
  };

  ws.onerror = (err) => {
    console.error('[WS] Error:', err);
  };

  ws.onclose = () => {
    console.log('[WS] Disconnected. Reconnecting in 2s...');
    ws = null;
    statusDot.classList.remove('connected');
    statusText.textContent = 'Disconnected – reconnecting...';
    reconnectTimer = setTimeout(connect, 2000);
  };
}

// Start connection
connect();

// ========== Toolbar button handlers ==========

// Reset View
document.getElementById('btn-reset-view').addEventListener('click', () => {
  camera.position.set(1.5, 1.5, 2.5);
  camera.lookAt(0, 0, 0);
  controls.reset();
});

// Lock Yaw toggle
const btnLockYaw = document.getElementById('btn-lock-yaw');
btnLockYaw.addEventListener('click', () => {
  lockYawOnly = !lockYawOnly;
  btnLockYaw.classList.toggle('active', lockYawOnly);
});

// Pause / Resume toggle
const btnPause = document.getElementById('btn-pause');
btnPause.addEventListener('click', () => {
  isPaused = !isPaused;
  btnPause.textContent = isPaused ? 'Resume' : 'Pause';
  btnPause.classList.toggle('active', isPaused);
});

// Clear North Offset
document.getElementById('btn-clear-north').addEventListener('click', () => {
  northOffsetDeg = 0;
  document.getElementById('manual-heading').value = 0;
});

// Set Heading manually (user says "IMU is currently pointing at X degrees")
document.getElementById('btn-set-heading').addEventListener('click', () => {
  const userDeg = parseFloat(document.getElementById('manual-heading').value) || 0;
  northOffsetDeg = (rawHeadingDeg - userDeg + 360) % 360;
});
