/**
 * robot_visualizer.js
 * Three.js top-down visualization + control panel for Amiga robot.
 *
 * Connects to WebSocket server (ws://localhost:8796 by default),
 * receives JSON telemetry, and provides WASD / velocity / E-Stop controls.
 */

import * as THREE from 'three';

// ========== Config ==========
const WS_URL = `ws://${window.location.hostname}:${Number(window.location.port || 8795) + 1}`;

// ========== Scene setup ==========
const container = document.getElementById('canvas-container');

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x0d1117, 1);
container.insertBefore(renderer.domElement, container.firstChild);

const scene = new THREE.Scene();

// Top-down camera
const VIEW_HEIGHT = 6.0;
const camera = new THREE.OrthographicCamera(-3, 3, 3, -3, 0.1, 100);
camera.position.set(0, VIEW_HEIGHT, 0);
camera.lookAt(0, 0, 0);

// ========== Lighting ==========
scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5, 10, 5);
scene.add(dirLight);

// ========== Ground grid ==========
const gridHelper = new THREE.GridHelper(8, 16, 0x303040, 0x20202c);
scene.add(gridHelper);

// ========== Robot model ==========
const robotGroup = new THREE.Group();
scene.add(robotGroup);

// Chassis (top-down rectangle)
const chassisGeo = new THREE.BoxGeometry(1.2, 0.15, 0.8);
const chassisMat = new THREE.MeshPhongMaterial({
  color: 0x2a5a3a,
  specular: 0x44aa66,
  shininess: 40,
  emissive: 0x0a1a0e,
});
const chassisMesh = new THREE.Mesh(chassisGeo, chassisMat);
chassisMesh.position.y = 0.08;
robotGroup.add(chassisMesh);

// Chassis edge highlight
const edgesGeo = new THREE.EdgesGeometry(chassisGeo);
const edgesMat = new THREE.LineBasicMaterial({ color: 0x3fb950 });
chassisMesh.add(new THREE.LineSegments(edgesGeo, edgesMat));

// Wheels (4 dark boxes at corners)
const wheelGeo = new THREE.BoxGeometry(0.15, 0.12, 0.25);
const wheelMat = new THREE.MeshPhongMaterial({ color: 0x222222 });
const wheelPositions = [
  [0.55, 0.06, 0.45],
  [0.55, 0.06, -0.45],
  [-0.55, 0.06, 0.45],
  [-0.55, 0.06, -0.45],
];
wheelPositions.forEach(([x, y, z]) => {
  const wheel = new THREE.Mesh(wheelGeo, wheelMat);
  wheel.position.set(x, y, z);
  robotGroup.add(wheel);
});

// Direction arrow (cone pointing forward = +X)
const arrowLen = 0.9;
const dirArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0.2, 0),
  arrowLen, 0xff8800, 0.18, 0.12
);
robotGroup.add(dirArrow);

// Front marker (small green box at +X end)
const frontMarkerGeo = new THREE.BoxGeometry(0.08, 0.06, 0.3);
const frontMarkerMat = new THREE.MeshPhongMaterial({ color: 0x3fb950, emissive: 0x1a5a2a });
const frontMarker = new THREE.Mesh(frontMarkerGeo, frontMarkerMat);
frontMarker.position.set(0.62, 0.12, 0);
robotGroup.add(frontMarker);

// Speed vector arrow (dynamic length)
const speedArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0.25, 0),
  0.01, 0x39d0d8, 0.1, 0.06
);
speedArrow.visible = false;
robotGroup.add(speedArrow);

// ========== Heading trail dots ==========
const TRAIL_MAX = 60;
const trailPoints = [];
const trailGeo = new THREE.BufferGeometry();
const trailMat = new THREE.PointsMaterial({ color: 0x58a6ff, size: 3, sizeAttenuation: false, transparent: true, opacity: 0.5 });
const trailDots = new THREE.Points(trailGeo, trailMat);
scene.add(trailDots);
let trailCounter = 0;

// ========== State ==========
let isPaused = false;
let currentHeadingDeg = 0;
let currentSpeed = 0;

// ========== Resize handler ==========
function onResize() {
  const w = container.clientWidth;
  const h = container.clientHeight;
  const aspect = w / h;
  const viewSize = 3;
  camera.left = -viewSize * aspect;
  camera.right = viewSize * aspect;
  camera.top = viewSize;
  camera.bottom = -viewSize;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h, false);
}
const resizeObserver = new ResizeObserver(onResize);
resizeObserver.observe(container);
onResize();

// ========== Animation loop ==========
function animate() {
  requestAnimationFrame(animate);

  // Rotate robot group based on heading (Y-axis rotation, heading 0=+X)
  const headingRad = -currentHeadingDeg * Math.PI / 180;
  robotGroup.rotation.y = headingRad;

  // Speed vector arrow
  const absSpeed = Math.abs(currentSpeed);
  if (absSpeed > 0.01) {
    speedArrow.visible = true;
    const dir = currentSpeed >= 0
      ? new THREE.Vector3(1, 0, 0)
      : new THREE.Vector3(-1, 0, 0);
    speedArrow.setDirection(dir);
    speedArrow.setLength(Math.min(absSpeed * 2, 2.0), 0.1, 0.06);
  } else {
    speedArrow.visible = false;
  }

  // Trail dots (record position every 3 frames)
  trailCounter++;
  if (trailCounter % 3 === 0 && absSpeed > 0.01) {
    trailPoints.push(new THREE.Vector3(0, 0.02, 0)); // robot is always at origin in this view
  }

  renderer.render(scene, camera);
}
animate();

// ========== Data panel helpers ==========
function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function fmt(v, decimals = 3) {
  if (v === undefined || v === null) return '\u2013';
  return Number(v).toFixed(decimals);
}

// State color mapping
const STATE_DOT_CLASS = {
  'BOOT': 'boot',
  'MANUAL_READY': 'ready',
  'MANUAL_ACTIVE': 'active',
  'AUTO_READY': 'ready',
  'AUTO_ACTIVE': 'active',
  'CC_ACTIVE': 'active',
  'ESTOPPED': 'estopped',
};

function updateSpeedBar(barId, value, maxVal = 1.0) {
  const bar = document.getElementById(barId);
  if (!bar) return;
  // value ranges from -maxVal to +maxVal
  // bar center is at 50% (zero), extends left for negative, right for positive
  const pct = Math.min(Math.abs(value) / maxVal, 1.0) * 50;
  if (value >= 0) {
    bar.style.left = '50%';
    bar.style.width = pct + '%';
  } else {
    bar.style.left = (50 - pct) + '%';
    bar.style.width = pct + '%';
  }
}

function updateBattery(soc) {
  setText('batteryPct', soc + '%');
  const bar = document.getElementById('batteryBar');
  if (!bar) return;
  bar.style.width = soc + '%';
  if (soc > 50) bar.style.background = 'var(--green)';
  else if (soc > 20) bar.style.background = 'var(--yellow)';
  else bar.style.background = 'var(--red)';
}

function formatUptime(seconds) {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = Math.floor(seconds % 60);
  if (h > 0) return `${h}h ${m}m ${s}s`;
  if (m > 0) return `${m}m ${s}s`;
  return `${s}s`;
}

function updatePanel(data) {
  // State
  const stateName = data.state_name || 'BOOT';
  setText('stateName', stateName);
  const stateDot = document.getElementById('stateDot');
  if (stateDot) {
    const cls = STATE_DOT_CLASS[stateName] || 'unknown';
    stateDot.className = 'state-dot ' + cls;
  }
  setText('firmwareStatus', data.firmware_status || 'UNKNOWN');

  // Battery
  updateBattery(data.soc || 0);

  // Speed
  setText('measSpeed', fmt(data.meas_speed));
  setText('cmdSpeed', fmt(data.cmd_speed));
  updateSpeedBar('measSpeedBar', data.meas_speed || 0);

  // Angular Rate
  setText('measAngRate', fmt(data.meas_ang_rate));
  setText('cmdAngRate', fmt(data.cmd_ang_rate));
  updateSpeedBar('measAngRateBar', data.meas_ang_rate || 0);

  // Odometry
  setText('headingDeg', fmt(data.heading_est_deg, 1) + '\u00B0');
  setText('totalDistance', fmt(data.total_distance_m, 2) + ' m');

  // Status
  setText('hzDisplay', data.hz !== undefined && data.hz !== null ? `${data.hz} Hz` : '\u2013 Hz');
  setText('uptimeDisplay', 'Uptime: ' + formatUptime(data.uptime_s || 0));

  // Update toggle button appearance
  const toggleBtn = document.getElementById('btn-toggle-state');
  if (toggleBtn) {
    const isActive = stateName.includes('ACTIVE');
    toggleBtn.classList.toggle('is-active', isActive);
    toggleBtn.textContent = isActive ? 'Set READY' : 'Set ACTIVE';
  }
}

// ========== WebSocket connection ==========
const statusDot  = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');

let ws = null;
let reconnectTimer = null;

function wsSend(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

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
      currentHeadingDeg = data.heading_est_deg || 0;
      currentSpeed = data.meas_speed || 0;
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
    statusText.textContent = 'Disconnected \u2013 reconnecting...';
    reconnectTimer = setTimeout(connect, 2000);
  };
}

connect();

// ========== WASD button handlers ==========
const wasdButtons = document.querySelectorAll('.wasd-btn');
wasdButtons.forEach(btn => {
  btn.addEventListener('mousedown', () => {
    const key = btn.dataset.key;
    if (key !== undefined) {
      wsSend({ wasd: key });
    }
  });
});

// ========== Keyboard controls ==========
const keysDown = new Set();

document.addEventListener('keydown', (e) => {
  // Skip if focus is on an input element
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

  const key = e.key.toLowerCase();

  // Throttle: don't re-send if already held
  if (keysDown.has(key)) return;
  keysDown.add(key);

  // Visual feedback on buttons
  const btnMap = { 'w': 'btn-w', 'a': 'btn-a', 's': 'btn-s', 'd': 'btn-d', ' ': 'btn-stop' };
  const btnId = btnMap[key];
  if (btnId) {
    const btn = document.getElementById(btnId);
    if (btn) btn.classList.add('pressed');
  }

  if (['w', 'a', 's', 'd', ' '].includes(key)) {
    e.preventDefault();
    wsSend({ wasd: key });
  } else if (key === 'enter') {
    e.preventDefault();
    wsSend({ wasd: '\r' });
  }
});

document.addEventListener('keyup', (e) => {
  const key = e.key.toLowerCase();
  keysDown.delete(key);

  const btnMap = { 'w': 'btn-w', 'a': 'btn-a', 's': 'btn-s', 'd': 'btn-d', ' ': 'btn-stop' };
  const btnId = btnMap[key];
  if (btnId) {
    const btn = document.getElementById(btnId);
    if (btn) btn.classList.remove('pressed');
  }
});

// ========== Toggle Ready/Active ==========
document.getElementById('btn-toggle-state').addEventListener('click', () => {
  wsSend({ wasd: '\r' });
});

// ========== Sliders ==========
const speedSlider = document.getElementById('slider-speed');
const angRateSlider = document.getElementById('slider-ang-rate');

function sendSliderVelocity() {
  const speed = parseFloat(speedSlider.value);
  const angRate = parseFloat(angRateSlider.value);
  setText('sliderSpeedVal', speed.toFixed(3));
  setText('sliderAngRateVal', angRate.toFixed(3));
  wsSend({ velocity: { speed, ang_rate: angRate } });
}

speedSlider.addEventListener('input', sendSliderVelocity);
angRateSlider.addEventListener('input', sendSliderVelocity);

// Double-click slider to reset to 0
speedSlider.addEventListener('dblclick', () => { speedSlider.value = 0; sendSliderVelocity(); });
angRateSlider.addEventListener('dblclick', () => { angRateSlider.value = 0; sendSliderVelocity(); });

// ========== E-Stop ==========
document.getElementById('btn-estop').addEventListener('click', () => {
  wsSend({ estop: true });
  // Reset sliders to 0
  speedSlider.value = 0;
  angRateSlider.value = 0;
  setText('sliderSpeedVal', '0.000');
  setText('sliderAngRateVal', '0.000');
});

// ========== Toolbar buttons ==========

// Reset View
document.getElementById('btn-reset-view').addEventListener('click', () => {
  camera.position.set(0, VIEW_HEIGHT, 0);
  camera.lookAt(0, 0, 0);
});

// Pause / Resume
const btnPause = document.getElementById('btn-pause');
btnPause.addEventListener('click', () => {
  isPaused = !isPaused;
  btnPause.textContent = isPaused ? 'Resume' : 'Pause';
  btnPause.classList.toggle('active', isPaused);
});
