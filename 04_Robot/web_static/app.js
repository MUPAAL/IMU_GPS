// Farm Robot Control — Application Logic
// Connects to robot_bridge WebSocket for joystick / state control.
// IMU, RTK, nav_status data are proxied from nav_bridge via robot_bridge.

// ── Config ─────────────────────────────────────────────────
const WS_PORT = parseInt(window.location.port, 10) + 1;
const WS_URL  = `ws://${window.location.hostname}:${WS_PORT}/`;
const HEARTBEAT_INTERVAL_MS   = 500;
const JOYSTICK_SEND_INTERVAL_MS = 100;  // 10 Hz
const DEADZONE = 0.15;

// MAX_LINEAR_VEL / MAX_ANGULAR_VEL injected by server as data-* attributes
const MAX_LINEAR  = parseFloat(document.documentElement.dataset.maxLinear  || "1.0");
const MAX_ANGULAR = parseFloat(document.documentElement.dataset.maxAngular || "1.0");

// ── Speed ratio ──────────────────────────────────────────────
let speedRatio = 0.5;

// ── State ───────────────────────────────────────────────────
let ws = null;
let joystickActive = false;
let currentLinear  = 0.0;
let currentAngular = 0.0;
let currentForce   = 0.0;
let heartbeatTimer = null;
let joySendTimer   = null;

let controlStateActive = false;
let isRecording = false;

function toggleControlState() {
  sendMsg({ type: "toggle_state" });
  document.getElementById('state-btn').disabled = true;
}

function updateStateBtn() {
  const btn = document.getElementById('state-btn');
  const lbl = document.getElementById('state-label');
  if (controlStateActive) {
    btn.className = 'state-active';
    lbl.textContent = 'DEACTIVATE';
  } else {
    btn.className = 'state-ready';
    lbl.textContent = 'ACTIVATE';
  }
}

function toggleRecording() {
  sendMsg({ type: "set_recording", enabled: !isRecording });
}

function handleRecStatus(msg) {
  isRecording = msg.recording;
  const btn      = document.getElementById('rec-btn');
  const filename = document.getElementById('rec-filename');
  if (isRecording) {
    btn.className   = 'rec-active';
    filename.textContent = msg.filename;
  } else {
    btn.className   = 'rec-idle';
    filename.textContent = '—';
  }
}

// ── WebSocket ───────────────────────────────────────────────
function connect() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    setStatus(true);
    scheduleHeartbeat();
    scheduleJoySend();
  };

  ws.onclose = () => {
    setStatus(false);
    clearTimers();
    setTimeout(connect, 2000);
  };

  ws.onerror = () => { ws.close(); };

  ws.onmessage = (evt) => {
    try {
      const msg = JSON.parse(evt.data);
      const handlers = {
        imu:          handleIMU,
        rtk:          handleRTK,
        odom:         handleOdom,
        state_status: handleStateStatus,
        rec_status:   handleRecStatus,
      };
      const handler = handlers[msg.type];
      if (handler) handler(msg);
    } catch(e) {}
  };
}

function sendMsg(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// ── Timers ──────────────────────────────────────────────────
function scheduleHeartbeat() {
  heartbeatTimer = setInterval(() => sendMsg({ type: "heartbeat" }), HEARTBEAT_INTERVAL_MS);
}

function scheduleJoySend() {
  joySendTimer = setInterval(() => {
    if (joystickActive) {
      sendMsg({ type: "joystick", linear: currentLinear, angular: currentAngular, force: currentForce });
    }
  }, JOYSTICK_SEND_INTERVAL_MS);
}

function clearTimers() {
  if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
  if (joySendTimer)   { clearInterval(joySendTimer);   joySendTimer   = null; }
}

// ── Status UI ───────────────────────────────────────────────
function setStatus(online) {
  const dot  = document.getElementById('status-dot');
  const text = document.getElementById('status-text');
  const warn = document.getElementById('warn-banner');
  const btn  = document.getElementById('state-btn');
  if (online) {
    dot.className = 'online';
    text.textContent = 'ONLINE';
    warn.classList.remove('visible');
    btn.disabled = false;
  } else {
    dot.className = 'offline';
    text.textContent = 'OFFLINE';
    warn.classList.add('visible');
    btn.disabled = true;
  }
}

// ── Odometry ─────────────────────────────────────────────────
function handleOdom(msg) {}

// ── IMU HUD ──────────────────────────────────────────────────
function handleIMU(msg) {}

// ── RTK HUD ──────────────────────────────────────────────────
function handleRTK(msg) {}

// ── State status ─────────────────────────────────────────────
function handleStateStatus(msg) {
  controlStateActive = msg.active;
  updateStateBtn();
  document.getElementById('state-btn').disabled = false;
}

// ── Direction label ───────────────────────────────────────────
function dirLabel(linear, angular) {
  const fwd   = linear  >  0.05;
  const back  = linear  < -0.05;
  const left  = angular >  0.05;
  const right = angular < -0.05;
  if (!fwd && !back && !left && !right) return '■ STOP';
  let s = '';
  if (fwd)   s += '↑';
  if (back)  s += '↓';
  if (left)  s += '←';
  if (right) s += '→';
  return s;
}

function updateJoyUI() {
  document.getElementById('joy-force').textContent   = currentForce.toFixed(2);
  document.getElementById('joy-linear').textContent  = fmt2(currentLinear)  + ' m/s';
  document.getElementById('joy-angular').textContent = fmt2(currentAngular) + ' r/s';
  document.getElementById('joy-dir').textContent     = dirLabel(currentLinear, currentAngular);
}

// ── Init ─────────────────────────────────────────────────────
window.addEventListener('load', () => {
  const zone = document.getElementById('joystick-zone');
  const hint = document.getElementById('joystick-hint');

  const manager = nipplejs.create({
    zone:        zone,
    mode:        'static',
    position:    { left: '50%', top: '45%' },
    size:        160,
    color:       '#00d4ff',
    restOpacity: 0.6,
  });

  manager.on('start', () => {
    joystickActive = true;
    hint.style.display = 'none';
  });

  manager.on('move', (evt, data) => {
    const force    = Math.min(data.force, 1.0);
    const angleRad = data.angle.radian;
    const rawX     = Math.cos(angleRad) * force;
    const rawY     = Math.sin(angleRad) * force;

    if (force < DEADZONE) {
      currentLinear = currentAngular = currentForce = 0.0;
    } else {
      currentLinear  =  rawY * MAX_LINEAR  * speedRatio;
      currentAngular = -rawX * MAX_ANGULAR * speedRatio;
      currentForce   = force;
    }
    updateJoyUI();
  });

  manager.on('end', () => {
    joystickActive = false;
    currentLinear = currentAngular = currentForce = 0.0;
    updateJoyUI();
    sendMsg({ type: "joystick", linear: 0.0, angular: 0.0, force: 0.0 });
    hint.style.display = 'block';
  });

  // State button
  const stateBtn = document.getElementById('state-btn');
  stateBtn.addEventListener('click', toggleControlState);
  stateBtn.addEventListener('touchend', (e) => { e.preventDefault(); toggleControlState(); });

  // REC button
  const recBtn = document.getElementById('rec-btn');
  recBtn.addEventListener('click', toggleRecording);

  // Speed slider
  const speedSlider = document.getElementById('speed-slider');
  speedSlider.addEventListener('input', (e) => {
    speedRatio = parseInt(e.target.value) / 100;
    document.getElementById('speed-value').textContent = e.target.value + '%';
  });

  connect();
});
