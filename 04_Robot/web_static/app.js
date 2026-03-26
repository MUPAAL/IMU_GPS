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
let navActive = false;

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
        nav_status:   handleNavStatus,
        nav_complete: handleNavComplete,
        nav_warning:  handleNavWarning,
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
    if (joystickActive && !navActive) {
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
// AmigaControlState integers from Amiga SDK
const AMIGA_STATE = { 0: 'BOOT', 1: 'INIT', 2: 'STOPPED', 3: 'READY', 4: 'ACTIVE', 5: 'ERROR' };

function handleOdom(msg) {
  document.getElementById('odom-v').textContent = (msg.v   != null) ? msg.v.toFixed(3)   : '--';
  document.getElementById('odom-w').textContent = (msg.w   != null) ? msg.w.toFixed(3)   : '--';
  document.getElementById('amiga-soc').textContent = (msg.soc != null) ? msg.soc + '%' : '--%';
  const stateEl = document.getElementById('odom-state');
  if (msg.state != null) {
    stateEl.textContent = AMIGA_STATE[msg.state] || String(msg.state);
    stateEl.style.color = msg.state === 4 ? 'var(--green)' : 'var(--dim)';
  }
}

// ── IMU HUD ──────────────────────────────────────────────────
function fmt1(v) { return (v >= 0 ? '+' : '') + v.toFixed(1) + '°'; }
function fmt2(v) { return (v >= 0 ? '+' : '') + v.toFixed(2); }

function handleIMU(msg) {
  const euler = msg.euler || {};
  if (euler.roll  !== undefined) document.getElementById('imu-roll').textContent  = fmt1(euler.roll);
  if (euler.pitch !== undefined) document.getElementById('imu-pitch').textContent = fmt1(euler.pitch);
  if (euler.yaw   !== undefined) document.getElementById('imu-yaw').textContent   = fmt1(euler.yaw);
}

// ── RTK HUD ──────────────────────────────────────────────────
const FIX_LABELS = {
  0: { text: "NO FIX",  cls: "" },
  1: { text: "GPS",     cls: "fix-gps" },
  2: { text: "DGPS",    cls: "fix-gps" },
  4: { text: "RTK FIX", cls: "fix-rtk" },
  5: { text: "RTK FLT", cls: "fix-gps" },
};

function handleRTK(msg) {
  const dot   = document.getElementById('rtk-live-dot');
  const badge = document.getElementById('rtk-fix-badge');

  if (!msg.available) {
    dot.className = ''; dot.title = 'RTK OFFLINE';
    badge.textContent = 'OFFLINE'; badge.className = '';
    return;
  }

  dot.className = 'live'; dot.title = 'RTK LIVE';
  const fi = FIX_LABELS[msg.fix_quality || 0] || { text: `FIX(${msg.fix_quality})`, cls: "fix-gps" };
  badge.textContent = fi.text; badge.className = fi.cls;

  const fmt = (v, d) => (v != null) ? v.toFixed(d) : '--';
  document.getElementById('rtk-lat').textContent  = fmt(msg.lat, 7);
  document.getElementById('rtk-lon').textContent  = fmt(msg.lon, 7);
  document.getElementById('rtk-alt').textContent  = fmt(msg.alt, 2);
  document.getElementById('rtk-sats').textContent = (msg.num_sats != null) ? msg.num_sats : '--';
  document.getElementById('rtk-hdop').textContent = fmt(msg.hdop, 2);
  document.getElementById('rtk-spd').textContent  = fmt(msg.speed_knots, 2);
}

// ── State status ─────────────────────────────────────────────
function handleStateStatus(msg) {
  controlStateActive = msg.active;
  updateStateBtn();
  document.getElementById('state-btn').disabled = false;
}

// ── Navigation status ─────────────────────────────────────────
function handleNavStatus(msg) {
  const state = msg.state || 'idle';
  navActive = (state === 'navigating');

  const statusPanel = document.getElementById('nav-status-panel');
  if (navActive || state === 'finished') {
    statusPanel.classList.add('visible');
  } else {
    statusPanel.classList.remove('visible');
  }

  const overlay = document.getElementById('auto-overlay');
  if (navActive) { overlay.classList.add('visible'); }
  else           { overlay.classList.remove('visible'); }

  const prog = msg.progress || [0, 0];
  document.getElementById('nav-progress').textContent    = prog[0] + ' / ' + prog[1];
  document.getElementById('nav-dist').textContent        = (msg.distance_m     != null) ? msg.distance_m.toFixed(1)     : '--';
  document.getElementById('nav-bearing').textContent     = (msg.target_bearing != null) ? msg.target_bearing.toFixed(0) + '°' : '--';
  document.getElementById('nav-mode-disp').textContent   = (msg.nav_mode    || '--').toUpperCase();
  document.getElementById('nav-filter-disp').textContent = (msg.filter_mode || '--').toUpperCase();
  document.getElementById('nav-tol').textContent         = (msg.tolerance_m != null) ? msg.tolerance_m.toFixed(1) : '--';

  // Update heading display
  if (msg.target_bearing != null) {
    document.getElementById('nav-heading').textContent  = msg.target_bearing.toFixed(1) + '°';
    document.getElementById('nav-cardinal').textContent = bearingToCardinal(msg.target_bearing);
  }
}

function handleNavComplete(msg) {
  navActive = false;
  document.getElementById('auto-overlay').classList.remove('visible');
  document.getElementById('nav-progress').textContent = '✓ DONE (' + (msg.total_wp || 0) + ')';
  document.getElementById('nav-status-panel').classList.add('visible');
}

function handleNavWarning(msg) {
  const warn = document.getElementById('warn-banner');
  warn.textContent = '⚠ ' + (msg.msg || 'Navigation warning').toUpperCase();
  warn.classList.add('visible');
  setTimeout(() => {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    warn.classList.remove('visible');
    warn.textContent = '⚠ CONNECTION LOST — ROBOT STOPPED';
  }, 5000);
}

// ── Bearing helpers ───────────────────────────────────────────
function bearingToCardinal(deg) {
  const dirs = ['N','NE','E','SE','S','SW','W','NW'];
  return dirs[Math.round(((deg % 360) + 360) % 360 / 45) % 8];
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

  // Speed slider
  const speedSlider = document.getElementById('speed-slider');
  speedSlider.addEventListener('input', (e) => {
    speedRatio = parseInt(e.target.value) / 100;
    document.getElementById('speed-value').textContent = e.target.value + '%';
  });

  connect();
});
