// PathFollower Control — Application Logic
// Connects to WebSocket server, handles nipplejs joystick, IMU/RTK HUD, heading control.

// ── Config ─────────────────────────────────────────
const WS_URL = `ws://${window.location.hostname}:8891/`;
const HEARTBEAT_INTERVAL_MS = 500;
const JOYSTICK_SEND_INTERVAL_MS = 100;
const DEADZONE = 0.15;

// ── State ───────────────────────────────────────────
let ws = null;
let joystickActive = false;
let currentLinear  = 0.0;
let currentAngular = 0.0;
let currentForce   = 0.0;
let heartbeatTimer = null;
let joySendTimer   = null;
let yawLocked = false;

// ── WebSocket ───────────────────────────────────────
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
      console.debug('WS message:', msg);
      const msgTypeEl = document.getElementById('last-msg-type');
      if (msgTypeEl) msgTypeEl.textContent = msg.type || 'unknown';
      const handlers = {
        imu:    handleIMU,
        status: handleStatus,
        rtk:    handleRTK,
      };
      const handler = handlers[msg.type];
      if (handler) handler(msg);
    } catch (e) {
      console.warn('WS message error:', e, evt.data);
    }
  };
}

function sendMsg(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// ── Timers ──────────────────────────────────────────
function scheduleHeartbeat() {
  heartbeatTimer = setInterval(() => { sendMsg({ type: "heartbeat" }); }, HEARTBEAT_INTERVAL_MS);
}

function scheduleJoySend() {
  joySendTimer = setInterval(() => {
    if (joystickActive) {
      sendMsg({ type: "joystick", linear: currentLinear, angular: currentAngular });
    }
  }, JOYSTICK_SEND_INTERVAL_MS);
}

function clearTimers() {
  if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
  if (joySendTimer)   { clearInterval(joySendTimer);   joySendTimer   = null; }
}

// ── Status UI ───────────────────────────────────────
function setStatus(online) {
  const dot  = document.getElementById('status-dot');
  const text = document.getElementById('status-text');
  const warn = document.getElementById('warn-banner');
  if (online) {
    dot.className = 'online';
    text.textContent = 'ONLINE';
    warn.classList.remove('visible');
  } else {
    dot.className = 'offline';
    text.textContent = 'OFFLINE';
    warn.classList.add('visible');
  }
}

// ── IMU HUD update ──────────────────────────────────
function fmt2(v) { return (v >= 0 ? '+' : '') + v.toFixed(2); }

function safeFixed(v, d) {
  return (typeof v === 'number' && !Number.isNaN(v)) ? v.toFixed(d) : '--';
}

function handleIMU(msg) {
  if (typeof msg.heading === 'number' && !Number.isNaN(msg.heading)) {
    const bearing = msg.heading;
    document.getElementById('compass-needle').setAttribute('transform', `rotate(${bearing.toFixed(1)},50,50)`);
    document.getElementById('compass-bearing').textContent  = bearing.toFixed(1) + '°';

    let cardinal = '--';
    if (bearing <= 22.5 || bearing > 337.5) cardinal = 'N';
    else if (bearing <= 67.5) cardinal = 'NE';
    else if (bearing <= 112.5) cardinal = 'E';
    else if (bearing <= 157.5) cardinal = 'SE';
    else if (bearing <= 202.5) cardinal = 'S';
    else if (bearing <= 247.5) cardinal = 'SW';
    else if (bearing <= 292.5) cardinal = 'W';
    else cardinal = 'NW';
    document.getElementById('compass-cardinal').textContent = cardinal;
  }

  document.getElementById('roll-val').textContent  = (typeof msg.roll === 'number' && !Number.isNaN(msg.roll)) ? fmt2(msg.roll) : '--';
  document.getElementById('pitch-val').textContent = (typeof msg.pitch === 'number' && !Number.isNaN(msg.pitch)) ? fmt2(msg.pitch) : '--';
  document.getElementById('yaw-val').textContent   = (typeof msg.yaw === 'number' && !Number.isNaN(msg.yaw)) ? fmt2(msg.yaw) : '--';
}

function handleStatus(msg) {
  const mode = (msg.mode === 'joystick') ? 'JOYSTICK' : (msg.mode === 'heading_follow') ? 'HEADING' : 'P2P';
  document.getElementById('mode-val').textContent = mode;
  document.getElementById('linear-val').textContent = safeFixed(msg.linear_vel, 2);
  document.getElementById('angular-val').textContent = safeFixed(msg.angular_vel, 2);

  const targetHeading = (typeof msg.target_heading === 'number' && !Number.isNaN(msg.target_heading)) ? msg.target_heading : null;
  document.getElementById('heading-target-val').textContent = (targetHeading !== null) ? targetHeading.toFixed(1) + '°' : '--';

  const headingError = (typeof msg.heading_error === 'number' && !Number.isNaN(msg.heading_error)) ? msg.heading_error : null;
  document.getElementById('heading-error-val').textContent = (headingError !== null) ? fmt2(headingError) + '°' : '--';

  const warn = document.getElementById('warn-banner');
  if (msg.watchdog) {
    warn.classList.add('visible');
  } else {
    warn.classList.remove('visible');
  }
}

// ── RTK HUD update ──────────────────────────────────
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

  dot.className = 'live'; dot.title = 'RTK LIVE';
  const fq = msg.fix_quality || 0;
  const fi = FIX_LABELS[fq] || { text: `FIX(${fq})`, cls: "fix-gps" };
  badge.textContent = fi.text; badge.className = fi.cls;

  const fmtCoord = (v, d) => (v !== null && v !== undefined) ? v.toFixed(d) : '--';
  document.getElementById('rtk-lat').textContent  = fmtCoord(msg.lat, 7);
  document.getElementById('rtk-lon').textContent  = fmtCoord(msg.lon, 7);
  document.getElementById('rtk-alt').textContent  = fmtCoord(msg.alt, 2);
  document.getElementById('rtk-sats').textContent = (msg.num_sats !== null && msg.num_sats !== undefined) ? msg.num_sats : '--';
}

// ── Direction label ──────────────────────────────────
function dirLabel(linear, angular) {
  const fwd  = linear  >  0.05;
  const back = linear  < -0.05;
  const left = angular >  0.05;
  const right= angular < -0.05;
  if (!fwd && !back && !left && !right) return '■ STOP';
  let s = '';
  if (fwd)   s += '↑';
  if (back)  s += '↓';
  if (left)  s += '←';
  if (right) s += '→';
  return s;
}

// ── Joystick update ──────────────────────────────────
function updateJoyUI() {
  document.getElementById('joy-force').textContent   = currentForce.toFixed(2);
  document.getElementById('joy-linear').textContent  = fmt2(currentLinear)  + ' m/s';
  document.getElementById('joy-angular').textContent = fmt2(currentAngular) + ' r/s';
  document.getElementById('joy-dir').textContent     = dirLabel(currentLinear, currentAngular);
}

// ── nipplejs setup ───────────────────────────────────
window.addEventListener('load', () => {
  const zone = document.getElementById('joystick-zone');
  const hint = document.getElementById('joystick-hint');

  const manager = nipplejs.create({
    zone: zone,
    mode: 'static',
    position: { left: '50%', top: '45%' },
    size: 160,
    color: '#00d4ff',
    restOpacity: 0.6,
  });

  manager.on('start', () => {
    joystickActive = true;
    hint.style.display = 'none';
  });

  manager.on('move', (evt, data) => {
    const force = Math.min(data.force, 1.0);
    const angleRad = data.angle.radian;
    let rawX = Math.cos(angleRad) * force;
    let rawY = Math.sin(angleRad) * force;

    if (force < DEADZONE) {
      currentLinear  = 0.0;
      currentAngular = 0.0;
      currentForce   = 0.0;
    } else {
      currentLinear  =  rawY * 1.0;
      currentAngular = -rawX * 1.0;
      currentForce   = force;
    }
    updateJoyUI();
  });

  manager.on('end', () => {
    joystickActive = false;
    currentLinear  = 0.0;
    currentAngular = 0.0;
    currentForce   = 0.0;
    updateJoyUI();
    sendMsg({ type: "joystick", linear: 0.0, angular: 0.0 });
    hint.style.display = 'block';
  });

  // Heading input
  const headingBtn = document.getElementById('btn-set-heading');
  const headingInput = document.getElementById('heading-input');
  headingBtn.addEventListener('click', () => {
    const h = parseInt(headingInput.value) || 0;
    sendMsg({ type: 'set_heading', heading_deg: h });
  });
  headingBtn.addEventListener('touchend', (e) => { e.preventDefault(); headingBtn.click(); });

  // Lock yaw button
  const lockYawBtn = document.getElementById('btn-lock-yaw');
  lockYawBtn.addEventListener('click', () => {
    yawLocked = !yawLocked;
    lockYawBtn.classList.toggle('active', yawLocked);
  });
  lockYawBtn.addEventListener('touchend', (e) => { e.preventDefault(); lockYawBtn.click(); });

  // Start WebSocket
  connect();
});
