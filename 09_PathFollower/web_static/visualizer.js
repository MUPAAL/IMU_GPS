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

// ── Navigation state ────────────────────────────────────────
let speedRatio = 0.5;
let navWpCount = 0;
let navMode    = "p2p";

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
        imu:              handleIMU,
        status:           handleStatus,
        rtk:              handleRTK,
        waypoints_loaded: handleWaypointsLoaded,
        state_status:     handleStateStatus,
        nav_status:       handleNavStatus,
        nav_complete:     handleNavComplete,
        coverage_ready:   handleCoverageReady,
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
  const btn  = document.getElementById('state-btn');

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

// ── State status ─────────────────────────────────────────────
function handleStateStatus(msg) {
  controlStateActive = msg.active;
  updateStateBtn();
  document.getElementById('state-btn').disabled = false;
}

// ── Navigation handlers ──────────────────────────────────────
function handleWaypointsLoaded(msg) {
  navWpCount = msg.count || 0;
  const el = document.getElementById('nav-wp-count');
  el.textContent = navWpCount + ' WP';
  if (msg.error) {
    el.style.color = 'var(--red)';
  } else {
    el.style.color = navWpCount > 0 ? 'var(--green)' : 'var(--dim)';
  }
}

function handleNavStatus(msg) {
  const state = msg.state || 'idle';
  navActive = (state === 'navigating');

  const autoBtn = document.getElementById('nav-auto-btn');
  if (navActive) {
    autoBtn.className = 'nav-active'; autoBtn.textContent = '■ STOP';
  } else {
    autoBtn.className = 'nav-idle';   autoBtn.textContent = '▶ AUTO';
  }

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
  const headingDeg = (msg.heading_deg != null) ? msg.heading_deg : msg.target_bearing;
  const headingDir = (msg.heading_dir != null) ? msg.heading_dir : null;
  document.getElementById('nav-progress').textContent    = prog[0] + ' / ' + prog[1];
  document.getElementById('nav-dist').textContent        = (msg.distance_m     != null) ? msg.distance_m.toFixed(1)     : '--';
  document.getElementById('nav-bearing').textContent     = (headingDeg          != null) ? headingDeg.toFixed(0) + '°'          : '--';
  document.getElementById('nav-mode-disp').textContent   = (msg.nav_mode    || '--').toUpperCase();
  document.getElementById('nav-filter-disp').textContent = (msg.filter_mode || '--').toUpperCase();
  document.getElementById('nav-tol').textContent         = (msg.tolerance_m != null) ? msg.tolerance_m.toFixed(1) : '--';

  // Update heading display
  if (headingDeg != null) {
    document.getElementById('nav-heading').textContent  = headingDeg.toFixed(1) + '°';
    document.getElementById('nav-cardinal').textContent = headingDir || bearingToCardinal(headingDeg);
  }
}

function handleNavComplete(msg) {
  navActive = false;
  document.getElementById('auto-overlay').classList.remove('visible');
  document.getElementById('nav-auto-btn').className   = 'nav-idle';
  document.getElementById('nav-auto-btn').textContent = '▶ AUTO';
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

function handleCoverageReady(msg) {
  const statusEl = document.getElementById('cov-status');
  const genBtn   = document.getElementById('cov-gen-btn');
  genBtn.disabled = false;
  if (msg.error) {
    statusEl.style.color = 'var(--red)';
    statusEl.textContent = '✗ ' + msg.error;
  } else {
    const count = msg.count || 0;
    statusEl.style.color = count > 0 ? 'var(--green)' : 'var(--warn)';
    statusEl.textContent = count > 0
      ? `✓ ${count} waypoints loaded`
      : '⚠ 0 waypoints generated — check boundary/spacing';
    const wpEl = document.getElementById('nav-wp-count');
    wpEl.textContent = count + ' WP';
    wpEl.style.color = count > 0 ? 'var(--green)' : 'var(--dim)';
  }
}

function parseBoundaryText(text) {
  const lines = text.trim().split('\n');
  const pts = [];
  for (const line of lines) {
    const parts = line.trim().split(/[\s,]+/);
    if (parts.length >= 2) {
      const lat = parseFloat(parts[0]);
      const lon = parseFloat(parts[1]);
      if (!isNaN(lat) && !isNaN(lon)) pts.push([lat, lon]);
    }
  }
  return pts;
}

function sendGenerateCoverage() {
  const boundaryText = document.getElementById('cov-boundary').value;
  const boundary     = parseBoundaryText(boundaryText);
  const statusEl     = document.getElementById('cov-status');
  const genBtn       = document.getElementById('cov-gen-btn');

  if (boundary.length < 3) {
    statusEl.style.color = 'var(--red)';
    statusEl.textContent = '✗ Need at least 3 boundary points';
    return;
  }

  const rowSpacing   = parseFloat(document.getElementById('cov-spacing').value)   || 1.0;
  const directionDeg = parseFloat(document.getElementById('cov-direction').value)  || 0.0;
  const overlapPct   = parseFloat(document.getElementById('cov-overlap').value)    || 0.0;
  const toleranceM   = parseFloat(document.getElementById('cov-tolerance').value)  || 1.0;

  statusEl.style.color = 'var(--dim)';
  statusEl.textContent = 'Generating...';
  genBtn.disabled = true;

  sendMsg({
    type:          'generate_coverage',
    boundary:      boundary,
    row_spacing:   rowSpacing,
    direction_deg: directionDeg,
    overlap:       overlapPct / 100.0,
    tolerance_m:   toleranceM,
    max_speed:     0.5,
  });
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

function updateJoyUI() {
  document.getElementById('joy-force').textContent   = currentForce.toFixed(2);
  document.getElementById('joy-linear').textContent  = fmt2(currentLinear)  + ' m/s';
  document.getElementById('joy-angular').textContent = fmt2(currentAngular) + ' r/s';
  document.getElementById('joy-dir').textContent     = dirLabel(currentLinear, currentAngular);
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
      currentLinear  =  rawY * 1.0 * speedRatio;
      currentAngular = -rawX * 1.0 * speedRatio;
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

  // CSV upload
  const csvInput  = document.getElementById('csv-file-input');
  const uploadBtn = document.getElementById('nav-upload-btn');
  uploadBtn.addEventListener('click',    () => csvInput.click());
  uploadBtn.addEventListener('touchend', (e) => { e.preventDefault(); csvInput.click(); });
  csvInput.addEventListener('change', (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => sendMsg({ type: 'upload_waypoints', csv: ev.target.result });
    reader.readAsText(file);
    csvInput.value = '';
  });

  // Nav mode buttons
  document.getElementById('btn-p2p').addEventListener('click', () => {
    navMode = 'p2p';
    document.getElementById('btn-p2p').classList.add('active');
    document.getElementById('btn-pursuit').classList.remove('active');
    sendMsg({ type: 'nav_mode', mode: 'p2p' });
  });
  document.getElementById('btn-pursuit').addEventListener('click', () => {
    navMode = 'pure_pursuit';
    document.getElementById('btn-p2p').classList.remove('active');
    document.getElementById('btn-pursuit').classList.add('active');
    sendMsg({ type: 'nav_mode', mode: 'pure_pursuit' });
  });

  // AUTO start/stop button
  const autoBtn = document.getElementById('nav-auto-btn');
  function toggleNav() {
    if (navActive) sendMsg({ type: 'nav_stop' });
    else           sendMsg({ type: 'nav_start' });
  }
  autoBtn.addEventListener('click', toggleNav);
  autoBtn.addEventListener('touchend', (e) => { e.preventDefault(); toggleNav(); });

  // FORCE button
  const forceBtn = document.getElementById('nav-force-btn');
  forceBtn.addEventListener('click', () => sendMsg({ type: 'nav_start_force' }));
  forceBtn.addEventListener('touchend', (e) => { e.preventDefault(); sendMsg({ type: 'nav_start_force' }); });

  // Coverage planner modal
  const covToggleBtn = document.getElementById('cov-toggle-btn');
  const covModal     = document.getElementById('cov-modal');
  const covGenBtn    = document.getElementById('cov-gen-btn');
  const covCloseBtn  = document.getElementById('cov-close-btn');

  function openCovModal()  { covModal.classList.add('visible'); }
  function closeCovModal() {
    covModal.classList.remove('visible');
    document.getElementById('cov-status').textContent = '';
    document.getElementById('cov-gen-btn').disabled = false;
  }

  covToggleBtn.addEventListener('click',    openCovModal);
  covToggleBtn.addEventListener('touchend', (e) => { e.preventDefault(); openCovModal(); });
  covCloseBtn.addEventListener('click',    closeCovModal);
  covCloseBtn.addEventListener('touchend', (e) => { e.preventDefault(); closeCovModal(); });
  covGenBtn.addEventListener('click',    sendGenerateCoverage);
  covGenBtn.addEventListener('touchend', (e) => { e.preventDefault(); sendGenerateCoverage(); });

  // Start WebSocket
  connect();
});
