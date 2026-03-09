// Farm Robot Control — Application Logic
// Connects to WebSocket server, handles joystick, IMU/RTK HUD, navigation controls.

// ── Config ─────────────────────────────────────────────────
const WS_URL = `ws://${window.location.hostname}:8889/`;
const HEARTBEAT_INTERVAL_MS = 500;
const JOYSTICK_SEND_INTERVAL_MS = 100;  // 10Hz throttle
const DEADZONE = 0.15;

// MAX_LINEAR_VEL / MAX_ANGULAR_VEL injected by server as data attributes or fallback 1.0
const MAX_LINEAR  = parseFloat(document.documentElement.dataset.maxLinear  || "1.0");
const MAX_ANGULAR = parseFloat(document.documentElement.dataset.maxAngular || "1.0");

// ── Speed ratio (Frontend Scaling Default 50) ─────────────────────────
let speedRatio = 0.5;

// ── State ───────────────────────────────────────────────────
let ws = null;
let joystickActive = false;
let currentLinear  = 0.0;
let currentAngular = 0.0;
let currentForce   = 0.0;
let heartbeatTimer = null;
let joySendTimer   = null;

// ── State machine (client-side tracking) ────────────────────
let controlStateActive = false;  // false = AUTO_READY, true = AUTO_ACTIVE

// ── Navigation state ────────────────────────────────────────
let navActive  = false;
let navWpCount = 0;
let navMode    = "p2p";
let filterMode = "moving_avg";

function toggleControlState() {
  sendMsg({ type: "toggle_state" });
  // Disable button until server confirms new state
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
        imu:              handleIMU,
        status:           handleStatus,
        rtk:              handleRTK,
        odom:             handleOdom,
        record_status:    handleRecordStatus,
        state_status:     handleStateStatus,
        waypoints_loaded: handleWaypointsLoaded,
        nav_status:       handleNavStatus,
        nav_complete:     handleNavComplete,
        nav_warning:      handleNavWarning,
        coverage_ready:   handleCoverageReady,
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
  heartbeatTimer = setInterval(() => { sendMsg({ type: "heartbeat" }); }, HEARTBEAT_INTERVAL_MS);
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

// ── Odometry HUD update ──────────────────────────────────────
function handleOdom(msg) {
  document.getElementById('odom-v').textContent = (msg.v !== null && msg.v !== undefined) ? msg.v.toFixed(3) : '--';
  document.getElementById('odom-w').textContent = (msg.w !== null && msg.w !== undefined) ? msg.w.toFixed(3) : '--';
  const socEl = document.getElementById('amiga-soc');
  socEl.textContent = (msg.soc !== null && msg.soc !== undefined) ? msg.soc + '%' : '--%';
}

// ── IMU HUD update ──────────────────────────────────────────
function fmt3(v) { return (v >= 0 ? '+' : '') + v.toFixed(3); }
function fmt2(v) { return (v >= 0 ? '+' : '') + v.toFixed(2); }

function handleIMU(msg) {
  if (msg.accel) {
    document.getElementById('acc-x').textContent = fmt3(msg.accel.x);
    document.getElementById('acc-y').textContent = fmt3(msg.accel.y);
    document.getElementById('acc-z').textContent = fmt3(msg.accel.z);
  }
  if (msg.gyro) {
    document.getElementById('gyro-x').textContent = fmt3(msg.gyro.x);
    document.getElementById('gyro-y').textContent = fmt3(msg.gyro.y);
    document.getElementById('gyro-z').textContent = fmt3(msg.gyro.z);
  }
  if (msg.compass) {
    const c = msg.compass;
    document.getElementById('compass-needle').setAttribute('transform', `rotate(${c.bearing.toFixed(1)},50,50)`);
    document.getElementById('compass-bearing').textContent  = c.bearing.toFixed(1) + '°';
    document.getElementById('compass-cardinal').textContent = c.cardinal;
    const acc = (c.accuracy !== undefined) ? Math.min(Math.max(c.accuracy, 0), 3) : (c.calibrated ? 3 : 0);
    const accLabels = ['● UNCAL', '● LOW', '● GOOD', '● PRECISE'];
    const accEl = document.getElementById('compass-accuracy');
    accEl.textContent = accLabels[acc];
    accEl.className = `acc-${acc}`;
  }
}

function handleStatus(msg) {
  // Reserved for serial/imu status if needed
}

// ── RTK HUD update ──────────────────────────────────────────
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
  const fq = msg.fix_quality || 0;
  const fi = FIX_LABELS[fq] || { text: `FIX(${fq})`, cls: "fix-gps" };
  badge.textContent = fi.text; badge.className = fi.cls;

  const fmtCoord = (v, d) => (v !== null && v !== undefined) ? v.toFixed(d) : '--';
  document.getElementById('rtk-lat').textContent  = fmtCoord(msg.lat, 7);
  document.getElementById('rtk-lon').textContent  = fmtCoord(msg.lon, 7);
  document.getElementById('rtk-alt').textContent  = fmtCoord(msg.alt, 2);
  document.getElementById('rtk-sats').textContent = (msg.num_sats !== null && msg.num_sats !== undefined) ? msg.num_sats : '--';
  document.getElementById('rtk-hdop').textContent = fmtCoord(msg.hdop, 2);
  document.getElementById('rtk-spd').textContent  = fmtCoord(msg.speed_knots, 2);
}

// ── State status update ──────────────────────────────────────
function handleStateStatus(msg) {
  controlStateActive = msg.active;
  updateStateBtn();
  document.getElementById('state-btn').disabled = false;
}

// ── Record status update ─────────────────────────────────────
function handleRecordStatus(msg) {
  const btn    = document.getElementById('rec-btn');
  const fnSpan = document.getElementById('rec-filename');
  if (msg.recording) {
    btn.className = 'rec-active';
    btn.textContent = '■ STOP';
    fnSpan.textContent = msg.filename || '—';
  } else {
    btn.className = 'rec-idle';
    btn.textContent = '● REC';
    fnSpan.textContent = '—';
  }
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

  const statusPanel = document.getElementById('nav-status-panel');
  if (navActive || state === 'finished') {
    statusPanel.classList.add('visible');
  } else {
    statusPanel.classList.remove('visible');
  }

  const autoBtn = document.getElementById('nav-auto-btn');
  if (navActive) {
    autoBtn.className = 'nav-active'; autoBtn.textContent = '■ STOP';
  } else {
    autoBtn.className = 'nav-idle';   autoBtn.textContent = '▶ AUTO';
  }

  const overlay = document.getElementById('auto-overlay');
  if (navActive) { overlay.classList.add('visible'); }
  else           { overlay.classList.remove('visible'); }

  const prog = msg.progress || [0, 0];
  document.getElementById('nav-progress').textContent  = prog[0] + ' / ' + prog[1];
  document.getElementById('nav-dist').textContent      = msg.distance_m   !== null && msg.distance_m   !== undefined ? msg.distance_m.toFixed(1)   : '--';
  document.getElementById('nav-bearing').textContent   = msg.target_bearing !== null && msg.target_bearing !== undefined ? msg.target_bearing.toFixed(0) + '°' : '--';
  document.getElementById('nav-mode-disp').textContent   = (msg.nav_mode    || '--').toUpperCase();
  document.getElementById('nav-filter-disp').textContent = (msg.filter_mode || '--').toUpperCase();
  document.getElementById('nav-tol').textContent       = msg.tolerance_m  !== null && msg.tolerance_m  !== undefined ? msg.tolerance_m.toFixed(1)  : '--';
}

function handleNavComplete(msg) {
  navActive = false;
  document.getElementById('auto-overlay').classList.remove('visible');
  document.getElementById('nav-auto-btn').className   = 'nav-idle';
  document.getElementById('nav-auto-btn').textContent = '▶ AUTO';
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

// ── Coverage path planner ─────────────────────────────────────
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
    // Update waypoint count in bottom bar
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

// ── Nav mode & filter toggle ─────────────────────────────────
function setNavMode(mode) {
  navMode = mode;
  document.getElementById('btn-p2p').classList.toggle('active',     mode === 'p2p');
  document.getElementById('btn-pursuit').classList.toggle('active', mode === 'pure_pursuit');
  sendMsg({ type: 'nav_mode', mode: mode });
}

function setFilterMode(mode) {
  filterMode = mode;
  document.getElementById('btn-mavg').classList.toggle('active',   mode === 'moving_avg');
  document.getElementById('btn-kalman').classList.toggle('active', mode === 'kalman');
  sendMsg({ type: 'filter_mode', mode: mode });
}

// ── Direction label ──────────────────────────────────────────
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

// ── Joystick update (elements hidden, values still sent) ─────
function updateJoyUI() {
  document.getElementById('joy-force').textContent   = currentForce.toFixed(2);
  document.getElementById('joy-linear').textContent  = fmt2(currentLinear)  + ' m/s';
  document.getElementById('joy-angular').textContent = fmt2(currentAngular) + ' r/s';
  document.getElementById('joy-dir').textContent     = dirLabel(currentLinear, currentAngular);
}

// ── nipplejs setup ───────────────────────────────────────────
window.addEventListener('load', () => {
  const zone = document.getElementById('joystick-zone');
  const hint = document.getElementById('joystick-hint');

  const manager = nipplejs.create({
    zone: zone,
    mode: 'static',
    position: { left: '50%', top: '45%' },  // slightly above center to account for bottom bar
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
    let rawX = Math.cos(angleRad) * force;  // right positive
    let rawY = Math.sin(angleRad) * force;  // up positive

    if (force < DEADZONE) {
      currentLinear  = 0.0;
      currentAngular = 0.0;
      currentForce   = 0.0;
    } else {
      currentLinear  =  rawY * MAX_LINEAR  * speedRatio;   // up → forward
      currentAngular = -rawX * MAX_ANGULAR * speedRatio;   // right → negative angular
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
    sendMsg({ type: "joystick", linear: 0.0, angular: 0.0, force: 0.0 });
    hint.style.display = 'block';
  });

  // State toggle button
  const stateBtn = document.getElementById('state-btn');
  stateBtn.addEventListener('click', toggleControlState);
  stateBtn.addEventListener('touchend', (e) => { e.preventDefault(); toggleControlState(); });

  // Record button
  const recBtn = document.getElementById('rec-btn');
  recBtn.addEventListener('click', () => sendMsg({ type: "toggle_record" }));
  recBtn.addEventListener('touchend', (e) => { e.preventDefault(); sendMsg({ type: "toggle_record" }); });

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
    csvInput.value = '';  // allow re-selecting same file
  });

  // Nav mode buttons
  document.getElementById('btn-p2p').addEventListener('click',     () => setNavMode('p2p'));
  document.getElementById('btn-pursuit').addEventListener('click', () => setNavMode('pure_pursuit'));
  document.getElementById('btn-mavg').addEventListener('click',   () => setFilterMode('moving_avg'));
  document.getElementById('btn-kalman').addEventListener('click',  () => setFilterMode('kalman'));

  // AUTO start/stop button
  const autoBtn = document.getElementById('nav-auto-btn');
  function toggleNav() {
    if (navActive) sendMsg({ type: 'nav_stop' });
    else           sendMsg({ type: 'nav_start' });
  }
  autoBtn.addEventListener('click', toggleNav);
  autoBtn.addEventListener('touchend', (e) => { e.preventDefault(); toggleNav(); });

  // FORCE button — start navigation ignoring GPS fix check
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
