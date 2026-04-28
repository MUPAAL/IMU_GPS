// AutoNav Dashboard
// Single WS connection to autonav_bridge :8806
// IMU and RTK raw data are proxied through autonav_status messages

const AUTONAV_PORT = Number(window.location.port || 8805) + 1;  // 8806
const AUTONAV_URL  = `ws://${window.location.hostname}:${AUTONAV_PORT}`;

const JSON_THROTTLE_MS = 300;

// ── DOM refs ─────────────────────────────────────────────────
const statusDot    = document.getElementById('status-dot');
const statusText   = document.getElementById('status-text');
const stateBadge   = document.getElementById('state-badge');
const jsonImu      = document.getElementById('json-imu');
const jsonRtk      = document.getElementById('json-rtk');
const jsonCmd      = document.getElementById('json-cmd');
const jsonStatus   = document.getElementById('json-status');
const imuAge       = document.getElementById('imu-age');
const rtkAge       = document.getElementById('rtk-age');
const needleHdg    = document.getElementById('needle-heading');
const needleTgt    = document.getElementById('needle-target');

const mHeading  = document.getElementById('m-heading');
const mTarget   = document.getElementById('m-target');
const mError    = document.getElementById('m-error');
const mDist     = document.getElementById('m-dist');
const mProgress = document.getElementById('m-progress');
const mLinear   = document.getElementById('m-linear');
const mAngular  = document.getElementById('m-angular');
const mGpsAge   = document.getElementById('m-gps-age');
const mImuAge   = document.getElementById('m-imu-age');

// ── State ────────────────────────────────────────────────────
let navWs = null;
const lastUpdate = { imu: 0, rtk: 0, status: 0 };

// ── Helpers ──────────────────────────────────────────────────
function fmt(v, d = 1, unit = '') {
  return v != null ? v.toFixed(d) + unit : '—';
}

function setNeedle(el, angleDeg) {
  if (angleDeg == null) return;
  const rad = (angleDeg - 90) * Math.PI / 180;
  el.setAttribute('x2', (100 + 75 * Math.cos(rad)).toFixed(1));
  el.setAttribute('y2', (100 + 75 * Math.sin(rad)).toFixed(1));
}

function setAge(el, ageS, threshold) {
  if (ageS == null || ageS > 999) {
    el.textContent = 'NO DATA';
    el.style.color = 'var(--red)';
  } else {
    el.textContent = ageS.toFixed(2) + 's';
    el.style.color = ageS > threshold ? 'var(--red)' : '';
  }
}

// ── AutoNav WebSocket ─────────────────────────────────────────
function connectAutoNav() {
  navWs = new WebSocket(AUTONAV_URL);

  navWs.onopen = () => {
    statusDot.className = 'connected';
    statusText.textContent = 'CONNECTED';
  };

  navWs.onclose = () => {
    statusDot.className = '';
    statusText.textContent = 'DISCONNECTED';
    navWs = null;
    setTimeout(connectAutoNav, 3000);
  };

  navWs.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data);
      if (msg.type !== 'autonav_status') return;
      handleNavStatus(msg);
    } catch (_) {}
  };
}

function handleNavStatus(msg) {
  const now = Date.now();

  // State badge
  const state = msg.state || 'idle';
  stateBadge.textContent = state.toUpperCase();
  stateBadge.className = `state-${state}`;

  // Compass needles
  setNeedle(needleHdg, msg.heading_deg);
  setNeedle(needleTgt, msg.target_bearing_deg);

  // Metrics
  mHeading.textContent  = fmt(msg.heading_deg, 1, '°');
  mTarget.textContent   = fmt(msg.target_bearing_deg, 1, '°');
  const err = msg.bearing_error_deg;
  mError.textContent    = err != null ? (err >= 0 ? '+' : '') + err.toFixed(1) + '°' : '—';
  mDist.textContent     = fmt(msg.dist_to_wp_m, 1, ' m');
  mProgress.textContent = (msg.current_wp_idx != null && msg.total_wp != null)
    ? `${msg.current_wp_idx} / ${msg.total_wp}` : '—';
  mLinear.textContent   = fmt(msg.linear, 2, ' m/s');
  mAngular.textContent  = fmt(msg.angular, 2, ' r/s');
  setAge(mGpsAge, msg.gps_age_s, 2.0);
  setAge(mImuAge, msg.imu_age_s, 2.0);

  // Waypoint window
  updateWpTable(msg.waypoints_window);

  // Heading calibration panel
  updateCalibPanel(msg);

  // Input JSON panels (throttled) — data proxied from 01/02 via bridge
  if (now - lastUpdate.imu >= JSON_THROTTLE_MS && msg.imu_raw) {
    lastUpdate.imu = now;
    jsonImu.textContent = JSON.stringify(msg.imu_raw, null, 2);
  }
  if (now - lastUpdate.rtk >= JSON_THROTTLE_MS && msg.rtk_raw) {
    lastUpdate.rtk = now;
    jsonRtk.textContent = JSON.stringify(msg.rtk_raw, null, 2);
  }

  // Output JSON panels (throttled)
  if (now - lastUpdate.status >= JSON_THROTTLE_MS) {
    lastUpdate.status = now;
    const statusDisplay = Object.fromEntries(
      Object.entries(msg).filter(([k]) => k !== 'imu_raw' && k !== 'rtk_raw')
    );
    jsonStatus.textContent = JSON.stringify(statusDisplay, null, 2);
    jsonCmd.textContent = JSON.stringify({
      type: 'joystick',
      linear: msg.linear,
      angular: msg.angular,
    }, null, 2);
  }
}

// ── Manual drive ─────────────────────────────────────────
const MANUAL_SPEED = 0.4;  // m/s straight drive speed

function startManual(linear) { sendCmd('manual_drive', { linear }); }
function stopManual()         { sendCmd('manual_drive', { linear: 0.0 }); }

const btnFwd = document.getElementById('btn-fwd');
const btnBwd = document.getElementById('btn-bwd');

btnFwd.addEventListener('pointerdown',  () => startManual( MANUAL_SPEED));
btnBwd.addEventListener('pointerdown',  () => startManual(-MANUAL_SPEED));
btnFwd.addEventListener('pointerup',    stopManual);
btnBwd.addEventListener('pointerup',    stopManual);
btnFwd.addEventListener('pointerleave', stopManual);
btnBwd.addEventListener('pointerleave', stopManual);

document.addEventListener('keydown', (e) => {
  if (e.repeat) return;
  if (e.key === 'w' || e.key === 'W') startManual( MANUAL_SPEED);
  if (e.key === 's' || e.key === 'S') startManual(-MANUAL_SPEED);
});
document.addEventListener('keyup', (e) => {
  if (e.key === 'w' || e.key === 'W' || e.key === 's' || e.key === 'S') stopManual();
});

// ── Heading calibration ───────────────────────────────────
const calibMarked  = document.getElementById('calib-marked');
const calibCurrent = document.getElementById('calib-current');
const calibBearing = document.getElementById('calib-bearing');
const calibOffset  = document.getElementById('calib-offset');

function fmtLatLon(lat, lon) {
  if (lat == null || lon == null) return '—';
  return `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
}

function bearing(lat1, lon1, lat2, lon2) {
  const toRad = d => d * Math.PI / 180;
  const dLon = toRad(lon2 - lon1);
  const x = Math.sin(dLon) * Math.cos(toRad(lat2));
  const y = Math.cos(toRad(lat1)) * Math.sin(toRad(lat2))
          - Math.sin(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.cos(dLon);
  return ((Math.atan2(x, y) * 180 / Math.PI) + 360) % 360;
}

let _calibMark = null;
let _curLat = null, _curLon = null;

function updateCalibPanel(msg) {
  const calib = msg.calib || {};
  const rtk   = msg.rtk_raw || {};

  _curLat = rtk.lat ?? null;
  _curLon = rtk.lon ?? null;
  _calibMark = calib.mark || null;

  calibCurrent.textContent = fmtLatLon(_curLat, _curLon);

  if (_calibMark) {
    calibMarked.textContent = fmtLatLon(_calibMark.lat, _calibMark.lon);
    if (_curLat != null) {
      const b = bearing(_curLat, _curLon, _calibMark.lat, _calibMark.lon);
      calibBearing.textContent = b.toFixed(1) + '°';
    }
  } else {
    calibMarked.textContent = '—';
    calibBearing.textContent = '—';
  }

  if (calib.offset_applied != null) {
    calibOffset.textContent = calib.offset_applied.toFixed(2) + '°';
    calibOffset.className = 'applied';
  } else {
    calibOffset.textContent = '—';
    calibOffset.className = '';
  }
}

document.getElementById('btn-mark').addEventListener('click', () => sendCmd('calib_mark'));
document.getElementById('btn-calibrate').addEventListener('click', () => {
  if (!_calibMark) { alert('先按 MARK POS 记录前方位置'); return; }
  sendCmd('calib_apply');
});

// ── Waypoint table ────────────────────────────────────────
const wpTbody = document.getElementById('wp-tbody');

function updateWpTable(wps) {
  if (!wps || !wps.length) {
    wpTbody.innerHTML = '<tr><td colspan="3" style="color:var(--dim);text-align:center">no waypoints</td></tr>';
    return;
  }
  wpTbody.innerHTML = wps.map(wp => {
    const cls = wp.current ? ' class="wp-current"' : '';
    return `<tr${cls}><td>${wp.idx}</td><td>${wp.lat.toFixed(7)}</td><td>${wp.lon.toFixed(7)}</td></tr>`;
  }).join('');
}

// ── Control buttons ───────────────────────────────────────────
function sendCmd(type, extra = {}) {
  if (navWs && navWs.readyState === WebSocket.OPEN) {
    navWs.send(JSON.stringify({ type, ...extra }));
  }
}

document.getElementById('btn-start').addEventListener('click',  () => sendCmd('start'));
document.getElementById('btn-stop').addEventListener('click',   () => sendCmd('stop'));
document.getElementById('btn-pause').addEventListener('click',  () => sendCmd('pause'));
document.getElementById('btn-resume').addEventListener('click', () => sendCmd('resume'));

// ── Speed slider ──────────────────────────────────────────────
const speedSlider = document.getElementById('speed-slider');
speedSlider.addEventListener('input', (e) => {
  const pct = parseInt(e.target.value);
  document.getElementById('speed-value').textContent = pct + '%';
  sendCmd('set_speed', { ratio: pct / 100 });
});

// ── CSV import ───────────────────────────────────────────────
const csvInput    = document.getElementById('csv-input');
const csvFilename = document.getElementById('csv-filename');

csvInput.addEventListener('change', (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (ev) => {
    sendCmd('load_csv', { content: ev.target.result });
    csvFilename.textContent = file.name;
  };
  reader.readAsText(file);
  csvInput.value = '';  // allow re-selecting the same file
});

// ── Boot ─────────────────────────────────────────────────────
connectAutoNav();
