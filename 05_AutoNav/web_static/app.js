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

// ── Boot ─────────────────────────────────────────────────────
connectAutoNav();
