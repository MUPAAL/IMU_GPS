/**
 * visualizer.js — PathFollower web UI controller
 *
 * Handles:
 *  - WebSocket connection to web_controller.py
 *  - Virtual joystick rendering and input
 *  - Heading target setting
 *  - Live telemetry display (IMU, RTK, status)
 *  - Heartbeat keep-alive
 */

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

const WS_URL = `ws://${window.location.hostname}:${parseInt(window.location.port) + 1}`;
const HEARTBEAT_INTERVAL_MS = 1000;

// ─────────────────────────────────────────────────────────────────────────────
// Global state
// ─────────────────────────────────────────────────────────────────────────────

let ws = null;
let joystick = {
    canvas: null,
    ctx: null,
    centerX: 0,
    centerY: 0,
    radius: 0,
    innerRadius: 0,
    isPressed: false,
    touchId: null,
};

let controlState = {
    linear: 0.0,
    angular: 0.0,
    targetHeading: 0.0,
    mode: 'joystick',
};

let sensorData = {
    imu: {
        heading: 0,
        roll: 0,
        pitch: 0,
        yaw: 0,
        hz: 0,
    },
    rtk: {
        lat: 0,
        lon: 0,
        alt: 0,
        numSats: 0,
        fixQuality: 0,
    },
    status: {
        watchdog: false,
        mode: 'joystick',
        linearVel: 0.0,
        angularVel: 0.0,
        headingError: 0.0,
    },
};

// ─────────────────────────────────────────────────────────────────────────────
// WebSocket connection
// ─────────────────────────────────────────────────────────────────────────────

function connectWebSocket() {
    console.log(`Connecting to ${WS_URL}...`);
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        console.log('WebSocket connected');
        // Start heartbeat
        setInterval(() => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'heartbeat' }));
            }
        }, HEARTBEAT_INTERVAL_MS);
    };

    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            handleServerMessage(msg);
        } catch (e) {
            console.error('Failed to parse message:', e);
        }
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
    };

    ws.onclose = () => {
        console.log('WebSocket closed. Reconnecting in 3s...');
        setTimeout(connectWebSocket, 3000);
    };
}

function handleServerMessage(msg) {
    const type = msg.type;

    if (type === 'imu') {
        sensorData.imu = {
            heading: msg.heading || 0,
            roll: msg.roll || 0,
            pitch: msg.pitch || 0,
            yaw: msg.yaw || 0,
            hz: msg.hz || 0,
        };
        updateIMUDisplay();
    } else if (type === 'rtk') {
        sensorData.rtk = {
            lat: msg.lat || 0,
            lon: msg.lon || 0,
            alt: msg.alt || 0,
            numSats: msg.num_sats || 0,
            fixQuality: msg.fix_quality || 0,
        };
        updateRTKDisplay();
    } else if (type === 'status') {
        sensorData.status = {
            watchdog: msg.watchdog || false,
            mode: msg.mode || 'joystick',
            linearVel: msg.linear_vel || 0.0,
            angularVel: msg.angular_vel || 0.0,
            headingError: msg.heading_error !== null ? msg.heading_error : 0.0,
        };
        updateStatusDisplay();
    }
}

function sendJoystickCommand(linear, angular) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'joystick',
            linear: linear,
            angular: angular,
        }));
    }
}

function sendHeadingCommand(heading) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'set_heading',
            heading_deg: heading,
        }));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Joystick rendering and input
// ─────────────────────────────────────────────────────────────────────────────

function initJoystick() {
    joystick.canvas = document.getElementById('joystick');
    joystick.ctx = joystick.canvas.getContext('2d');

    const rect = joystick.canvas.getBoundingClientRect();
    joystick.centerX = joystick.canvas.width / 2;
    joystick.centerY = joystick.canvas.height / 2;
    joystick.radius = Math.min(joystick.centerX, joystick.centerY) * 0.9;
    joystick.innerRadius = joystick.radius * 0.5;

    // Draw static background
    drawJoystickBackground();

    // Mouse events
    joystick.canvas.addEventListener('mousedown', onJoystickMouseDown);
    joystick.canvas.addEventListener('mousemove', onJoystickMouseMove);
    joystick.canvas.addEventListener('mouseup', onJoystickMouseUp);
    joystick.canvas.addEventListener('mouseleave', onJoystickMouseUp);

    // Touch events
    joystick.canvas.addEventListener('touchstart', onJoystickTouchStart);
    joystick.canvas.addEventListener('touchmove', onJoystickTouchMove);
    joystick.canvas.addEventListener('touchend', onJoystickTouchEnd);
}

function drawJoystickBackground() {
    const ctx = joystick.ctx;
    ctx.clearRect(0, 0, joystick.canvas.width, joystick.canvas.height);

    // Outer circle
    ctx.strokeStyle = '#58a6ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(joystick.centerX, joystick.centerY, joystick.radius, 0, Math.PI * 2);
    ctx.stroke();

    // Inner circle
    ctx.strokeStyle = '#30363d';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(joystick.centerX, joystick.centerY, joystick.innerRadius, 0, Math.PI * 2);
    ctx.stroke();

    // Crosshair
    ctx.strokeStyle = '#30363d';
    ctx.lineWidth = 1;
    const cross = joystick.radius * 0.3;
    ctx.beginPath();
    ctx.moveTo(joystick.centerX - cross, joystick.centerY);
    ctx.lineTo(joystick.centerX + cross, joystick.centerY);
    ctx.moveTo(joystick.centerX, joystick.centerY - cross);
    ctx.lineTo(joystick.centerX, joystick.centerY + cross);
    ctx.stroke();
}

function drawJoystickHandle(x, y) {
    const ctx = joystick.ctx;
    drawJoystickBackground();

    // Draw handle
    ctx.fillStyle = '#58a6ff';
    ctx.beginPath();
    ctx.arc(x, y, joystick.innerRadius * 0.6, 0, Math.PI * 2);
    ctx.fill();
}

function onJoystickMouseDown(e) {
    joystick.isPressed = true;
    updateJoystickFromEvent(e);
}

function onJoystickMouseMove(e) {
    if (joystick.isPressed) {
        updateJoystickFromEvent(e);
    }
}

function onJoystickMouseUp(e) {
    joystick.isPressed = false;
    controlState.linear = 0.0;
    controlState.angular = 0.0;
    drawJoystickBackground();
    updateJoystickDisplay();
    sendJoystickCommand(0.0, 0.0);
}

function onJoystickTouchStart(e) {
    e.preventDefault();
    joystick.isPressed = true;
    const touch = e.touches[0];
    joystick.touchId = touch.identifier;
    updateJoystickFromEvent(touch);
}

function onJoystickTouchMove(e) {
    e.preventDefault();
    const touch = Array.from(e.touches).find(t => t.identifier === joystick.touchId);
    if (touch) {
        updateJoystickFromEvent(touch);
    }
}

function onJoystickTouchEnd(e) {
    e.preventDefault();
    joystick.isPressed = false;
    joystick.touchId = null;
    controlState.linear = 0.0;
    controlState.angular = 0.0;
    drawJoystickBackground();
    updateJoystickDisplay();
    sendJoystickCommand(0.0, 0.0);
}

function updateJoystickFromEvent(event) {
    const rect = joystick.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    // Compute vector from center
    const dx = x - joystick.centerX;
    const dy = y - joystick.centerY;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const angle = Math.atan2(dy, dx);

    // Clamp to radius
    let clampedDist = Math.min(dist, joystick.radius);
    const handleX = joystick.centerX + Math.cos(angle) * clampedDist;
    const handleY = joystick.centerY + Math.sin(angle) * clampedDist;

    // Normalize to [-1, 1]
    let normalizedDist = clampedDist / joystick.radius;

    // Map to velocities
    // Forward/backward (Y axis): negative dy = forward
    // Left/right (X axis): positive dx = right turn
    controlState.linear = -normalizedDist * Math.cos(angle);
    controlState.angular = normalizedDist * Math.sin(angle);

    drawJoystickHandle(handleX, handleY);
    updateJoystickDisplay();
    sendJoystickCommand(controlState.linear, controlState.angular);
}

function updateJoystickDisplay() {
    document.getElementById('joy-linear').textContent = controlState.linear.toFixed(2);
    document.getElementById('joy-angular').textContent = controlState.angular.toFixed(2);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor display updates
// ─────────────────────────────────────────────────────────────────────────────

function updateIMUDisplay() {
    const h = sensorData.imu.heading;
    let dir = '—';
    if (h !== null && h !== undefined) {
        if (h <= 22.5 || h > 337.5) dir = 'N';
        else if (h <= 67.5) dir = 'NE';
        else if (h <= 112.5) dir = 'E';
        else if (h <= 157.5) dir = 'SE';
        else if (h <= 202.5) dir = 'S';
        else if (h <= 247.5) dir = 'SW';
        else if (h <= 292.5) dir = 'W';
        else dir = 'NW';
    }

    document.getElementById('imu-heading').textContent = (h !== null && h !== undefined) ? h.toFixed(1) : '—';
    document.getElementById('imu-heading-dir').textContent = dir;
    document.getElementById('imu-roll').textContent = (sensorData.imu.roll !== null) ? sensorData.imu.roll.toFixed(1) : '—';
    document.getElementById('imu-pitch').textContent = (sensorData.imu.pitch !== null) ? sensorData.imu.pitch.toFixed(1) : '—';
    document.getElementById('imu-yaw').textContent = (sensorData.imu.yaw !== null) ? sensorData.imu.yaw.toFixed(1) : '—';
    document.getElementById('imu-hz').textContent = sensorData.imu.hz.toFixed(1);
}

function updateRTKDisplay() {
    const formatCoord = (val) => (val !== null && val !== 0) ? val.toFixed(6) : '—';
    document.getElementById('rtk-lat').textContent = formatCoord(sensorData.rtk.lat);
    document.getElementById('rtk-lon').textContent = formatCoord(sensorData.rtk.lon);
    document.getElementById('rtk-alt').textContent = (sensorData.rtk.alt !== null) ? sensorData.rtk.alt.toFixed(2) : '—';
    document.getElementById('rtk-sats').textContent = sensorData.rtk.numSats;

    const fixNames = ['No Fix', 'GPS', 'DGPS', '?', 'RTK Fixed', 'RTK Float'];
    const fixName = fixNames[sensorData.rtk.fixQuality] || '?';
    document.getElementById('rtk-fix').textContent = fixName;
}

function updateStatusDisplay() {
    const s = sensorData.status;
    document.getElementById('status-mode').textContent = s.mode === 'joystick' ? 'Joystick' : 'Heading Follow';
    const watchdogSpan = document.getElementById('status-watchdog');
    if (s.watchdog) {
        watchdogSpan.classList.remove('watchdog-ok');
        watchdogSpan.classList.add('watchdog-alert');
        watchdogSpan.textContent = '✗ Timeout';
    } else {
        watchdogSpan.classList.remove('watchdog-alert');
        watchdogSpan.classList.add('watchdog-ok');
        watchdogSpan.textContent = '✓ Active';
    }
    document.getElementById('status-linear').textContent = s.linearVel.toFixed(2);
    document.getElementById('status-angular').textContent = s.angularVel.toFixed(2);

    if (s.mode === 'heading_follow') {
        const target = document.getElementById('target-heading');
        const current = sensorData.imu.heading;
        const error = (s.headingError !== null && s.headingError !== undefined) ? s.headingError.toFixed(1) : '—';
        document.getElementById('heading-target').textContent = target.value;
        document.getElementById('heading-current').textContent = (current !== null) ? current.toFixed(1) : '—';
        document.getElementById('heading-error').textContent = error;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Heading control UI
// ─────────────────────────────────────────────────────────────────────────────

function initHeadingControl() {
    const btnSetHeading = document.getElementById('btn-set-heading');
    const inputHeading = document.getElementById('target-heading');

    btnSetHeading.addEventListener('click', () => {
        const heading = parseFloat(inputHeading.value) || 0.0;
        if (heading < 0 || heading >= 360) {
            alert('Heading must be 0–359 degrees');
            return;
        }
        controlState.targetHeading = heading;
        sendHeadingCommand(heading);
        console.log(`Heading target set to ${heading}°`);
    });

    // Allow Enter key to set heading
    inputHeading.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') {
            btnSetHeading.click();
        }
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Initialization
// ─────────────────────────────────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
    console.log('Initializing PathFollower UI...');
    initJoystick();
    initHeadingControl();
    connectWebSocket();
});
