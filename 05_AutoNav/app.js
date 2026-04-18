/**
 * app.js — AutoNav browser client
 *
 * Reads config injected as data-* attributes on <html>:
 *   data-max-linear, data-max-angular, data-cam1-url, data-cam2-url, data-ws-port
 *
 * WebSocket messages OUT:
 *   { type: "joystick",     linear: float, angular: float, force: float }
 *   { type: "toggle_state" }
 *   { type: "heartbeat" }
 *
 * WebSocket messages IN:
 *   { type: "odom",        v, w, state, soc }
 *   { type: "imu",         ... }
 *   { type: "rtk",         available, lat, lon, fix_quality }
 *   { type: "nav_status",  state, progress, distance_m, heading_deg, nav_mode }
 *   { type: "state_status", active: bool }
 */

(function () {
  "use strict";

  // ── Config from injected HTML attributes ────────────────────────────────
  const root       = document.documentElement;
  const MAX_LIN    = parseFloat(root.dataset.maxLinear  || "1.0");
  const MAX_ANG    = parseFloat(root.dataset.maxAngular || "1.0");
  const CAM1_URL   = root.dataset.cam1Url  || "http://10.95.76.11:8080";
  const CAM2_URL   = root.dataset.cam2Url  || "http://10.95.76.10:8081";
  const WS_PORT    = parseInt(root.dataset.wsPort || "8806", 10);

  // ── DOM refs ─────────────────────────────────────────────────────────────
  const wsPill      = document.getElementById("ws-status");
  const robotPill   = document.getElementById("robot-status");
  const gpsPill     = document.getElementById("gps-status");
  const socLabel    = document.getElementById("soc-label");
  const camFeed     = document.getElementById("cam-feed");
  const camOverlay  = document.getElementById("cam-overlay");
  const camLabel    = document.getElementById("cam-label");
  const cam1Btn     = document.getElementById("cam1-btn");
  const cam2Btn     = document.getElementById("cam2-btn");
  const velDisplay  = document.getElementById("vel-display");
  const headingVal  = document.getElementById("heading-val");
  const distVal     = document.getElementById("dist-val");
  const navStateVal = document.getElementById("nav-state-val");
  const wpVal       = document.getElementById("wp-val");
  const toggleBtn   = document.getElementById("toggle-btn");
  const toggleLabel = document.getElementById("toggle-label");
  const navModeVal  = document.getElementById("nav-mode-val");
  const joystickRing  = document.querySelector(".joystick-ring");
  const joystickThumb = document.getElementById("joystick-thumb");
  const linBar      = document.getElementById("lin-bar");
  const angBar      = document.getElementById("ang-bar");
  const linVal      = document.getElementById("lin-val");
  const angVal      = document.getElementById("ang-val");

  // ── State ────────────────────────────────────────────────────────────────
  let ws              = null;
  let wsConnected     = false;
  let robotActive     = false;
  let currentCam      = 1;
  let joystickActive  = false;
  let joystickCenter  = { x: 0, y: 0 };
  let joystickRadius  = 0;
  let currentLinear   = 0;
  let currentAngular  = 0;
  let sendLoopId      = null;
  let heartbeatId     = null;

  // ── Camera ───────────────────────────────────────────────────────────────
  function switchCam(n) {
    currentCam = n;
    const url = n === 1 ? CAM1_URL : CAM2_URL;
    camFeed.src = url + "?t=" + Date.now();
    camLabel.textContent = `OAK-D · CAM ${n}`;
    cam1Btn.classList.toggle("active", n === 1);
    cam2Btn.classList.toggle("active", n === 2);
  }

  camFeed.addEventListener("load", () => {
    camOverlay.classList.add("hidden");
  });
  camFeed.addEventListener("error", () => {
    camOverlay.classList.remove("hidden");
    // Retry after 3s
    setTimeout(() => switchCam(currentCam), 3000);
  });

  // Expose to HTML
  window.switchCam = switchCam;

  // ── WebSocket ────────────────────────────────────────────────────────────
  function wsConnect() {
    const url = `ws://${location.hostname}:${WS_PORT}/`;
    ws = new WebSocket(url);

    ws.addEventListener("open", () => {
      wsConnected = true;
      setPill(wsPill, "ok", "WS");
      startHeartbeat();
    });

    ws.addEventListener("close", () => {
      wsConnected = false;
      setPill(wsPill, "error", "WS");
      setPill(robotPill, "", "ROBOT");
      setPill(gpsPill, "", "GPS");
      clearInterval(heartbeatId);
      // Send stop on disconnect
      currentLinear  = 0;
      currentAngular = 0;
      updateVelDisplay(0, 0);
      setTimeout(wsConnect, 2000);
    });

    ws.addEventListener("message", (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }
      handleMessage(msg);
    });
  }

  function wsSend(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj));
    }
  }

  function startHeartbeat() {
    clearInterval(heartbeatId);
    heartbeatId = setInterval(() => wsSend({ type: "heartbeat" }), 500);
  }

  // ── Message handler ───────────────────────────────────────────────────────
  function handleMessage(msg) {
    switch (msg.type) {

      case "odom":
        updateVelDisplay(msg.v ?? 0, msg.w ?? 0);
        if (msg.soc != null) {
          socLabel.textContent = msg.soc + "%";
        }
        if (msg.state != null) {
          setRobotActive(msg.state === 1);
        }
        break;

      case "state_status":
        setRobotActive(!!msg.active);
        break;

      case "rtk":
        setPill(gpsPill, msg.available ? "ok" : "warn", "GPS");
        break;

      case "nav_status":
        headingVal.textContent  = msg.heading_deg != null
          ? Math.round(msg.heading_deg) + "°" : "—°";
        distVal.textContent     = msg.distance_m != null
          ? msg.distance_m.toFixed(1) + "m" : "—m";
        navStateVal.textContent = (msg.state || "IDLE").toUpperCase();
        if (Array.isArray(msg.progress)) {
          wpVal.textContent = msg.progress[0] + "/" + msg.progress[1];
        }
        navModeVal.textContent  = (msg.nav_mode || "—").toUpperCase();
        break;
    }
  }

  // ── Robot active state ────────────────────────────────────────────────────
  function setRobotActive(active) {
    robotActive = active;
    setPill(robotPill, active ? "ok" : "warn", "ROBOT");
    toggleBtn.classList.toggle("active", active);
    toggleLabel.textContent = active ? "STOP" : "START";
    const icon = toggleBtn.querySelector(".toggle-icon");
    icon.textContent = active ? "■" : "▶";
  }

  // ── Toggle state ──────────────────────────────────────────────────────────
  function toggleState() {
    wsSend({ type: "toggle_state" });
  }
  window.toggleState = toggleState;

  // ── Status pill helper ────────────────────────────────────────────────────
  function setPill(el, cls, text) {
    el.className = "status-pill" + (cls ? " " + cls : "");
    el.textContent = text;
  }

  // ── Velocity display ──────────────────────────────────────────────────────
  function updateVelDisplay(lin, ang) {
    const absLin = Math.abs(lin);
    const absAng = Math.abs(ang);
    velDisplay.textContent = `${lin.toFixed(2)} m/s · ${ang.toFixed(2)} rad/s`;
    linVal.textContent = lin.toFixed(2);
    angVal.textContent = ang.toFixed(2);
    linBar.style.width = (absLin / MAX_LIN * 100).toFixed(1) + "%";
    angBar.style.width = (absAng / MAX_ANG * 100).toFixed(1) + "%";
  }

  // ── Joystick ──────────────────────────────────────────────────────────────
  function getJoystickCenter() {
    const rect = joystickRing.getBoundingClientRect();
    joystickCenter = {
      x: rect.left + rect.width  / 2,
      y: rect.top  + rect.height / 2,
    };
    joystickRadius = rect.width / 2;
  }

  function joystickMove(clientX, clientY) {
    const dx = clientX - joystickCenter.x;
    const dy = clientY - joystickCenter.y;
    const maxR = joystickRadius - 22; // thumb margin
    const dist  = Math.min(Math.hypot(dx, dy), maxR);
    const angle = Math.atan2(dy, dx);

    const nx = Math.cos(angle) * dist;
    const ny = Math.sin(angle) * dist;

    joystickThumb.style.left = (joystickCenter.x - joystickRing.getBoundingClientRect().left + nx) + "px";
    joystickThumb.style.top  = (joystickCenter.y - joystickRing.getBoundingClientRect().top  + ny) + "px";
    joystickThumb.style.transform = "translate(-50%, -50%)";

    // Map: Y-axis → linear (up=positive), X-axis → angular (left=positive)
    const normX = nx / maxR;
    const normY = -ny / maxR; // invert Y

    currentLinear  = normY * MAX_LIN;
    currentAngular = -normX * MAX_ANG; // right stick = negative angular (turn right)
  }

  function joystickRelease() {
    joystickActive = false;
    joystickRing.classList.remove("active");
    joystickThumb.style.left      = "50%";
    joystickThumb.style.top       = "50%";
    joystickThumb.style.transform = "translate(-50%, -50%)";
    currentLinear  = 0;
    currentAngular = 0;
    wsSend({ type: "joystick", linear: 0, angular: 0, force: 0.0 });
    updateVelDisplay(0, 0);
  }

  // Mouse events
  joystickRing.addEventListener("mousedown", (e) => {
    e.preventDefault();
    joystickActive = true;
    joystickRing.classList.add("active");
    getJoystickCenter();
    joystickMove(e.clientX, e.clientY);
  });

  document.addEventListener("mousemove", (e) => {
    if (!joystickActive) return;
    joystickMove(e.clientX, e.clientY);
  });

  document.addEventListener("mouseup", () => {
    if (joystickActive) joystickRelease();
  });

  // Touch events
  joystickRing.addEventListener("touchstart", (e) => {
    e.preventDefault();
    joystickActive = true;
    joystickRing.classList.add("active");
    getJoystickCenter();
    const t = e.touches[0];
    joystickMove(t.clientX, t.clientY);
  }, { passive: false });

  document.addEventListener("touchmove", (e) => {
    if (!joystickActive) return;
    e.preventDefault();
    const t = e.touches[0];
    joystickMove(t.clientX, t.clientY);
  }, { passive: false });

  document.addEventListener("touchend", () => {
    if (joystickActive) joystickRelease();
  });

  // ── Joystick send loop (20 Hz) ────────────────────────────────────────────
  function startSendLoop() {
    sendLoopId = setInterval(() => {
      if (!joystickActive) return;
      wsSend({
        type:    "joystick",
        linear:  parseFloat(currentLinear.toFixed(3)),
        angular: parseFloat(currentAngular.toFixed(3)),
        force:   1.0,
      });
      updateVelDisplay(currentLinear, currentAngular);
    }, 50);
  }

  // ── Keyboard support ──────────────────────────────────────────────────────
  const keysDown = new Set();

  document.addEventListener("keydown", (e) => {
    if (keysDown.has(e.key)) return;
    keysDown.add(e.key);
    applyKeys();
  });

  document.addEventListener("keyup", (e) => {
    keysDown.delete(e.key);
    applyKeys();
  });

  function applyKeys() {
    let lin = 0, ang = 0;
    if (keysDown.has("w") || keysDown.has("ArrowUp"))    lin  =  MAX_LIN * 0.5;
    if (keysDown.has("s") || keysDown.has("ArrowDown"))  lin  = -MAX_LIN * 0.5;
    if (keysDown.has("a") || keysDown.has("ArrowLeft"))  ang  =  MAX_ANG * 0.6;
    if (keysDown.has("d") || keysDown.has("ArrowRight")) ang  = -MAX_ANG * 0.6;
    if (keysDown.has("x")) { lin = 0; ang = 0; }
    if (keysDown.size === 0) { lin = 0; ang = 0; }

    currentLinear  = lin;
    currentAngular = ang;

    if (keysDown.size > 0 && (lin !== 0 || ang !== 0)) {
      wsSend({ type: "joystick", linear: lin, angular: ang, force: 1.0 });
      updateVelDisplay(lin, ang);
    } else if (keysDown.size === 0) {
      wsSend({ type: "joystick", linear: 0, angular: 0, force: 0.0 });
      updateVelDisplay(0, 0);
    }
  }

  // ── Init ─────────────────────────────────────────────────────────────────
  switchCam(1);
  wsConnect();
  startSendLoop();

})();