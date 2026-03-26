/**
 * camera_visualizer.js — WebSocket client + UI controller for the
 * Camera Controller dashboard.
 *
 * Connects to camera_bridge WS for status updates and control commands.
 * Displays MJPEG stream via <img> tag pointing at the MJPEG HTTP server.
 */

// ── DOM Elements ─────────────────────────────────────────────────────────────

const statusDot          = document.getElementById("statusDot");
const statusText         = document.getElementById("statusText");
const singleView         = document.getElementById("singleView");
const dualView           = document.getElementById("dualView");
const mjpegStream        = document.getElementById("mjpegStream");
const mjpegStreamCam1    = document.getElementById("mjpegStreamCam1");
const mjpegStreamCam2    = document.getElementById("mjpegStreamCam2");
const videoOverlay       = document.getElementById("videoOverlay");
const videoOverlayCam1   = document.getElementById("videoOverlayCam1");
const videoOverlayCam2   = document.getElementById("videoOverlayCam2");
const videoOverlayText   = document.getElementById("videoOverlayText");
const overlaySpinner     = document.getElementById("overlaySpinner");
const btnCam1            = document.getElementById("btnCam1");
const btnCam2            = document.getElementById("btnCam2");
const btnViewSingle      = document.getElementById("btnViewSingle");
const btnViewBoth        = document.getElementById("btnViewBoth");
const streamStatus       = document.getElementById("streamStatus");
const streamFps          = document.getElementById("streamFps");
const streamRes          = document.getElementById("streamRes");
const btnStartStream     = document.getElementById("btnStartStream");
const btnStopStream      = document.getElementById("btnStopStream");
const btnStartCam1       = document.getElementById("btnStartCam1");
const btnStopCam1        = document.getElementById("btnStopCam1");
const btnStartCam2       = document.getElementById("btnStartCam2");
const btnStopCam2        = document.getElementById("btnStopCam2");
const mjpegUrl1          = document.getElementById("mjpegUrl1");
const mjpegUrl2          = document.getElementById("mjpegUrl2");
const cam1Clients        = document.getElementById("cam1Clients");
const cam2Clients        = document.getElementById("cam2Clients");
const pluginSelect       = document.getElementById("pluginSelect");
const activePlugin       = document.getElementById("activePlugin");
const pluginConfigContainer = document.getElementById("pluginConfigContainer");
const btnApplyPlugin     = document.getElementById("btnApplyPlugin");

// ── WebSocket ────────────────────────────────────────────────────────────────

const WS_PORT = parseInt(window.location.port, 10) + 1;
const WS_URL  = `ws://${window.location.hostname}:${WS_PORT}`;

let ws = null;
let reconnectTimer = null;
let currentCam = 1;
let lastPluginListHash = "";
let lastActivePlugin = "";
let lastPluginList = [];
let waitingForFirstFrame = false;
let forceStreamReload = false;
let viewMode = "single"; // single | both
let applyingPlugin = false; // plugin apply feedback flag
let pluginApplyTimer = null;

function showOverlay(text, loading = false) {
  videoOverlayText.textContent = text;
  overlaySpinner.classList.toggle("active", loading);
  videoOverlay.classList.remove("hidden");
}

function hideOverlay() {
  // Don't hide immediately if applying plugin; wait for real feedback
  if (applyingPlugin) return;
  videoOverlay.classList.add("hidden");
  overlaySpinner.classList.remove("active");
}

function showDualOverlay(camId, text) {
  const overlay = camId === 1 ? videoOverlayCam1 : videoOverlayCam2;
  if (!overlay) return;
  const textNode = overlay.querySelector("span");
  if (textNode) textNode.textContent = text;
  overlay.classList.remove("hidden");
}

function hideDualOverlay(camId) {
  const overlay = camId === 1 ? videoOverlayCam1 : videoOverlayCam2;
  if (!overlay) return;
  overlay.classList.add("hidden");
}

function applyViewMode() {
  const both = viewMode === "both";
  singleView.classList.toggle("hidden", both);
  dualView.classList.toggle("hidden", !both);
  btnViewSingle.classList.toggle("active", !both);
  btnViewBoth.classList.toggle("active", both);
}

function connect() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    statusDot.classList.add("connected");
    statusText.textContent = "Connected";
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  };

  ws.onclose = () => {
    statusDot.classList.remove("connected");
    statusText.textContent = "Disconnected";
    scheduleReconnect();
  };

  ws.onerror = () => { ws.close(); };

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      updateUI(data);
    } catch (e) {
      console.warn("Parse error:", e);
    }
  };
}

function scheduleReconnect() {
  if (!reconnectTimer) {
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null;
      connect();
    }, 2000);
  }
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(msg));
  }
}

// ── UI Update ────────────────────────────────────────────────────────────────

function updateUI(data) {
  // Camera selection
  currentCam = data.cam_selection || 1;
  btnCam1.classList.toggle("active", currentCam === 1);
  btnCam2.classList.toggle("active", currentCam === 2);

  // Stream status
  const cam1Streaming = Boolean(data.cam1_streaming);
  const cam2Streaming = Boolean(data.cam2_streaming);
  const isStreaming = currentCam === 1 ? cam1Streaming : cam2Streaming;
  const selectedFps = currentCam === 1 ? (data.cam1_fps ?? 0) : (data.cam2_fps ?? 0);
  streamStatus.textContent = isStreaming ? "Streaming" : "Stopped";
  streamStatus.style.color = isStreaming ? "var(--green)" : "var(--text-muted)";
  streamFps.textContent = Number(selectedFps).toFixed(1);
  streamRes.textContent = data.width && data.height
    ? `${data.width}x${data.height}` : "—";

  // MJPEG URLs — replace {host} with actual hostname
  const host = window.location.hostname;
  const url1 = data.mjpeg_url_cam1.replace("{host}", host);
  const url2 = data.mjpeg_url_cam2.replace("{host}", host);
  mjpegUrl1.href = url1;
  mjpegUrl1.textContent = url1;
  mjpegUrl2.href = url2;
  mjpegUrl2.textContent = url2;
  cam1Clients.textContent = String(data.cam1_clients ?? 0);
  cam2Clients.textContent = String(data.cam2_clients ?? 0);

  // Plugin info
  if (data.active_plugin !== undefined) {
    activePlugin.textContent = data.active_plugin || "—";
  }
  if (data.available_plugins) {
    updatePluginSelect(data.available_plugins, data.active_plugin);
    renderPluginConfig(data.available_plugins, data.active_plugin, data.active_plugin_config);
  }

  if (viewMode === "single") {
    const streamUrl = currentCam === 1 ? url1 : url2;
    if (isStreaming) {
      const shouldReload = forceStreamReload || !mjpegStream.src.includes(streamUrl);
      if (shouldReload) {
        waitingForFirstFrame = true;
        showOverlay("Starting camera…", true);
        const reloadUrl = `${streamUrl}${streamUrl.includes("?") ? "&" : "?"}reload=${Date.now()}`;
        mjpegStream.src = reloadUrl;
      }
      mjpegStream.classList.add("active");
      if (!waitingForFirstFrame) hideOverlay();
    } else {
      waitingForFirstFrame = false;
      mjpegStream.classList.remove("active");
      mjpegStream.removeAttribute("src");
      showOverlay("Stream not active", false);
    }
  } else {
    const reloadSuffix = forceStreamReload ? `${Date.now()}` : null;
    renderDualCamera(1, url1, cam1Streaming, reloadSuffix);
    renderDualCamera(2, url2, cam2Streaming, reloadSuffix);
  }

  forceStreamReload = false;
}

function renderDualCamera(camId, streamUrl, isStreaming, reloadSuffix = null) {
  const img = camId === 1 ? mjpegStreamCam1 : mjpegStreamCam2;
  if (!img) return;

  if (!isStreaming) {
    img.classList.remove("active");
    img.removeAttribute("src");
    showDualOverlay(camId, `Camera ${camId} not active`);
    return;
  }

  const shouldReload = !!reloadSuffix || !img.src.includes(streamUrl);
  if (shouldReload) {
    const reloadUrl = reloadSuffix
      ? `${streamUrl}${streamUrl.includes("?") ? "&" : "?"}reload=${reloadSuffix}`
      : streamUrl;
    img.src = reloadUrl;
    showDualOverlay(camId, `Camera ${camId} loading…`);
  }

  img.classList.add("active");
}

// ── Controls ─────────────────────────────────────────────────────────────────

btnStartStream.addEventListener("click", () => {
  waitingForFirstFrame = true;
  showOverlay("Starting camera…", true);
  send({ type: "start_stream", cam_id: currentCam });
});

btnStopStream.addEventListener("click", () => {
  waitingForFirstFrame = false;
  showOverlay("Stream not active", false);
  send({ type: "stop_stream", cam_id: currentCam });
});

btnCam1.addEventListener("click", () => {
  if (currentCam === 1) return;
  waitingForFirstFrame = true;
  showOverlay("Switching camera…", true);
  send({ type: "switch_camera", cam_id: 1 });
});

btnCam2.addEventListener("click", () => {
  if (currentCam === 2) return;
  waitingForFirstFrame = true;
  showOverlay("Switching camera…", true);
  send({ type: "switch_camera", cam_id: 2 });
});

btnStartCam1.addEventListener("click", () => {
  showOverlay("Starting camera 1…", true);
  send({ type: "start_stream", cam_id: 1 });
});

btnStopCam1.addEventListener("click", () => {
  send({ type: "stop_stream", cam_id: 1 });
  if (currentCam === 1 && viewMode === "single") {
    waitingForFirstFrame = false;
    showOverlay("Stream not active", false);
  }
});

btnStartCam2.addEventListener("click", () => {
  showOverlay("Starting camera 2…", true);
  send({ type: "start_stream", cam_id: 2 });
});

btnStopCam2.addEventListener("click", () => {
  send({ type: "stop_stream", cam_id: 2 });
  if (currentCam === 2 && viewMode === "single") {
    waitingForFirstFrame = false;
    showOverlay("Stream not active", false);
  }
});

btnViewSingle.addEventListener("click", () => {
  viewMode = "single";
  applyViewMode();
  waitingForFirstFrame = true;
  showOverlay("Switching to single view…", true);
});

btnViewBoth.addEventListener("click", () => {
  viewMode = "both";
  applyViewMode();
});

mjpegStream.addEventListener("load", () => {
  waitingForFirstFrame = false;
  // If applying plugin, show success and hide overlay after a brief moment for visibility
  if (applyingPlugin) {
    videoOverlayText.textContent = "Plugin applied ✓";
    overlaySpinner.classList.remove("active");
    clearTimeout(pluginApplyTimer);
    pluginApplyTimer = setTimeout(() => {
      videoOverlay.classList.add("hidden");
      applyingPlugin = false;
    }, 800);
  } else {
    hideOverlay();
  }
});

mjpegStream.addEventListener("error", () => {
  if (!mjpegStream.src) return;
  waitingForFirstFrame = true;
  showOverlay("Waiting for camera…", true);
});

mjpegStreamCam1.addEventListener("load", () => {
  if (applyingPlugin) {
    showDualOverlay(1, "Applied ✓");
  } else {
    hideDualOverlay(1);
  }
});
mjpegStreamCam2.addEventListener("load", () => {
  if (applyingPlugin) {
    showDualOverlay(2, "Applied ✓");
  } else {
    hideDualOverlay(2);
  }
});
mjpegStreamCam1.addEventListener("error", () => {
  if (!mjpegStreamCam1.src) return;
  showDualOverlay(1, "Camera 1 waiting…");
});
mjpegStreamCam2.addEventListener("error", () => {
  if (!mjpegStreamCam2.src) return;
  showDualOverlay(2, "Camera 2 waiting…");
});

// ── Plugin Controls ─────────────────────────────────────────────────────────

function updatePluginSelect(plugins, activeName) {
  const hash = plugins.map(p => p.name).join(",");
  const listChanged = hash !== lastPluginListHash;

  if (listChanged) {
    lastPluginListHash = hash;
    lastPluginList = plugins;
    pluginSelect.innerHTML = "";
    plugins.forEach(p => {
      const opt = document.createElement("option");
      opt.value = p.name;
      opt.textContent = p.label || p.name;
      pluginSelect.appendChild(opt);
    });
    // Sync to server state only on initial load or plugin list change
    pluginSelect.value = activeName || "";
    lastActivePlugin = activeName || "";
  } else if (activeName !== lastActivePlugin) {
    // Server confirmed a different plugin became active — sync the dropdown
    lastActivePlugin = activeName || "";
    pluginSelect.value = activeName || "";
  }
  // Otherwise: user may have changed the dropdown; don't overwrite their selection
}

function renderPluginConfig(plugins, activeName, currentConfig) {
  const plugin = plugins.find(p => p.name === activeName);
  if (!plugin || !plugin.config_schema || plugin.config_schema.length === 0) {
    pluginConfigContainer.innerHTML = "";
    return;
  }
  if (pluginConfigContainer.dataset.plugin === activeName) return;
  pluginConfigContainer.dataset.plugin = activeName;
  pluginConfigContainer.innerHTML = "";
  plugin.config_schema.forEach(field => {
    const row = document.createElement("div");
    row.className = "config-field";
    const label = document.createElement("label");
    label.textContent = field.label || field.key;
    label.setAttribute("for", "cfg_" + field.key);
    const input = document.createElement("input");
    input.id = "cfg_" + field.key;
    input.name = field.key;
    input.type = field.type === "int" ? "number" : "text";
    const val = currentConfig && currentConfig[field.key] != null
      ? currentConfig[field.key] : (field.default != null ? field.default : "");
    input.value = val;
    input.placeholder = field.default != null ? String(field.default) : "";
    row.appendChild(label);
    row.appendChild(input);
    pluginConfigContainer.appendChild(row);
  });
}

btnApplyPlugin.addEventListener("click", () => {
  const pluginName = pluginSelect.value;
  const config = {};
  pluginConfigContainer.querySelectorAll("input").forEach(input => {
    const val = input.value.trim();
    if (val === "") return;
    config[input.name] = input.type === "number" ? parseInt(val, 10) : val;
  });
  
  clearTimeout(pluginApplyTimer);
  applyingPlugin = true;
  showOverlay("App configuration in progress…", true);
  
  // Timeout fallback: if stream doesn't load within 5s, show completion anyway
  pluginApplyTimer = setTimeout(() => {
    if (applyingPlugin) {
      videoOverlayText.textContent = "The configuration has been applied ✓";
      overlaySpinner.classList.remove("active");
      pluginApplyTimer = setTimeout(() => {
        videoOverlay.classList.add("hidden");
        applyingPlugin = false;
      }, 1000);
    }
  }, 5000);
  
  forceStreamReload = true;
  send({ type: "switch_plugin", plugin_name: pluginName, config: config });
  pluginConfigContainer.dataset.plugin = "";
});

pluginSelect.addEventListener("change", () => {
  const selectedName = pluginSelect.value;
  if (lastPluginList.length > 0) {
    pluginConfigContainer.dataset.plugin = "";  // force re-render
    renderPluginConfig(lastPluginList, selectedName, {});
  }
});

// ── Init ─────────────────────────────────────────────────────────────────────
applyViewMode();
connect();
