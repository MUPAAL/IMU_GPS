/**
 * camera_visualizer.js — WebSocket client + UI controller for the
 * Camera Controller dashboard.
 *
 * Connects to camera_bridge WS for status updates and control commands.
 * Displays MJPEG stream via <img> tag pointing at the MJPEG HTTP server.
 */

// ── DOM Elements ─────────────────────────────────────────────────────────────

const statusDot     = document.getElementById("statusDot");
const statusText    = document.getElementById("statusText");
const mjpegStream   = document.getElementById("mjpegStream");
const videoOverlay  = document.getElementById("videoOverlay");
const btnCam1       = document.getElementById("btnCam1");
const btnCam2       = document.getElementById("btnCam2");
const streamStatus  = document.getElementById("streamStatus");
const streamFps     = document.getElementById("streamFps");
const streamRes     = document.getElementById("streamRes");
const btnStartStream = document.getElementById("btnStartStream");
const btnStopStream  = document.getElementById("btnStopStream");
const mjpegUrl1     = document.getElementById("mjpegUrl1");
const mjpegUrl2     = document.getElementById("mjpegUrl2");
const pluginSelect  = document.getElementById("pluginSelect");
const activePlugin  = document.getElementById("activePlugin");
const pluginConfigContainer = document.getElementById("pluginConfigContainer");
const btnApplyPlugin = document.getElementById("btnApplyPlugin");

// ── WebSocket ────────────────────────────────────────────────────────────────

const WS_PORT = parseInt(window.location.port, 10) + 1;
const WS_URL  = `ws://${window.location.hostname}:${WS_PORT}`;

let ws = null;
let reconnectTimer = null;
let currentCam = 1;
let lastPluginListHash = "";

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
  const isStreaming = data.streaming;
  streamStatus.textContent = isStreaming ? "Streaming" : "Stopped";
  streamStatus.style.color = isStreaming ? "var(--green)" : "var(--text-muted)";
  streamFps.textContent = data.fps.toFixed(1);
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

  // Plugin info
  if (data.active_plugin !== undefined) {
    activePlugin.textContent = data.active_plugin || "—";
  }
  if (data.available_plugins) {
    updatePluginSelect(data.available_plugins, data.active_plugin);
    renderPluginConfig(data.available_plugins, data.active_plugin, data.active_plugin_config);
  }

  // Update MJPEG <img> source
  if (isStreaming) {
    const streamUrl = currentCam === 1 ? url1 : url2;
    if (mjpegStream.src !== streamUrl) {
      mjpegStream.src = streamUrl;
    }
    mjpegStream.classList.add("active");
    videoOverlay.classList.add("hidden");
  } else {
    mjpegStream.classList.remove("active");
    mjpegStream.removeAttribute("src");
    videoOverlay.classList.remove("hidden");
  }
}

// ── Controls ─────────────────────────────────────────────────────────────────

btnCam1.addEventListener("click", () => {
  send({ type: "switch_camera", cam_id: 1 });
});

btnCam2.addEventListener("click", () => {
  send({ type: "switch_camera", cam_id: 2 });
});

btnStartStream.addEventListener("click", () => {
  send({ type: "start_stream", cam_id: currentCam });
});

btnStopStream.addEventListener("click", () => {
  send({ type: "stop_stream", cam_id: currentCam });
});

// ── Plugin Controls ─────────────────────────────────────────────────────────

function updatePluginSelect(plugins, activeName) {
  // Hash to avoid unnecessary DOM rebuilds
  const hash = plugins.map(p => p.name).join(",");
  if (hash === lastPluginListHash) {
    // Just update selection
    pluginSelect.value = activeName || "";
    return;
  }
  lastPluginListHash = hash;
  pluginSelect.innerHTML = "";
  plugins.forEach(p => {
    const opt = document.createElement("option");
    opt.value = p.name;
    opt.textContent = p.label || p.name;
    pluginSelect.appendChild(opt);
  });
  pluginSelect.value = activeName || "";
}

function renderPluginConfig(plugins, activeName, currentConfig) {
  const plugin = plugins.find(p => p.name === activeName);
  if (!plugin || !plugin.config_schema || plugin.config_schema.length === 0) {
    pluginConfigContainer.innerHTML = "";
    return;
  }
  // Only rebuild if plugin changed
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
  send({ type: "switch_plugin", plugin_name: pluginName, config: config });
  // Force config re-render on next update
  pluginConfigContainer.dataset.plugin = "";
});

// ── Init ─────────────────────────────────────────────────────────────────────
connect();
