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
const btnRestartCamera   = document.getElementById("btnRestartCamera");
const settingsFps        = document.getElementById("settingsFps");
const settingsWidth      = document.getElementById("settingsWidth");
const settingsHeight     = document.getElementById("settingsHeight");

// ── WebSocket ────────────────────────────────────────────────────────────────

const WS_PORT = parseInt(window.location.port, 10) + 1;
const WS_URL  = `ws://${window.location.hostname}:${WS_PORT}`;

let ws = null;
let reconnectTimer = null;
let currentCam = 1;
let lastPluginListHash = "";
let lastActivePlugin = "";
let lastPluginList = [];
let pendingPluginSelection = null;
let waitingForFirstFrame = false;
let viewMode = "single"; // single | both
let applyingPlugin = false;
let pluginApplyTimer = null;
let activePluginName = "";
let liveUpdateTimer = null;

function showOverlay(text, loading = false) {
  videoOverlayText.textContent = text;
  overlaySpinner.classList.toggle("active", loading);
  videoOverlay.classList.remove("hidden");
}

function hideOverlay() {
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

  // MJPEG URLs
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
    activePluginName = data.active_plugin || "";
    activePlugin.textContent = data.active_plugin || "—";
  }
  if (data.available_plugins) {
    updatePluginSelect(data.available_plugins, data.active_plugin);
    const selectedPlugin = pluginSelect.value || data.active_plugin;
    const selectedConfig = selectedPlugin === data.active_plugin ? data.active_plugin_config : {};
    renderPluginConfig(data.available_plugins, selectedPlugin, selectedConfig);
  }

  // When server confirms restart finished (streaming resumed after restart)
  if (applyingPlugin && isStreaming) {
    finishApply();
    hideOverlay();
  }

  if (viewMode === "single") {
    const streamUrl = currentCam === 1 ? url1 : url2;
    if (isStreaming) {
      if (!mjpegStream.src.includes(streamUrl)) {
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
    renderDualCamera(1, url1, cam1Streaming);
    renderDualCamera(2, url2, cam2Streaming);
  }
}

function renderDualCamera(camId, streamUrl, isStreaming) {
  const img = camId === 1 ? mjpegStreamCam1 : mjpegStreamCam2;
  if (!img) return;

  if (!isStreaming) {
    img.classList.remove("active");
    img.removeAttribute("src");
    showDualOverlay(camId, `Camera ${camId} not active`);
    return;
  }

  if (!img.src.includes(streamUrl)) {
    img.src = streamUrl;
    showDualOverlay(camId, `Camera ${camId} loading…`);
  }
  img.classList.add("active");
}

// ── Controls ─────────────────────────────────────────────────────────────────

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
  hideOverlay();
});

mjpegStream.addEventListener("error", () => {
  if (!mjpegStream.src) return;
  waitingForFirstFrame = true;
  showOverlay("Waiting for camera…", true);
});

mjpegStreamCam1.addEventListener("load", () => { hideDualOverlay(1); });
mjpegStreamCam2.addEventListener("load", () => { hideDualOverlay(2); });
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
  const previousSelection = pluginSelect.value;
  const hash = plugins.map(p => p.name).join(",");
  const listChanged = hash !== lastPluginListHash;
  const hasPlugin = (name) => !!name && plugins.some(p => p.name === name);

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

    const preferredSelection =
      (hasPlugin(pendingPluginSelection) && pendingPluginSelection) ||
      (hasPlugin(previousSelection) && previousSelection) ||
      (activeName || "");
    pluginSelect.value = preferredSelection;
    lastActivePlugin = activeName || "";
  } else if (activeName !== lastActivePlugin) {
    lastActivePlugin = activeName || "";
    if (!pendingPluginSelection || pendingPluginSelection === lastActivePlugin) {
      pluginSelect.value = lastActivePlugin;
      pendingPluginSelection = null;
    }
  }

  if (pendingPluginSelection && pendingPluginSelection === lastActivePlugin) {
    pendingPluginSelection = null;
  }
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
    const value = currentConfig && currentConfig[field.key] != null
      ? currentConfig[field.key]
      : (field.default != null ? field.default : "");

    if (field.type === "enum" && Array.isArray(field.options) && field.options.length > 0) {
      row.classList.add("config-field-radio");
      const title = document.createElement("div");
      title.className = "config-field-title";
      title.textContent = field.label || field.key;
      row.appendChild(title);

      const radioGroup = document.createElement("div");
      radioGroup.className = "radio-group";
      field.options.forEach((opt, index) => {
        const optVal = typeof opt === "string" ? opt : opt.value;
        const optLabel = typeof opt === "string" ? opt : (opt.label || opt.value);
        const item = document.createElement("label");
        item.className = "radio-item";
        const input = document.createElement("input");
        input.type = "radio";
        input.name = field.key;
        input.value = String(optVal);
        input.dataset.cfgType = "enum";
        input.id = `cfg_${field.key}_${index}`;
        input.checked = String(value) === String(optVal);
        const text = document.createElement("span");
        text.textContent = String(optLabel);
        item.appendChild(input);
        item.appendChild(text);
        radioGroup.appendChild(item);
      });
      row.appendChild(radioGroup);
    } else if (field.type === "bool") {
      row.classList.add("config-field-checkbox");
      const label = document.createElement("label");
      label.className = "checkbox-item";
      const input = document.createElement("input");
      input.type = "checkbox";
      input.name = field.key;
      input.dataset.cfgType = "bool";
      input.checked = Boolean(value);
      const text = document.createElement("span");
      text.textContent = field.label || field.key;
      label.appendChild(input);
      label.appendChild(text);
      row.appendChild(label);
    } else if (field.type === "range") {
      row.classList.add("config-field-range");
      const label = document.createElement("label");
      label.textContent = field.label || field.key;
      label.setAttribute("for", "cfg_" + field.key);

      const sliderWrap = document.createElement("div");
      sliderWrap.className = "range-wrap";

      const input = document.createElement("input");
      input.id = "cfg_" + field.key;
      input.name = field.key;
      input.type = "range";
      input.min = String(field.min ?? 0);
      input.max = String(field.max ?? 100);
      input.step = String(field.step ?? 1);
      input.value = String(value === "" ? (field.default ?? 0) : value);
      input.dataset.cfgType = "range";

      const valueText = document.createElement("span");
      valueText.className = "range-value";
      valueText.textContent = String(input.value);

      input.addEventListener("input", () => {
        valueText.textContent = input.value;
        if (pluginSelect.value !== activePluginName) return;
        clearTimeout(liveUpdateTimer);
        liveUpdateTimer = setTimeout(() => {
          send({
            type: "update_plugin_config",
            config: { [field.key]: parseInt(input.value, 10) },
          });
        }, 120);
      });

      sliderWrap.appendChild(input);
      sliderWrap.appendChild(valueText);
      row.appendChild(label);
      row.appendChild(sliderWrap);
    } else {
      const label = document.createElement("label");
      label.textContent = field.label || field.key;
      label.setAttribute("for", "cfg_" + field.key);
      const input = document.createElement("input");
      input.id = "cfg_" + field.key;
      input.name = field.key;
      input.type = field.type === "int" ? "number" : "text";
      input.dataset.cfgType = field.type === "int" ? "int" : "str";
      input.value = value;
      input.placeholder = field.default != null ? String(field.default) : "";
      row.appendChild(label);
      row.appendChild(input);
    }

    pluginConfigContainer.appendChild(row);
  });
}

function setApplyLoading(loading, label = "Applying…") {
  btnApplyPlugin.disabled = loading;
  btnApplyPlugin.textContent = loading ? label : "Apply Plugin";
}

function finishApply() {
  clearTimeout(pluginApplyTimer);
  applyingPlugin = false;
  setApplyLoading(false);
}

btnApplyPlugin.addEventListener("click", () => {
  const pluginName = pluginSelect.value;
  pendingPluginSelection = pluginName;
  const config = {};

  pluginConfigContainer.querySelectorAll('input[type="checkbox"]').forEach(input => {
    config[input.name] = input.checked;
  });
  pluginConfigContainer.querySelectorAll('input[type="radio"]:checked').forEach(input => {
    config[input.name] = input.value;
  });
  pluginConfigContainer.querySelectorAll('input[type="number"], input[type="text"], input[type="range"]').forEach(input => {
    const val = input.value.trim();
    if (val === "") return;
    config[input.name] = input.type === "number" || input.type === "range"
      ? parseInt(val, 10) : val;
  });

  clearTimeout(pluginApplyTimer);
  applyingPlugin = true;
  pluginConfigContainer.dataset.plugin = "";

  // All plugins are processors — always instant, no stream restart
  setApplyLoading(true, "Applying…");
  send({ type: "switch_plugin", plugin_name: pluginName, config: config });
  pluginApplyTimer = setTimeout(finishApply, 1000);
});

pluginSelect.addEventListener("change", () => {
  const selectedName = pluginSelect.value;
  pendingPluginSelection = selectedName;
  if (lastPluginList.length > 0) {
    pluginConfigContainer.dataset.plugin = "";
    renderPluginConfig(lastPluginList, selectedName, {});
  }
});

// ── Advanced Settings ─────────────────────────────────────────────────────────

if (btnRestartCamera) {
  btnRestartCamera.addEventListener("click", () => {
    const fps    = parseInt(settingsFps.value, 10) || 30;
    const width  = parseInt(settingsWidth.value, 10) || 1280;
    const height = parseInt(settingsHeight.value, 10) || 720;

    if (!confirm(`Restart camera with ${width}x${height} @ ${fps}fps?\nThe stream will be interrupted briefly.`)) {
      return;
    }

    applyingPlugin = true;
    showOverlay("Restarting camera…", true);
    btnRestartCamera.disabled = true;
    btnRestartCamera.textContent = "Restarting…";

    send({ type: "restart_camera", fps, width, height });

    // Re-enable button after timeout (server broadcasts new status when done)
    setTimeout(() => {
      btnRestartCamera.disabled = false;
      btnRestartCamera.textContent = "Restart Camera";
    }, 8000);
  });
}

// ── Init ─────────────────────────────────────────────────────────────────────
applyViewMode();
connect();
