/**
 * recorder_visualizer.js — WebSocket client + UI controller for the
 * Data Recorder dashboard.
 *
 * Connects to recorder_bridge WS, displays recording status, data source
 * connection states, and file management (download / delete).
 */

// ── DOM Elements ─────────────────────────────────────────────────────────────

const statusDot       = document.getElementById("statusDot");
const statusText      = document.getElementById("statusText");
const recordIndicator = document.getElementById("recordIndicator");
const recordStatusText = document.getElementById("recordStatusText");
const currentFile     = document.getElementById("currentFile");
const rowCount        = document.getElementById("rowCount");
const elapsed         = document.getElementById("elapsed");
const btnStart        = document.getElementById("btnStart");
const btnStop         = document.getElementById("btnStop");
const srcImu          = document.getElementById("srcImu");
const srcRtk          = document.getElementById("srcRtk");
const srcRobot        = document.getElementById("srcRobot");
const fileListEl      = document.getElementById("fileList");

// ── WebSocket ────────────────────────────────────────────────────────────────

const WS_PORT = parseInt(window.location.port, 10) + 1;
const WS_URL  = `ws://${window.location.hostname}:${WS_PORT}`;

let ws = null;
let reconnectTimer = null;

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

  ws.onerror = () => {
    ws.close();
  };

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
  // Recording status
  const isRecording = data.recording;
  recordIndicator.classList.toggle("recording", isRecording);
  recordStatusText.textContent = isRecording ? "Recording..." : "Stopped";
  currentFile.textContent = data.current_filename || "—";
  rowCount.textContent = data.row_count.toLocaleString();
  elapsed.textContent = formatElapsed(data.elapsed_s);

  // Button states
  btnStart.disabled = isRecording;
  btnStop.disabled  = !isRecording;

  // Data source indicators
  const sources = data.sources || {};
  srcImu.classList.toggle("connected", !!sources.imu);
  srcRtk.classList.toggle("connected", !!sources.rtk);
  srcRobot.classList.toggle("connected", !!sources.robot);

  // File list
  updateFileList(data.file_list || []);
}

function formatElapsed(s) {
  if (s < 60) return s.toFixed(1) + "s";
  const m = Math.floor(s / 60);
  const sec = (s % 60).toFixed(0);
  return `${m}m ${sec}s`;
}

function updateFileList(files) {
  if (files.length === 0) {
    fileListEl.innerHTML = '<div class="file-empty">No files yet</div>';
    return;
  }

  fileListEl.innerHTML = files.map(f => `
    <div class="file-item" data-name="${f.name}">
      <div class="file-info">
        <span class="file-name">${f.name}</span>
        <span class="file-meta">${f.size_kb} KB · ${f.modified}</span>
      </div>
      <div class="file-actions">
        <button class="btn-download" onclick="downloadFile('${f.name}')">Download</button>
        <button class="btn-delete" onclick="deleteFile('${f.name}')">Delete</button>
      </div>
    </div>
  `).join("");
}

// ── Actions ──────────────────────────────────────────────────────────────────

btnStart.addEventListener("click", () => {
  send({ type: "start_recording" });
});

btnStop.addEventListener("click", () => {
  send({ type: "stop_recording" });
});

function downloadFile(filename) {
  const a = document.createElement("a");
  a.href = `/data/${encodeURIComponent(filename)}`;
  a.download = filename;
  a.click();
}

function deleteFile(filename) {
  if (!confirm(`Delete ${filename}?`)) return;
  send({ type: "delete_file", filename });
}

// ── Init ─────────────────────────────────────────────────────────────────────
connect();
