"""
recorder_bridge.py — Multi-source data recorder: subscribes to IMU, RTK, and
Robot WebSocket feeds and writes timestamped CSV files for offline analysis.

Data flow:
    imu_bridge(WS)   ──→ ImuWsClient.latest   ─┐
    rtk_bridge(WS)   ──→ RtkWsClient.latest   ─┤── RecordLoop(5Hz) → RecorderPipeline → CSV
    robot_bridge(WS)  ──→ RobotWsClient.latest ─┘
                                                       ↓
                                   WebSocketServer.broadcast() → browser (status)

    Browser → WebSocketServer → _handle_client_message → start/stop/list/delete

Usage:
    python recorder_bridge.py --ws-port 8825 \
        --imu-ws ws://localhost:8766 --rtk-ws ws://localhost:8776 \
        --robot-ws ws://localhost:8796 --record-hz 5.0 --data-dir ./data_log
    # Browser: http://localhost:8825
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import logging
import os
import socketserver
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from datetime import datetime
from http.server import SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import unquote

import websockets


# ── Logger setup ─────────────────────────────────────────────────────────────

def _setup_logger() -> logging.Logger:
    """Configure module-level logger; output to recorder_bridge.log and stderr."""
    py_name = Path(__file__).stem
    output_path = Path(__file__).parent
    log_file = output_path / f"{py_name}.log"

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[
            logging.FileHandler(log_file, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    return logging.getLogger(__name__)


logger = _setup_logger()


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ═════════════════════════════════════════════════════════════════════════════

_CSV_HEADER = [
    "timestamp",
    # IMU fields
    "quat_i", "quat_j", "quat_k", "quat_r",
    "euler_yaw", "euler_pitch", "euler_roll",
    # RTK fields
    "lat", "lon", "alt",
    "fix_quality", "num_sats", "hdop",
    "speed_knots", "track_deg",
    # Robot fields
    "meas_speed", "meas_ang_rate",
    "robot_state", "robot_soc",
    "total_distance_m", "heading_est_deg",
]


@dataclass
class RecorderFrame:
    """
    Status payload broadcast to the browser UI each tick.

    Carries recording state, current file info, and connection status.
    """

    recording: bool = False
    current_filename: str = ""
    row_count: int = 0
    elapsed_s: float = 0.0
    file_list: list[dict] = field(default_factory=list)
    sources: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Serialize to broadcast-ready dict."""
        return {
            "recording": self.recording,
            "current_filename": self.current_filename,
            "row_count": self.row_count,
            "elapsed_s": round(self.elapsed_s, 1),
            "file_list": self.file_list,
            "sources": self.sources,
        }


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE
# ═════════════════════════════════════════════════════════════════════════════

class DataRecorder:
    """
    Thread-safe CSV writer for multi-source sensor data.

    Manages file lifecycle (start/stop) and provides row counting,
    elapsed time tracking, and file listing.
    """

    def __init__(self, data_dir: Path) -> None:
        self._data_dir = data_dir
        self._data_dir.mkdir(parents=True, exist_ok=True)
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        self._recording = False
        self._current_filename = ""
        self._row_count = 0
        self._start_time = 0.0

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def current_filename(self) -> str:
        return self._current_filename

    @property
    def row_count(self) -> int:
        return self._row_count

    @property
    def elapsed_s(self) -> float:
        if not self._recording:
            return 0.0
        return time.monotonic() - self._start_time

    def start(self) -> str:
        """Create a new CSV file and write the header row. Returns filename."""
        with self._lock:
            if self._recording:
                logger.warning("DataRecorder: start() while already recording — stopping previous")
                self._close_file()

            ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{ts_str}.csv"
            filepath = self._data_dir / filename

            try:
                self._file = open(filepath, "w", newline="", encoding="utf-8")
                self._writer = csv.writer(self._file)
                self._writer.writerow(_CSV_HEADER)
                self._file.flush()
                self._recording = True
                self._current_filename = filename
                self._row_count = 0
                self._start_time = time.monotonic()
                logger.info("DataRecorder: recording started -> %s", filepath)
                return filename
            except OSError as exc:
                logger.error("DataRecorder: failed to create CSV [%s]: %s", filepath, exc)
                self._file = None
                self._writer = None
                self._recording = False
                raise

    def stop(self) -> None:
        """Close the current CSV file and stop recording."""
        with self._lock:
            self._close_file()

    def record(self, imu: dict, rtk: dict, robot: dict) -> None:
        """Write one row to the CSV. Silently skips if not recording."""
        if not self._recording:
            return

        ts = datetime.now().isoformat(timespec="milliseconds")

        quat = imu.get("quaternion", {})
        euler = imu.get("euler", {})

        row = [
            ts,
            # IMU
            _fmt(quat.get("i")),
            _fmt(quat.get("j")),
            _fmt(quat.get("k")),
            _fmt(quat.get("r")),
            _fmt(euler.get("yaw"), 2),
            _fmt(euler.get("pitch"), 2),
            _fmt(euler.get("roll"), 2),
            # RTK
            _fmt(rtk.get("lat"), 8),
            _fmt(rtk.get("lon"), 8),
            _fmt(rtk.get("alt"), 3),
            rtk.get("fix_quality", ""),
            rtk.get("num_sats", ""),
            _fmt(rtk.get("hdop"), 2),
            _fmt(rtk.get("speed_knots"), 2),
            _fmt(rtk.get("track_deg"), 2),
            # Robot
            _fmt(robot.get("meas_speed"), 3),
            _fmt(robot.get("meas_ang_rate"), 3),
            robot.get("state", ""),
            robot.get("soc", ""),
            _fmt(robot.get("total_distance_m"), 2),
            _fmt(robot.get("heading_est_deg"), 1),
        ]

        with self._lock:
            if not self._recording or self._writer is None:
                return
            try:
                self._writer.writerow(row)
                self._file.flush()
                self._row_count += 1
            except OSError as exc:
                logger.error("DataRecorder: failed to write CSV row: %s", exc)
                self._recording = False
                self._close_file()

    def list_files(self) -> list[dict]:
        """Return list of CSV files in data_dir with metadata."""
        result = []
        try:
            for p in sorted(self._data_dir.glob("*.csv"), reverse=True):
                stat = p.stat()
                result.append({
                    "name": p.name,
                    "size_kb": round(stat.st_size / 1024, 1),
                    "modified": datetime.fromtimestamp(stat.st_mtime).isoformat(timespec="seconds"),
                })
        except OSError as exc:
            logger.error("DataRecorder: failed to list files: %s", exc)
        return result

    def delete_file(self, filename: str) -> bool:
        """Delete a CSV file by name. Returns True if successful."""
        filepath = self._data_dir / filename
        # Prevent path traversal
        if filepath.parent.resolve() != self._data_dir.resolve():
            logger.warning("DataRecorder: path traversal attempt blocked: %s", filename)
            return False
        try:
            filepath.unlink()
            logger.info("DataRecorder: deleted %s", filename)
            return True
        except OSError as exc:
            logger.error("DataRecorder: failed to delete %s: %s", filename, exc)
            return False

    def _close_file(self) -> None:
        """Close file handle. Must be called with self._lock held."""
        if self._file is not None:
            try:
                self._file.close()
                logger.info("DataRecorder: recording stopped, file closed: %s", self._current_filename)
            except OSError as exc:
                logger.warning("DataRecorder: error closing file: %s", exc)
            finally:
                self._file = None
                self._writer = None
        self._recording = False
        self._current_filename = ""
        self._row_count = 0


class RecorderPipeline:
    """
    Pipeline Pattern — samples WS client data, writes CSV, produces status.

        WS client latest snapshots
          → _sample()      → (imu, rtk, robot) dicts
          → _record()      → write CSV row (if recording)
          → _serialize()   → RecorderFrame JSON for browser
    """

    def __init__(self, recorder: DataRecorder) -> None:
        self._recorder = recorder

    def process(self, imu: dict, rtk: dict, robot: dict, sources: dict) -> str:
        """
        CORE — Run the full pipeline on one tick.

        Receives latest snapshots from WS clients (INPUT) and returns a
        serialized JSON string ready for WebSocketServer to broadcast (OUTPUT).
        """
        self._record(imu, rtk, robot)
        return self._serialize(sources)

    def _record(self, imu: dict, rtk: dict, robot: dict) -> None:
        """Stage 1: write a CSV row if recording."""
        self._recorder.record(imu, rtk, robot)

    def _serialize(self, sources: dict) -> str:
        """Stage 2: build RecorderFrame status and serialize to JSON."""
        frame = RecorderFrame(
            recording=self._recorder.is_recording,
            current_filename=self._recorder.current_filename,
            row_count=self._recorder.row_count,
            elapsed_s=self._recorder.elapsed_s,
            file_list=self._recorder.list_files(),
            sources=sources,
        )
        return json.dumps(frame.to_dict())


def _fmt(value, decimals: int = 6) -> str:
    """Format a numeric value to string, or empty string if None."""
    if value is None:
        return ""
    try:
        return f"{float(value):.{decimals}f}"
    except (TypeError, ValueError):
        return str(value)


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class _WsClient:
    """
    Base WebSocket client that connects to a bridge and stores the latest frame.

    Reconnects automatically every 3s on failure.
    """

    RECONNECT_DELAY_S = 3.0

    def __init__(self, name: str, ws_url: str) -> None:
        self._name = name
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._connected = False

    @property
    def latest(self) -> dict:
        """Thread-safe access to the most recent frame."""
        with self._lock:
            return dict(self._latest)

    @property
    def connected(self) -> bool:
        return self._connected

    async def run(self) -> None:
        """Connect to upstream WS and continuously read frames."""
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("%s: connected to %s", self._name, self._url)
                    self._connected = True
                    # ── INPUT ──────────────────────────────────────────────────────────
                    # JSON frames arrive here from the upstream bridge via WebSocket.
                    # If data looks wrong, start debugging from this line.
                    async for msg in ws:
                    # ───────────────────────────────────────────────────────────────────
                        try:
                            data = json.loads(msg)
                            with self._lock:
                                self._latest = data
                        except json.JSONDecodeError as exc:
                            logger.warning("%s: JSON parse error: %s", self._name, exc)
            except Exception as exc:
                logger.warning(
                    "%s: connection to %s failed: %s — retrying in %.0fs",
                    self._name, self._url, exc, self.RECONNECT_DELAY_S,
                )
            finally:
                self._connected = False
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class ImuWsClient(_WsClient):
    """WebSocket client for imu_bridge."""

    def __init__(self, ws_url: str) -> None:
        super().__init__("ImuWsClient", ws_url)


class RtkWsClient(_WsClient):
    """WebSocket client for rtk_bridge."""

    def __init__(self, ws_url: str) -> None:
        super().__init__("RtkWsClient", ws_url)


class RobotWsClient(_WsClient):
    """WebSocket client for robot_bridge (read-only, odometry/SOC data)."""

    def __init__(self, ws_url: str) -> None:
        super().__init__("RobotWsClient", ws_url)


class RecordLoop:
    """
    Asyncio coroutine: sample latest WS client data at N Hz,
    run RecorderPipeline, and push status into the broadcast queue.
    """

    def __init__(
        self,
        imu_client: ImuWsClient,
        rtk_client: RtkWsClient,
        robot_client: RobotWsClient,
        pipeline: RecorderPipeline,
        queue: asyncio.Queue,
        hz: float,
    ) -> None:
        self._imu = imu_client
        self._rtk = rtk_client
        self._robot = robot_client
        self._pipeline = pipeline
        self._queue = queue
        self._period = 1.0 / max(hz, 0.1)

    async def run(self) -> None:
        """Main loop: sample, record, enqueue status."""
        while True:
            imu = self._imu.latest
            rtk = self._rtk.latest
            robot = self._robot.latest

            sources = {
                "imu": self._imu.connected,
                "rtk": self._rtk.connected,
                "robot": self._robot.connected,
            }

            result = self._pipeline.process(imu, rtk, robot, sources)
            try:
                self._queue.put_nowait(result)
            except asyncio.QueueFull:
                logger.debug("RecordLoop: queue full, dropping status frame")

            await asyncio.sleep(self._period)


class WebSocketServer:
    """
    WebSocket server for browser clients.

    Broadcasts RecorderFrame status messages and accepts control commands
    (start_recording, stop_recording, list_files, delete_file).
    """

    def __init__(
        self,
        port: int,
        queue: asyncio.Queue,
        on_client_message=None,
    ) -> None:
        self._port = port
        self._queue = queue
        self._clients: set = set()
        self._on_client_message = on_client_message

    async def broadcast(self) -> None:
        """Continuously dequeue messages and forward to all connected clients."""
        while True:
            message: str = await self._queue.get()
            if not self._clients:
                continue

            dead: set = set()
            for ws in self._clients.copy():
                try:
                    # ── OUTPUT ─────────────────────────────────────────────────────────
                    # Recorder status JSON exits the program here to the browser.
                    # If the browser receives wrong data, start debugging here.
                    await ws.send(message)
                    # ───────────────────────────────────────────────────────────────────
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
                except Exception as exc:
                    logger.warning("WebSocketServer: error sending to client: %s", exc)
                    dead.add(ws)

            self._clients.difference_update(dead)

    async def handle_client(self, websocket) -> None:
        """Handle a single browser WebSocket connection lifecycle."""
        addr = websocket.remote_address
        logger.info("WebSocketServer: client connected: %s", addr)
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                if self._on_client_message is not None:
                    try:
                        self._on_client_message(raw)
                    except Exception as exc:
                        logger.warning(
                            "WebSocketServer: error handling message from %s: %s",
                            addr, exc,
                        )
        except websockets.exceptions.ConnectionClosed:
            logger.debug("WebSocketServer: connection closed normally for %s", addr)
        except Exception as exc:
            logger.warning("WebSocketServer: handler error for %s: %s", addr, exc)
        finally:
            self._clients.discard(websocket)
            logger.info("WebSocketServer: client disconnected: %s", addr)

    async def serve(self) -> None:
        """Start the WebSocket server and the broadcast coroutine."""
        logger.info("WebSocketServer: starting on ws://localhost:%d", self._port)
        asyncio.create_task(self.broadcast())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("WebSocketServer: running.")
            await asyncio.Future()  # run forever


class HttpFileServer:
    """
    Serve static files and CSV data downloads over HTTP.

    Routes:
      /           → web_static/index.html
      /data/*     → CSV files from data_dir
      everything else → web_static/*

    Runs in a dedicated daemon thread.
    """

    def __init__(self, static_dir: Path, data_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._data_dir = data_dir
        self._port = port

    def run(self) -> None:
        """Entry point for the HTTP server thread."""
        static_dir = self._static_dir
        data_dir = self._data_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def do_GET(self):
                if self.path.startswith("/data/"):
                    self._serve_data_file()
                else:
                    super().do_GET()

            def _serve_data_file(self):
                # Extract filename from /data/<filename>
                filename = unquote(self.path[6:])  # strip "/data/"
                filepath = data_dir / filename
                # Security: prevent path traversal
                try:
                    filepath.resolve().relative_to(data_dir.resolve())
                except ValueError:
                    self.send_error(403, "Forbidden")
                    return

                if not filepath.is_file():
                    self.send_error(404, "File not found")
                    return

                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "text/csv")
                    self.send_header(
                        "Content-Disposition",
                        f'attachment; filename="{filepath.name}"',
                    )
                    self.send_header("Content-Length", str(filepath.stat().st_size))
                    self.end_headers()
                    with open(filepath, "rb") as f:
                        self.wfile.write(f.read())
                except OSError as exc:
                    logger.error("HttpFileServer: error serving %s: %s", filename, exc)

            def log_message(self, fmt, *args):
                logger.debug("HTTP %s - %s", self.address_string(), fmt % args)

        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self._port), _Handler) as httpd:
                logger.info(
                    "HttpFileServer: serving static=%s data=%s on port %d",
                    self._static_dir, self._data_dir, self._port,
                )
                httpd.serve_forever()
        except Exception as exc:
            logger.error("HttpFileServer: failed to start on port %d: %s", self._port, exc)


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

class RecorderBridge:
    """
    Top-level orchestrator.  Assembles and starts all components:
      1. ImuWsClient      — reads from imu_bridge WS
      2. RtkWsClient      — reads from rtk_bridge WS
      3. RobotWsClient    — reads from robot_bridge WS
      4. DataRecorder      — CSV writer
      5. RecorderPipeline  — sample + write + status
      6. RecordLoop        — N Hz sampling coroutine
      7. HttpFileServer    — serves web_static + CSV downloads
      8. WebSocketServer   — broadcasts status to browser
    """

    def __init__(
        self,
        ws_port: int,
        imu_ws_url: str,
        rtk_ws_url: str,
        robot_ws_url: str,
        record_hz: float,
        data_dir: Path,
        static_dir: Path,
    ) -> None:
        self._http_port = ws_port
        self._ws_port = ws_port + 1
        self._imu_ws_url = imu_ws_url
        self._rtk_ws_url = rtk_ws_url
        self._robot_ws_url = robot_ws_url
        self._record_hz = record_hz
        self._data_dir = data_dir
        self._static_dir = static_dir
        self._recorder: DataRecorder | None = None

    def run(self) -> None:
        """Synchronous entry point — blocks until Ctrl-C."""
        logger.info(
            "RecorderBridge starting | http=:%d ws=:%d hz=%.1f "
            "imu=%s rtk=%s robot=%s data_dir=%s",
            self._http_port, self._ws_port, self._record_hz,
            self._imu_ws_url, self._rtk_ws_url, self._robot_ws_url,
            self._data_dir,
        )

        # HTTP file server
        if self._static_dir.exists():
            http_server = HttpFileServer(
                self._static_dir, self._data_dir, self._http_port,
            )
            threading.Thread(
                target=http_server.run, daemon=True, name="http-server",
            ).start()
        else:
            logger.warning(
                "web_static directory not found at %s; HTTP server not started.",
                self._static_dir,
            )

        url = f"http://localhost:{self._http_port}"
        logger.info("Open browser at %s", url)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("RecorderBridge: stopped by user.")

    async def _run_async(self) -> None:
        """Start all async components."""
        queue: asyncio.Queue = asyncio.Queue(maxsize=20)

        # Sensor clients
        imu_client = ImuWsClient(self._imu_ws_url)
        rtk_client = RtkWsClient(self._rtk_ws_url)
        robot_client = RobotWsClient(self._robot_ws_url)

        # Recorder + pipeline
        self._recorder = DataRecorder(self._data_dir)
        pipeline = RecorderPipeline(self._recorder)

        # Record loop
        record_loop = RecordLoop(
            imu_client=imu_client,
            rtk_client=rtk_client,
            robot_client=robot_client,
            pipeline=pipeline,
            queue=queue,
            hz=self._record_hz,
        )

        # WebSocket server
        ws_server = WebSocketServer(
            port=self._ws_port,
            queue=queue,
            on_client_message=self._handle_client_message,
        )

        await asyncio.gather(
            imu_client.run(),
            rtk_client.run(),
            robot_client.run(),
            record_loop.run(),
            ws_server.serve(),
        )

    def _handle_client_message(self, raw: str) -> None:
        """Parse and dispatch JSON control messages from the browser."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning(
                "RecorderBridge: invalid JSON from browser: %s | raw: %s",
                exc, raw[:120],
            )
            return

        recorder = self._recorder
        if recorder is None:
            return

        msg_type = msg.get("type", "")

        if msg_type == "start_recording":
            try:
                recorder.start()
            except OSError:
                pass  # already logged inside DataRecorder

        elif msg_type == "stop_recording":
            recorder.stop()

        elif msg_type == "delete_file":
            filename = msg.get("filename", "")
            if filename:
                recorder.delete_file(filename)

        else:
            logger.debug("RecorderBridge: unknown message type: %s", msg_type)


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Multi-source data recorder: IMU + RTK + Robot → CSV"
    )
    parser.add_argument(
        "--ws-port", type=int, default=8825,
        help="HTTP port for web UI; WebSocket uses ws-port+1 (default: 8825)",
    )
    parser.add_argument(
        "--imu-ws", default="ws://localhost:8766",
        help="WebSocket URL for imu_bridge (default: ws://localhost:8766)",
    )
    parser.add_argument(
        "--rtk-ws", default="ws://localhost:8776",
        help="WebSocket URL for rtk_bridge (default: ws://localhost:8776)",
    )
    parser.add_argument(
        "--robot-ws", default="ws://localhost:8796",
        help="WebSocket URL for robot_bridge (default: ws://localhost:8796)",
    )
    parser.add_argument(
        "--record-hz", type=float, default=5.0,
        help="Recording sample rate in Hz (default: 5.0)",
    )
    parser.add_argument(
        "--data-dir", default="./data_log",
        help="Directory for CSV data files (default: ./data_log)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    RecorderBridge(
        ws_port=args.ws_port,
        imu_ws_url=args.imu_ws,
        rtk_ws_url=args.rtk_ws,
        robot_ws_url=args.robot_ws,
        record_hz=args.record_hz,
        data_dir=Path(args.data_dir),
        static_dir=Path(__file__).parent / "web_static",
    ).run()
