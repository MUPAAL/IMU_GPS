"""
imu_bridge.py - BNO085 serial-to-WebSocket bridge (OOP + Pipeline design).

Data flow:
    Serial Port → SerialReader → IMUPipeline → asyncio.Queue → WebSocketServer → Browser
                                      ↑
               (Pipeline: _parse → _enrich_euler → _enrich_hz → _serialize)

Usage:
    python imu_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765
    # Browser: http://localhost:8765
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

import argparse
import asyncio
import json
import logging
import math
import socketserver
import subprocess
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import serial
import websockets


# ── Logger setup ─────────────────────────────────────────────────────────────
def _setup_logger() -> logging.Logger:
    """Configure module-level logger; output to imu_bridge.log and stderr."""
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

@dataclass
class IMUFrame:
    """
    Single IMU data frame flowing through the pipeline.

    Quaternion fields (BNO085 convention: i, j, k, r) are stored as top-level
    attributes.  All other fields forwarded from the ESP32 payload (lin_accel,
    gyro, mag, cal, …) are kept in `extra` for transparent pass-through.
    """

    qi: float
    qj: float
    qk: float
    qr: float
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    heading_raw: float | None = None
    heading: float | None = None
    heading_dir: str | None = None
    hz: float | None = None
    north_offset_deg: float = 0.0              # north correction applied to yaw
    extra: dict = field(default_factory=dict)   # passthrough fields (cal, gyro, …)

    def to_dict(self) -> dict:
        """Serialise frame to the outbound dict shape expected by the browser."""
        payload: dict = {
            "rot": {"qi": self.qi, "qj": self.qj, "qk": self.qk, "qr": self.qr},
        }
        if self.roll is not None:
            payload["euler"] = {
                "roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw,                          # offset already applied
                "north_offset_deg": self.north_offset_deg,
            }
        if self.hz is not None:
            payload["hz"] = self.hz
        if self.heading is not None:
            payload["heading"] = {
                "raw": self.heading_raw,
                "deg": self.heading,
                "dir": self.heading_dir,
            }
        payload.update(self.extra)
        return payload


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE
# ═════════════════════════════════════════════════════════════════════════════

class FrameRateTracker:
    """Rolling-window frame-rate calculator."""

    def __init__(self, window: int = 50) -> None:
        self._window = window
        self._times: list[float] = []

    def tick(self) -> float:
        """Record a new frame timestamp and return the current Hz estimate."""
        now = time.monotonic()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times = self._times[-self._window:]

        if len(self._times) < 2:
            return 0.0

        elapsed = self._times[-1] - self._times[0]
        if elapsed <= 0:
            return 0.0

        return round((len(self._times) - 1) / elapsed, 1)


class IMUPipeline:
    """
    Pipeline Pattern — transforms a raw serial JSON string into an outbound
    JSON string ready for WebSocket broadcast.

        raw JSON str
          → _parse()         → IMUFrame
          → _enrich_euler()  → IMUFrame  (adds roll / pitch / yaw)
          → _enrich_hz()     → IMUFrame  (adds hz)
          → _serialize()     → JSON str
    """

    def __init__(self, hz_tracker: FrameRateTracker, north_offset_deg: float = 0.0) -> None:
        self._hz_tracker = hz_tracker
        self._north_offset_deg: float = north_offset_deg
        self._last_good_heading: float | None = None  # Store last reliable heading

    @property
    def north_offset_deg(self) -> float:
        """Current north offset in degrees applied to yaw."""
        return self._north_offset_deg

    @north_offset_deg.setter
    def north_offset_deg(self, value: float) -> None:
        self._north_offset_deg = value
        logger.info(f"North offset updated to {value:.2f} deg")

    # ── Public entry point ────────────────────────────────────────────────────

    def process(self, raw_line: str) -> str | None:
        """
        CORE — Run the full pipeline on one raw line.

        Receives validated text from SerialReader (INPUT) and returns a
        serialized JSON string ready for WebSocketServer to broadcast (OUTPUT).
        Returns None if the line should be skipped (bad JSON, missing fields, etc.).
        """
        frame = self._parse(raw_line)
        if frame is None:
            return None
        frame = self._enrich_euler(frame)
        frame = self._enrich_heading(frame)
        frame = self._enrich_hz(frame)
        return self._serialize(frame)

    # ── Pipeline stages (private) ─────────────────────────────────────────────

    def _parse(self, line: str) -> IMUFrame | None:
        """Stage 1: decode JSON line → IMUFrame, or None on failure."""
        try:
            data = json.loads(line)
        except json.JSONDecodeError as exc:
            logger.warning(f"JSON parse error: {exc} | raw: {line[:80]}")
            return None

        # Expand compact format (single-letter keys + arrays) from Arduino
        if "r" in data and isinstance(data["r"], list):
            data = self._expand_compact(data)

        rot = data.get("rot")
        if not rot:
            logger.warning("Frame missing 'rot' field, skipping.")
            return None

        # Extract quaternion; keep all other top-level fields in extra
        extra = {k: v for k, v in data.items() if k != "rot"}
        return IMUFrame(
            qi=rot.get("qi", 0.0),
            qj=rot.get("qj", 0.0),
            qk=rot.get("qk", 0.0),
            qr=rot.get("qr", 1.0),
            north_offset_deg=self._north_offset_deg,
            extra=extra,
        )

    @staticmethod
    def _expand_compact(data: dict) -> dict:
        """Expand compact single-letter keys + arrays into full named structure.

        Arduino sends compact JSON to stay under the 256-byte USB CDC-ACM
        buffer limit.  This method restores full field names so the rest of
        the pipeline (IMUFrame, to_dict, frontend JS) needs no changes.

        Compact key mapping:
            t  → ts          (scalar, millis)
            r  → rot         [qi, qj, qk, qr, acc]
            g  → game_rot    [qi, qj, qk, qr]
            a  → accel       [x, y, z]
            l  → lin_accel   [x, y, z]
            v  → gravity     [x, y, z]
            w  → gyro        [x, y, z]
            m  → mag         [x, y, z]
            s  → steps       (scalar)
            c  → cal         (scalar, 0-3)
        """
        out: dict = {}

        if "t" in data:
            out["ts"] = data["t"]

        r = data.get("r")
        if r and len(r) >= 4:
            out["rot"] = {
                "qi": r[0], "qj": r[1], "qk": r[2], "qr": r[3],
                "acc": r[4] if len(r) > 4 else 0.0,
            }

        g = data.get("g")
        if g and len(g) >= 4:
            out["game_rot"] = {"qi": g[0], "qj": g[1], "qk": g[2], "qr": g[3]}

        a = data.get("a")
        if a and len(a) >= 3:
            out["accel"] = {"x": a[0], "y": a[1], "z": a[2]}

        l_ = data.get("l")
        if l_ and len(l_) >= 3:
            out["lin_accel"] = {"x": l_[0], "y": l_[1], "z": l_[2]}

        v = data.get("v")
        if v and len(v) >= 3:
            out["gravity"] = {"x": v[0], "y": v[1], "z": v[2]}

        w = data.get("w")
        if w and len(w) >= 3:
            out["gyro"] = {"x": w[0], "y": w[1], "z": w[2]}

        m = data.get("m")
        if m and len(m) >= 3:
            out["mag"] = {"x": m[0], "y": m[1], "z": m[2]}

        if "s" in data:
            out["steps"] = data["s"]

        if "c" in data:
            out["cal"] = data["c"]

        return out

    def _enrich_euler(self, frame: IMUFrame) -> IMUFrame:
        """Stage 2: compute ZYX Euler angles from quaternion and attach."""
        qi, qj, qk, qr = frame.qi, frame.qj, frame.qk, frame.qr

        sinr_cosp = 2.0 * (qr * qi + qj * qk)
        cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qr * qj - qk * qi)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qr * qk + qi * qj)
        cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        raw_yaw_deg = math.degrees(yaw)
        frame.roll = round(math.degrees(roll), 2)
        frame.pitch = round(math.degrees(pitch), 2)
        frame.yaw = round((raw_yaw_deg + self._north_offset_deg) % 360, 2)
        return frame

    def _enrich_hz(self, frame: IMUFrame) -> IMUFrame:
        """Stage 3: measure and attach frame rate."""
        frame.hz = self._hz_tracker.tick()
        return frame

    @staticmethod
    def _heading_to_direction(heading_deg: float) -> str:
        """Convert heading angle (0-360) to 16-point compass direction."""
        dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        idx = int(round((heading_deg % 360.0) / 22.5)) % 16
        return dirs[idx]

    @staticmethod
    def _quat_mul(
        a: tuple[float, float, float, float],
        b: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        """Quaternion multiply: a * b, components in (x, y, z, w)."""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _normalize_quat(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / norm
        return (x * inv, y * inv, z * inv, w * inv)

    def _enrich_heading(self, frame: IMUFrame) -> IMUFrame:
        """Stage 3: compute web-aligned heading from quaternion + north offset."""
        source = frame.extra.get("game_rot") or {
            "qi": frame.qi,
            "qj": frame.qj,
            "qk": frame.qk,
            "qr": frame.qr,
        }

        q_src = self._normalize_quat((
            float(source.get("qi", 0.0)),
            float(source.get("qj", 0.0)),
            float(source.get("qk", 0.0)),
            float(source.get("qr", 1.0)),
        ))

        # Same frame correction as frontend: BNO085 Z-up -> Three.js Y-up
        frame_correction = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
        qx, qy, qz, qw = self._normalize_quat(self._quat_mul(frame_correction, q_src))

        # Chip X-axis projected on horizontal plane (Three.js XZ)
        chip_x_world_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        chip_x_world_z = 2.0 * (qx * qz - qw * qy)
        raw_heading = (math.degrees(math.atan2(chip_x_world_z, chip_x_world_x)) + 360.0) % 360.0
        corrected_heading = (raw_heading + self._north_offset_deg) % 360.0

        frame.heading_raw = round(raw_heading, 3)
        corrected_heading = (raw_heading + self._north_offset_deg) % 360.0

        # Use fallback if calibration is unreliable
        cal = frame.extra.get("cal", 0)
        if cal == 0 and self._last_good_heading is not None:
            frame.heading = round(self._last_good_heading, 3)
            frame.heading_dir = self._heading_to_direction(self._last_good_heading)
        else:
            frame.heading = round(corrected_heading, 3)
            frame.heading_dir = self._heading_to_direction(corrected_heading)

        # Store last good heading if calibration is medium or high
        if cal >= 2 and frame.heading is not None:
            self._last_good_heading = frame.heading

        return frame

    def _serialize(self, frame: IMUFrame) -> str:
        """Stage 5: convert IMUFrame to outbound JSON string."""
        return json.dumps(frame.to_dict())


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class SerialReader:
    """
    Read JSON lines from a serial port, process them through the pipeline,
    and push the resulting JSON strings into an asyncio.Queue.

    Runs in a dedicated daemon thread (call `run()` via threading.Thread).
    Automatically retries the serial connection every 3 s on failure.
    """

    _RETRY_DELAY_S = 3.0

    def __init__(
        self,
        port: str,
        baud: int,
        pipeline: IMUPipeline,
        loop: asyncio.AbstractEventLoop,
        queue: asyncio.Queue,
        command_queue: asyncio.Queue,
        stop_event: threading.Event | None = None,
    ) -> None:
        self._port = port
        self._baud = baud
        self._pipeline = pipeline
        self._loop = loop
        self._queue = queue
        self._command_queue = command_queue
        self._stop_event = stop_event or threading.Event()

    def run(self) -> None:
        """Entry point for the serial reader thread."""
        logger.info(f"Opening serial port {self._port} at {self._baud} baud.")
        while not self._stop_event.is_set():
            try:
                with serial.Serial(self._port, self._baud, timeout=1.0) as ser:
                    logger.info(f"Serial port {self._port} opened successfully.")
                    self._read_loop(ser)
            except serial.SerialException as exc:
                logger.error(
                    f"Cannot open serial port {self._port}: {exc}. "
                    f"Retrying in {self._RETRY_DELAY_S}s."
                )
            except Exception as exc:
                logger.error(
                    f"Serial thread unexpected error: {exc}. "
                    f"Retrying in {self._RETRY_DELAY_S}s."
                )
            time.sleep(self._RETRY_DELAY_S)

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _read_loop(self, ser: serial.Serial) -> None:
        """Inner read loop; exits on serial error to trigger reconnect."""
        while not self._stop_event.is_set():
            try:
                # Check for outgoing commands to send to IMU
                while not self._command_queue.empty():
                    cmd = self._command_queue.get_nowait()
                    ser.write((cmd + '\n').encode('utf-8'))
                    logger.info(f"Sent command to IMU: {cmd}")

                # ── INPUT ──────────────────────────────────────────────────────────
                # Raw JSON bytes arrive here from the ESP32 over serial.
                # If data looks wrong, start debugging from this line.
                raw = ser.readline()
                # ───────────────────────────────────────────────────────────────────
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if line.startswith("#") or not line.startswith("{"):
                    continue

                result = self._pipeline.process(line)
                if result is None:
                    continue

                # Thread-safe push into the asyncio event loop's queue
                asyncio.run_coroutine_threadsafe(
                    self._queue.put(result), self._loop
                )

            except serial.SerialException as exc:
                logger.error(f"Serial read error: {exc}")
                break
            except Exception as exc:
                logger.error(f"Unexpected error in serial read loop: {exc}")
                break


class WebSocketServer:
    """
    Manage WebSocket client connections and broadcast queue messages.

    Combines the broadcaster coroutine and per-client connection handler into
    a single cohesive class.
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
        self._on_client_message = on_client_message  # callable(str) | None

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
                    # Processed JSON frame exits the program here to the browser.
                    # If the browser receives wrong data, start debugging here.
                    await ws.send(message)
                    # ───────────────────────────────────────────────────────────────────
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
                except Exception as exc:
                    logger.warning(f"Error sending to client: {exc}")
                    dead.add(ws)

            self._clients.difference_update(dead)

    async def handle_client(self, websocket) -> None:
        """Handle a single WebSocket client connection lifecycle."""
        addr = websocket.remote_address
        logger.info(f"WebSocket client connected: {addr}")
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                if self._on_client_message is not None:
                    try:
                        self._on_client_message(raw)
                    except Exception as exc:
                        logger.warning(f"Error handling client message from {addr}: {exc}")
        except websockets.exceptions.ConnectionClosed:
            logger.debug(f"WebSocket connection closed normally for {addr}.")
        except Exception as exc:
            logger.warning(f"WebSocket handler error for {addr}: {exc}")
        finally:
            self._clients.discard(websocket)
            logger.info(f"WebSocket client disconnected: {addr}")

    @staticmethod
    def _is_websocket_upgrade_request(request) -> bool:
        headers = request.headers
        connection = headers.get("Connection", "")
        upgrade = headers.get("Upgrade", "")
        return "upgrade" in connection.lower() and upgrade.lower() == "websocket"

    async def _process_request(self, connection, request):
        if self._is_websocket_upgrade_request(request):
            return None

        http_port = self._port - 1
        body = (
            "<html><head><title>WebSocket endpoint</title></head><body>"
            "<h1>WebSocket endpoint</h1>"
            f"<p>This port only accepts WebSocket connections.</p>"
            f"<p>Open the web UI at <a href=\"http://localhost:{http_port}\">http://localhost:{http_port}</a>.</p>"
            "</body></html>"
        ).encode("utf-8")
        headers = [
            ("Content-Type", "text/html; charset=utf-8"),
            ("Content-Length", str(len(body))),
        ]
        return 400, headers, body

    async def serve(self) -> None:
        """Start the WebSocket server and the broadcast coroutine."""
        logger.info(f"Starting WebSocket server on ws://localhost:{self._port}")
        asyncio.create_task(self.broadcast())
        async with websockets.serve(
            self.handle_client,
            "0.0.0.0",
            self._port,
            process_request=self._process_request,
        ):
            logger.info("WebSocket server running.")
            await asyncio.Future()  # run forever


class HttpFileServer:
    """
    Serve static files from a directory over HTTP.

    Runs in a dedicated daemon thread (call `run()` via threading.Thread).
    """

    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port = port

    def run(self) -> None:
        """Entry point for the HTTP server thread."""
        static_dir = self._static_dir  # capture for closure

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def log_message(self, fmt, *args):  # suppress default stderr noise
                logger.debug(f"HTTP {self.address_string()} - " + fmt % args)

        socketserver.TCPServer.allow_reuse_address = True
        with socketserver.TCPServer(("", self._port), _Handler) as httpd:
            logger.info(
                f"HTTP server serving {self._static_dir} on port {self._port}"
            )
            httpd.serve_forever()


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

class IMUBridge:
    """
    Top-level application object.

    Assembles SerialReader, IMUPipeline, WebSocketServer, and HttpFileServer,
    then starts them all with correct threading / async boundaries.
    """

    def __init__(
        self,
        serial_port: str,
        baud: int,
        ws_port: int,
        static_dir: Path,
        north_offset_deg: float = 0.0,
    ) -> None:
        self._serial_port = serial_port
        self._baud = baud
        self._http_port = ws_port          # HTTP port (user-facing)
        self._ws_port = ws_port + 1        # WebSocket port (browser auto-detects)
        self._static_dir = static_dir
        self._north_offset_deg = north_offset_deg
        self._pipeline: IMUPipeline | None = None  # set in _run_async
        self._command_queue: asyncio.Queue | None = None  # set in _run_async
        self._queue: asyncio.Queue | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._serial_stop_event: threading.Event | None = None
        self._reader_thread: threading.Thread | None = None

    def run(self) -> None:
        """Synchronous entry point — blocks until Ctrl-C."""
        logger.info(
            f"Starting BNO085 bridge | serial={self._serial_port}@{self._baud} "
            f"| http=:{self._http_port} | ws=:{self._ws_port}"
        )
        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Bridge stopped by user.")

    async def _run_async(self) -> None:
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue = asyncio.Queue(maxsize=200)
        command_queue: asyncio.Queue = asyncio.Queue(maxsize=10)

        # -- Pipeline ----------------------------------------------------------
        hz_tracker = FrameRateTracker(window=50)
        self._pipeline = IMUPipeline(hz_tracker, north_offset_deg=self._north_offset_deg)
        self._queue = queue
        self._command_queue = command_queue
        self._loop = loop

        # -- Serial reader thread ----------------------------------------------
        self._start_serial_reader()

        # -- HTTP static file server thread ------------------------------------
        if not self._static_dir.exists():
            logger.warning(
                f"web_static directory not found at {self._static_dir}; "
                "HTTP server not started."
            )
        else:
            http_server = HttpFileServer(self._static_dir, self._http_port)
            threading.Thread(
                target=http_server.run, daemon=True, name="http-server"
            ).start()

        # -- Open browser after short delay ------------------------------------
        url = f"http://localhost:{self._http_port}"
        logger.info(f"Open browser at {url}")
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        # -- WebSocket server (runs forever in event loop) ---------------------
        ws_server = WebSocketServer(
            port=self._ws_port,
            queue=queue,
            on_client_message=self._handle_client_message,
        )
        await ws_server.serve()

    def _start_serial_reader(self) -> None:
        if self._loop is None or self._queue is None or self._command_queue is None or self._pipeline is None:
            logger.error("Cannot start serial reader; bridge not fully initialized.")
            return
        self._serial_stop_event = threading.Event()
        reader = SerialReader(
            port=self._serial_port,
            baud=self._baud,
            pipeline=self._pipeline,
            loop=self._loop,
            queue=self._queue,
            command_queue=self._command_queue,
            stop_event=self._serial_stop_event,
        )
        self._reader_thread = threading.Thread(
            target=reader.run,
            daemon=True,
            name="serial-reader",
        )
        self._reader_thread.start()
        logger.info("Serial reader thread started.")

    def _stop_serial_reader(self, timeout: float = 5.0) -> None:
        if self._serial_stop_event is None or self._reader_thread is None:
            return
        self._serial_stop_event.set()
        logger.info("Stopping serial reader thread for firmware flash...")
        self._reader_thread.join(timeout)
        if self._reader_thread.is_alive():
            logger.warning("Serial reader thread did not stop in time.")
        else:
            logger.info("Serial reader thread stopped.")
        self._reader_thread = None
        self._serial_stop_event = None

    def send_command_to_imu(self, cmd: str) -> None:
        """Enqueue a command to send to the IMU over serial."""
        if self._command_queue is not None:
            asyncio.create_task(self._command_queue.put(cmd))
            logger.info(f"Enqueued command to IMU: {cmd}")

    def _flash_firmware(self) -> None:
        script_path = Path(__file__).resolve().parent / "flash_imu_firmware.py"
        if not script_path.exists():
            logger.error(f"Firmware flash script not found: {script_path}")
            return

        self._stop_serial_reader()
        cmd = [sys.executable, str(script_path), "--port", self._serial_port]
        logger.info(f"Starting firmware flash: {' '.join(cmd)}")
        try:
            subprocess.run(cmd, cwd=script_path.parent, check=True)
            logger.info("Firmware flash completed successfully.")
        except subprocess.CalledProcessError as exc:
            logger.error(f"Firmware flash failed: {exc}")
        except FileNotFoundError as exc:
            logger.error(f"Firmware flash helper not found: {exc}")
        finally:
            self._start_serial_reader()

    def _handle_client_message(self, raw: str) -> None:
        """Parse and apply JSON control messages sent from the browser."""
        try:
            # ── INPUT ──────────────────────────────────────────────────────────
            # north_offset_deg control messages arrive here from the browser.
            # If the north offset control doesn't work, start debugging here.
            msg = json.loads(raw)
            # ───────────────────────────────────────────────────────────────────
        except json.JSONDecodeError as exc:
            logger.warning(f"Invalid JSON from browser client: {exc} | raw: {raw[:80]}")
            return
        if "set_north_offset" in msg:
            if self._pipeline is None:
                logger.warning("Pipeline not ready; ignoring set_north_offset.")
                return
            try:
                north_offset_deg = float(msg["set_north_offset"])
            except (ValueError, TypeError) as exc:
                logger.warning(f"Invalid set_north_offset value: {exc}")
                return
            self._pipeline.north_offset_deg = north_offset_deg
            self.send_command_to_imu(json.dumps({
                "cmd": "set_heading",
                "heading_deg": north_offset_deg,
            }))
            if msg.get("flash_after_heading"):
                threading.Thread(target=self._flash_firmware, daemon=True).start()
        elif "save_cal" in msg:
            self.send_command_to_imu('{"cmd":"save_cal"}')
        elif "reset_cal" in msg:
            self.send_command_to_imu('{"cmd":"reset_cal"}')


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="BNO085 serial-to-WebSocket bridge"
    )
    parser.add_argument(
        "--port",
        default=_cfg.IMU_SERIAL_PORT if _cfg else "/dev/cu.usbmodem101",
        help="Serial port device",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=_cfg.IMU_BAUD if _cfg else 921600,
        help="Serial baud rate",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=_cfg.IMU_WS_PORT if _cfg else 8765,
        help="HTTP port for web UI; WebSocket uses ws-port+1",
    )
    parser.add_argument(
        "--north-offset",
        type=float,
        default=_cfg.IMU_NORTH_OFFSET if _cfg else 0.0,
        help="Initial north offset in degrees applied to yaw",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    static_dir = Path(__file__).parent / "web_static"
    IMUBridge(
        serial_port=args.port,
        baud=args.baud,
        ws_port=args.ws_port,
        static_dir=static_dir,
        north_offset_deg=args.north_offset,
    ).run()
