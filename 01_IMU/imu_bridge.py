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

import argparse
import asyncio
import json
import logging
import math
import socketserver
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
    hz: float | None = None
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
                "yaw": self.yaw,
            }
        if self.hz is not None:
            payload["hz"] = self.hz
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

    def __init__(self, hz_tracker: FrameRateTracker) -> None:
        self._hz_tracker = hz_tracker

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
            extra=extra,
        )

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

        frame.roll = round(math.degrees(roll), 2)
        frame.pitch = round(math.degrees(pitch), 2)
        frame.yaw = round(math.degrees(yaw), 2)
        return frame

    def _enrich_hz(self, frame: IMUFrame) -> IMUFrame:
        """Stage 3: measure and attach frame rate."""
        frame.hz = self._hz_tracker.tick()
        return frame

    def _serialize(self, frame: IMUFrame) -> str:
        """Stage 4: convert IMUFrame to outbound JSON string."""
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
    ) -> None:
        self._port = port
        self._baud = baud
        self._pipeline = pipeline
        self._loop = loop
        self._queue = queue

    def run(self) -> None:
        """Entry point for the serial reader thread."""
        logger.info(f"Opening serial port {self._port} at {self._baud} baud.")
        while True:
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
        while True:
            try:
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

    def __init__(self, port: int, queue: asyncio.Queue) -> None:
        self._port = port
        self._queue = queue
        self._clients: set = set()

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
            await websocket.wait_closed()
        except Exception as exc:
            logger.warning(f"WebSocket handler error for {addr}: {exc}")
        finally:
            self._clients.discard(websocket)
            logger.info(f"WebSocket client disconnected: {addr}")

    async def serve(self) -> None:
        """Start the WebSocket server and the broadcast coroutine."""
        logger.info(f"Starting WebSocket server on ws://localhost:{self._port}")
        asyncio.create_task(self.broadcast())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
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
    ) -> None:
        self._serial_port = serial_port
        self._baud = baud
        self._http_port = ws_port          # HTTP port (user-facing)
        self._ws_port = ws_port + 1        # WebSocket port (browser auto-detects)
        self._static_dir = static_dir

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

        # -- Pipeline ----------------------------------------------------------
        hz_tracker = FrameRateTracker(window=50)
        pipeline = IMUPipeline(hz_tracker)

        # -- Serial reader thread ----------------------------------------------
        reader = SerialReader(
            port=self._serial_port,
            baud=self._baud,
            pipeline=pipeline,
            loop=loop,
            queue=queue,
        )
        threading.Thread(
            target=reader.run, daemon=True, name="serial-reader"
        ).start()

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
        ws_server = WebSocketServer(port=self._ws_port, queue=queue)
        await ws_server.serve()


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="BNO085 serial-to-WebSocket bridge"
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port device (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=921600,
        help="Serial baud rate (default: 921600)",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8765,
        help="HTTP port for web UI; WebSocket uses ws-port+1 (default: 8765)",
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
    ).run()
