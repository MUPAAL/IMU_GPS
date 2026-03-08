"""
robot_bridge.py - Amiga robot serial-to-WebSocket bridge (OOP + Pipeline design).

Data flow:
    Serial Port → SerialReader → RobotPipeline → asyncio.Queue → WebSocketServer → Browser
                                      ↑
               (Pipeline: _parse → _enrich_state → _enrich_hz → _enrich_odometry → _serialize)

    Browser → WebSocketServer → _handle_client_message → SerialReader.send_command → Serial Port

Usage:
    python robot_bridge.py --port /dev/ttyACM0 --baud 115200 --ws-port 8795
    # Browser: http://localhost:8795
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
    """Configure module-level logger; output to robot_bridge.log and stderr."""
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

AMIGA_STATE_NAMES = {
    0: "BOOT",
    1: "MANUAL_READY",
    2: "MANUAL_ACTIVE",
    3: "CC_ACTIVE",
    4: "AUTO_READY",
    5: "AUTO_ACTIVE",
    6: "ESTOPPED",
}


@dataclass
class RobotFrame:
    """
    Single robot telemetry frame flowing through the pipeline.

    Raw fields from O: line, enriched fields from pipeline stages,
    odometry from integration, and command echo from latest sent command.
    """

    # --- Raw from O: line ---
    meas_speed: float = 0.0
    meas_ang_rate: float = 0.0
    state: int = 0
    soc: int = 0

    # --- Enriched ---
    state_name: str = "BOOT"
    hz: float | None = None
    firmware_status: str = "UNKNOWN"
    uptime_s: float = 0.0

    # --- Odometry (integrated) ---
    total_distance_m: float = 0.0
    heading_est_deg: float = 0.0

    # --- Command echo (latest sent) ---
    cmd_speed: float = 0.0
    cmd_ang_rate: float = 0.0

    def to_dict(self) -> dict:
        """Flat dict of all fields for JSON serialization."""
        return {
            "meas_speed": round(self.meas_speed, 3),
            "meas_ang_rate": round(self.meas_ang_rate, 3),
            "state": self.state,
            "soc": self.soc,
            "state_name": self.state_name,
            "hz": self.hz,
            "firmware_status": self.firmware_status,
            "uptime_s": round(self.uptime_s, 1),
            "total_distance_m": round(self.total_distance_m, 2),
            "heading_est_deg": round(self.heading_est_deg, 1),
            "cmd_speed": round(self.cmd_speed, 3),
            "cmd_ang_rate": round(self.cmd_ang_rate, 3),
        }


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


class RobotPipeline:
    """
    Pipeline Pattern — transforms raw serial lines into outbound JSON.

    Holds accumulated RobotFrame state (like NMEAPipeline in rtk_bridge).

        raw serial line
          → _parse()            → update _frame fields (O: or S:)
          → _enrich_state()     → map state int → state_name
          → _enrich_hz()        → FrameRateTracker.tick()
          → _enrich_odometry()  → distance += |speed|*dt, heading += ang_rate*dt
          → _serialize()        → json.dumps(frame.to_dict())
    """

    def __init__(self, hz_tracker: FrameRateTracker) -> None:
        self._hz_tracker = hz_tracker
        self._frame = RobotFrame()
        self._start_time = time.monotonic()
        self._last_tick_time = time.monotonic()

    # ── Public entry point ────────────────────────────────────────────────────

    def process(self, raw_line: str) -> str | None:
        """
        CORE — Run the full pipeline on one raw line.

        Receives validated text from SerialReader (INPUT) and returns a
        serialized JSON string ready for WebSocketServer to broadcast (OUTPUT).
        Returns None for S: lines (status updates only) or unparseable lines.
        """
        line_type = self._parse(raw_line)
        if line_type is None:
            return None
        if line_type == "S":
            # S: lines update firmware_status but don't produce output
            return None

        self._enrich_state()
        self._enrich_hz()
        self._enrich_odometry()
        return self._serialize()

    def set_cmd(self, speed: float, ang_rate: float) -> None:
        """Update command echo from WS command callback."""
        self._frame.cmd_speed = speed
        self._frame.cmd_ang_rate = ang_rate

    # ── Pipeline stages (private) ─────────────────────────────────────────────

    def _parse(self, line: str) -> str | None:
        """Stage 1: parse O: or S: line and update _frame fields."""
        if line.startswith("O:"):
            # Format: O:{speed},{ang_rate},{state},{soc}
            try:
                parts = line[2:].split(",")
                if len(parts) >= 4:
                    self._frame.meas_speed = float(parts[0])
                    self._frame.meas_ang_rate = float(parts[1])
                    self._frame.state = int(parts[2])
                    self._frame.soc = int(parts[3])
                    return "O"
            except (ValueError, IndexError) as exc:
                logger.warning(f"Failed to parse O: line: {exc} | raw: {line[:80]}")
                return None
        elif line.startswith("S:"):
            # Format: S:READY or S:ACTIVE
            status = line[2:].strip()
            self._frame.firmware_status = status
            logger.info(f"Firmware status update: {status}")
            return "S"

        return None

    def _enrich_state(self) -> None:
        """Stage 2: map state int to human-readable name."""
        self._frame.state_name = AMIGA_STATE_NAMES.get(
            self._frame.state, f"UNKNOWN({self._frame.state})"
        )

    def _enrich_hz(self) -> None:
        """Stage 3: measure and attach frame rate."""
        self._frame.hz = self._hz_tracker.tick()

    def _enrich_odometry(self) -> None:
        """Stage 4: integrate speed and angular rate for odometry."""
        now = time.monotonic()
        dt = now - self._last_tick_time
        self._last_tick_time = now

        # Uptime
        self._frame.uptime_s = now - self._start_time

        # Clamp dt to avoid jumps on reconnect
        if dt > 1.0:
            return

        # Distance: integrate |speed| * dt
        self._frame.total_distance_m += abs(self._frame.meas_speed) * dt

        # Heading: integrate angular rate (rad/s → deg)
        heading_delta_deg = self._frame.meas_ang_rate * dt * (180.0 / math.pi)
        self._frame.heading_est_deg = (
            self._frame.heading_est_deg + heading_delta_deg
        ) % 360.0

    def _serialize(self) -> str:
        """Stage 5: convert RobotFrame to outbound JSON string."""
        return json.dumps(self._frame.to_dict())


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class SerialReader:
    """
    Read lines from a serial port, process O:/S: lines through the pipeline,
    and push results into an asyncio.Queue.

    Also provides thread-safe send_command() for writing to serial (WASD/V).
    Runs in a dedicated daemon thread (call run() via threading.Thread).
    Automatically retries the serial connection every 3s on failure.
    """

    _RETRY_DELAY_S = 3.0

    def __init__(
        self,
        port: str,
        baud: int,
        pipeline: RobotPipeline,
        loop: asyncio.AbstractEventLoop,
        queue: asyncio.Queue,
    ) -> None:
        self._port = port
        self._baud = baud
        self._pipeline = pipeline
        self._loop = loop
        self._queue = queue
        self._ser: serial.Serial | None = None
        self._write_lock = threading.Lock()

    def send_command(self, cmd: str) -> None:
        """Thread-safe write to serial port."""
        with self._write_lock:
            if self._ser is not None and self._ser.is_open:
                try:
                    self._ser.write(cmd.encode("ascii"))
                    self._ser.flush()
                except serial.SerialException as exc:
                    logger.error(f"Serial write error: {exc}")
            else:
                logger.warning("Serial port not open; command dropped.")

    def run(self) -> None:
        """Entry point for the serial reader thread."""
        logger.info(f"Opening serial port {self._port} at {self._baud} baud.")
        while True:
            try:
                with serial.Serial(self._port, self._baud, timeout=1.0) as ser:
                    self._ser = ser
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
            finally:
                self._ser = None
            time.sleep(self._RETRY_DELAY_S)

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _read_loop(self, ser: serial.Serial) -> None:
        """Inner read loop; exits on serial error to trigger reconnect."""
        while True:
            try:
                # ── INPUT ──────────────────────────────────────────────────────────
                # Raw bytes arrive here from the CIRCUITPY device over USB serial.
                # If data looks wrong, start debugging from this line.
                raw = ser.readline()
                # ───────────────────────────────────────────────────────────────────
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line.startswith(("O:", "S:")):
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

    Runs in a dedicated daemon thread (call run() via threading.Thread).
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

class RobotBridge:
    """
    Top-level application object.

    Assembles SerialReader, RobotPipeline, WebSocketServer, and HttpFileServer,
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
        self._http_port = ws_port
        self._ws_port = ws_port + 1
        self._static_dir = static_dir
        self._pipeline: RobotPipeline | None = None
        self._serial_reader: SerialReader | None = None

    def run(self) -> None:
        """Synchronous entry point — blocks until Ctrl-C."""
        logger.info(
            f"Starting Amiga Robot bridge | serial={self._serial_port}@{self._baud} "
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
        self._pipeline = RobotPipeline(hz_tracker)

        # -- Serial reader thread ----------------------------------------------
        self._serial_reader = SerialReader(
            port=self._serial_port,
            baud=self._baud,
            pipeline=self._pipeline,
            loop=loop,
            queue=queue,
        )
        threading.Thread(
            target=self._serial_reader.run, daemon=True, name="serial-reader"
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
        ws_server = WebSocketServer(
            port=self._ws_port,
            queue=queue,
            on_client_message=self._handle_client_message,
        )
        await ws_server.serve()

    def _handle_client_message(self, raw: str) -> None:
        """Parse and apply JSON control messages sent from the browser."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning(f"Invalid JSON from browser client: {exc} | raw: {raw[:80]}")
            return

        reader = self._serial_reader

        # WASD single-character command
        if "wasd" in msg:
            char = str(msg["wasd"])
            if char in ("w", "a", "s", "d", " ", "\r"):
                logger.debug(f"WASD command: {repr(char)}")
                if reader:
                    reader.send_command(char)

        # Direct velocity command
        elif "velocity" in msg:
            vel = msg["velocity"]
            try:
                speed = max(-1.0, min(1.0, float(vel.get("speed", 0.0))))
                ang_rate = max(-1.0, min(1.0, float(vel.get("ang_rate", 0.0))))
                cmd = f"V{speed:.3f},{ang_rate:.3f}\n"
                logger.debug(f"Velocity command: {cmd.strip()}")
                if reader:
                    reader.send_command(cmd)
                if self._pipeline:
                    self._pipeline.set_cmd(speed, ang_rate)
            except (ValueError, TypeError) as exc:
                logger.warning(f"Invalid velocity command: {exc}")

        # E-Stop
        elif msg.get("estop"):
            logger.info("E-STOP triggered from browser!")
            if reader:
                reader.send_command(" ")  # space = stop
            if self._pipeline:
                self._pipeline.set_cmd(0.0, 0.0)


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Amiga robot serial-to-WebSocket bridge"
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM1",
        help="Serial port device (default: /dev/ttyACM1)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8795,
        help="HTTP port for web UI; WebSocket uses ws-port+1 (default: 8795)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    static_dir = Path(__file__).parent / "web_static"
    RobotBridge(
        serial_port=args.port,
        baud=args.baud,
        ws_port=args.ws_port,
        static_dir=static_dir,
    ).run()
