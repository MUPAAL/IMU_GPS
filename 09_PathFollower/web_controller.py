"""
web_controller.py — HTTP + WebSocket server for path follower web UI.

Architecture:
  - Asyncio event loop (main thread) for HTTP server, WebSocket server, control loops
  - IMUReader daemon thread → sensor snapshot
  - RTKReader daemon thread → sensor snapshot
  - Serial writer for Feather M4 velocity commands

WebSocket protocol (JSON):
  Client → Server:
    {"type": "heartbeat"}                          # keep-alive
    {"type": "joystick", "linear": 0.5, "angular": 0.2}
    {"type": "set_heading", "heading_deg": 45.0}

  Server → Client:
    {"type": "imu", "heading": 45.0, "roll": 10.0, ...}
    {"type": "rtk", "lat": 38.94, "lon": -92.31, ...}
    {"type": "status", "watchdog": true, "mode": "joystick", ...}
"""

import sys
import json
import logging
import asyncio
import threading
import time
import socketserver
from pathlib import Path
from http.server import SimpleHTTPRequestHandler
from dataclasses import asdict

import serial
import websockets

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config
from heading_controller import HeadingController
from sensor_threads import IMUReader, RTKReader

logger = logging.getLogger(__name__)

# Global state (protected by locks)
_last_linear = 0.0
_last_angular = 0.0
_vel_lock = threading.Lock()


class StaticFileHandler(SimpleHTTPRequestHandler):
    """HTTP handler for static files."""

    def do_GET(self):
        if self.path == "/":
            self.path = "/index.html"

        try:
            static_dir = Path(__file__).parent / "web_static"
            file_path = static_dir / self.path.lstrip("/")

            if not file_path.exists():
                self.send_error(404)
                return

            with open(file_path, "rb") as f:
                content = f.read()

            content_type = (
                "text/html"
                if file_path.suffix == ".html"
                else "text/css"
                if file_path.suffix == ".css"
                else "application/javascript"
                if file_path.suffix == ".js"
                else "application/octet-stream"
            )

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        except Exception as e:
            logger.error(f"HTTP: {e}")
            self.send_error(500)

    def log_message(self, fmt, *args):
        logger.debug(f"HTTP: {fmt % args}")


def _start_http_server() -> None:
    """Start HTTP server in a daemon thread."""
    server = socketserver.TCPServer(
        ("0.0.0.0", config.PATHFOLLOWER_WS_PORT),
        StaticFileHandler,
    )
    server.allow_reuse_address = True
    t = threading.Thread(target=server.serve_forever, name="HTTPServer", daemon=True)
    t.start()
    logger.info(
        f"HTTP server started: http://0.0.0.0:{config.PATHFOLLOWER_WS_PORT}/"
    )


class PathFollowerController:
    """Main web controller: serial I/O, WebSocket, heading control."""

    def __init__(self):
        self._ser: serial.Serial | None = None
        self._ser_lock = threading.Lock()
        self._clients: set = set()
        self._clients_lock = asyncio.Lock()

        self._imu_reader: IMUReader | None = None
        self._rtk_reader: RTKReader | None = None
        self._heading_controller = HeadingController()

        self._last_heartbeat = time.time()
        self._watchdog_active = False
        self._mode = "joystick"  # "joystick" or "heading_follow"

    def start_sensors(self) -> None:
        """Start IMU and RTK reader threads."""
        logger.info("Starting sensor reader threads...")
        self._imu_reader = IMUReader()
        self._imu_reader.start()

        self._rtk_reader = RTKReader()
        self._rtk_reader.start()

        logger.info("Sensor threads started")

    def open_serial(self) -> None:
        """Open serial connection to Feather M4."""
        try:
            self._ser = serial.Serial(
                config.ROBOT_SERIAL_PORT,
                config.ROBOT_SERIAL_BAUD,
                timeout=config.ROBOT_SERIAL_TIMEOUT,
            )
            logger.info(
                f"Serial port opened: {config.ROBOT_SERIAL_PORT} @ {config.ROBOT_SERIAL_BAUD} baud"
            )
        except serial.SerialException as e:
            logger.error(
                f"Failed to open serial port [{config.ROBOT_SERIAL_PORT}]: {e}"
            )

    def close_serial(self) -> None:
        """Close serial connection."""
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
                logger.info("Serial port closed")

    def _send_velocity(self, linear: float, angular: float) -> None:
        """Send velocity command to Feather M4."""
        global _last_linear, _last_angular

        # Clamp to limits
        linear = max(-config.PATHFOLLOWER_MAX_LINEAR_VEL,
                     min(config.PATHFOLLOWER_MAX_LINEAR_VEL, linear))
        angular = max(-config.PATHFOLLOWER_MAX_ANGULAR_VEL,
                      min(config.PATHFOLLOWER_MAX_ANGULAR_VEL, angular))

        cmd = f"V{linear:.2f},{angular:.2f}\n".encode()
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                logger.warning("Serial port not open, cannot send velocity")
                return
            try:
                self._ser.write(cmd)
                logger.debug(f"Velocity: {linear:.2f} m/s, {angular:.2f} rad/s")
            except serial.SerialException as e:
                logger.error(f"Serial write failed: {e}")

        with _vel_lock:
            _last_linear = linear
            _last_angular = angular

    async def _broadcast(self, obj: dict) -> None:
        """Broadcast JSON to all connected WebSocket clients."""
        msg = json.dumps(obj)
        async with self._clients_lock:
            clients = set(self._clients)

        dead = set()
        for ws in clients:
            try:
                await ws.send(msg)
            except websockets.exceptions.ConnectionClosed:
                dead.add(ws)
            except Exception as e:
                logger.warning(f"Broadcast error: {e}")
                dead.add(ws)

        async with self._clients_lock:
            self._clients.difference_update(dead)

    async def _handle_client(self, websocket) -> None:
        """Handle WebSocket client connection."""
        addr = websocket.remote_address
        logger.info(f"WebSocket client connected: {addr}")
        async with self._clients_lock:
            self._clients.add(websocket)

        try:
            async for raw in websocket:
                self._handle_client_message(raw)
        except websockets.exceptions.ConnectionClosed:
            logger.debug(f"WebSocket disconnected: {addr}")
        except Exception as e:
            logger.warning(f"WebSocket error: {e}")
        finally:
            async with self._clients_lock:
                self._clients.discard(websocket)
            logger.info(f"WebSocket client gone: {addr}")

    def _handle_client_message(self, raw: str) -> None:
        """Parse and process JSON messages from browser."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return

        msg_type = msg.get("type")

        if msg_type == "heartbeat":
            self._last_heartbeat = time.time()
            self._watchdog_active = False

        elif msg_type == "joystick":
            self._last_heartbeat = time.time()
            linear = float(msg.get("linear", 0.0))
            angular = float(msg.get("angular", 0.0))
            self._mode = "joystick"
            self._send_velocity(linear, angular)

        elif msg_type == "set_heading":
            self._last_heartbeat = time.time()
            heading = float(msg.get("heading_deg", 0.0))
            self._heading_controller.set_target_heading(heading)
            self._mode = "heading_follow"
            logger.info(f"Target heading set to {heading:.1f}°")

    async def _control_loop(self) -> None:
        """Main control loop (20 Hz): read sensors, compute heading control, send velocity."""
        dt = 0.05  # 20 Hz
        next_time = time.time()

        while True:
            now = time.time()
            if now < next_time:
                await asyncio.sleep(next_time - now)
                now = time.time()

            next_time = now + dt

            # Check watchdog
            if now - self._last_heartbeat > config.ROBOT_WATCHDOG_TIMEOUT:
                if not self._watchdog_active:
                    logger.warning("Watchdog timeout: stopping motors")
                    self._watchdog_active = True
                self._send_velocity(0.0, 0.0)
                continue

            # Get sensor data
            imu_frame = self._imu_reader.snapshot() if self._imu_reader else None
            rtk_frame = self._rtk_reader.snapshot() if self._rtk_reader else None

            # Heading control mode
            if self._mode == "heading_follow" and imu_frame:
                angular = self._heading_controller.compute(imu_frame.heading, dt)
                with _vel_lock:
                    linear = _last_linear
                self._send_velocity(linear, angular)

            # In joystick mode, velocity is already sent via _handle_client_message

    async def _broadcast_loop_imu(self) -> None:
        """Broadcast IMU data at 20 Hz."""
        dt = 0.05  # 20 Hz
        next_time = time.time()

        while True:
            now = time.time()
            if now < next_time:
                await asyncio.sleep(next_time - now)
            next_time = now + dt

            frame = self._imu_reader.snapshot() if self._imu_reader else None
            if frame:
                obj = {
                    "type": "imu",
                    "heading": frame.heading,
                    "roll": frame.roll,
                    "pitch": frame.pitch,
                    "yaw": frame.yaw,
                    "hz": frame.hz,
                }
                await self._broadcast(obj)

    async def _broadcast_loop_rtk(self) -> None:
        """Broadcast RTK data at 1 Hz."""
        dt = 1.0  # 1 Hz
        next_time = time.time()

        while True:
            now = time.time()
            if now < next_time:
                await asyncio.sleep(next_time - now)
            next_time = now + dt

            frame = self._rtk_reader.snapshot() if self._rtk_reader else None
            if frame:
                obj = {
                    "type": "rtk",
                    "lat": frame.lat,
                    "lon": frame.lon,
                    "alt": frame.alt,
                    "num_sats": frame.num_sats,
                    "fix_quality": frame.fix_quality,
                }
                await self._broadcast(obj)

    async def _broadcast_loop_status(self) -> None:
        """Broadcast system status at 2 Hz."""
        dt = 0.5  # 2 Hz
        next_time = time.time()

        while True:
            now = time.time()
            if now < next_time:
                await asyncio.sleep(next_time - now)
            next_time = now + dt

            with _vel_lock:
                lin, ang = _last_linear, _last_angular

            heading_error = (
                self._heading_controller.heading_error
                if self._mode == "heading_follow"
                else None
            )

            obj = {
                "type": "status",
                "watchdog": self._watchdog_active,
                "mode": self._mode,
                "linear_vel": lin,
                "angular_vel": ang,
                "heading_error": heading_error,
            }
            await self._broadcast(obj)

    async def _run_ws_server(self) -> None:
        """Run WebSocket server."""
        logger.info(
            f"WebSocket server starting on ws://0.0.0.0:{config.PATHFOLLOWER_WS_PORT + 1}"
        )
        async with websockets.serve(
            self._handle_client, "0.0.0.0", config.PATHFOLLOWER_WS_PORT + 1
        ):
            logger.info("WebSocket server running")
            await asyncio.Future()  # run forever

    async def run_async(self) -> None:
        """Run all async tasks concurrently."""
        await asyncio.gather(
            self._run_ws_server(),
            self._control_loop(),
            self._broadcast_loop_imu(),
            self._broadcast_loop_rtk(),
            self._broadcast_loop_status(),
        )


async def main():
    """Main entry point."""
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Create controller
    controller = PathFollowerController()

    # Start HTTP server
    _start_http_server()

    # Start sensor readers
    controller.start_sensors()

    # Open serial port
    controller.open_serial()

    # Run async event loop
    try:
        await controller.run_async()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        controller.close_serial()
        if controller._imu_reader:
            controller._imu_reader.stop()
        if controller._rtk_reader:
            controller._rtk_reader.stop()


if __name__ == "__main__":
    asyncio.run(main())
