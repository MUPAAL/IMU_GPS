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
from enum import Enum

import serial
import websockets

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import config
from heading_controller import HeadingController
from sensor_threads import IMUReader, RTKReader
from navigation.waypoint import WaypointManager, Waypoint
from navigation.geo_utils import bearing_to_target, haversine_distance, normalize_angle

logger = logging.getLogger(__name__)


class NavState(Enum):
    """Navigation state machine."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    FINISHED = "finished"

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
    """Main web controller: serial I/O, WebSocket, heading control, autonomous navigation."""

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

        # Navigation state machine
        self._nav_state = NavState.IDLE
        self._nav_mode = "p2p"  # "p2p" or "pure_pursuit"
        self._waypoint_manager = WaypointManager()
        self._heading_offset_deg = 0.0
        self._speed_ratio = 0.5  # speed scaling from frontend (0.0-1.0)
        self._nav_lock = threading.Lock()  # protect nav state
        self._last_gps_check_time = time.time()

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
        linear = max(-config.PATHFOLLOWER_MAX_LINEAR_VEL, min(config.PATHFOLLOWER_MAX_LINEAR_VEL, linear))
        angular = max(-config.PATHFOLLOWER_MAX_ANGULAR_VEL, min(config.PATHFOLLOWER_MAX_ANGULAR_VEL, angular))

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
            # Apply speed ratio scaling
            speed_ratio = msg.get("speed_ratio", self._speed_ratio)
            if isinstance(speed_ratio, (int, float)):
                self._speed_ratio = max(0.0, min(1.0, float(speed_ratio)))
            linear *= self._speed_ratio
            angular *= self._speed_ratio
            self._mode = "joystick"
            self._send_velocity(linear, angular)

        elif msg_type == "set_heading":
            self._last_heartbeat = time.time()
            heading = float(msg.get("heading_deg", 0.0))
            self._heading_controller.set_mode("p2p")
            self._heading_controller.set_target_heading(heading)
            self._mode = "heading_follow"
            logger.info(f"Target heading set to {heading:.1f}°")

        elif msg_type == "upload_waypoints":
            self._handle_upload_waypoints(msg)

        elif msg_type == "nav_mode":
            self._handle_nav_mode(msg)

        elif msg_type == "nav_start":
            self._handle_nav_start(msg, force=False)

        elif msg_type == "nav_start_force":
            self._handle_nav_start(msg, force=True)

        elif msg_type == "nav_stop":
            self._handle_nav_stop()

        elif msg_type == "set_heading_offset":
            self._handle_set_heading_offset(msg)

    def _handle_upload_waypoints(self, msg: dict) -> None:
        """Handle waypoint upload from CSV."""
        try:
            csv_content = msg.get("csv", "")
            count = self._waypoint_manager.load_csv(csv_content)
            error = None
            logger.info(f"Waypoints loaded: {count}")
        except Exception as e:
            count = 0
            error = str(e)
            logger.error(f"Waypoint upload error: {e}")

        # Load waypoints into heading controller for Pure Pursuit mode
        if count > 0:
            waypoints = self._waypoint_manager.waypoints
            self._heading_controller.load_waypoints(waypoints)

        # Broadcast result to UI
        asyncio.create_task(self._broadcast({
            "type": "waypoints_loaded",
            "count": count,
            "error": error,
        }))

    def _handle_nav_mode(self, msg: dict) -> None:
        """Handle navigation mode selection."""
        mode = msg.get("mode", "p2p")
        if mode not in ("p2p", "pure_pursuit"):
            logger.warning(f"Invalid nav mode: {mode}")
            return

        with self._nav_lock:
            self._nav_mode = mode
        self._heading_controller.set_mode(mode)
        logger.info(f"Navigation mode set to {mode}")

    def _handle_nav_start(self, msg: dict, force: bool = False) -> None:
        """Handle navigation start."""
        with self._nav_lock:
            if self._nav_state == NavState.NAVIGATING:
                logger.warning("Navigation already in progress")
                return

        # Check GPS quality unless forced
        rtk_frame = self._rtk_reader.snapshot() if self._rtk_reader else None
        if not force and rtk_frame:
            if rtk_frame.fix_quality < 4 or rtk_frame.num_sats < 6:
                logger.warning(
                    f"GPS quality insufficient for navigation: "
                    f"fix_quality={rtk_frame.fix_quality}, sats={rtk_frame.num_sats}"
                )
                asyncio.create_task(self._broadcast({
                    "type": "nav_warning",
                    "message": f"GPS quality poor: {rtk_frame.fix_quality}, sats: {rtk_frame.num_sats}",
                }))
                return

        with self._nav_lock:
            self._nav_state = NavState.NAVIGATING
            self._waypoint_manager.reset()

        self._mode = "navigation"
        logger.info(f"Navigation started (forced={force})")
        asyncio.create_task(self._broadcast({
            "type": "nav_started",
            "mode": self._nav_mode,
        }))

    def _handle_nav_stop(self) -> None:
        """Handle navigation stop."""
        with self._nav_lock:
            self._nav_state = NavState.IDLE
        self._send_velocity(0.0, 0.0)  # Stop motors immediately
        self._mode = "joystick"
        logger.info("Navigation stopped")
        asyncio.create_task(self._broadcast({
            "type": "nav_stopped",
        }))

    def _handle_set_heading_offset(self, msg: dict) -> None:
        """Handle heading offset setting."""
        heading_offset = float(msg.get("heading_deg", 0.0))
        with self._nav_lock:
            self._heading_offset_deg = normalize_angle(heading_offset)
        self._heading_controller.set_heading_offset(self._heading_offset_deg)
        logger.info(f"Heading offset set to {self._heading_offset_deg:.1f}°")

    async def _control_loop(self) -> None:
        """Main control loop (20 Hz): read sensors, compute heading control, send velocity, manage navigation."""
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
                angular = self._heading_controller.compute_p2p(imu_frame.heading, dt)
                with _vel_lock:
                    linear = _last_linear
                self._send_velocity(linear, angular)

            # Navigation mode (autonomous path following)
            elif self._mode == "navigation":
                with self._nav_lock:
                    nav_state = self._nav_state
                    nav_mode = self._nav_mode

                if nav_state == NavState.IDLE:
                    self._send_velocity(0.0, 0.0)
                elif nav_state == NavState.NAVIGATING and imu_frame and rtk_frame:
                    self._control_navigation(imu_frame, rtk_frame, dt)
                elif nav_state == NavState.FINISHED:
                    self._send_velocity(0.0, 0.0)

            # In joystick mode, velocity is already sent via _handle_client_message

    def _control_navigation(self, imu_frame, rtk_frame, dt: float) -> None:
        """Execute navigation control based on current mode."""
        with self._nav_lock:
            nav_mode = self._nav_mode
            heading_offset = self._heading_offset_deg

        if nav_mode == "p2p":
            # Backward compatibility: point-to-point mode
            angular = self._heading_controller.compute_p2p(imu_frame.heading, dt)
            with _vel_lock:
                linear = _last_linear * self._speed_ratio
            self._send_velocity(linear, angular)

        elif nav_mode == "pure_pursuit":
            # Pure Pursuit with waypoint tracking
            current_wp = self._waypoint_manager.current
            if current_wp is None:
                # No more waypoints
                with self._nav_lock:
                    self._nav_state = NavState.FINISHED
                self._send_velocity(0.0, 0.0)
                logger.info("Navigation complete - all waypoints visited")
                asyncio.create_task(self._broadcast({
                    "type": "nav_complete",
                }))
                return

            # Compute Pure Pursuit command
            linear, angular = self._heading_controller.compute_pure_pursuit(
                rtk_frame.lat,
                rtk_frame.lon,
                imu_frame.heading,
                dt,
            )

            # Apply speed ratio scaling
            linear *= self._speed_ratio
            angular *= self._speed_ratio

            self._send_velocity(linear, angular)

            # Check for waypoint arrival
            distance_to_wp = haversine_distance(
                rtk_frame.lat, rtk_frame.lon,
                current_wp.lat, current_wp.lon
            )

            arrived = self._waypoint_manager.update(
                distance_to_wp,
                rtk_frame.fix_quality
            )

            if arrived:
                logger.info(
                    f"Waypoint {current_wp.id} reached. "
                    f"Distance: {distance_to_wp:.2f}m"
                )
                asyncio.create_task(self._broadcast({
                    "type": "waypoint_reached",
                    "waypoint_id": current_wp.id,
                    "distance_m": distance_to_wp,
                }))

                # Check if all waypoints visited
                if self._waypoint_manager.is_finished:
                    with self._nav_lock:
                        self._nav_state = NavState.FINISHED
                    self._send_velocity(0.0, 0.0)
                    logger.info("Navigation complete - all waypoints visited")
                    asyncio.create_task(self._broadcast({
                        "type": "nav_complete",
                    }))

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
            target_heading = (
                self._heading_controller.target_heading
                if self._mode == "heading_follow"
                else None
            )

            # Navigation status
            with self._nav_lock:
                nav_state = self._nav_state.value
                nav_mode = self._nav_mode
                heading_offset = self._heading_offset_deg

            nav_status = None
            nav_progress = None
            distance_to_wp = None
            target_bearing = None
            current_wp_lat = None
            current_wp_lon = None

            if self._mode == "navigation":
                nav_status = nav_state
                current_wp = self._waypoint_manager.current
                progress_idx, progress_total = self._waypoint_manager.progress
                nav_progress = [progress_idx, progress_total]

                if current_wp:
                    rtk_frame = self._rtk_reader.snapshot() if self._rtk_reader else None
                    if rtk_frame:
                        distance_to_wp = haversine_distance(
                            rtk_frame.lat, rtk_frame.lon,
                            current_wp.lat, current_wp.lon
                        )
                        target_bearing = bearing_to_target(
                            rtk_frame.lat, rtk_frame.lon,
                            current_wp.lat, current_wp.lon
                        )
                        # Apply heading offset to bearing
                        target_bearing = normalize_angle(target_bearing + heading_offset)

                    current_wp_lat = current_wp.lat
                    current_wp_lon = current_wp.lon

            obj = {
                "type": "status",
                "watchdog": self._watchdog_active,
                "mode": self._mode,
                "linear_vel": lin,
                "angular_vel": ang,
                "heading_error": heading_error,
                "target_heading": target_heading,
                "nav_state": nav_status,
                "nav_mode": nav_mode,
                "nav_progress": nav_progress,
                "distance_to_wp": distance_to_wp,
                "target_bearing": target_bearing,
                "current_wp_lat": current_wp_lat,
                "current_wp_lon": current_wp_lon,
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
