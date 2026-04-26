"""
autonav_bridge.py — Autonomous navigation: Pure Pursuit + PID path tracking.

Data flow:
    imu_bridge(WS:8766)  ──→ ImuWsClient.latest ─┐
                                                   ├─ AutoNavLoop(10Hz) → AutoNavController → NavCommand
    rtk_bridge(WS:8776)  ──→ RtkWsClient.latest ─┘        │                                      │
                                                            │                                      ↓
    path.csv ───────────────────────────────────────────────┘              RobotWsClient → robot_bridge(WS:8889)
                                                                                                   │
                                                                     AutoNavWsServer(WS:8806) ◄────┘
                                                                     (status broadcast for debugging)

Control commands (via ws://localhost:8806):
    {"type": "start"}   — begin navigation
    {"type": "stop"}    — stop and hold
    {"type": "pause"}   — pause in place
    {"type": "resume"}  — resume from paused

Usage:
    python autonav_bridge.py
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

import asyncio
import collections
import csv
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
from typing import Literal

import websockets

# ── Configuration ─────────────────────────────────────────────────────────────

HTTP_PORT           = _cfg.AUTONAV_WS_PORT          if _cfg else 8805
WS_PORT             = HTTP_PORT + 1                  # 8806
IMU_WS_URL          = _cfg.AUTONAV_IMU_WS            if _cfg else "ws://localhost:8766"
RTK_WS_URL          = _cfg.AUTONAV_RTK_WS            if _cfg else "ws://localhost:8776"
# NOTE: config.py has an incorrect port 8796; 04_Robot actually listens on 8889.
ROBOT_WS_URL        = "ws://localhost:8889"

MAX_LINEAR_VEL      = _cfg.AUTONAV_MAX_LINEAR_VEL    if _cfg else 0.5   # m/s
MAX_ANGULAR_VEL     = _cfg.AUTONAV_MAX_ANGULAR_VEL   if _cfg else 0.8   # rad/s
MIN_LINEAR_VEL      = 0.10                                               # m/s, minimum crawl speed

PID_KP              = _cfg.AUTONAV_PID_KP             if _cfg else 0.8
PID_KI              = _cfg.AUTONAV_PID_KI             if _cfg else 0.01
PID_KD              = _cfg.AUTONAV_PID_KD             if _cfg else 0.05
LOOKAHEAD_M         = _cfg.AUTONAV_LOOKAHEAD_M        if _cfg else 2.0
DECEL_RADIUS_M      = _cfg.AUTONAV_DECEL_RADIUS_M     if _cfg else 3.0
ARRIVE_FRAMES       = _cfg.AUTONAV_ARRIVE_FRAMES      if _cfg else 5
GPS_TIMEOUT_S       = _cfg.AUTONAV_GPS_TIMEOUT_S      if _cfg else 5.0
MA_WINDOW           = _cfg.AUTONAV_MA_WINDOW          if _cfg else 10
REACH_TOLERANCE_M   = 1.5                             # waypoint arrival radius (m)

CONTROL_HZ          = 10.0                            # navigation loop frequency
HEARTBEAT_INTERVAL  = 1.0                             # seconds between idle heartbeats

# angular sign: +1 = positive error → positive angular (robot turns left/CCW).
# If robot turns the wrong way, set to -1 and restart.
ANGULAR_SIGN        = 1

PATH_FILE           = Path(__file__).parent / "path.csv"


# ── Logger setup ──────────────────────────────────────────────────────────────

def _setup_logger() -> logging.Logger:
    py_name = Path(__file__).stem
    log_file = Path(__file__).parent / f"{py_name}.log"
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
class Waypoint:
    id: int
    lat: float
    lng: float
    tolerance_m: float = REACH_TOLERANCE_M
    max_speed: float = MAX_LINEAR_VEL


@dataclass
class SensorSnapshot:
    heading_deg: float | None    # 0~360, true north (from IMU)
    lat: float | None            # decimal degrees
    lon: float | None            # decimal degrees
    imu_ts: float                # time.monotonic() of last IMU frame
    gps_ts: float                # time.monotonic() of last GPS frame


@dataclass
class NavCommand:
    linear: float                # m/s
    angular: float               # rad/s
    state: str                   # "idle" | "running" | "arrived" | "paused"
    current_wp_idx: int
    total_wp: int
    dist_to_wp_m: float | None
    target_bearing_deg: float | None
    bearing_error_deg: float | None
    heading_deg: float | None
    gps_age_s: float
    imu_age_s: float


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE (CORE)
# ═════════════════════════════════════════════════════════════════════════════

class GeoMath:
    """Static geodesic helpers."""

    EARTH_RADIUS_M = 6_371_000

    @staticmethod
    def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Great-circle distance in metres between two WGS-84 points."""
        r = math.radians
        d_lat = r(lat2 - lat1)
        d_lon = r(lon2 - lon1)
        a = (
            math.sin(d_lat / 2) ** 2
            + math.cos(r(lat1)) * math.cos(r(lat2)) * math.sin(d_lon / 2) ** 2
        )
        return 2 * GeoMath.EARTH_RADIUS_M * math.asin(math.sqrt(a))

    @staticmethod
    def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Initial bearing from point 1 to point 2, in degrees [0, 360)."""
        r = math.radians
        d_lon = r(lon2 - lon1)
        x = math.sin(d_lon) * math.cos(r(lat2))
        y = (
            math.cos(r(lat1)) * math.sin(r(lat2))
            - math.sin(r(lat1)) * math.cos(r(lat2)) * math.cos(d_lon)
        )
        return (math.degrees(math.atan2(x, y)) + 360) % 360

    @staticmethod
    def normalize_angle(deg: float) -> float:
        """Normalize an angle to (-180, 180]."""
        deg = deg % 360
        if deg > 180:
            deg -= 360
        return deg


class PIDController:
    """Discrete PID with anti-windup integral clamping."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float,
        output_max: float,
    ) -> None:
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._out_min = output_min
        self._out_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0
        # Anti-windup: clamp integral contribution to output range
        self._integral_limit = (output_max - output_min) / (2.0 * max(ki, 1e-9))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3
        p = self._kp * error
        self._integral += error * dt
        self._integral = max(-self._integral_limit,
                             min(self._integral_limit, self._integral))
        i = self._ki * self._integral
        d = self._kd * (error - self._prev_error) / dt
        self._prev_error = error
        raw = p + i + d
        return max(self._out_min, min(self._out_max, raw))


class MovingAverage:
    """Simple window-based moving average."""

    def __init__(self, window: int) -> None:
        self._buf: collections.deque = collections.deque(maxlen=max(window, 1))

    def update(self, value: float) -> float:
        self._buf.append(value)
        return sum(self._buf) / len(self._buf)

    def reset(self) -> None:
        self._buf.clear()


class PathLoader:
    """Load waypoints from path.csv.

    Expected columns (tab or comma separated):
        id    lat    lon    tolerance_m    max_speed
    """

    @staticmethod
    def load(path: Path) -> list[Waypoint]:
        waypoints: list[Waypoint] = []
        with open(path, newline="", encoding="utf-8") as f:
            # Auto-detect delimiter (tab or comma)
            sample = f.read(4096)
            f.seek(0)
            dialect = csv.Sniffer().sniff(sample, delimiters="\t,")
            reader = csv.DictReader(f, dialect=dialect)
            for row in reader:
                waypoints.append(Waypoint(
                    id=int(row["id"]),
                    lat=float(row["lat"]),
                    lng=float(row["lon"]),
                    tolerance_m=float(row.get("tolerance_m", REACH_TOLERANCE_M)),
                    max_speed=float(row.get("max_speed", MAX_LINEAR_VEL)),
                ))
        logger.info("PathLoader: loaded %d waypoints from %s", len(waypoints), path)
        return waypoints


class PurePursuitPlanner:
    """
    Simplified Pure Pursuit: advance the current waypoint index when the robot
    arrives within REACH_TOLERANCE_M, then select the first waypoint at least
    LOOKAHEAD_M ahead as the steering target.
    """

    def __init__(
        self,
        waypoints: list[Waypoint],
        lookahead_m: float,
        arrive_frames: int,
    ) -> None:
        self._waypoints = waypoints
        self._lookahead_m = lookahead_m
        self._arrive_frames = arrive_frames
        self._current_idx = 0
        self._arrive_counter = 0

    def reset(self) -> None:
        self._current_idx = 0
        self._arrive_counter = 0

    @property
    def current_idx(self) -> int:
        return self._current_idx

    def update(self, lat: float, lon: float) -> tuple[int, Waypoint | None]:
        """
        Return (current_idx, target_waypoint).
        target_waypoint is None when all waypoints have been reached.
        """
        wps = self._waypoints

        # Advance past already-reached waypoints
        while self._current_idx < len(wps):
            wp = wps[self._current_idx]
            dist = GeoMath.haversine(lat, lon, wp.lat, wp.lng)
            if dist < wp.tolerance_m:
                self._arrive_counter += 1
                if self._arrive_counter >= self._arrive_frames:
                    logger.info(
                        "PurePursuit: waypoint %d/%d reached (%.2f m)",
                        self._current_idx + 1, len(wps), dist,
                    )
                    self._current_idx += 1
                    self._arrive_counter = 0
                break
            else:
                self._arrive_counter = 0
                break

        if self._current_idx >= len(wps):
            return self._current_idx, None  # all done

        # Find lookahead target: first wp at distance >= lookahead_m from current pos
        target = wps[-1]  # fallback: last waypoint
        for wp in wps[self._current_idx:]:
            d = GeoMath.haversine(lat, lon, wp.lat, wp.lng)
            if d >= self._lookahead_m:
                target = wp
                break

        return self._current_idx, target


NavState = Literal["idle", "running", "arrived", "paused"]


class AutoNavController:
    """
    Navigation state machine + velocity command computation.

    State transitions:
      idle    ─start()─►  running  ─arrived─►  arrived
              ◄─stop()─              ─timeout─► paused ─resume()─► running
    """

    def __init__(
        self,
        waypoints: list[Waypoint],
        pid: PIDController,
        ma: MovingAverage,
        planner: PurePursuitPlanner,
        max_linear: float,
        max_angular: float,
        decel_radius_m: float,
        gps_timeout_s: float,
    ) -> None:
        self._waypoints = waypoints
        self._pid = pid
        self._ma = ma
        self._planner = planner
        self._max_linear = max_linear
        self._max_angular = max_angular
        self._decel_radius_m = decel_radius_m
        self._gps_timeout_s = gps_timeout_s
        self._state: NavState = "idle"
        self._prev_ts: float = time.monotonic()
        self._paused_by_timeout = False

    # ── State control ─────────────────────────────────────────────────────

    def start(self) -> None:
        if not self._waypoints:
            logger.warning("AutoNavController: start() called with empty waypoint list")
            return
        self._planner.reset()
        self._pid.reset()
        self._ma.reset()
        self._state = "running"
        self._paused_by_timeout = False
        logger.info("AutoNavController: started navigation (%d waypoints)", len(self._waypoints))

    def stop(self) -> None:
        self._state = "idle"
        self._pid.reset()
        self._ma.reset()
        logger.info("AutoNavController: stopped")

    def pause(self) -> None:
        if self._state == "running":
            self._state = "paused"
            self._paused_by_timeout = False
            logger.info("AutoNavController: paused by command")

    def resume(self) -> None:
        if self._state == "paused":
            self._state = "running"
            self._paused_by_timeout = False
            self._pid.reset()
            logger.info("AutoNavController: resumed")

    @property
    def state(self) -> NavState:
        return self._state

    # ── Main compute ──────────────────────────────────────────────────────

    def compute(self, snapshot: SensorSnapshot) -> NavCommand:
        """
        Run one control tick. Returns a NavCommand with linear/angular velocities.
        """
        now = time.monotonic()
        dt = now - self._prev_ts
        self._prev_ts = now

        gps_age = now - snapshot.gps_ts
        imu_age = now - snapshot.imu_ts

        # Safety: timeout handling
        if self._state == "running":
            timed_out = (
                snapshot.lat is None
                or snapshot.lon is None
                or snapshot.heading_deg is None
                or gps_age > self._gps_timeout_s
                or imu_age > self._gps_timeout_s
            )
            if timed_out:
                self._state = "paused"
                self._paused_by_timeout = True
                logger.warning(
                    "AutoNavController: sensor timeout — GPS age=%.1fs IMU age=%.1fs",
                    gps_age, imu_age,
                )
                return self._zero_cmd("paused", gps_age, imu_age)

        if self._state == "paused" and self._paused_by_timeout:
            # Auto-resume when sensors recover
            if (
                snapshot.lat is not None
                and snapshot.lon is not None
                and snapshot.heading_deg is not None
                and gps_age < self._gps_timeout_s
                and imu_age < self._gps_timeout_s
            ):
                self._state = "running"
                self._paused_by_timeout = False
                self._pid.reset()
                logger.info("AutoNavController: sensors recovered — auto-resuming")

        if self._state != "running":
            return self._zero_cmd(self._state, gps_age, imu_age)

        lat = snapshot.lat
        lon = snapshot.lon
        heading = snapshot.heading_deg

        # Pure Pursuit: get current idx and steering target
        wp_idx, target_wp = self._planner.update(lat, lon)

        if target_wp is None:
            self._state = "arrived"
            logger.info("AutoNavController: arrived at destination")
            return self._zero_cmd("arrived", gps_age, imu_age)

        # Distance to current (immediate) waypoint
        cur_wp = self._waypoints[min(wp_idx, len(self._waypoints) - 1)]
        dist_to_wp = GeoMath.haversine(lat, lon, cur_wp.lat, cur_wp.lng)

        # Bearing to lookahead target
        target_bearing = GeoMath.bearing(lat, lon, target_wp.lat, target_wp.lng)

        # Bearing error in (-180, 180]
        error = GeoMath.normalize_angle(target_bearing - heading)

        # PID → angular velocity
        raw_angular = self._pid.compute(error * ANGULAR_SIGN, dt)
        angular = self._ma.update(raw_angular)
        angular = max(-self._max_angular, min(self._max_angular, angular))

        # Linear velocity: use current waypoint's max_speed, decelerate near final
        wp_max_speed = self._waypoints[min(wp_idx, len(self._waypoints) - 1)].max_speed
        dist_to_final = GeoMath.haversine(
            lat, lon, self._waypoints[-1].lat, self._waypoints[-1].lng
        )
        if dist_to_final < self._decel_radius_m:
            linear = wp_max_speed * (dist_to_final / self._decel_radius_m)
            linear = max(MIN_LINEAR_VEL, linear)
        else:
            linear = wp_max_speed

        return NavCommand(
            linear=round(linear, 3),
            angular=round(angular, 3),
            state="running",
            current_wp_idx=wp_idx,
            total_wp=len(self._waypoints),
            dist_to_wp_m=round(dist_to_wp, 2),
            target_bearing_deg=round(target_bearing, 1),
            bearing_error_deg=round(error, 1),
            heading_deg=round(heading, 1),
            gps_age_s=round(gps_age, 2),
            imu_age_s=round(imu_age, 2),
        )

    def _zero_cmd(self, state: str, gps_age: float, imu_age: float) -> NavCommand:
        return NavCommand(
            linear=0.0,
            angular=0.0,
            state=state,
            current_wp_idx=self._planner.current_idx,
            total_wp=len(self._waypoints),
            dist_to_wp_m=None,
            target_bearing_deg=None,
            bearing_error_deg=None,
            heading_deg=None,
            gps_age_s=round(gps_age, 2),
            imu_age_s=round(imu_age, 2),
        )


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class ImuWsClient:
    """WebSocket client for imu_bridge; stores latest frame thread-safely."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._last_ts: float = 0.0

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def last_ts(self) -> float:
        with self._lock:
            return self._last_ts

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("ImuWsClient: connected to %s", self._url)
                    # ── INPUT boundary ─────────────────────────────────────────
                    # IMU JSON frames enter autonav_bridge here.
                    # Debug IMU data issues by inspecting messages at this point.
                    async for msg in ws:
                    # ──────────────────────────────────────────────────────────
                        try:
                            data = json.loads(msg)
                            with self._lock:
                                self._latest = data
                                self._last_ts = time.monotonic()
                        except json.JSONDecodeError as exc:
                            logger.warning("ImuWsClient: JSON error: %s", exc)
            except Exception as exc:
                logger.warning(
                    "ImuWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S
                )
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class RtkWsClient:
    """WebSocket client for rtk_bridge; stores latest frame thread-safely."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._last_ts: float = 0.0

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def last_ts(self) -> float:
        with self._lock:
            return self._last_ts

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("RtkWsClient: connected to %s", self._url)
                    # ── INPUT boundary ─────────────────────────────────────────
                    # RTK JSON frames enter autonav_bridge here.
                    async for msg in ws:
                    # ──────────────────────────────────────────────────────────
                        try:
                            data = json.loads(msg)
                            with self._lock:
                                self._latest = data
                                self._last_ts = time.monotonic()
                        except json.JSONDecodeError as exc:
                            logger.warning("RtkWsClient: JSON error: %s", exc)
            except Exception as exc:
                logger.warning(
                    "RtkWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S
                )
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class RobotWsClient:
    """
    WebSocket client that sends joystick commands to robot_bridge.

    Uses an asyncio.Queue to decouple command production from network I/O.
    Sends heartbeat (zero-velocity joystick) when state is idle/paused
    to prevent robot_bridge watchdog timeout.
    """

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=5)
        self._last_sent_ts: float = 0.0

    async def send_command(self, linear: float, angular: float, force: float = 1.0) -> None:
        """Enqueue a joystick command (non-blocking; drops if queue full)."""
        msg = json.dumps({"type": "joystick", "linear": linear,
                          "angular": angular, "force": force})
        try:
            self._queue.put_nowait(msg)
        except asyncio.QueueFull:
            # Control loop is faster than network; drop oldest
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(msg)
            except asyncio.QueueEmpty:
                pass

    async def run(self) -> None:
        """Maintain WS connection and consume send queue."""
        asyncio.create_task(self._heartbeat_loop())
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("RobotWsClient: connected to %s", self._url)
                    while True:
                        msg: str = await self._queue.get()
                        # ── OUTPUT boundary ────────────────────────────────────
                        # Joystick commands exit autonav_bridge here to robot_bridge.
                        # Debug wrong robot motion by inspecting messages here.
                        await ws.send(msg)
                        # ──────────────────────────────────────────────────────
                        self._last_sent_ts = time.monotonic()
            except Exception as exc:
                logger.warning(
                    "RobotWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S
                )
                await asyncio.sleep(self.RECONNECT_DELAY_S)

    async def _heartbeat_loop(self) -> None:
        """Send zero-velocity heartbeat to prevent robot watchdog timeout."""
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            age = time.monotonic() - self._last_sent_ts
            if age >= HEARTBEAT_INTERVAL:
                await self.send_command(0.0, 0.0)


class AutoNavWsServer:
    """
    WebSocket server that broadcasts autonav_status to debug clients
    and accepts control commands (start/stop/pause/resume).
    """

    def __init__(self, port: int, queue: asyncio.Queue, on_client_message) -> None:
        self._port = port
        self._queue = queue
        self._clients: set = set()
        self._on_client_message = on_client_message

    async def broadcast(self) -> None:
        while True:
            message: str = await self._queue.get()
            if not self._clients:
                continue
            dead: set = set()
            for ws in self._clients.copy():
                try:
                    await ws.send(message)
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
                except Exception as exc:
                    logger.warning("AutoNavWsServer: send error: %s", exc)
                    dead.add(ws)
            self._clients.difference_update(dead)

    async def handle_client(self, websocket) -> None:
        addr = websocket.remote_address
        logger.info("AutoNavWsServer: client connected: %s", addr)
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                if self._on_client_message is not None:
                    try:
                        await self._on_client_message(raw)
                    except Exception as exc:
                        logger.warning("AutoNavWsServer: message error: %s", exc)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as exc:
            logger.warning("AutoNavWsServer: handler error: %s", exc)
        finally:
            self._clients.discard(websocket)
            logger.info("AutoNavWsServer: client disconnected: %s", addr)

    async def serve(self) -> None:
        logger.info("AutoNavWsServer: starting on ws://localhost:%d", self._port)
        asyncio.create_task(self.broadcast())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("AutoNavWsServer: running.")
            await asyncio.Future()  # run forever


class HttpFileServer:
    """Serve static files from a directory over HTTP in a daemon thread."""

    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port = port

    def run(self) -> None:
        static_dir = self._static_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def log_message(self, fmt, *args):
                logger.debug("HTTP %s - %s", self.address_string(), fmt % args)

        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self._port), _Handler) as httpd:
                logger.info("HttpFileServer: serving on port %d", self._port)
                httpd.serve_forever()
        except Exception as exc:
            logger.error("HttpFileServer: failed on port %d: %s", self._port, exc)


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

class AutoNavLoop:
    """
    10 Hz control loop: reads sensor clients, calls AutoNavController,
    sends commands to robot, and enqueues status for broadcast.
    """

    def __init__(
        self,
        imu_client: ImuWsClient,
        rtk_client: RtkWsClient,
        robot_client: RobotWsClient,
        controller: AutoNavController,
        status_queue: asyncio.Queue,
    ) -> None:
        self._imu = imu_client
        self._rtk = rtk_client
        self._robot = robot_client
        self._ctrl = controller
        self._queue = status_queue

    async def run(self) -> None:
        period = 1.0 / CONTROL_HZ
        while True:
            imu = self._imu.latest
            rtk = self._rtk.latest

            # Build sensor snapshot
            heading_deg: float | None = None
            heading_block = imu.get("heading", {})
            if heading_block.get("deg") is not None:
                heading_deg = float(heading_block["deg"])

            lat: float | None = rtk.get("lat")
            lon: float | None = rtk.get("lon")
            if lat is not None:
                lat = float(lat)
            if lon is not None:
                lon = float(lon)

            imu_ts = self._imu.last_ts or time.monotonic()
            gps_ts = self._rtk.last_ts or time.monotonic()

            snapshot = SensorSnapshot(
                heading_deg=heading_deg,
                lat=lat,
                lon=lon,
                imu_ts=imu_ts,
                gps_ts=gps_ts,
            )

            cmd = self._ctrl.compute(snapshot)

            # Send to robot
            await self._robot.send_command(cmd.linear, cmd.angular)

            # Enqueue status broadcast
            status = {
                "type": "autonav_status",
                "version": 1,
                "state": cmd.state,
                "current_wp_idx": cmd.current_wp_idx,
                "total_wp": cmd.total_wp,
                "dist_to_wp_m": cmd.dist_to_wp_m,
                "target_bearing_deg": cmd.target_bearing_deg,
                "heading_deg": cmd.heading_deg,
                "bearing_error_deg": cmd.bearing_error_deg,
                "linear": cmd.linear,
                "angular": cmd.angular,
                "gps_age_s": cmd.gps_age_s,
                "imu_age_s": cmd.imu_age_s,
            }
            try:
                self._queue.put_nowait(json.dumps(status))
            except asyncio.QueueFull:
                pass

            await asyncio.sleep(period)


class AutoNavBridge:
    """
    Top-level orchestrator. Assembles and starts all components.
    """

    def __init__(self) -> None:
        self._controller: AutoNavController | None = None

    def run(self) -> None:
        logger.info(
            "AutoNavBridge starting | http=:%d  ws=:%d  imu=%s  rtk=%s  robot=%s",
            HTTP_PORT, WS_PORT, IMU_WS_URL, RTK_WS_URL, ROBOT_WS_URL,
        )

        static_dir = Path(__file__).parent / "web_static"
        if static_dir.exists():
            http_server = HttpFileServer(static_dir, HTTP_PORT)
            threading.Thread(target=http_server.run, daemon=True, name="http").start()
        else:
            logger.info("No web_static directory — HTTP server skipped.")

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("AutoNavBridge: stopped by user.")

    async def _run_async(self) -> None:
        # Load path
        waypoints = PathLoader.load(PATH_FILE)
        if not waypoints:
            logger.error("No waypoints loaded from %s — exiting.", PATH_FILE)
            return

        # Build pipeline
        pid = PIDController(PID_KP, PID_KI, PID_KD, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        ma = MovingAverage(MA_WINDOW)
        planner = PurePursuitPlanner(waypoints, LOOKAHEAD_M, ARRIVE_FRAMES)
        self._controller = AutoNavController(
            waypoints=waypoints,
            pid=pid,
            ma=ma,
            planner=planner,
            max_linear=MAX_LINEAR_VEL,
            max_angular=MAX_ANGULAR_VEL,
            decel_radius_m=DECEL_RADIUS_M,
            gps_timeout_s=GPS_TIMEOUT_S,
        )

        # I/O clients
        imu_client = ImuWsClient(IMU_WS_URL)
        rtk_client = RtkWsClient(RTK_WS_URL)
        robot_client = RobotWsClient(ROBOT_WS_URL)

        status_queue: asyncio.Queue = asyncio.Queue(maxsize=20)

        nav_loop = AutoNavLoop(
            imu_client=imu_client,
            rtk_client=rtk_client,
            robot_client=robot_client,
            controller=self._controller,
            status_queue=status_queue,
        )

        ws_server = AutoNavWsServer(
            port=WS_PORT,
            queue=status_queue,
            on_client_message=self._handle_client_message,
        )

        logger.info("AutoNavBridge: loaded %d waypoints. Send {\"type\":\"start\"} to begin.", len(waypoints))

        await asyncio.gather(
            imu_client.run(),
            rtk_client.run(),
            robot_client.run(),
            nav_loop.run(),
            ws_server.serve(),
        )

    async def _handle_client_message(self, raw: str) -> None:
        """Dispatch control commands from debug WS clients."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning("AutoNavBridge: invalid JSON: %s", exc)
            return

        if self._controller is None:
            return

        msg_type = msg.get("type", "")
        if msg_type == "start":
            self._controller.start()
        elif msg_type == "stop":
            self._controller.stop()
        elif msg_type == "pause":
            self._controller.pause()
        elif msg_type == "resume":
            self._controller.resume()
        else:
            logger.warning("AutoNavBridge: unknown message type: %s", msg_type)


if __name__ == "__main__":
    AutoNavBridge().run()
