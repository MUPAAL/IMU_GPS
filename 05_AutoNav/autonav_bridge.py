"""
autonav_bridge.py — Autonomous navigation I/O bridge.

Data flow:
    imu_bridge(WS:8766)  ──→ ImuWsClient ─┐
                                           ├─ AutoNavLoop → algo.compute() → NavCommand
    rtk_bridge(WS:8776)  ──→ RtkWsClient ─┘       │                              │
                                                    │                              ↓
    path.csv ──────────────────────────────────────┘         RobotWsClient → robot_bridge(WS:8889)
                                                                                   │
                                                          AutoNavWsServer(WS:8806)◄┘

Control commands (via ws://localhost:8806):
    {"type": "start"}   — begin navigation
    {"type": "stop"}    — stop and hold
    {"type": "pause"}   — pause in place
    {"type": "resume"}  — resume from paused

Algorithm is in autonav_algo.py — edit that file to change steering behaviour.

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
import csv
import json
import logging
import socketserver
import threading
import time
import webbrowser
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import websockets

import autonav_algo as algo

# ── Configuration ─────────────────────────────────────────────────────────────

def _c(attr, default):
    return getattr(_cfg, attr, default) if _cfg else default

HTTP_PORT          = _c("AUTONAV_WS_PORT",           8805)
WS_PORT            = HTTP_PORT + 1                        # 8806
IMU_WS_URL         = _c("AUTONAV_IMU_WS",           "ws://localhost:8766")
RTK_WS_URL         = _c("AUTONAV_RTK_WS",           "ws://localhost:8776")
ROBOT_WS_URL       = "ws://localhost:8889"
GPS_TIMEOUT_S      = _c("AUTONAV_GPS_TIMEOUT_S",     5.0)
CONTROL_HZ         = _c("AUTONAV_CONTROL_HZ",        5.0)
MANUAL_SPEED       = _c("AUTONAV_MANUAL_SPEED",      0.4)
ARRIVE_FRAMES      = _c("AUTONAV_ARRIVE_FRAMES",     1)
HEARTBEAT_INTERVAL = 1.0

PATH_FILE = Path(__file__).parent / "path.csv"

# ── Logger setup ──────────────────────────────────────────────────────────────

def _setup_logger() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[logging.StreamHandler()],
    )
    return logging.getLogger(__name__)

logger = _setup_logger()


# ═════════════════════════════════════════════════════════════════════════════
# WAYPOINT LOADER
# ═════════════════════════════════════════════════════════════════════════════

def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle bearing from point 1 → point 2, in degrees [0, 360)."""
    import math
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _load_waypoints(path: Path) -> list[dict]:
    """Load path.csv → list of {lat, lon} dicts."""
    waypoints = []
    with open(path, newline="", encoding="utf-8") as f:
        sample = f.read(4096)
        f.seek(0)
        dialect = csv.Sniffer().sniff(sample, delimiters="\t,")
        for row in csv.DictReader(f, dialect=dialect):
            waypoints.append({"lat": float(row["lat"]), "lon": float(row["lon"])})
    logger.info("Loaded %d waypoints from %s", len(waypoints), path)
    return waypoints


def _parse_csv_content(content: str) -> list[dict]:
    """Parse CSV text content → list of {lat, lon} dicts."""
    waypoints = []
    try:
        sample = content[:4096]
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters="\t,")
        except csv.Error:
            dialect = csv.excel  # fall back to standard comma-separated
        reader = csv.DictReader(content.splitlines(), dialect=dialect)
        for row in reader:
            waypoints.append({"lat": float(row["lat"]), "lon": float(row["lon"])})
        logger.info("Parsed %d waypoints from uploaded CSV", len(waypoints))
    except Exception as exc:
        logger.warning("_parse_csv_content: failed to parse CSV: %s", exc)
    return waypoints


# ═════════════════════════════════════════════════════════════════════════════
# I/O CLIENTS
# ═════════════════════════════════════════════════════════════════════════════

class ImuWsClient:
    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str) -> None:
        self._url = url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._last_ts: float = 0.0
        self._send_queue: asyncio.Queue = asyncio.Queue(maxsize=8)
        self._ws = None  # live websocket reference for sending

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def last_ts(self) -> float:
        with self._lock:
            return self._last_ts

    async def set_north_offset(self, offset_deg: float) -> None:
        """Send north_offset calibration to 01_IMU bridge."""
        msg = json.dumps({"set_north_offset": round(offset_deg, 4)})
        try:
            self._send_queue.put_nowait(msg)
        except asyncio.QueueFull:
            pass

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    self._ws = ws
                    async def _sender():
                        while True:
                            out = await self._send_queue.get()
                            try:
                                await ws.send(out)
                            except Exception:
                                pass
                    sender_task = asyncio.create_task(_sender())
                    try:
                        # ── INPUT boundary ─────────────────────────────────────────
                        async for msg in ws:
                        # ──────────────────────────────────────────────────────────
                            try:
                                data = json.loads(msg)
                                with self._lock:
                                    self._latest = data
                                    self._last_ts = time.monotonic()
                            except json.JSONDecodeError as exc:
                                logger.warning("ImuWsClient: JSON error: %s", exc)
                    finally:
                        sender_task.cancel()
                        self._ws = None
            except Exception as exc:
                logger.warning("ImuWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S)
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class RtkWsClient:
    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str) -> None:
        self._url = url
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
                    # ── INPUT boundary ─────────────────────────────────────────
                    async for msg in ws:
                    # ──────────────────────────────────────────────────────────
                        try:
                            data = json.loads(msg)
                            with self._lock:
                                self._latest = data
                                if data.get("fix_quality", 0) > 0:
                                    self._last_ts = time.monotonic()
                        except json.JSONDecodeError as exc:
                            logger.warning("RtkWsClient: JSON error: %s", exc)
            except Exception as exc:
                logger.warning("RtkWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S)
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class RobotWsClient:
    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str) -> None:
        self._url = url
        self._ws = None           # live WS reference; None when disconnected
        self._last_sent_ts: float = 0.0

    async def send(self, linear: float, angular: float) -> None:
        if self._ws is None:
            logger.warning("RobotWsClient.send: ws not connected, dropping (%.2f, %.2f)", linear, angular)
            return
        msg = json.dumps({"type": "joystick", "linear": linear, "angular": angular})
        try:
            # ── OUTPUT boundary ────────────────────────────────────────────────
            await self._ws.send(msg)
            # ──────────────────────────────────────────────────────────────────
            self._last_sent_ts = time.monotonic()
            if linear == 0.0 and angular == 0.0:
                logger.info("RobotWsClient: sent STOP (0,0)")
        except Exception as e:
            logger.warning("RobotWsClient.send: error %s", e)
            self._ws = None       # mark disconnected; run() will reconnect

    def flush_stop(self) -> None:
        """Schedule an immediate stop — creates a fire-and-forget task."""
        asyncio.create_task(self.send(0.0, 0.0))

    async def run(self) -> None:
        asyncio.create_task(self._heartbeat_loop())
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    self._ws = ws
                    await ws.wait_closed()
            except Exception as exc:
                logger.warning("RobotWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S)
            finally:
                self._ws = None
            await asyncio.sleep(self.RECONNECT_DELAY_S)

    async def _heartbeat_loop(self) -> None:
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            if time.monotonic() - self._last_sent_ts >= HEARTBEAT_INTERVAL:
                await self.send(0.0, 0.0)


# ═════════════════════════════════════════════════════════════════════════════
# WS SERVER (status broadcast + control commands)
# ═════════════════════════════════════════════════════════════════════════════

class AutoNavWsServer:
    def __init__(self, port: int, queue: asyncio.Queue, on_message) -> None:
        self._port = port
        self._queue = queue
        self._clients: set = set()
        self._on_message = on_message

    async def broadcast(self) -> None:
        while True:
            msg = await self._queue.get()
            dead = set()
            for ws in self._clients.copy():
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            self._clients.difference_update(dead)

    async def handle_client(self, websocket) -> None:
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                if self._on_message:
                    try:
                        await self._on_message(raw)
                    except Exception as exc:
                        logger.warning("WsServer: message error: %s", exc)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.discard(websocket)

    async def serve(self) -> None:
        asyncio.create_task(self.broadcast())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            await asyncio.Future()


# ═════════════════════════════════════════════════════════════════════════════
# HTTP FILE SERVER
# ═════════════════════════════════════════════════════════════════════════════

class HttpFileServer:
    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port = port

    def run(self) -> None:
        static_dir = self._static_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)
            def log_message(self, fmt, *args):
                pass
            def end_headers(self):
                self.send_header("Cache-Control", "no-store")
                super().end_headers()

        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self._port), _Handler) as httpd:
                httpd.serve_forever()
        except Exception as exc:
            logger.error("HttpFileServer: failed on port %d: %s", self._port, exc)


# ═════════════════════════════════════════════════════════════════════════════
# NAVIGATION LOOP
# ═════════════════════════════════════════════════════════════════════════════

class AutoNavLoop:
    """
    Control loop at CONTROL_HZ.

    Handles: sensor reading, timeout safety, state machine.
    All geometry, filtering, and steering logic is in autonav_algo.py.
    """

    def __init__(self, imu: ImuWsClient, rtk: RtkWsClient,
                 robot: RobotWsClient, waypoints: list[dict],
                 status_queue: asyncio.Queue) -> None:
        self._imu = imu
        self._rtk = rtk
        self._robot = robot
        self._waypoints = waypoints
        self._queue = status_queue

        self._state = "idle"            # idle | running | paused | arrived
        self._paused_by_timeout = False
        self._wp_idx = 0
        self._prev_ts = time.monotonic()
        self._speed_ratio: float = 1.0
        self._manual_linear: float = 0.0   # set by manual_drive command

        # arrival / confirmation state moved from algo into the bridge
        self._arrive_counter: int = 0
        self._waiting_at_wp: bool = False
        self._confirm_advance: bool = False

        # ── Heading calibration state ─────────────────────────────────────────
        self._calib_mark: dict | None = None   # {lat, lon} of marked forward point
        self._calib_offset_applied: float | None = None  # last applied offset

    # --------------------- Algorithm control helpers ---------------------
    def reset(self) -> None:
        """Reset internal arrival/waiting state (moved from algo)."""
        self._arrive_counter = 0
        self._waiting_at_wp = False
        self._confirm_advance = False

    def confirm_wp(self) -> None:
        """Called when UI confirms advancing from a waiting waypoint."""
        self._confirm_advance = True

    # ── State control (called from WS message handler) ────────────────────────

    def cmd_start(self) -> None:
        if not self._waypoints:
            logger.warning("start: no waypoints loaded")
            return
        self._wp_idx = 0
        self._paused_by_timeout = False
        self.reset()
        self._state = "running"
        logger.info("Navigation started (%d waypoints)", len(self._waypoints))

    def cmd_stop(self) -> None:
        self.reset()
        self._state = "idle"
        self._manual_linear = 0.0
        self._robot.flush_stop()
        logger.info("Navigation stopped")

    def cmd_mark_pos(self) -> str:
        """Record current RTK position as the forward calibration point."""
        rtk = self._rtk.latest
        lat, lon = rtk.get("lat"), rtk.get("lon")
        if lat is None or lon is None:
            logger.warning("calib mark_pos: no RTK fix")
            return "no_fix"
        self._calib_mark = {"lat": float(lat), "lon": float(lon)}
        logger.info("Calib mark set: lat=%.7f lon=%.7f", lat, lon)
        return "ok"

    async def cmd_calibrate(self) -> str:
        """Compute heading from current pos → marked pos and apply to 01_IMU."""
        if self._calib_mark is None:
            logger.warning("calib calibrate: no mark set")
            return "no_mark"
        rtk = self._rtk.latest
        imu = self._imu.latest
        cur_lat, cur_lon = rtk.get("lat"), rtk.get("lon")
        heading_raw = imu.get("heading", {}).get("raw")
        if cur_lat is None or cur_lon is None:
            return "no_fix"
        if heading_raw is None:
            return "no_imu"
        # bearing from current (origin) → marked (forward) point
        bearing = _bearing(float(cur_lat), float(cur_lon),
                           self._calib_mark["lat"], self._calib_mark["lon"])
        north_offset = (bearing - float(heading_raw)) % 360.0
        await self._imu.set_north_offset(north_offset)
        self._calib_offset_applied = north_offset
        logger.info("Calib applied: bearing=%.2f raw=%.2f offset=%.2f",
                    bearing, heading_raw, north_offset)
        return "ok"

    def calib_status(self) -> dict:
        mark = self._calib_mark
        return {
            "mark": mark,
            "offset_applied": self._calib_offset_applied,
        }

    def cmd_manual(self, linear: float) -> None:
        if self._state == "idle":
            self._manual_linear = linear

    def cmd_load_waypoints(self, waypoints: list[dict]) -> None:
        self.cmd_stop()
        self._waypoints = waypoints
        self._wp_idx = 0
        logger.info("Waypoints replaced: %d waypoints loaded", len(waypoints))

    def _get_wp_window(self, window: int = 7) -> list[dict]:
        """Return up to `window` waypoints centered around current index."""
        wps = self._waypoints
        if not wps:
            return []
        half = window // 2
        start = max(0, self._wp_idx - half)
        end   = min(len(wps), start + window)
        start = max(0, end - window)
        return [
            {"idx": i, "lat": wps[i]["lat"], "lon": wps[i]["lon"], "current": i == self._wp_idx}
            for i in range(start, end)
        ]

    def cmd_pause(self) -> None:
        if self._state == "running":
            self._state = "paused"
            self._paused_by_timeout = False
            self._manual_linear = 0.0
            self._robot.flush_stop()

    def cmd_resume(self) -> None:
        if self._state == "paused":
            self.reset()
            self._state = "running"
            self._paused_by_timeout = False

    def cmd_set_speed(self, ratio: float) -> None:
        self._speed_ratio = max(0.0, min(1.0, ratio))
        logger.info("Speed ratio set to %.0f%%", self._speed_ratio * 100)

    # ── Main loop ─────────────────────────────────────────────────────────────

    async def run(self) -> None:
        period = 1.0 / CONTROL_HZ
        while True:
            now = time.monotonic()
            dt  = now - self._prev_ts
            self._prev_ts = now

            # ── Read raw sensor dicts ─────────────────────────────────────────
            imu_raw = self._imu.latest
            rtk_raw = self._rtk.latest

            # ── Extract values from raw dicts ─────────────────────────────────
            heading = None
            hblock = imu_raw.get("heading", {})
            if hblock.get("deg") is not None:
                heading = float(hblock["deg"])

            lat = rtk_raw.get("lat")
            lon = rtk_raw.get("lon")
            if lat is not None: lat = float(lat)
            if lon is not None: lon = float(lon)

            imu_ts = self._imu.last_ts if self._imu.last_ts > 0 else now - 9999
            gps_ts = self._rtk.last_ts if self._rtk.last_ts > 0 else now - 9999
            gps_age = now - gps_ts
            imu_age = now - imu_ts

            # ─────────────────────────────────────────────────────────────────

            linear, angular = 0.0, 0.0

            # ── State machine ─────────────────────────────────────────────────
            sensors_ok = (
                heading is not None and lat is not None and lon is not None
                and gps_age < GPS_TIMEOUT_S and imu_age < GPS_TIMEOUT_S
            )

            if self._state == "running" and not sensors_ok:
                self._state = "paused"
                self._paused_by_timeout = True
                logger.warning("Sensor timeout — GPS age=%.1fs IMU age=%.1fs", gps_age, imu_age)

            if self._state == "paused" and self._paused_by_timeout and sensors_ok:
                self.reset()
                self._state = "running"
                self._paused_by_timeout = False
                logger.info("Sensors recovered — auto-resuming")

            if self._state == "running" and sensors_ok:
                # ── Call algorithm ────────────────────────────────────────────
                prev_wp = self._wp_idx
                linear, angular, arrived = algo.compute(
                    lat=lat, lon=lon, heading_deg=heading,
                    waypoints=self._waypoints, wp_idx=self._wp_idx, dt_s=dt,
                )
                # algo.compute() returns arrived=True when within REACH_TOL_M of current wp
                # Bridge handles ARRIVE_FRAMES counting and waypoint confirmation.
                if self._waiting_at_wp:
                    # Already at a waypoint, waiting for user to confirm advance
                    if self._confirm_advance:
                        self._wp_idx += 1
                        self._arrive_counter = 0
                        self._confirm_advance = False
                        self._waiting_at_wp = False
                    else:
                        # Hold position
                        linear, angular = 0.0, 0.0
                else:
                    # Try to reach current waypoint
                    if arrived:
                        self._arrive_counter += 1
                        if self._arrive_counter >= ARRIVE_FRAMES:
                            # Confirmed at waypoint for ARRIVE_FRAMES cycles
                            self._arrive_counter = 0
                            if self._wp_idx >= len(self._waypoints) - 1:
                                # Final waypoint reached
                                self._state = "arrived"
                                logger.info("Arrived at destination")
                            else:
                                # Intermediate waypoint: enter waiting state
                                self._waiting_at_wp = True
                                linear, angular = 0.0, 0.0
                    else:
                        self._arrive_counter = 0
                
                if self._wp_idx > prev_wp:
                    logger.info("Waypoint %d/%d reached", self._wp_idx, len(self._waypoints))

            # ── Send to robot ─────────────────────────────────────────────────
            if self._state == "idle":
                send_linear  = self._manual_linear
                send_angular = 0.0
            elif self._state == "running":
                send_linear  = linear * self._speed_ratio
                send_angular = angular * self._speed_ratio
            else:  # paused / arrived — explicit stop
                send_linear  = 0.0
                send_angular = 0.0
            await self._robot.send(round(send_linear, 3), round(send_angular, 3))

            # ── Derived display metrics ───────────────────────────────────────
            dist_to_wp_m       = None
            target_bearing_deg = None
            bearing_error_deg  = None
            if lat is not None and lon is not None and self._wp_idx < len(self._waypoints):
                wp = self._waypoints[self._wp_idx]
                dist_to_wp_m = algo._fast_distance_m(lat, lon, wp["lat"], wp["lon"])
                target_bearing_deg = algo._fast_bearing(lat, lon, wp["lat"], wp["lon"])
                if heading is not None:
                    bearing_error_deg = round((target_bearing_deg - heading + 180) % 360 - 180, 1)
                dist_to_wp_m = round(dist_to_wp_m, 2)
                target_bearing_deg = round(target_bearing_deg, 1)

            # ── Broadcast status ──────────────────────────────────────────────
            status = {
                "type":               "autonav_status",
                "version":            1,
                "state":              self._state,
                "current_wp_idx":     self._wp_idx,
                "total_wp":           len(self._waypoints),
                "heading_deg":        heading,
                "target_bearing_deg": target_bearing_deg,
                "bearing_error_deg":  bearing_error_deg,
                "dist_to_wp_m":       dist_to_wp_m,
                "linear":             round(linear, 3),
                "angular":            round(angular, 3),
                "gps_age_s":          round(gps_age, 2),
                "imu_age_s":          round(imu_age, 2),
                "speed_ratio":        self._speed_ratio,
                "manual_speed":       MANUAL_SPEED,
                "calib":              self.calib_status(),
                "waypoints_window":   self._get_wp_window(),
                "waiting_at_wp":      self._waiting_at_wp,
                "waiting_wp_idx":     self._wp_idx if self._waiting_at_wp else None,
                "imu_raw":            imu_raw,
                "rtk_raw":            rtk_raw,
            }
            try:
                self._queue.put_nowait(json.dumps(status))
            except asyncio.QueueFull:
                pass

            await asyncio.sleep(period)


# ═════════════════════════════════════════════════════════════════════════════
# TOP-LEVEL ORCHESTRATOR
# ═════════════════════════════════════════════════════════════════════════════

class AutoNavBridge:
    def __init__(self) -> None:
        self._nav_loop: AutoNavLoop | None = None
        self._robot: RobotWsClient | None = None

    def run(self) -> None:
        static_dir = Path(__file__).parent / "web_static"
        if static_dir.exists():
            http_server = HttpFileServer(static_dir, HTTP_PORT)
            threading.Thread(target=http_server.run, daemon=True, name="http").start()
            threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{HTTP_PORT}")).start()
        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Stopped by user.")

    async def _run_async(self) -> None:
        try:
            waypoints = _load_waypoints(PATH_FILE)
        except FileNotFoundError:
            logger.warning("path.csv not found — starting with no waypoints. Use LOAD CSV in the UI.")
            waypoints = []
        if not waypoints:
            logger.warning("No waypoints loaded from %s — use LOAD CSV in the UI to load a path.", PATH_FILE)

        imu_client   = ImuWsClient(IMU_WS_URL)
        rtk_client   = RtkWsClient(RTK_WS_URL)
        robot_client = RobotWsClient(ROBOT_WS_URL)
        self._robot  = robot_client
        status_queue: asyncio.Queue = asyncio.Queue(maxsize=20)

        self._nav_loop = AutoNavLoop(imu_client, rtk_client, robot_client,
                                     waypoints, status_queue)

        ws_server = AutoNavWsServer(WS_PORT, status_queue, self._handle_message)

        logger.info("AutoNavBridge ready | http=:%d ws=:%d | %d waypoints",
                    HTTP_PORT, WS_PORT, len(waypoints))

        await asyncio.gather(
            imu_client.run(),
            rtk_client.run(),
            robot_client.run(),
            self._nav_loop.run(),
            ws_server.serve(),
        )

    async def _handle_message(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        if self._nav_loop is None:
            return
        t = msg.get("type", "")
        if   t == "start":     self._nav_loop.cmd_start()
        elif t == "stop":      self._nav_loop.cmd_stop()
        elif t == "pause":     self._nav_loop.cmd_pause()
        elif t == "resume":    self._nav_loop.cmd_resume()
        elif t == "set_speed": self._nav_loop.cmd_set_speed(float(msg.get("ratio", 1.0)))
        elif t == "load_csv":
            waypoints = _parse_csv_content(msg.get("content", ""))
            if waypoints:
                self._nav_loop.cmd_load_waypoints(waypoints)
            else:
                logger.warning("load_csv: no valid waypoints parsed from uploaded content")
        elif t == "calib_mark":
            self._nav_loop.cmd_mark_pos()
        elif t == "calib_apply":
            await self._nav_loop.cmd_calibrate()
        elif t == "manual_drive":
            self._nav_loop.cmd_manual(float(msg.get("linear", 0.0)))
        elif t == "confirm_wp":
            # forward to nav loop wrapper
            try:
                self._nav_loop.confirm_wp()
            except Exception:
                pass


if __name__ == "__main__":
    AutoNavBridge().run()
