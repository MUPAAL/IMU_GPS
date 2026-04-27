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
import math
import socketserver
import threading
import time
import webbrowser
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import websockets

import autonav_algo as algo

# ── Configuration ─────────────────────────────────────────────────────────────

HTTP_PORT          = _cfg.AUTONAV_WS_PORT if _cfg else 8805
WS_PORT            = HTTP_PORT + 1                        # 8806
IMU_WS_URL         = _cfg.AUTONAV_IMU_WS  if _cfg else "ws://localhost:8766"
RTK_WS_URL         = _cfg.AUTONAV_RTK_WS  if _cfg else "ws://localhost:8776"
ROBOT_WS_URL       = "ws://localhost:8889"
GPS_TIMEOUT_S      = _cfg.AUTONAV_GPS_TIMEOUT_S if _cfg else 5.0
CONTROL_HZ         = _cfg.AUTONAV_CONTROL_HZ    if _cfg else 5.0
HEARTBEAT_INTERVAL = 1.0

# angular sign: +1 = positive error → turn left/CCW.
# Set to -1 if robot turns the wrong way.
ANGULAR_SIGN = 1

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
# GEO HELPERS
# ═════════════════════════════════════════════════════════════════════════════

def _haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres."""
    R = 6_371_000
    r = math.radians
    dlat = r(lat2 - lat1)
    dlon = r(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(r(lat1))*math.cos(r(lat2))*math.sin(dlon/2)**2
    return 2 * R * math.asin(math.sqrt(a))

def _bearing(lat1, lon1, lat2, lon2):
    """Initial bearing from (lat1,lon1) to (lat2,lon2), degrees [0,360)."""
    r = math.radians
    dlon = r(lon2 - lon1)
    x = math.sin(dlon) * math.cos(r(lat2))
    y = math.cos(r(lat1))*math.sin(r(lat2)) - math.sin(r(lat1))*math.cos(r(lat2))*math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


# ═════════════════════════════════════════════════════════════════════════════
# WAYPOINT LOADER
# ═════════════════════════════════════════════════════════════════════════════

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
                                self._last_ts = time.monotonic()
                        except json.JSONDecodeError as exc:
                            logger.warning("ImuWsClient: JSON error: %s", exc)
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
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=5)
        self._last_sent_ts: float = 0.0

    async def send(self, linear: float, angular: float) -> None:
        msg = json.dumps({"type": "joystick", "linear": linear, "angular": angular})
        try:
            self._queue.put_nowait(msg)
        except asyncio.QueueFull:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(msg)
            except asyncio.QueueEmpty:
                pass

    async def run(self) -> None:
        asyncio.create_task(self._heartbeat_loop())
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    while True:
                        msg = await self._queue.get()
                        # ── OUTPUT boundary ────────────────────────────────────
                        await ws.send(msg)
                        # ──────────────────────────────────────────────────────
                        self._last_sent_ts = time.monotonic()
            except Exception as exc:
                logger.warning("RobotWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S)
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

    Handles: sensor reading, timeout safety, waypoint selection (Pure Pursuit),
             state machine, and calls algo.compute() for linear/angular output.
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
        self._arrive_counter = 0
        self._prev_ts = time.monotonic()

    # ── State control (called from WS message handler) ────────────────────────

    def cmd_start(self) -> None:
        if not self._waypoints:
            logger.warning("start: no waypoints loaded")
            return
        self._wp_idx = 0
        self._arrive_counter = 0
        self._paused_by_timeout = False
        algo.reset()
        self._state = "running"
        logger.info("Navigation started (%d waypoints)", len(self._waypoints))

    def cmd_stop(self) -> None:
        algo.reset()
        self._state = "idle"
        logger.info("Navigation stopped")

    def cmd_pause(self) -> None:
        if self._state == "running":
            self._state = "paused"
            self._paused_by_timeout = False

    def cmd_resume(self) -> None:
        if self._state == "paused":
            algo.reset()
            self._state = "running"
            self._paused_by_timeout = False

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

            if algo.ALGO_DEBUG:
                print(f"[RAW IMU] {json.dumps(imu_raw)}")
                print(f"[RAW RTK] {json.dumps(rtk_raw)}")

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

            if algo.ALGO_DEBUG:
                print(f"[EXTRACTED] heading={heading} lat={lat} lon={lon} "
                      f"gps_age={gps_age:.2f}s imu_age={imu_age:.2f}s")
            # ─────────────────────────────────────────────────────────────────

            linear, angular = 0.0, 0.0
            target_bearing = None
            bearing_error  = None
            dist_to_wp     = None
            dist_to_final  = None

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
                algo.reset()
                self._state = "running"
                self._paused_by_timeout = False
                logger.info("Sensors recovered — auto-resuming")

            if self._state == "running" and sensors_ok:
                wps = self._waypoints

                # Advance waypoint index when close enough
                wp = wps[self._wp_idx]
                d  = _haversine(lat, lon, wp["lat"], wp["lon"])
                if d < algo.REACH_TOL_M:
                    self._arrive_counter += 1
                    if self._arrive_counter >= algo.ARRIVE_FRAMES:
                        self._wp_idx += 1
                        self._arrive_counter = 0
                        logger.info("Waypoint %d/%d reached", self._wp_idx, len(wps))
                else:
                    self._arrive_counter = 0

                # Check if all waypoints done
                if self._wp_idx >= len(wps):
                    self._state = "arrived"
                    logger.info("Arrived at destination")
                else:
                    # Pure Pursuit: pick first waypoint at least LOOKAHEAD_M away
                    target = wps[-1]
                    for w in wps[self._wp_idx:]:
                        if _haversine(lat, lon, w["lat"], w["lon"]) >= algo.LOOKAHEAD_M:
                            target = w
                            break

                    target_bearing = _bearing(lat, lon, target["lat"], target["lon"])
                    dist_to_wp     = _haversine(lat, lon, wps[self._wp_idx]["lat"], wps[self._wp_idx]["lon"])
                    dist_to_final  = _haversine(lat, lon, wps[-1]["lat"], wps[-1]["lon"])
                    bearing_error  = (target_bearing - heading + 540) % 360 - 180

                    # ── Call algorithm ────────────────────────────────────────
                    linear, angular = algo.compute(
                        heading_deg        = heading,
                        target_bearing_deg = target_bearing,
                        dist_to_wp_m       = dist_to_wp,
                        dist_to_final_m    = dist_to_final,
                        dt_s               = dt,
                    )
                    angular *= ANGULAR_SIGN
                    # ─────────────────────────────────────────────────────────

                    if algo.ALGO_DEBUG:
                        print(f"[CMD OUT] linear={linear:.3f} angular={angular:.3f} "
                              f"error={bearing_error:.1f}° dist={dist_to_wp:.1f}m "
                              f"wp={self._wp_idx}/{len(wps)}")

            # ── Send to robot ─────────────────────────────────────────────────
            await self._robot.send(round(linear, 3), round(angular, 3))

            # ── Broadcast status ──────────────────────────────────────────────
            status = {
                "type":               "autonav_status",
                "version":            1,
                "state":              self._state,
                "current_wp_idx":     self._wp_idx,
                "total_wp":           len(self._waypoints),
                "dist_to_wp_m":       round(dist_to_wp, 2)    if dist_to_wp    is not None else None,
                "target_bearing_deg": round(target_bearing, 1) if target_bearing is not None else None,
                "heading_deg":        round(heading, 1)        if heading        is not None else None,
                "bearing_error_deg":  round(bearing_error, 1)  if bearing_error  is not None else None,
                "linear":             round(linear, 3),
                "angular":            round(angular, 3),
                "gps_age_s":          round(gps_age, 2),
                "imu_age_s":          round(imu_age, 2),
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
        waypoints = _load_waypoints(PATH_FILE)
        if not waypoints:
            logger.error("No waypoints in %s — exiting.", PATH_FILE)
            return

        imu_client   = ImuWsClient(IMU_WS_URL)
        rtk_client   = RtkWsClient(RTK_WS_URL)
        robot_client = RobotWsClient(ROBOT_WS_URL)
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
        if   t == "start":  self._nav_loop.cmd_start()
        elif t == "stop":   self._nav_loop.cmd_stop()
        elif t == "pause":  self._nav_loop.cmd_pause()
        elif t == "resume": self._nav_loop.cmd_resume()


if __name__ == "__main__":
    AutoNavBridge().run()
