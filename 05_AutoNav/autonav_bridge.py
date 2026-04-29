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
HEARTBEAT_INTERVAL = 0.2
ROBOT_SEND_TIMEOUT_S = _c("AUTONAV_ROBOT_SEND_TIMEOUT_S", 2.0)

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


def _load_default_waypoints(path: Path) -> list[dict]:
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


def _convert_csv_to_waypoints(content: str) -> list[dict]:
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


async def _ws_session_loop(client_name: str, url: str, reconnect_delay_s: float,
                           on_connect=None, on_message=None, on_disconnect=None) -> None:
    """Reconnect forever and let the caller customize connect/message/cleanup hooks."""
    while True:
        try:
            async with websockets.connect(url) as ws:
                if on_connect is not None:
                    await on_connect(ws)
                if on_message is None:
                    await ws.wait_closed()
                else:
                    async for msg in ws:
                        await on_message(msg)
        except Exception as exc:
            logger.warning("%s: %s — retrying in %.0fs", client_name, exc, reconnect_delay_s)
        finally:
            if on_disconnect is not None:
                await on_disconnect()
        await asyncio.sleep(reconnect_delay_s)


def _extract_imu_heading(imu_raw: dict) -> float | None:
    """Extract heading in degrees from the raw IMU payload."""
    heading = None
    hblock = imu_raw.get("heading", {})
    if hblock.get("deg") is not None:
        heading = float(hblock["deg"])
    return heading


def _extract_rtk_sample(rtk_raw: dict) -> tuple[float | None, float | None, int]:
    """Extract RTK position and fix quality from the raw RTK payload."""
    lat = rtk_raw.get("lat")
    lon = rtk_raw.get("lon")
    if lat is not None:
        lat = float(lat)
    if lon is not None:
        lon = float(lon)
    fix_quality = int(rtk_raw.get("fix_quality", 0) or 0)
    return lat, lon, fix_quality


def _sensor_ages(now: float, imu_client: "ImuWsClient", rtk_client: "RtkWsClient") -> tuple[float, float, float]:
    """Return IMU age, RTK fix age, and RTK packet age in seconds."""
    imu_ts = imu_client.last_ts if imu_client.last_ts > 0 else now - 9999
    gps_ts = rtk_client.last_ts if rtk_client.last_ts > 0 else now - 9999
    gps_packet_ts = rtk_client.last_packet_ts if rtk_client.last_packet_ts > 0 else now - 9999
    return now - imu_ts, now - gps_ts, now - gps_packet_ts


def _sensor_block_reason(fix_quality: int, gps_age: float, gps_packet_age: float, imu_age: float) -> str:
    """Explain why the control loop paused for sensor safety."""
    if fix_quality <= 0:
        return f"rtk_fix_quality={fix_quality}"
    if gps_packet_age >= GPS_TIMEOUT_S:
        return f"rtk_packet_age={gps_packet_age:.1f}s"
    if gps_age >= GPS_TIMEOUT_S:
        return f"rtk_fix_age={gps_age:.1f}s"
    if imu_age >= GPS_TIMEOUT_S:
        return f"imu_age={imu_age:.1f}s"
    return "sensor_unavailable"


def _sensors_ok(heading: float | None, lat: float | None, lon: float | None,
                fix_quality: int, gps_age: float, gps_packet_age: float, imu_age: float) -> bool:
    """Return True only when all inputs are present and fresh enough."""
    return (
        heading is not None and lat is not None and lon is not None
        and fix_quality > 0
        and gps_age < GPS_TIMEOUT_S and gps_packet_age < GPS_TIMEOUT_S
        and imu_age < GPS_TIMEOUT_S
    )


def _apply_waypoint_progress(loop: "AutoNavLoop", linear: float, angular: float,
                             arrived: bool) -> tuple[float, float, bool, bool]:
    """Update waypoint progress and return final command, arrival flag, and advance flag."""
    waypoint_advanced = False

    if loop._waiting_at_wp:
        if loop._confirm_advance:
            loop._wp_idx += 1
            loop._arrive_counter = 0
            loop._confirm_advance = False
            loop._waiting_at_wp = False
            return 0.0, 0.0, False, True
        return 0.0, 0.0, False, False

    if not arrived:
        loop._arrive_counter = 0
        return linear, angular, False, False

    loop._arrive_counter += 1
    if loop._arrive_counter < ARRIVE_FRAMES:
        return linear, angular, True, False

    loop._arrive_counter = 0
    if loop._wp_idx >= len(loop._waypoints) - 1:
        loop._state = "arrived"
        logger.info("Arrived at destination")
        return 0.0, 0.0, False, False

    loop._waiting_at_wp = True
    return 0.0, 0.0, False, False


def _scaled_robot_command(state: str, linear: float, angular: float,
                          manual_linear: float, speed_ratio: float) -> tuple[float, float]:
    """Convert controller output into final robot command."""
    if state == "idle":
        return manual_linear, 0.0
    if state == "running":
        return linear * speed_ratio, angular * speed_ratio
    return 0.0, 0.0


def _build_autonav_status(loop: "AutoNavLoop", heading: float | None,
                          lat: float | None, lon: float | None,
                          gps_age: float, gps_packet_age: float,
                          imu_age: float, linear: float, angular: float,
                          imu_raw: dict, rtk_raw: dict) -> dict:
    """Package the current navigation snapshot for the dashboard."""
    dist_to_wp_m = None
    target_bearing_deg = None
    bearing_error_deg = None
    if lat is not None and lon is not None and loop._wp_idx < len(loop._waypoints):
        wp = loop._waypoints[loop._wp_idx]
        dist_to_wp_m = algo._fast_distance_m(lat, lon, wp["lat"], wp["lon"])
        target_bearing_deg = algo._fast_bearing(lat, lon, wp["lat"], wp["lon"])
        if heading is not None:
            bearing_error_deg = round((target_bearing_deg - heading + 180) % 360 - 180, 1)
        dist_to_wp_m = round(dist_to_wp_m, 2)
        target_bearing_deg = round(target_bearing_deg, 1)

    return {
        "type":               "autonav_status",
        "version":            1,
        "state":              loop._state,
        "current_wp_idx":     loop._wp_idx,
        "total_wp":           len(loop._waypoints),
        "heading_deg":        heading,
        "target_bearing_deg": target_bearing_deg,
        "bearing_error_deg":  bearing_error_deg,
        "dist_to_wp_m":       dist_to_wp_m,
        "linear":             round(linear, 3),
        "angular":            round(angular, 3),
        "gps_age_s":          round(gps_age, 2),
        "gps_packet_age_s":   round(gps_packet_age, 2),
        "gps_fix_quality":    int(rtk_raw.get("fix_quality", 0) or 0),
        "sensor_block_reason": loop._sensor_block_reason,
        "imu_age_s":          round(imu_age, 2),
        "speed_ratio":        loop._speed_ratio,
        "manual_speed":       MANUAL_SPEED,
        "calib":              loop.calib_status(),
        "waypoints_window":   loop._get_wp_window(),
        "waiting_at_wp":      loop._waiting_at_wp,
        "waiting_wp_idx":     loop._wp_idx if loop._waiting_at_wp else None,
        "imu_raw":            imu_raw,
        "rtk_raw":            rtk_raw,
    }


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
        sender_task: asyncio.Task | None = None

        async def _handle_message(msg: str) -> None:
            try:
                data = json.loads(msg)
                with self._lock:
                    self._latest = data
                    self._last_ts = time.monotonic()
            except json.JSONDecodeError as exc:
                logger.warning("ImuWsClient: JSON error: %s", exc)

        async def _on_connect(ws) -> None:
            nonlocal sender_task
            self._ws = ws

            async def _sender() -> None:
                while True:
                    out = await self._send_queue.get()
                    try:
                        await ws.send(out)
                    except Exception:
                        pass

            sender_task = asyncio.create_task(_sender())

        async def _on_disconnect() -> None:
            nonlocal sender_task
            if sender_task is not None:
                sender_task.cancel()
                sender_task = None
            self._ws = None

        await _ws_session_loop(
            "ImuWsClient",
            self._url,
            self.RECONNECT_DELAY_S,
            on_connect=_on_connect,
            on_message=_handle_message,
            on_disconnect=_on_disconnect,
        )


class RtkWsClient:
    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str) -> None:
        self._url = url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._last_ts: float = 0.0
        self._last_packet_ts: float = 0.0

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def last_ts(self) -> float:
        with self._lock:
            return self._last_ts

    @property
    def last_packet_ts(self) -> float:
        with self._lock:
            return self._last_packet_ts

    async def run(self) -> None:
        async def _handle_message(msg: str) -> None:
            try:
                data = json.loads(msg)
                with self._lock:
                    self._latest = data
                    self._last_packet_ts = time.monotonic()
                    if data.get("fix_quality", 0) > 0:
                        self._last_ts = self._last_packet_ts
            except json.JSONDecodeError as exc:
                logger.warning("RtkWsClient: JSON error: %s", exc)

        await _ws_session_loop(
            "RtkWsClient",
            self._url,
            self.RECONNECT_DELAY_S,
            on_message=_handle_message,
        )


class RobotWsClient:
    RECONNECT_DELAY_S = 1.0

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
            await asyncio.wait_for(self._ws.send(msg), timeout=ROBOT_SEND_TIMEOUT_S)
            # ──────────────────────────────────────────────────────────────────
            self._last_sent_ts = time.monotonic()
            if linear == 0.0 and angular == 0.0:
                logger.debug("RobotWsClient: sent STOP (0,0)")
        except asyncio.TimeoutError:
            logger.warning("RobotWsClient.send: timeout after %.2fs, closing stale connection", ROBOT_SEND_TIMEOUT_S)
            if self._ws is not None:
                try:
                    await self._ws.close()
                except Exception:
                    pass
            self._ws = None
        except Exception as e:
            logger.warning("RobotWsClient.send: error %s, closing stale connection", e)
            if self._ws is not None:
                try:
                    await self._ws.close()
                except Exception:
                    pass
            self._ws = None

    async def run(self) -> None:
        asyncio.create_task(self._heartbeat_loop())

        async def _session(ws) -> None:
            self._ws = ws
            try:
                async for _ in ws:  # drain incoming messages (odom/state) to prevent TCP buffer fill
                    pass
            finally:
                self._ws = None

        await _ws_session_loop("RobotWsClient", self._url, self.RECONNECT_DELAY_S, on_connect=_session)

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
            def log_message(self, format, *args):
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
        self._sensor_block_reason: str | None = None

        # ── Heading calibration state ─────────────────────────────────────────
        self._calib_mark: dict | None = None   # {lat, lon} of marked forward point
        self._calib_offset_applied: float | None = None  # last applied offset

        # ── Stop blocking mechanism ────────────────────────────────────────────
        # When stop() is called, _resume_event is cleared, blocking the main loop
        # Resume or start will set it, unblocking the loop
        self._resume_event: asyncio.Event = asyncio.Event()
        self._resume_event.set()  # Initially unblocked

    # --------------------- Algorithm control helpers ---------------------
    def reset(self) -> None:
        """Reset internal arrival/waiting state (moved from algo)."""
        self._arrive_counter = 0
        self._waiting_at_wp = False
        self._confirm_advance = False
        self._sensor_block_reason = None

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
        self._resume_event.set()  # Unblock main loop if it was stopped
        logger.info("Navigation started (%d waypoints)", len(self._waypoints))

    async def cmd_stop(self) -> None:
        self.reset()
        self._state = "idle"
        self._manual_linear = 0.0
        await self._robot.send(0.0, 0.0)  # Send stop command immediately
        self._resume_event.set()  # Keep loop running so idle status is pushed to frontend
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
        bearing = algo._fast_bearing(float(cur_lat), float(cur_lon),float(self._calib_mark["lat"]), float(self._calib_mark["lon"]))
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

    async def cmd_load_waypoints(self, waypoints: list[dict]) -> None:
        await self.cmd_stop()
        self._waypoints = waypoints
        self._wp_idx = 0
        self._resume_event.set()  # keep loop running in idle so status keeps streaming
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

    async def cmd_pause(self) -> None:
        if self._state == "running":
            self._state = "paused"
            self._paused_by_timeout = False
            self._manual_linear = 0.0
            await self._robot.send(0.0, 0.0)

    def cmd_resume(self) -> None:
        if self._state == "paused":
            self.reset()
            self._state = "running"
            self._paused_by_timeout = False
            self._resume_event.set()  # Unblock main loop

    def cmd_set_speed(self, ratio: float) -> None:
        self._speed_ratio = max(0.0, min(1.0, ratio))
        logger.info("Speed ratio set to %.0f%%", self._speed_ratio * 100)

    # ── Main loop ─────────────────────────────────────────────────────────────

    async def run(self) -> None:
        period = 1.0 / CONTROL_HZ
        while True:
            await self._resume_event.wait()  # Block until resume/start unblocks

            now = time.monotonic()
            dt  = now - self._prev_ts
            self._prev_ts = now

            imu_raw = self._imu.latest
            rtk_raw = self._rtk.latest
            heading = _extract_imu_heading(imu_raw)
            lat, lon, fix_quality = _extract_rtk_sample(rtk_raw)
            imu_age, gps_age, gps_packet_age = _sensor_ages(now, self._imu, self._rtk)
            sensors_ok = _sensors_ok(heading, lat, lon, fix_quality, gps_age, gps_packet_age, imu_age)

            linear, angular = 0.0, 0.0

            if self._state == "running" and not sensors_ok:
                self._state = "paused"
                self._paused_by_timeout = True
                self._sensor_block_reason = _sensor_block_reason(fix_quality, gps_age, gps_packet_age, imu_age)
                logger.warning(
                    "Sensor timeout — reason=%s GPS age=%.1fs packet age=%.1fs IMU age=%.1fs",
                    self._sensor_block_reason, gps_age, gps_packet_age, imu_age,
                )

            if self._state == "paused" and self._paused_by_timeout and sensors_ok:
                self._arrive_counter = 0
                self._confirm_advance = False
                self._state = "running"
                self._paused_by_timeout = False
                self._sensor_block_reason = None
                logger.info("Sensors recovered — auto-resuming")

            waypoint_advanced = False
            if self._state == "running" and sensors_ok:
                linear, angular, arrived = algo.compute(
                    lat=lat, lon=lon, heading_deg=heading,
                    waypoints=self._waypoints, wp_idx=self._wp_idx, dt_s=dt,
                )
                linear, angular, arrived, waypoint_advanced = _apply_waypoint_progress(
                    self, linear, angular, arrived
                )
                if waypoint_advanced:
                    logger.info("Waypoint %d/%d reached", self._wp_idx, len(self._waypoints))

            send_linear, send_angular = _scaled_robot_command(
                self._state, linear, angular, self._manual_linear, self._speed_ratio
            )
            await self._robot.send(round(send_linear, 3), round(send_angular, 3))

            status = _build_autonav_status(
                self, heading, lat, lon, gps_age, gps_packet_age, imu_age,
                linear, angular, imu_raw, rtk_raw,
            )
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
            waypoints = _load_default_waypoints(PATH_FILE)
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
        elif t == "stop":      await self._nav_loop.cmd_stop()
        elif t == "pause":     await self._nav_loop.cmd_pause()
        elif t == "resume":    self._nav_loop.cmd_resume()
        elif t == "set_speed": self._nav_loop.cmd_set_speed(float(msg.get("ratio", 1.0)))
        elif t == "load_csv":
            waypoints = _convert_csv_to_waypoints(msg.get("content", ""))
            if waypoints:
                await self._nav_loop.cmd_load_waypoints(waypoints)
            else:
                logger.warning("load_csv: no valid waypoints parsed from uploaded content")
        elif t == "calib_mark":
            self._nav_loop.cmd_mark_pos()
        elif t == "calib_apply":
            await self._nav_loop.cmd_calibrate()
        elif t == "manual_drive":
            self._nav_loop.cmd_manual(float(msg.get("linear", 0.0)))
        elif t == "confirm_wp":
            logger.info("confirm_wp received — waiting=%s confirm=%s wp=%s",
                        self._nav_loop._waiting_at_wp,
                        self._nav_loop._confirm_advance,
                        self._nav_loop._wp_idx)
            self._nav_loop.confirm_wp()


if __name__ == "__main__":
    AutoNavBridge().run()
