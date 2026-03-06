"""
nav_bridge.py - Navigation fusion bridge.

Subscribes to IMU WebSocket (port 8766) and RTK WebSocket (port 8776),
fuses sensor data, performs waypoint navigation with PD control,
broadcasts combined state to web clients, and serves the static nav UI.

Architecture:
  async tasks:
    connect_imu_ws  — IMU client, updates _imu_state
    connect_rtk_ws  — RTK client, updates _rtk_state
    nav_loop        — fusion + path planning + command generation (10 Hz)
    broadcast_loop  — sends fused state to all web WS clients
  threads:
    HTTP server     — serves web_static/
    RobotSerial     — sends V commands, receives O feedback

Degradation:
  - If IMU disconnects: heading falls back to RTK track_deg
  - If RTK disconnects: position/speed unavailable, nav pauses
  - If robot serial unavailable: nav still runs, commands logged only

Usage:
    python nav_bridge.py \\
        --imu-ws ws://localhost:8766 \\
        --rtk-ws ws://localhost:8776 \\
        --robot-port /dev/ttyUSB1 --robot-baud 115200 \\
        --nav-port 8785
    # Browser: http://localhost:8785
"""

# **************** IMPORTS ****************

import argparse
import asyncio
import json
import logging
import math
import socketserver
import threading
import time
import webbrowser
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import numpy as np
import websockets

from robot_serial import RobotSerial

# **************** LOGGING SETUP ****************

_py_name = Path(__file__).stem
OUTPUT_PATH = Path(__file__).parent
log_file_name = OUTPUT_PATH / f"{_py_name}.log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_file_name, encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

# **************** NAVIGATION TUNING CONSTANTS ****************

ARRIVAL_RADIUS_M   = 1.0    # metres  — waypoint considered reached within this radius
TURN_THRESHOLD_DEG = 15.0   # degrees — rotate in place if |heading_error| > threshold
MAX_SPEED_MS       = 1.0    # m/s     — maximum allowed forward speed
MAX_ANG_RATE_RADS  = 0.8    # rad/s   — angular rate clamp (both directions)
KP_ANG             = 0.025  # PD P-gain: heading_error_deg → ang_rate_rads
KD_ANG             = 0.005  # PD D-gain: rate of heading error change
SLOWDOWN_DIST_M    = 3.0    # metres  — start decelerating when closer than this

RECONNECT_DELAY_S  = 3.0    # seconds — wait before retrying a dropped WebSocket
NAV_LOOP_HZ        = 10.0   # Hz      — nav_loop tick rate
DATA_STALE_S       = 5.0    # seconds — sensor data older than this is ignored

# **************** GLOBAL SHARED STATE ****************
#
#  Three coroutines write/read these via _state_lock:
#    connect_imu_ws  → writes _imu_state, _imu_ts_last
#    connect_rtk_ws  → writes _rtk_state, _rtk_ts_last
#    nav_loop        → reads both, writes _fused, _wp_index, _nav_active
#
# _ws_clients and _robot are only modified in the event loop thread (no extra lock needed).

_imu_state: dict = {}           # latest raw IMU frame received from WS (8766)
_rtk_state: dict = {}           # latest raw RTK frame received from WS (8776)
_imu_connected = False          # True while IMU WS connection is alive
_rtk_connected = False          # True while RTK WS connection is alive

_imu_ts_last = 0.0              # time.time() of most recent IMU frame
_rtk_ts_last = 0.0              # time.time() of most recent RTK frame

_state_lock = threading.Lock()  # protects _imu_state, _rtk_state, timestamps

# Navigation planning state
_waypoints: list[dict] = []     # ordered list of {"lat": ..., "lon": ...}
_wp_index: int = 0              # index of the current target waypoint
_nav_active: bool = False       # True when the robot should be navigating

_last_heading_error: float = 0.0  # previous-cycle error for PD derivative term

# Fused output payload (built every nav_loop tick, broadcast by broadcast_loop)
_fused: dict = {}

# Connected web-client WebSocket objects
_ws_clients: set = set()

# Optional robot serial interface (None if --robot-port not provided)
_robot: RobotSerial | None = None

# Stop command string (defined here; also used inside nav_loop)
CMD_STOP_STR = "V0.00,0.000\n"


# **************** GEO MATH UTILITIES ****************

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return great-circle distance in metres between two WGS-84 coordinates."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Return initial bearing in degrees from point 1 to point 2.
    Convention: 0 = North, increases clockwise, range [0, 360).
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def normalize_angle(deg: float) -> float:
    """Normalize an angle to the range [-180, 180] degrees."""
    deg = deg % 360.0
    if deg > 180.0:
        deg -= 360.0
    return deg


def quat_to_yaw(qi: float, qj: float, qk: float, qr: float) -> float:
    """
    Extract yaw from BNO085 quaternion (i, j, k, r).
    Returns mathematical yaw in [-180, 180] degrees.
    Convention: right-hand Z-up, 0 at +X, positive CCW.
    """
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def yaw_to_compass_heading(yaw_deg: float) -> float:
    """
    Convert mathematical yaw to compass heading.

    Input yaw convention:
      - right-hand, Z-up
      - 0 deg at +X axis
      - positive CCW

    Output heading convention (used by navigation bearing):
      - 0 deg at North
      - positive CW
      - [0, 360)
    """
    return (90.0 - yaw_deg) % 360.0


# ****************
# **************** DATA INPUT: IMU WebSocket → _imu_state (8766 → this process)
# ****************

async def connect_imu_ws(uri: str) -> None:
    """
    Persistent async client for the IMU WebSocket server.

    On each received message:
      JSON frame → _imu_state (raw IMU data, used by nav_loop)

    On disconnect: waits RECONNECT_DELAY_S then retries indefinitely.
    Degradation: if this coroutine fails, nav_loop falls back to RTK track_deg for heading.

    Expected incoming frame keys (from serial_bridge.py):
      rot.qi/qj/qk/qr   — absolute orientation quaternion
      euler.yaw/pitch/roll
      lin_accel.x/y/z   — linear acceleration [m/s²]
      gyro.x/y/z        — angular rate [rad/s]
      mag.x/y/z         — magnetometer [µT]
      cal                — calibration status 0-3
      hz                 — measured data rate
    """
    global _imu_connected, _imu_state, _imu_ts_last
    while True:
        try:
            logger.info("IMU WS: connecting to %s", uri)
            async with websockets.connect(uri, ping_interval=20, ping_timeout=10) as ws:
                _imu_connected = True
                logger.info("IMU WS: connected")
                async for msg in ws:
                    try:
                        data = json.loads(msg)
                        # ── INJECT received IMU frame into shared state ──
                        with _state_lock:
                            _imu_state    = data
                            _imu_ts_last  = time.time()
                    except json.JSONDecodeError as e:
                        logger.warning("IMU WS: invalid JSON: %s", e)
        except (websockets.ConnectionClosed, OSError) as e:
            logger.warning("IMU WS: disconnected (%s), retry in %.1fs", e, RECONNECT_DELAY_S)
        except Exception as e:
            logger.error("IMU WS: unexpected error: %s", e)
        finally:
            _imu_connected = False

        await asyncio.sleep(RECONNECT_DELAY_S)


# ****************
# **************** DATA INPUT: RTK WebSocket → _rtk_state (8776 → this process)
# ****************

async def connect_rtk_ws(uri: str) -> None:
    """
    Persistent async client for the RTK WebSocket server.

    On each received message:
      JSON frame → _rtk_state (raw RTK data, used by nav_loop)

    On disconnect: waits RECONNECT_DELAY_S then retries indefinitely.
    Degradation: if this coroutine fails, navigation pauses (no position).

    Expected incoming frame keys (from rtk_bridge.py):
      lat, lon, alt         — WGS-84 position
      fix_quality           — 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
      num_sats, hdop        — satellite count and dilution of precision
      speed_knots           — ground speed [knots]
      track_deg             — true course over ground [degrees, 0=North]
      source                — "rtk" or "default" (fallback position)
    """
    global _rtk_connected, _rtk_state, _rtk_ts_last
    while True:
        try:
            logger.info("RTK WS: connecting to %s", uri)
            async with websockets.connect(uri, ping_interval=20, ping_timeout=10) as ws:
                _rtk_connected = True
                logger.info("RTK WS: connected")
                async for msg in ws:
                    try:
                        data = json.loads(msg)
                        # ── INJECT received RTK frame into shared state ──
                        with _state_lock:
                            _rtk_state    = data
                            _rtk_ts_last  = time.time()
                    except json.JSONDecodeError as e:
                        logger.warning("RTK WS: invalid JSON: %s", e)
        except (websockets.ConnectionClosed, OSError) as e:
            logger.warning("RTK WS: disconnected (%s), retry in %.1fs", e, RECONNECT_DELAY_S)
        except Exception as e:
            logger.error("RTK WS: unexpected error: %s", e)
        finally:
            _rtk_connected = False

        await asyncio.sleep(RECONNECT_DELAY_S)


# **************** PD CONTROLLER ****************

def _compute_command(
    current_heading: float,
    target_bearing: float,
    distance_m: float,
) -> tuple[float, float, str]:
    """
    Compute velocity command using PD heading control.

    Algorithm:
      heading_error = normalize(target_bearing - current_heading)  [-180, 180]
      ang_rate = Kp * error + Kd * d(error)/dt
      if |error| > TURN_THRESHOLD_DEG: rotate in place (speed = 0)
      else: forward with proportional slowdown near waypoint

    Args:
      current_heading  — robot compass heading [0, 360) degrees
      target_bearing   — bearing to next waypoint [0, 360) degrees
      distance_m       — distance to next waypoint [metres]

    Returns:
      speed_ms        — linear velocity [m/s], >= 0
      ang_rate_rads   — angular velocity [rad/s], left positive
      nav_state_str   — "TURNING" | "NAVIGATING"
    """
    global _last_heading_error

    heading_error = normalize_angle(target_bearing - current_heading)
    d_error = heading_error - _last_heading_error
    _last_heading_error = heading_error

    ang_rate = KP_ANG * heading_error + KD_ANG * d_error
    ang_rate = max(-MAX_ANG_RATE_RADS, min(MAX_ANG_RATE_RADS, ang_rate))

    if abs(heading_error) > TURN_THRESHOLD_DEG:
        # Heading too far off — rotate in place, no forward motion
        return 0.0, ang_rate, "TURNING"

    # Within acceptable heading band — move forward with deceleration ramp
    speed_factor = min(1.0, distance_m / SLOWDOWN_DIST_M)
    speed = MAX_SPEED_MS * speed_factor
    speed = max(0.05, speed)  # minimum creep speed to avoid stall

    return speed, ang_rate, "NAVIGATING"


# **************** NAVIGATION LOOP (core logic, 10 Hz) ****************

async def nav_loop() -> None:
    """
    Main navigation coroutine — runs at NAV_LOOP_HZ (default 10 Hz).

    Each tick:
      1. Snapshot IMU + RTK state (thread-safe copy)
      2. Extract heading: IMU quaternion yaw (priority) → RTK track_deg (fallback)
      3. Extract position / speed from RTK
      4. Classify motion state (STOPPED / MOVING / TURNING)
      5. If nav active + position valid: compute distance/bearing to waypoint
         a. If within ARRIVAL_RADIUS_M → advance to next waypoint
         b. Else → _compute_command() → V command string
      6. Send V command to robot (if serial connected)
      7. Read robot O feedback
      8. Assemble _fused payload → broadcast_loop picks it up
    """
    global _wp_index, _nav_active, _fused, _last_heading_error

    period = 1.0 / NAV_LOOP_HZ

    while True:
        now = time.time()

        # ── Step 1: Snapshot shared state (thread-safe) ───
        with _state_lock:
            imu       = dict(_imu_state)
            rtk       = dict(_rtk_state)
            imu_fresh = (now - _imu_ts_last) < DATA_STALE_S
            rtk_fresh = (now - _rtk_ts_last) < DATA_STALE_S

        # ── Step 2: Heading fusion ─────────────────────────
        #    Priority: IMU quaternion yaw (absolute, mag-referenced)
        #    Fallback: RTK track_deg (only valid when moving)
        heading_deg = None
        imu_cal     = 0

        if imu_fresh and imu:
            try:
                rot        = imu.get("rot", {})
                qi         = rot.get("qi", 0.0)
                qj         = rot.get("qj", 0.0)
                qk         = rot.get("qk", 0.0)
                qr         = rot.get("qr", 1.0)
                yaw_deg    = quat_to_yaw(qi, qj, qk, qr)
                heading_deg = yaw_to_compass_heading(yaw_deg)
                imu_cal    = imu.get("cal", 0)
            except Exception as e:
                logger.warning("nav_loop: IMU heading extraction failed: %s", e)

        if heading_deg is None and rtk_fresh and rtk:
            heading_deg = rtk.get("track_deg")   # degrees true north

        # ── Step 3: RTK position and speed ────────────────
        lat         = rtk.get("lat")          if rtk_fresh else None
        lon         = rtk.get("lon")          if rtk_fresh else None
        alt         = rtk.get("alt")
        fix_quality = rtk.get("fix_quality", 0)
        speed_knots = rtk.get("speed_knots")
        track_deg   = rtk.get("track_deg")
        speed_ms    = (speed_knots * 0.5144)  if speed_knots is not None else 0.0

        # ── Step 4: Motion state classification ───────────
        SPEED_THRESHOLD   = 0.1   # m/s — below this → not translating
        ANG_THRESHOLD_DPS = 5.0   # deg/s — above this → rotating

        gyro_z = 0.0
        if imu_fresh and imu:
            gyro   = imu.get("gyro", {})
            gyro_z = abs(math.degrees(gyro.get("z", 0.0)))

        if speed_ms > SPEED_THRESHOLD:
            motion = "MOVING"
        elif gyro_z > ANG_THRESHOLD_DPS:
            motion = "TURNING"
        else:
            motion = "STOPPED"

        # ── Step 5: Waypoint navigation planning ──────────
        nav_info: dict = {
            "active":            _nav_active,
            "waypoint_index":    _wp_index,
            "waypoints":         _waypoints,
            "distance_m":        None,
            "bearing_deg":       None,
            "heading_error_deg": None,
        }

        # Default: issue stop command until overridden below
        cmd_info: dict = {
            "v_cmd":             CMD_STOP_STR,
            "speed_ms":          0.0,
            "ang_rate_rads":     0.0,
            "heading_error_deg": 0.0,
            "distance_m":        None,
            "target_waypoint":   _wp_index,
            "total_waypoints":   len(_waypoints),
            "nav_state":         "IDLE",
        }

        if _nav_active and lat is not None and lon is not None and heading_deg is not None:
            if _wp_index < len(_waypoints):
                wp   = _waypoints[_wp_index]
                dist = haversine_m(lat, lon, wp["lat"], wp["lon"])
                brg  = bearing_deg(lat, lon, wp["lat"], wp["lon"])
                h_err = normalize_angle(brg - heading_deg)

                nav_info["distance_m"]        = round(dist, 2)
                nav_info["bearing_deg"]        = round(brg,  2)
                nav_info["heading_error_deg"]  = round(h_err, 2)

                cmd_info["distance_m"]        = round(dist, 2)
                cmd_info["heading_error_deg"] = round(h_err, 2)
                cmd_info["target_waypoint"]   = _wp_index
                cmd_info["total_waypoints"]   = len(_waypoints)

                if dist < ARRIVAL_RADIUS_M:
                    # Waypoint reached — advance index
                    logger.info(
                        "nav_loop: waypoint %d reached (dist=%.2f m)", _wp_index, dist
                    )
                    _wp_index += 1
                    _last_heading_error = 0.0

                    if _wp_index >= len(_waypoints):
                        logger.info("nav_loop: all waypoints reached, navigation complete")
                        _nav_active = False

                    cmd_info["nav_state"] = "ARRIVED"

                else:
                    # Compute motion command toward waypoint
                    spd, ang, nav_st = _compute_command(heading_deg, brg, dist)
                    v_cmd = f"V{spd:.2f},{ang:.3f}\n"
                    cmd_info.update({
                        "v_cmd":          v_cmd,
                        "speed_ms":       round(spd, 3),
                        "ang_rate_rads":  round(ang, 4),
                        "nav_state":      nav_st,
                    })
            else:
                _nav_active = False
                cmd_info["nav_state"] = "ARRIVED"

        # nav_state defaults to "IDLE" when !_nav_active or no position available

        # ── Step 6: Send V command to robot ───────────────
        if _robot is not None:
            _robot.send_velocity(cmd_info["speed_ms"], cmd_info["ang_rate_rads"])

        # ── Step 7: Read robot O feedback ─────────────────
        robot_info: dict = {
            "meas_speed_ms":      0.0,
            "meas_ang_rate_rads": 0.0,
            "state":              0,
            "state_name":         "UNKNOWN",
            "soc":                0,
            "connected":          False,
        }
        if _robot is not None:
            robot_info = _robot.get_feedback()

        # ── Step 8: Assemble _fused payload ───────────────
        payload = {
            "ts": now,
            "state": {
                "lat":         lat,
                "lon":         lon,
                "alt":         alt,
                "heading_deg": round(heading_deg, 2) if heading_deg is not None else None,
                "speed_ms":    round(speed_ms, 3),
                "fix_quality": fix_quality,
                "track_deg":   track_deg,
                "motion":      motion,
                "imu_cal":     imu_cal,
            },
            "imu_raw": {
                "euler":     imu.get("euler",     {}),
                "rot":       imu.get("rot",       {}),
                "lin_accel": imu.get("lin_accel", {}),
                "gyro":      imu.get("gyro",      {}),
                "mag":       imu.get("mag",       {}),
                "cal":       imu_cal,
                "hz":        imu.get("hz", 0),
            } if imu_fresh and imu else {},
            "rtk_raw": {
                "lat":         lat,
                "lon":         lon,
                "alt":         alt,
                "fix_quality": fix_quality,
                "num_sats":    rtk.get("num_sats", 0),
                "hdop":        rtk.get("hdop"),
                "speed_knots": speed_knots,
                "speed_ms":    round(speed_ms, 3),
                "track_deg":   track_deg,
                "source":      rtk.get("source", "unknown"),
            } if rtk_fresh and rtk else {},
            "nav":    nav_info,
            "cmd":    cmd_info,
            "robot":  robot_info,
            "source": {
                "imu":   imu_fresh and bool(imu),
                "rtk":   rtk_fresh and bool(rtk),
                "robot": robot_info.get("connected", False),
            },
        }

        _fused = payload   # broadcast_loop will send this to all web clients

        await asyncio.sleep(period)


# ****************
# **************** DATA INPUT: Web client → nav_bridge (commands from browser)
# ****************

async def ws_handler(websocket) -> None:
    """
    Handle a new WebSocket connection from the web client (browser).

    Registers the connection in _ws_clients so broadcast_loop can push data.
    Passes every incoming message to _handle_client_message().
    Cleans up on disconnect.
    """
    addr = websocket.remote_address
    logger.info("Nav WS: client connected: %s", addr)
    _ws_clients.add(websocket)
    try:
        async for msg in websocket:
            await _handle_client_message(msg, websocket)
    except websockets.ConnectionClosed:
        pass
    except Exception as e:
        logger.error("Nav WS: handler error for %s: %s", addr, e)
    finally:
        _ws_clients.discard(websocket)
        logger.info("Nav WS: client disconnected: %s", addr)


async def _handle_client_message(msg: str, ws) -> None:
    """
    Dispatch incoming JSON commands from the browser.

    Supported message types and their effect on navigation state:
      set_waypoints   → replace _waypoints list, reset index
      start_nav       → set _nav_active = True, reset index
      stop_nav        → set _nav_active = False (robot stops on next tick)
      clear_waypoints → empty _waypoints, stop navigation

    Expected JSON format:
      {"type": "set_waypoints", "waypoints": [{"lat": ..., "lon": ...}, ...]}
      {"type": "start_nav"}
      {"type": "stop_nav"}
      {"type": "clear_waypoints"}
    """
    global _waypoints, _wp_index, _nav_active, _last_heading_error

    try:
        data = json.loads(msg)
    except json.JSONDecodeError as e:
        logger.warning("Nav WS: invalid JSON from client: %s", e)
        return

    msg_type = data.get("type", "")

    if msg_type == "set_waypoints":
        wps        = data.get("waypoints", [])
        _waypoints = [{"lat": float(w["lat"]), "lon": float(w["lon"])} for w in wps]
        _wp_index  = 0
        _last_heading_error = 0.0
        logger.info("Nav WS: set_waypoints: %d waypoints", len(_waypoints))

    elif msg_type == "start_nav":
        if not _waypoints:
            logger.warning("Nav WS: start_nav called with no waypoints")
        else:
            _nav_active = True
            _wp_index   = 0
            _last_heading_error = 0.0
            logger.info("Nav WS: navigation started (%d waypoints)", len(_waypoints))

    elif msg_type == "stop_nav":
        _nav_active = False
        logger.info("Nav WS: navigation stopped by client")

    elif msg_type == "clear_waypoints":
        _waypoints  = []
        _wp_index   = 0
        _nav_active = False
        _last_heading_error = 0.0
        logger.info("Nav WS: waypoints cleared")

    else:
        logger.warning("Nav WS: unknown message type: %r", msg_type)


# ****************
# **************** DATA OUTPUT: _fused → all web clients (this process → browser)
# ****************

async def broadcast_loop(hz: float) -> None:
    """
    Push the latest _fused payload to every connected web client at `hz` Hz.

    Dead connections are silently removed from _ws_clients.

    Broadcast JSON structure (see nav_loop Step 8 for full schema):
      ts       — Unix timestamp
      state    — fused position / heading / speed / motion
      imu_raw  — raw IMU fields (euler, rot, accel, gyro, mag, cal, hz)
      rtk_raw  — raw RTK fields (lat, lon, fix, sats, hdop, speed, track)
      nav      — waypoint progress (active, index, distance, bearing, error)
      cmd      — V command + nav_state (IDLE / NAVIGATING / TURNING / ARRIVED)
      robot    — O feedback (meas_speed, meas_ang_rate, state, soc, connected)
      source   — boolean flags: imu/rtk/robot data freshness
    """
    period = 1.0 / max(hz, 0.5)
    while True:
        if _ws_clients and _fused:
            msg  = json.dumps(_fused)
            dead: set = set()
            for ws in _ws_clients.copy():
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            _ws_clients.difference_update(dead)
        await asyncio.sleep(period)


# **************** HTTP SERVER (static web UI) ****************

def start_http_server(static_dir: Path, http_port: int) -> None:
    """
    Serve the nav web UI from web_static/ directory.
    Runs in a dedicated daemon thread (not the asyncio loop).
    """
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def log_message(self, fmt, *args):
            logger.debug("HTTP %s - %s", self.address_string(), fmt % args)

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", http_port), Handler) as httpd:
        logger.info("HTTP server serving %s on port %d", static_dir, http_port)
        httpd.serve_forever()


# **************** MAIN ENTRY POINT ****************

async def main_async(args: argparse.Namespace) -> None:
    """
    Bootstrap sequence:
      1. Open robot serial (optional)
      2. Start HTTP server thread
      3. Launch async tasks: IMU WS client, RTK WS client, nav_loop, broadcast_loop
      4. Start nav WebSocket server and await forever
    """
    global _robot

    # ── 1. Robot serial (optional) ────────────────────────
    if args.robot_port:
        _robot = RobotSerial(args.robot_port, args.robot_baud)
        ok = _robot.start()
        if not ok:
            logger.warning("Robot serial unavailable, continuing without robot")
            _robot = None
    else:
        logger.info("No --robot-port specified, robot serial disabled")

    # ── 2. HTTP server ────────────────────────────────────
    static_dir = Path(__file__).parent / "web_static"
    if static_dir.exists():
        t = threading.Thread(
            target=start_http_server,
            args=(static_dir, args.nav_port),
            daemon=True,
            name="nav-http",
        )
        t.start()
    else:
        logger.warning("web_static directory not found: %s", static_dir)

    ws_port = args.nav_port + 1
    logger.info("Nav WS server starting on ws://localhost:%d", ws_port)
    logger.info("Open browser at http://localhost:%d", args.nav_port)

    if args.open_browser:
        url = f"http://localhost:{args.nav_port}"
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()

    # ── 3. Launch async tasks ─────────────────────────────
    asyncio.create_task(connect_imu_ws(args.imu_ws))    # IMU subscriber
    asyncio.create_task(connect_rtk_ws(args.rtk_ws))    # RTK subscriber
    asyncio.create_task(nav_loop())                      # fusion + planning
    asyncio.create_task(broadcast_loop(args.hz))         # web push

    # ── 4. WebSocket server (accepts browser connections) ──
    async with websockets.serve(ws_handler, "0.0.0.0", ws_port):
        logger.info("Nav bridge running. Press Ctrl+C to stop.")
        await asyncio.Future()   # run forever


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Navigation fusion bridge")
    parser.add_argument(
        "--imu-ws",
        default="ws://localhost:8766",
        help="IMU WebSocket URL (default: ws://localhost:8766)",
    )
    parser.add_argument(
        "--rtk-ws",
        default="ws://localhost:8776",
        help="RTK WebSocket URL (default: ws://localhost:8776)",
    )
    parser.add_argument(
        "--robot-port",
        default="",
        help="Robot serial port (e.g. /dev/ttyUSB1). Leave empty to disable.",
    )
    parser.add_argument(
        "--robot-baud",
        type=int,
        default=115200,
        help="Robot serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--nav-port",
        type=int,
        default=8785,
        help="HTTP port for nav web UI (WebSocket uses nav-port+1, default: 8785/8786)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Broadcast rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--open-browser",
        action="store_true",
        default=True,
        help="Auto-open browser after startup",
    )
    return parser.parse_args()


# **************** SCRIPT ENTRY ****************

if __name__ == "__main__":
    args = parse_args()
    logger.info(
        "Starting nav bridge | IMU=%s | RTK=%s | robot=%s | http=:%d | ws=:%d | hz=%.1f",
        args.imu_ws,
        args.rtk_ws,
        args.robot_port or "disabled",
        args.nav_port,
        args.nav_port + 1,
        args.hz,
    )
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("Nav bridge stopped by user.")
