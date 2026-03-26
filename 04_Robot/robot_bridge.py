"""
robot_bridge.py — Farm Robot Web Controller bridge (standalone, self-contained).

Data flow:
    ESP32/OAK-D IMU → IMUReader thread → IMU data dict ─────────────────────────┐
    RTK GPS serial  → RTKReader  thread → RTK data dict ─────────────────────────┤
    Feather M4 serial ← _send_velocity() ← NavigationEngine / joystick          │
                      → odometry lines  → _last_odom dict ──────────────────────┤
                                                                                  ↓
                                          asyncio.gather()
                                            ├─ _imu_broadcast_loop()  20Hz
                                            ├─ _rtk_broadcast_loop()   1Hz
                                            ├─ _odom_broadcast_loop() 20Hz
                                            ├─ _data_record_loop()     5Hz
                                            ├─ _status_broadcast_loop()  2Hz
                                            ├─ _watchdog_loop()         0.5Hz
                                            └─ websockets.serve() :ws_port+1
                                                  ↕ JSON messages
                                               Browser (joystick, nav, record)

    HttpFileServer :ws_port → web_static/ (index.html, app.js, style.css)

Usage:
    python robot_bridge.py --ws-port 8888 --serial-port /dev/ttyACM0
    # Browser:    http://localhost:8888
    # WebSocket:  ws://localhost:8889
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import io
import json
import logging
import math
import os
import platform
import socketserver
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime
from enum import Enum, auto
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Coroutine, List, Optional, Tuple

import numpy as np
import serial
import websockets


# ── Logger setup ─────────────────────────────────────────────────────────────

def _setup_logger() -> logging.Logger:
    """Configure module-level logger; output to robot_bridge.log and stderr."""
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
# BLOCK 1 — CONFIGURATION
# ═════════════════════════════════════════════════════════════════════════════

def _default_serial_port() -> str:
    return "/dev/cu.usbmodem11301" if platform.system() == "Darwin" else "/dev/ttyACM0"


# All defaults; overridden by CLI args or env vars at startup.
_CFG: dict = {}   # populated by _parse_args() at the bottom


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE
# ═════════════════════════════════════════════════════════════════════════════

# ── 2a. Geo utils ─────────────────────────────────────────────────────────────

_EARTH_RADIUS_M = 6_371_000.0


def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance (metres) between two WGS-84 points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    a = (
        math.sin((phi2 - phi1) / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    )
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(a))


def _bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """True-north bearing (degrees, 0–360) from point 1 to point 2."""
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
         - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlam))
    return math.degrees(math.atan2(x, y)) % 360.0


def _norm_angle(angle_deg: float) -> float:
    """Normalize angle to (-180, 180]."""
    a = angle_deg % 360.0
    return a - 360.0 if a > 180.0 else a


def _project_on_segment(
    p_lat: float, p_lon: float,
    a_lat: float, a_lon: float,
    b_lat: float, b_lon: float,
) -> Tuple[float, float, float]:
    """Project point P onto segment A→B; return (proj_lat, proj_lon, t)."""
    cos_lat = math.cos(math.radians(a_lat))

    def _local(lat: float, lon: float) -> Tuple[float, float]:
        return (math.radians(lat - a_lat) * _EARTH_RADIUS_M,
                math.radians(lon - a_lon) * _EARTH_RADIUS_M * cos_lat)

    py, px = _local(p_lat, p_lon)
    by, bx = _local(b_lat, b_lon)
    ab_sq = bx * bx + by * by
    if ab_sq < 1e-12:
        return a_lat, a_lon, 0.0
    t = max(0.0, min(1.0, (px * bx + py * by) / ab_sq))
    proj_lat = a_lat + math.degrees((t * by) / _EARTH_RADIUS_M)
    proj_lon = a_lon + math.degrees((t * bx) / (_EARTH_RADIUS_M * cos_lat + 1e-12))
    return proj_lat, proj_lon, t


# ── 2b. Waypoint ──────────────────────────────────────────────────────────────

@dataclass
class Waypoint:
    id: int
    lat: float
    lon: float
    tolerance_m: float
    max_speed: float


class WaypointManager:
    """Manages the waypoint sequence and arrival logic."""

    def __init__(self, arrive_frames: int = 5) -> None:
        self._arrive_frames = arrive_frames
        self._waypoints: list[Waypoint] = []
        self._idx = 0
        self._arrive_count = 0

    def load_csv(self, csv_text: str) -> int:
        self._waypoints = []
        self._idx = 0
        self._arrive_count = 0
        lines = csv_text.strip().splitlines()
        for raw in lines[1:]:
            raw = raw.strip()
            if not raw:
                continue
            try:
                parts = [p.strip() for p in raw.split(",")]
                if len(parts) < 5:
                    raise ValueError(f"expected 5 columns, got {len(parts)}")
                self._waypoints.append(Waypoint(
                    id=int(parts[0]), lat=float(parts[1]), lon=float(parts[2]),
                    tolerance_m=float(parts[3]), max_speed=float(parts[4]),
                ))
            except (ValueError, IndexError) as e:
                logger.warning("WaypointManager: skipping invalid row %r: %s", raw, e)
        count = len(self._waypoints)
        logger.info("WaypointManager: loaded %d waypoints", count)
        return count

    @property
    def current(self) -> Optional[Waypoint]:
        return self._waypoints[self._idx] if 0 <= self._idx < len(self._waypoints) else None

    @property
    def is_finished(self) -> bool:
        return self._idx >= len(self._waypoints)

    @property
    def progress(self) -> Tuple[int, int]:
        total = len(self._waypoints)
        return min(self._idx + 1, total), total

    @property
    def waypoints(self) -> list[Waypoint]:
        return list(self._waypoints)

    @property
    def current_index(self) -> int:
        return self._idx

    def update(self, distance_m: float, fix_quality: int) -> bool:
        wp = self.current
        if wp is None:
            return False
        tolerance = min(wp.tolerance_m, 0.5) if fix_quality == 4 else (2.0 if fix_quality == 5 else wp.tolerance_m)
        if distance_m < tolerance:
            self._arrive_count += 1
            if self._arrive_count >= self._arrive_frames:
                logger.info("WaypointManager: reached wp %d (dist=%.2fm)", wp.id, distance_m)
                self._idx += 1
                self._arrive_count = 0
                return True
        else:
            self._arrive_count = 0
        return False

    def reset(self) -> None:
        self._idx = 0
        self._arrive_count = 0


# ── 2c. GPS filters ───────────────────────────────────────────────────────────

class MovingAverageFilter:
    def __init__(self, window: int = 10) -> None:
        self._lat_buf: deque = deque(maxlen=window)
        self._lon_buf: deque = deque(maxlen=window)
        self._window = window

    def update(self, lat: float, lon: float) -> Tuple[float, float]:
        self._lat_buf.append(lat)
        self._lon_buf.append(lon)
        return sum(self._lat_buf) / len(self._lat_buf), sum(self._lon_buf) / len(self._lon_buf)

    def get_position(self) -> Tuple[float, float]:
        if not self._lat_buf:
            return 0.0, 0.0
        return sum(self._lat_buf) / len(self._lat_buf), sum(self._lon_buf) / len(self._lon_buf)

    def reset(self) -> None:
        self._lat_buf.clear()
        self._lon_buf.clear()

    @property
    def is_ready(self) -> bool:
        return len(self._lat_buf) >= self._window


class KalmanFilter:
    """4-state Kalman filter (position + velocity) for GPS smoothing."""

    def __init__(self, process_noise_std: float = 0.1, gps_noise_std: float = 2.0) -> None:
        self._pn_std = process_noise_std
        self._gps_std = gps_noise_std
        self._origin_lat = self._origin_lon = self._cos_lat = 0.0
        self._initialized = False
        self._x = np.zeros(4)
        self._P = np.eye(4) * 100.0
        self._H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)

    def init(self, lat: float, lon: float) -> None:
        self._origin_lat, self._origin_lon = lat, lon
        self._cos_lat = math.cos(math.radians(lat))
        self._x = np.zeros(4)
        self._P = np.eye(4) * 10.0
        self._initialized = True

    def predict(self, dt: float, a_north: float = 0.0, a_east: float = 0.0) -> None:
        if not self._initialized:
            return
        F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]], dtype=float)
        B = np.array([[.5*dt**2,0],[0,.5*dt**2],[dt,0],[0,dt]], dtype=float)
        q = self._pn_std ** 2
        Q = np.diag([q*dt**2, q*dt**2, q, q])
        self._x = F @ self._x + B @ np.array([a_north, a_east])
        self._P = F @ self._P @ F.T + Q

    def update(self, lat: float, lon: float, fix_quality: int = 1) -> Tuple[float, float]:
        if not self._initialized:
            self.init(lat, lon)
            return lat, lon
        r_std = {4: 0.03, 5: 0.5, 2: 1.0}.get(fix_quality, self._gps_std)
        R = np.eye(2) * r_std ** 2
        z = np.array([
            math.radians(lat - self._origin_lat) * _EARTH_RADIUS_M,
            math.radians(lon - self._origin_lon) * _EARTH_RADIUS_M * self._cos_lat,
        ])
        S = self._H @ self._P @ self._H.T + R
        K = self._P @ self._H.T @ np.linalg.inv(S)
        self._x += K @ (z - self._H @ self._x)
        self._P = (np.eye(4) - K @ self._H) @ self._P
        return self.get_position()

    def update_velocity(self, v_north: float, v_east: float) -> None:
        if not self._initialized:
            return
        z = np.array([v_north, v_east])
        H = np.array([[0,0,1,0],[0,0,0,1]], dtype=float)
        R_vel = np.eye(2) * 0.05
        y = z - H @ self._x
        S = H @ self._P @ H.T + R_vel
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x += K @ y
        self._P = (np.eye(4) - K @ H) @ self._P

    def get_position(self) -> Tuple[float, float]:
        if not self._initialized:
            return 0.0, 0.0
        return (
            self._origin_lat + math.degrees(self._x[0] / _EARTH_RADIUS_M),
            self._origin_lon + math.degrees(self._x[1] / (_EARTH_RADIUS_M * self._cos_lat + 1e-12)),
        )

    def reset(self) -> None:
        self._initialized = False
        self._x = np.zeros(4)
        self._P = np.eye(4) * 100.0

    @property
    def is_ready(self) -> bool:
        return self._initialized


# ── 2d. Controllers ───────────────────────────────────────────────────────────

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float = 30.0, output_limit: float = 1.0) -> None:
        self._kp, self._ki, self._kd = kp, ki, kd
        self._int_lim, self._out_lim = integral_limit, output_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self._integral = max(-self._int_lim, min(self._int_lim, self._integral + error * dt))
        derivative = 0.0 if self._first else (error - self._prev_error) / dt
        self._first = False
        self._prev_error = error
        out = self._kp * error + self._ki * self._integral + self._kd * derivative
        return max(-self._out_lim, min(self._out_lim, out))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True


class P2PController:
    def __init__(self, kp: float, ki: float, kd: float,
                 decel_radius: float, max_linear: float, max_angular: float) -> None:
        self._pid = PIDController(kp, ki, kd, output_limit=max_angular)
        self._decel_radius = decel_radius
        self._max_linear = max_linear

    def compute(self, robot_lat: float, robot_lon: float, robot_bearing: float,
                target: Waypoint, dt: float) -> Tuple[float, float]:
        dist = _haversine(robot_lat, robot_lon, target.lat, target.lon)
        target_bearing = _bearing_to(robot_lat, robot_lon, target.lat, target.lon)
        err = _norm_angle(target_bearing - robot_bearing)
        angular = self._pid.compute(err, dt)
        speed_f = min(1.0, dist / (self._decel_radius + 1e-6))
        head_f = max(0.0, 1.0 - abs(err) / 90.0)
        linear = min(target.max_speed, self._max_linear) * speed_f * head_f
        return linear, angular

    def reset(self) -> None:
        self._pid.reset()


class PurePursuitController:
    def __init__(self, kp: float, ki: float, kd: float,
                 decel_radius: float, max_linear: float, max_angular: float,
                 lookahead: float) -> None:
        self._p2p = P2PController(kp, ki, kd, decel_radius, max_linear, max_angular)
        self._lookahead = lookahead

    def compute(self, robot_lat: float, robot_lon: float, robot_bearing: float,
                waypoints: list, current_idx: int, dt: float) -> Tuple[float, float]:
        if current_idx >= len(waypoints):
            return 0.0, 0.0
        current_wp = waypoints[current_idx]
        if current_idx == 0 or len(waypoints) < 2:
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)
        prev_wp = waypoints[current_idx - 1]
        try:
            proj_lat, proj_lon, _ = _project_on_segment(
                robot_lat, robot_lon, prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)
        except Exception:
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)
        seg_bearing = _bearing_to(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)
        dist_to_wp = _haversine(proj_lat, proj_lon, current_wp.lat, current_wp.lon)
        if dist_to_wp < self._lookahead:
            lookahead_lat, lookahead_lon = current_wp.lat, current_wp.lon
        else:
            cos_lat = math.cos(math.radians(proj_lat))
            rad = math.radians(seg_bearing)
            lookahead_lat = proj_lat + math.degrees(self._lookahead * math.cos(rad) / _EARTH_RADIUS_M)
            lookahead_lon = proj_lon + math.degrees(
                self._lookahead * math.sin(rad) / (_EARTH_RADIUS_M * cos_lat + 1e-12))
        return self._p2p.compute(robot_lat, robot_lon, robot_bearing,
                                 replace(current_wp, lat=lookahead_lat, lon=lookahead_lon), dt)

    def reset(self) -> None:
        self._p2p.reset()


# ── 2e. Navigation engine ─────────────────────────────────────────────────────

class NavState(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    FINISHED   = auto()


class NavMode(Enum):
    P2P          = "p2p"
    PURE_PURSUIT = "pure_pursuit"


class FilterMode(Enum):
    MOVING_AVG = "moving_avg"
    KALMAN     = "kalman"


class NavigationEngine:
    """GPS + IMU autonomous path navigation state machine."""

    def __init__(
        self,
        send_velocity_fn: Callable[[float, float], None],
        broadcast_fn: Callable[[dict], Coroutine],
        loop: asyncio.AbstractEventLoop,
        max_linear: float,
        max_angular: float,
        gps_timeout: float,
        arrive_frames: int,
        kp: float, ki: float, kd: float,
        decel_radius: float,
        lookahead: float,
        ma_window: int,
        compass_min_accuracy: int,
    ) -> None:
        self._send_velocity = send_velocity_fn
        self._broadcast     = broadcast_fn
        self._loop          = loop
        self._max_linear    = max_linear
        self._max_angular   = max_angular
        self._gps_timeout   = gps_timeout
        self._compass_min_accuracy = compass_min_accuracy

        self._lock = threading.Lock()
        self._state:       NavState   = NavState.IDLE
        self._nav_mode:    NavMode    = NavMode.P2P
        self._filter_mode: FilterMode = FilterMode.MOVING_AVG

        self._wp_mgr    = WaypointManager(arrive_frames=arrive_frames)
        self._ma_filter = MovingAverageFilter(window=ma_window)
        self._kf_filter = KalmanFilter()
        self._p2p_ctrl  = P2PController(kp, ki, kd, decel_radius, max_linear, max_angular)
        self._pp_ctrl   = PurePursuitController(kp, ki, kd, decel_radius, max_linear, max_angular, lookahead)

        self._robot_bearing: float | None = None
        self._last_imu_ts    = 0.0
        self._last_control_ts = 0.0
        self._fix_quality    = 0
        self._last_gps_ts    = 0.0
        self._gps_warning_sent = False
        self._broadcast_counter = 0
        self._last_uncal_warn_ts = 0.0
        self._last_diag_ts       = 0.0
        self._last_status_log_ts = 0.0

    # ── Public API ────────────────────────────────────────────────────────────

    def load_waypoints(self, csv_text: str) -> int:
        with self._lock:
            return self._wp_mgr.load_csv(csv_text)

    def start(self, force: bool = False) -> bool:
        with self._lock:
            if self._state == NavState.NAVIGATING:
                return False
            if self._wp_mgr.current is None:
                return False
            if self._fix_quality < 1 and not force:
                return False
            self._wp_mgr.reset()
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
            self._gps_warning_sent = False
            self._state = NavState.NAVIGATING
            self._last_control_ts = time.time()
        self._schedule_broadcast()
        return True

    def stop(self) -> None:
        with self._lock:
            if self._state == NavState.IDLE:
                return
            self._state = NavState.IDLE
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
        self._send_velocity(0.0, 0.0)
        self._schedule_broadcast()

    def set_nav_mode(self, mode: NavMode) -> None:
        with self._lock:
            self._nav_mode = mode
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()

    def set_filter_mode(self, mode: FilterMode) -> None:
        with self._lock:
            self._filter_mode = mode

    def get_status(self) -> dict:
        with self._lock:
            wp   = self._wp_mgr.current
            prog = self._wp_mgr.progress
            distance_m = target_bearing = tolerance_m = None
            if wp is not None:
                filt = self._ma_filter if self._filter_mode == FilterMode.MOVING_AVG else self._kf_filter
                pos = filt.get_position()
                if pos != (0.0, 0.0):
                    distance_m     = _haversine(pos[0], pos[1], wp.lat, wp.lon)
                    target_bearing = _bearing_to(pos[0], pos[1], wp.lat, wp.lon)
                    tolerance_m    = wp.tolerance_m
            return {
                "type":           "nav_status",
                "state":          self._state.name.lower(),
                "progress":       list(prog),
                "distance_m":     round(distance_m, 2) if distance_m is not None else None,
                "target_bearing": round(target_bearing, 1) if target_bearing is not None else None,
                "nav_mode":       self._nav_mode.value,
                "filter_mode":    self._filter_mode.value,
                "tolerance_m":    tolerance_m,
                "fix_quality":    self._fix_quality,
            }

    def on_imu(self, imu_data: dict) -> None:
        try:
            compass    = imu_data.get("compass", {})
            calibrated = compass.get("calibrated", False)
            accuracy   = compass.get("accuracy", 0)
            if not calibrated and accuracy < self._compass_min_accuracy:
                now = time.time()
                with self._lock:
                    if (self._state == NavState.NAVIGATING
                            and now - self._last_uncal_warn_ts >= 5.0):
                        self._last_uncal_warn_ts = now
                        logger.warning("NavigationEngine: compass not calibrated (acc=%d/3)", accuracy)
                return

            bearing = compass.get("bearing", 0.0)
            now = time.time()
            dt  = now - self._last_imu_ts if self._last_imu_ts > 0 else 0.05
            self._last_imu_ts = now

            with self._lock:
                self._robot_bearing = bearing
                if self._filter_mode == FilterMode.KALMAN and self._kf_filter.is_ready:
                    accel = imu_data.get("accel", {})
                    ax, ay = accel.get("x", 0.0), accel.get("y", 0.0)
                    rad = math.radians(bearing)
                    self._kf_filter.predict(dt,
                                            ax * math.cos(rad) - ay * math.sin(rad),
                                            ax * math.sin(rad) + ay * math.cos(rad))
                if self._state != NavState.NAVIGATING:
                    return

            self._control_step(now)
        except Exception as e:
            logger.error("NavigationEngine.on_imu: %s", e)

    def on_rtk(self, rtk_data: dict) -> None:
        try:
            lat = rtk_data.get("lat")
            lon = rtk_data.get("lon")
            fq  = rtk_data.get("fix_quality", 0)
            with self._lock:
                self._fix_quality = fq
            if lat is None or lon is None or fq < 1:
                return
            with self._lock:
                self._last_gps_ts      = time.time()
                self._gps_warning_sent = False
                if self._filter_mode == FilterMode.MOVING_AVG:
                    self._ma_filter.update(lat, lon)
                else:
                    self._kf_filter.update(lat, lon, fq)
        except Exception as e:
            logger.error("NavigationEngine.on_rtk: %s", e)

    def on_odometry(self, v_linear: float, v_angular: float) -> None:
        try:
            with self._lock:
                if self._filter_mode != FilterMode.KALMAN or not self._kf_filter.is_ready:
                    return
                if self._robot_bearing is None:
                    return
                rad = math.radians(self._robot_bearing)
                self._kf_filter.update_velocity(v_linear * math.cos(rad), v_linear * math.sin(rad))
        except Exception as e:
            logger.error("NavigationEngine.on_odometry: %s", e)

    # ── Control step ──────────────────────────────────────────────────────────

    def _control_step(self, now: float) -> None:
        with self._lock:
            if self._state != NavState.NAVIGATING:
                return
            if self._last_gps_ts > 0 and (now - self._last_gps_ts) > self._gps_timeout:
                if not self._gps_warning_sent:
                    self._gps_warning_sent = True
                    logger.warning("NavigationEngine: GPS timeout — stopping")
                    self._send_velocity(0.0, 0.0)
                    self._schedule_broadcast_unsafe({"type": "nav_warning", "msg": "GPS timeout"})
                return
            if self._gps_warning_sent:
                return

            filt = self._ma_filter if self._filter_mode == FilterMode.MOVING_AVG else self._kf_filter
            if not filt.is_ready:
                if now - self._last_diag_ts >= 5.0:
                    self._last_diag_ts = now
                    logger.warning("NavigationEngine: waiting for GPS filter readiness")
                return

            pos = filt.get_position()
            robot_lat, robot_lon = pos

            if self._robot_bearing is None:
                if now - self._last_diag_ts >= 5.0:
                    self._last_diag_ts = now
                    logger.warning("NavigationEngine: no compass bearing — paused")
                return

            wp = self._wp_mgr.current
            if wp is None:
                return

            dt = now - self._last_control_ts
            self._last_control_ts = now
            if dt <= 0 or dt > 1.0:
                dt = 0.05

            dist = _haversine(robot_lat, robot_lon, wp.lat, wp.lon)
            switched = self._wp_mgr.update(dist, self._fix_quality)

            if switched:
                self._p2p_ctrl.reset()
                self._pp_ctrl.reset()
                if self._wp_mgr.is_finished:
                    self._state = NavState.FINISHED
                    total = self._wp_mgr.progress[1]
                    self._send_velocity(0.0, 0.0)
                    self._schedule_broadcast_unsafe({"type": "nav_complete", "total_wp": total})
                    self._schedule_broadcast_unsafe(self._get_status_unsafe())
                    return
                wp = self._wp_mgr.current

            bearing = self._robot_bearing
            if self._nav_mode == NavMode.PURE_PURSUIT:
                linear, angular = self._pp_ctrl.compute(
                    robot_lat, robot_lon, bearing,
                    self._wp_mgr.waypoints, self._wp_mgr.current_index, dt)
            else:
                linear, angular = self._p2p_ctrl.compute(
                    robot_lat, robot_lon, bearing, wp, dt)

            linear  = max(-self._max_linear,  min(self._max_linear,  linear))
            angular = max(-self._max_angular, min(self._max_angular, angular))

        self._send_velocity(linear, angular)

        self._broadcast_counter += 1
        if self._broadcast_counter % 5 == 0:
            self._schedule_broadcast()

    def _schedule_broadcast(self) -> None:
        asyncio.run_coroutine_threadsafe(self._broadcast(self.get_status()), self._loop)

    def _schedule_broadcast_unsafe(self, msg: dict) -> None:
        asyncio.run_coroutine_threadsafe(self._broadcast(msg), self._loop)

    def _get_status_unsafe(self) -> dict:
        wp   = self._wp_mgr.current
        prog = self._wp_mgr.progress
        return {
            "type": "nav_status", "state": self._state.name.lower(),
            "progress": list(prog), "distance_m": None, "target_bearing": None,
            "nav_mode": self._nav_mode.value, "filter_mode": self._filter_mode.value,
            "tolerance_m": wp.tolerance_m if wp else None, "fix_quality": self._fix_quality,
        }


# ── 2f. Coverage planner ──────────────────────────────────────────────────────

class CoveragePlanner:
    """Boustrophedon (lawnmower) coverage path generator."""

    def __init__(self, boundary: List[Tuple[float, float]],
                 row_spacing: float = 1.0, direction_deg: float = 0.0,
                 overlap: float = 0.0, tolerance_m: float = 1.0,
                 max_speed: float = 0.5) -> None:
        if len(boundary) < 3:
            raise ValueError("boundary needs at least 3 vertices")
        self._boundary    = list(boundary)
        self._row_spacing = max(0.05, row_spacing)
        self._direction   = direction_deg % 360.0
        self._overlap     = max(0.0, min(0.99, overlap))
        self._tolerance_m = tolerance_m
        self._max_speed   = max_speed
        lats = [p[0] for p in boundary]; lons = [p[1] for p in boundary]
        self._origin_lat = sum(lats) / len(lats)
        self._origin_lon = sum(lons) / len(lons)
        self._cos_lat    = math.cos(math.radians(self._origin_lat))

    def _to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        north = math.radians(lat - self._origin_lat) * _EARTH_RADIUS_M
        east  = math.radians(lon - self._origin_lon) * _EARTH_RADIUS_M * self._cos_lat
        return east, north

    def _from_enu(self, east_m: float, north_m: float) -> Tuple[float, float]:
        return (
            self._origin_lat + math.degrees(north_m / _EARTH_RADIUS_M),
            self._origin_lon + math.degrees(east_m / (_EARTH_RADIUS_M * self._cos_lat + 1e-12)),
        )

    def _enu_to_scan(self, east: float, north: float) -> Tuple[float, float]:
        rad = math.radians(self._direction)
        return east * math.sin(rad) + north * math.cos(rad), -east * math.cos(rad) + north * math.sin(rad)

    def _scan_to_enu(self, x: float, y: float) -> Tuple[float, float]:
        rad = math.radians(self._direction)
        return x * math.sin(rad) - y * math.cos(rad), x * math.cos(rad) + y * math.sin(rad)

    @staticmethod
    def _intersect_y(y: float, x1: float, y1: float, x2: float, y2: float) -> Optional[float]:
        if (y1 <= y < y2) or (y2 <= y < y1):
            return x1 + (y - y1) / (y2 - y1) * (x2 - x1)
        return None

    def _clip(self, y: float, polygon: list) -> list:
        xs = []
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]; x2, y2 = polygon[(i + 1) % n]
            xi = self._intersect_y(y, x1, y1, x2, y2)
            if xi is not None:
                xs.append(xi)
        return sorted(xs)

    def generate_csv(self) -> str:
        scan_pts = [self._enu_to_scan(*self._to_enu(lat, lon)) for lat, lon in self._boundary]
        ys = [p[1] for p in scan_pts]
        y_min, y_max = min(ys), max(ys)
        spacing = self._row_spacing * (1.0 - self._overlap)
        scan_ys = []
        y = y_min + self._row_spacing / 2.0
        while y <= y_max:
            scan_ys.append(y); y += spacing
        if not scan_ys:
            return "id,lat,lon,tolerance_m,max_speed\n"
        wps_scan: list = []
        reverse = False
        for sy in scan_ys:
            xs = self._clip(sy, scan_pts)
            if len(xs) < 2:
                continue
            xl, xr = xs[0], xs[-1]
            wps_scan.extend([(xr, sy), (xl, sy)] if reverse else [(xl, sy), (xr, sy)])
            reverse = not reverse
        if not wps_scan:
            return "id,lat,lon,tolerance_m,max_speed\n"
        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow(["id", "lat", "lon", "tolerance_m", "max_speed"])
        for i, (sx, sy) in enumerate(wps_scan):
            east, north = self._scan_to_enu(sx, sy)
            lat, lon = self._from_enu(east, north)
            w.writerow([i, f"{lat:.8f}", f"{lon:.8f}", self._tolerance_m, self._max_speed])
        return buf.getvalue()


# ── 2g. IMU readers ───────────────────────────────────────────────────────────

_CARDINALS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
_DEFAULT_IMU_DATA: dict = {
    "accel":   {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro":    {"x": 0.0, "y": 0.0, "z": 0.0},
    "compass": {"bearing": 0.0, "cardinal": "N", "calibrated": False, "accuracy": 0,
                "quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}},
    "ts": 0.0,
}


class ESP32IMUReader(threading.Thread):
    """Reads BNO085 roll/pitch/yaw from ESP32 over serial UART."""

    def __init__(self, port: str, baud: int,
                 compass_min_accuracy: int = 2, compass_offset_deg: float = 0.0) -> None:
        super().__init__(name="ESP32IMUReader", daemon=True)
        self._port = port
        self._baud = baud
        self._compass_min_accuracy = compass_min_accuracy
        self._compass_offset_deg   = compass_offset_deg
        self._lock = threading.Lock()
        self._data: dict = dict(_DEFAULT_IMU_DATA)
        self._available = False

    @property
    def is_available(self) -> bool:
        return self._available

    def get_data(self) -> dict:
        with self._lock:
            return dict(self._data)

    def run(self) -> None:
        logger.info("ESP32IMUReader: opening %s @ %d baud", self._port, self._baud)
        try:
            ser = serial.Serial(self._port, self._baud, timeout=2.0)
        except serial.SerialException as e:
            logger.error("ESP32IMUReader: cannot open %s: %s", self._port, e)
            return
        buf = b""
        while True:
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException as e:
                logger.error("ESP32IMUReader: read error: %s", e)
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw_bytes, buf = buf.split(b"\n", 1)
                raw = raw_bytes.decode("ascii", errors="replace").strip()
                if not raw or raw.startswith("#"):
                    continue
                self._process(raw)

    def _process(self, raw: str) -> None:
        parts = raw.strip().split(",")
        try:
            if len(parts) == 4:
                roll, pitch, yaw, accuracy = float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3])
            elif len(parts) == 3:
                roll, pitch, yaw, accuracy = float(parts[0]), float(parts[1]), float(parts[2]), -1
            else:
                return
        except ValueError:
            return
        bearing = ((90.0 - yaw) + self._compass_offset_deg) % 360.0
        cardinal = _CARDINALS[int((bearing + 22.5) / 45.0) % 8]
        eff_acc = max(0, accuracy)
        snap = {
            "accel":   {"x": 0.0, "y": 0.0, "z": 0.0},
            "gyro":    {"x": roll, "y": pitch, "z": yaw},
            "compass": {"bearing": bearing, "cardinal": cardinal,
                        "calibrated": eff_acc >= self._compass_min_accuracy,
                        "accuracy": eff_acc,
                        "quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}},
            "ts": time.time(),
        }
        with self._lock:
            self._data = snap
            if not self._available:
                self._available = True
                logger.info("ESP32IMUReader: first packet — bearing=%.1f°", bearing)


class OakDIMUReader(threading.Thread):
    """Reads BNO085 from OAK-D camera via depthai."""

    def __init__(self, compass_min_accuracy: int = 2,
                 compass_offset_deg: float = 0.0) -> None:
        super().__init__(name="OakDIMUReader", daemon=True)
        self._compass_min_accuracy = compass_min_accuracy
        self._compass_offset_deg   = compass_offset_deg
        self._lock = threading.Lock()
        self._data: dict = dict(_DEFAULT_IMU_DATA)
        self._available = False

    @property
    def is_available(self) -> bool:
        return self._available

    def get_data(self) -> dict:
        with self._lock:
            return dict(self._data)

    def run(self) -> None:
        try:
            import depthai as dai
        except ImportError:
            logger.warning("OakDIMUReader: depthai not installed — IMU unavailable")
            return
        try:
            with dai.Pipeline() as pipeline:
                imu_node = pipeline.create(dai.node.IMU)
                imu_node.enableIMUSensor(dai.IMUSensor.LINEAR_ACCELERATION, 400)
                imu_node.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
                imu_node.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
                imu_node.setBatchReportThreshold(1)
                imu_node.setMaxBatchReports(10)
                q = imu_node.out.createOutputQueue(maxSize=50, blocking=False)
                pipeline.start()
                self._available = True
                logger.info("OakDIMUReader: depthai IMU pipeline started")
                while pipeline.isRunning():
                    pkt_data = q.get()
                    if pkt_data is None:
                        continue
                    for pkt in pkt_data.packets:
                        self._process_packet(pkt)
        except Exception as e:
            logger.error("OakDIMUReader: pipeline failed: %s", e)

    def _process_packet(self, pkt) -> None:
        try:
            accel = pkt.acceleroMeter
            gyro  = pkt.gyroscope
            rot   = pkt.rotationVector
            w, xi, yj, zk = rot.real, rot.i, rot.j, rot.k
            try:
                accuracy = int(rot.accuracy)
            except Exception:
                accuracy = 0 if (w == 0.0 and xi == 0.0) else 3
            yaw_rad = math.atan2(2 * (w * zk + xi * yj), 1 - 2 * (yj * yj + zk * zk))
            bearing = (math.degrees(yaw_rad) + self._compass_offset_deg) % 360.0
            cardinal = _CARDINALS[int((bearing + 22.5) / 45.0) % 8]
            snap = {
                "accel":   {"x": accel.x, "y": accel.y, "z": accel.z},
                "gyro":    {"x": gyro.x,  "y": gyro.y,  "z": gyro.z},
                "compass": {"bearing": bearing, "cardinal": cardinal,
                            "calibrated": accuracy >= self._compass_min_accuracy,
                            "accuracy": accuracy,
                            "quat": {"w": w, "x": xi, "y": yj, "z": zk}},
                "ts": time.time(),
            }
            with self._lock:
                self._data = snap
        except Exception as e:
            logger.error("OakDIMUReader: packet error: %s", e)


# ── 2h. RTK reader ────────────────────────────────────────────────────────────

class RTKReader(threading.Thread):
    """Reads NMEA (GGA/RMC) from Emlid RS+ over serial."""

    def __init__(self, port: str, baud: int, timeout: float = 1.0) -> None:
        super().__init__(name="RTKReader", daemon=True)
        self._port    = port
        self._baud    = baud
        self._timeout = timeout
        self._lock    = threading.Lock()
        self._data: dict = {
            "lat": None, "lon": None, "alt": None,
            "fix_quality": 0, "num_sats": 0, "hdop": None,
            "speed_knots": None, "track_deg": None, "ts": 0.0, "raw_gga": "",
        }
        self._available = False

    @property
    def is_available(self) -> bool:
        return self._available

    def get_data(self) -> dict:
        with self._lock:
            return dict(self._data)

    def run(self) -> None:
        try:
            ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
        except serial.SerialException as e:
            logger.error("RTKReader: cannot open %s: %s", self._port, e)
            return
        self._available = True
        logger.info("RTKReader: opened %s @ %d", self._port, self._baud)
        try:
            while True:
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line:
                        self._dispatch(line)
                except serial.SerialException as e:
                    logger.error("RTKReader: read error: %s", e)
                    break
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self._available = False

    def _dispatch(self, line: str) -> None:
        if not line.startswith("$") or not self._verify_checksum(line):
            return
        body  = line[1:].split("*")[0]
        parts = body.split(",")
        if not parts:
            return
        stype = parts[0][2:]
        if stype == "GGA":
            self._parse_gga(parts)
        elif stype == "RMC":
            self._parse_rmc(parts)

    def _parse_gga(self, parts: list) -> None:
        try:
            if len(parts) < 11:
                return
            lat = self._nmea_dec(parts[2], parts[3]) if parts[2] and parts[3] else None
            lon = self._nmea_dec(parts[4], parts[5]) if parts[4] and parts[5] else None
            with self._lock:
                self._data.update({
                    "lat": lat, "lon": lon,
                    "alt": float(parts[9]) if parts[9] else None,
                    "fix_quality": int(parts[6]) if parts[6] else 0,
                    "num_sats": int(parts[7]) if parts[7] else 0,
                    "hdop": float(parts[8]) if parts[8] else None,
                    "ts": time.time(), "raw_gga": ",".join(parts),
                })
        except (ValueError, IndexError) as e:
            logger.warning("RTKReader: GGA parse error: %s", e)

    def _parse_rmc(self, parts: list) -> None:
        try:
            if len(parts) < 9 or parts[2] != "A":
                return
            with self._lock:
                self._data["speed_knots"] = float(parts[7]) if parts[7] else None
                self._data["track_deg"]   = float(parts[8]) if parts[8] else None
        except (ValueError, IndexError):
            pass

    @staticmethod
    def _verify_checksum(sentence: str) -> bool:
        try:
            star = sentence.rindex("*")
            expected = int(sentence[star + 1:star + 3], 16)
            computed = 0
            for ch in sentence[1:star]:
                computed ^= ord(ch)
            return computed == expected
        except (ValueError, IndexError):
            return False

    @staticmethod
    def _nmea_dec(raw: str, direction: str) -> float:
        dot = raw.index(".")
        deg = float(raw[:dot - 2]) + float(raw[dot - 2:]) / 60.0
        return -deg if direction in ("S", "W") else deg


# ── 2i. Data recorder ─────────────────────────────────────────────────────────

_CSV_HEADER = [
    "timestamp",
    "accel_x", "accel_y", "accel_z",
    "gyro_x",  "gyro_y",  "gyro_z",
    "compass_bearing",
    "lat", "lon", "alt",
    "fix_quality", "num_sats", "hdop",
    "linear_cmd", "angular_cmd",
    "odom_speed", "odom_angrate",
    "amiga_soc",
]


def _fmt(value, decimals: int = 6) -> str:
    if value is None:
        return ""
    try:
        return f"{float(value):.{decimals}f}"
    except (TypeError, ValueError):
        return str(value)


class DataRecorder:
    """CSV writer for robot sensor + command data (thread-safe)."""

    def __init__(self, log_dir: str) -> None:
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._file = self._writer = None
        self._lock = threading.Lock()
        self._recording = False
        self._current_filename = ""

    @property
    def is_recording(self) -> bool:
        return self._recording

    def start(self) -> str:
        with self._lock:
            if self._recording:
                self._close()
            ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = f"robot_data_{ts}.csv"
            fp = self._log_dir / fname
            try:
                self._file = open(fp, "w", newline="", encoding="utf-8")
                self._writer = csv.writer(self._file)
                self._writer.writerow(_CSV_HEADER)
                self._file.flush()
                self._recording = True
                self._current_filename = fname
                logger.info("DataRecorder: recording → %s", fp)
                return fname
            except OSError as e:
                logger.error("DataRecorder: failed to create file: %s", e)
                self._file = self._writer = None
                self._recording = False
                raise

    def stop(self) -> None:
        with self._lock:
            self._close()

    def record(self, imu_snap: dict, rtk_snap: dict,
               linear: float, angular: float, odom_snap: dict | None = None) -> None:
        if not self._recording:
            return
        ts   = datetime.now().isoformat(timespec="milliseconds")
        acc  = imu_snap.get("accel", {})
        gyro = imu_snap.get("gyro",  {})
        comp = imu_snap.get("compass", {})
        odom = odom_snap or {}
        row = [
            ts,
            _fmt(acc.get("x")), _fmt(acc.get("y")), _fmt(acc.get("z")),
            _fmt(gyro.get("x")), _fmt(gyro.get("y")), _fmt(gyro.get("z")),
            _fmt(comp.get("bearing")),
            _fmt(rtk_snap.get("lat"), 8), _fmt(rtk_snap.get("lon"), 8), _fmt(rtk_snap.get("alt"), 3),
            rtk_snap.get("fix_quality", ""), rtk_snap.get("num_sats", ""), _fmt(rtk_snap.get("hdop"), 2),
            _fmt(linear, 4), _fmt(angular, 4),
            _fmt(odom.get("v"), 4), _fmt(odom.get("w"), 4),
            odom.get("soc", ""),
        ]
        with self._lock:
            if not self._recording or self._writer is None:
                return
            try:
                self._writer.writerow(row)
                self._file.flush()
            except OSError as e:
                logger.error("DataRecorder: write error: %s", e)
                self._recording = False
                self._close()

    def _close(self) -> None:
        if self._file is not None:
            try:
                self._file.close()
                logger.info("DataRecorder: file closed: %s", self._current_filename)
            except OSError:
                pass
            finally:
                self._file = self._writer = None
        self._recording = False
        self._current_filename = ""


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class _StaticHandler(SimpleHTTPRequestHandler):
    """HTTP handler that injects velocity limits into index.html."""

    # class-level config (set by RobotBridge before serving)
    static_dir: Path = Path("web_static")
    max_linear:  float = 1.0
    max_angular: float = 1.0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(self.static_dir), **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_index()
        else:
            super().do_GET()

    def _serve_index(self):
        try:
            content = (self.static_dir / "index.html").read_text(encoding="utf-8")
            content = content.replace(
                '<html lang="en">',
                f'<html lang="en" data-max-linear="{self.max_linear}" '
                f'data-max-angular="{self.max_angular}">',
            )
            encoded = content.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)
        except Exception as e:
            logger.error("HTTP: failed to serve index.html: %s", e)
            self.send_error(500)

    def log_message(self, fmt, *args):
        logger.debug("HTTP %s - %s", self.address_string(), fmt % args)


class HttpFileServer:
    """Serve web_static/ files in a daemon thread."""

    def __init__(self, static_dir: Path, port: int,
                 max_linear: float, max_angular: float) -> None:
        _StaticHandler.static_dir  = static_dir
        _StaticHandler.max_linear  = max_linear
        _StaticHandler.max_angular = max_angular
        self._port = port
        self._static_dir = static_dir

    def run(self) -> None:
        server = ThreadingHTTPServer(("0.0.0.0", self._port), _StaticHandler)
        logger.info("HttpFileServer: serving %s on port %d", self._static_dir, self._port)
        server.serve_forever()


# ── WebSocket server ──────────────────────────────────────────────────────────

# Module-level shared state (set by RobotBridge._run_async, read from broadcast loops)
_imu_reader:    OakDIMUReader | ESP32IMUReader | None = None
_rtk_reader:    RTKReader | None = None
_data_recorder: DataRecorder | None = None
_vel_lock = threading.Lock()
_last_linear:  float = 0.0
_last_angular: float = 0.0
_odom_lock = threading.Lock()
_last_odom: dict = {}


class RobotWebSocketServer:
    """
    WebSocket server: receives joystick/nav commands from browser,
    broadcasts sensor data and status at various rates.
    """

    def __init__(
        self,
        port: int,
        serial_port: str,
        serial_baud: int,
        serial_timeout: float,
        max_linear: float,
        max_angular: float,
        watchdog_timeout: float,
        nav_engine: NavigationEngine,
    ) -> None:
        self._port             = port
        self._serial_port      = serial_port
        self._serial_baud      = serial_baud
        self._serial_timeout   = serial_timeout
        self._max_linear       = max_linear
        self._max_angular      = max_angular
        self._watchdog_timeout = watchdog_timeout
        self._nav_engine       = nav_engine

        self._ser: serial.Serial | None = None
        self._ser_lock    = threading.Lock()
        self._clients: set = set()
        self._clients_lock = asyncio.Lock()
        self._last_heartbeat = time.time()
        self._serial_ok    = False
        self._auto_active  = False
        self._loop: asyncio.AbstractEventLoop | None = None

    # ── Serial ────────────────────────────────────────────────────────────────

    def open_serial(self) -> None:
        try:
            self._ser = serial.Serial(self._serial_port, self._serial_baud,
                                      timeout=self._serial_timeout)
            self._serial_ok = True
            logger.info("Serial: opened %s @ %d", self._serial_port, self._serial_baud)
        except serial.SerialException as e:
            logger.error("Serial: cannot open %s: %s", self._serial_port, e)
            self._serial_ok = False

    def close_serial(self) -> None:
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
                logger.info("Serial: closed")

    def _send_velocity(self, linear: float, angular: float) -> None:
        global _last_linear, _last_angular
        cmd = f"V{linear:.2f},{angular:.2f}\n".encode()
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write(cmd)
            except serial.SerialException as e:
                logger.error("Serial: write error: %s", e)
                self._serial_ok = False
                return
        with _vel_lock:
            _last_linear  = linear
            _last_angular = angular

    def _send_raw(self, data: bytes) -> None:
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write(data)
            except serial.SerialException as e:
                logger.error("Serial: raw write error: %s", e)
                self._serial_ok = False

    # ── Serial reader thread ──────────────────────────────────────────────────

    def _start_serial_reader(self) -> None:
        t = threading.Thread(target=self._serial_reader_thread, name="SerialReader", daemon=True)
        t.start()

    def _serial_reader_thread(self) -> None:
        buf = b""
        while True:
            try:
                with self._ser_lock:
                    if self._ser is None or not self._ser.is_open:
                        buf = b""; time.sleep(0.1); continue
                    n     = self._ser.in_waiting
                    chunk = self._ser.read(n) if n > 0 else b""
            except serial.SerialException as e:
                logger.error("SerialReader: error: %s", e)
                buf = ""; time.sleep(0.1); continue
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._handle_serial_line(line.strip())
            else:
                time.sleep(0.01)

    def _handle_serial_line(self, line: bytes) -> None:
        if line == b"S:ACTIVE":
            new_state = True
        elif line == b"S:READY":
            new_state = False
        elif line.startswith(b"O:"):
            try:
                parts = line[2:].decode().split(",")
                v = float(parts[0]); w = float(parts[1])
                state_int = int(parts[2]) if len(parts) > 2 else None
                soc       = int(parts[3]) if len(parts) > 3 else None
                with _odom_lock:
                    _last_odom.update({"v": v, "w": w, "state": state_int, "soc": soc, "ts": time.time()})
                self._nav_engine.on_odometry(v, w)
            except (ValueError, IndexError):
                pass
            return
        else:
            return
        if self._auto_active == new_state:
            return
        self._auto_active = new_state
        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"type": "state_status", "active": new_state}),
                self._loop,
            )

    # ── Broadcast ────────────────────────────────────────────────────────────

    async def _broadcast(self, obj: dict) -> None:
        msg = json.dumps(obj)
        async with self._clients_lock:
            clients = set(self._clients)
        dead = set()
        for ws in clients:
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        if dead:
            async with self._clients_lock:
                self._clients -= dead

    # ── WebSocket handler ─────────────────────────────────────────────────────

    async def _ws_handler(self, websocket) -> None:
        async with self._clients_lock:
            self._clients.add(websocket)
        logger.info("WS: client connected: %s", websocket.remote_address)
        try:
            await websocket.send(json.dumps({"type": "state_status", "active": self._auto_active}))
        except Exception:
            pass
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                msg_type = msg.get("type")

                if msg_type in ("heartbeat", "joystick"):
                    self._last_heartbeat = time.time()

                if msg_type == "joystick":
                    nav_active = (self._nav_engine.get_status().get("state") == "navigating")
                    if not nav_active:
                        try:
                            lin = max(-self._max_linear,  min(self._max_linear,  float(msg.get("linear",  0.0))))
                            ang = max(-self._max_angular, min(self._max_angular, float(msg.get("angular", 0.0))))
                            self._send_velocity(lin, ang)
                        except (TypeError, ValueError):
                            pass

                elif msg_type == "toggle_state":
                    self._last_heartbeat = time.time()
                    self._send_raw(b"\r")

                elif msg_type == "toggle_record":
                    await self._handle_toggle_record()

                elif msg_type == "upload_waypoints":
                    csv_text = msg.get("csv", "")
                    if isinstance(csv_text, str) and csv_text.strip():
                        try:
                            count = self._nav_engine.load_waypoints(csv_text)
                            await self._broadcast({"type": "waypoints_loaded", "count": count})
                        except Exception as e:
                            await self._broadcast({"type": "waypoints_loaded", "count": 0, "error": str(e)})
                    else:
                        await self._broadcast({"type": "waypoints_loaded", "count": 0, "error": "empty CSV"})

                elif msg_type == "nav_start":
                    ok = self._nav_engine.start()
                    if not ok:
                        await self._broadcast({**self._nav_engine.get_status(),
                                               "error": "Unable to start navigation"})

                elif msg_type == "nav_start_force":
                    ok = self._nav_engine.start(force=True)
                    if not ok:
                        await self._broadcast({**self._nav_engine.get_status(),
                                               "error": "Unable to start navigation (no waypoints)"})

                elif msg_type == "nav_stop":
                    self._nav_engine.stop()
                    await self._broadcast(self._nav_engine.get_status())

                elif msg_type == "nav_mode":
                    try:
                        self._nav_engine.set_nav_mode(NavMode(msg.get("mode", "")))
                        await self._broadcast(self._nav_engine.get_status())
                    except ValueError:
                        pass

                elif msg_type == "filter_mode":
                    try:
                        self._nav_engine.set_filter_mode(FilterMode(msg.get("mode", "")))
                        await self._broadcast(self._nav_engine.get_status())
                    except ValueError:
                        pass

                elif msg_type == "generate_coverage":
                    await self._handle_generate_coverage(msg)

        except websockets.exceptions.ConnectionClosedError:
            pass
        except Exception as e:
            logger.error("WS: handler error: %s", e)
        finally:
            async with self._clients_lock:
                self._clients.discard(websocket)
            logger.info("WS: client disconnected: %s", websocket.remote_address)
            self._send_velocity(0.0, 0.0)

    async def _handle_toggle_record(self) -> None:
        if _data_recorder is None:
            return
        if _data_recorder.is_recording:
            _data_recorder.stop()
            msg = {"type": "record_status", "recording": False, "filename": ""}
        else:
            try:
                fname = _data_recorder.start()
                msg = {"type": "record_status", "recording": True, "filename": fname}
            except OSError:
                msg = {"type": "record_status", "recording": False, "filename": ""}
        await self._broadcast(msg)

    async def _handle_generate_coverage(self, msg: dict) -> None:
        try:
            raw_boundary = msg.get("boundary", [])
            if not isinstance(raw_boundary, list) or len(raw_boundary) < 3:
                await self._broadcast({"type": "coverage_ready", "count": 0,
                                       "error": "boundary needs at least 3 vertices"})
                return
            boundary    = [(float(p[0]), float(p[1])) for p in raw_boundary]
            row_spacing = float(msg.get("row_spacing",   1.0))
            direction   = float(msg.get("direction_deg", 0.0))
            overlap     = float(msg.get("overlap",       0.0))
            tolerance   = float(msg.get("tolerance_m",  1.0))
            max_speed   = float(msg.get("max_speed",     0.5))
            planner  = CoveragePlanner(boundary, row_spacing, direction, overlap, tolerance, max_speed)
            csv_text = planner.generate_csv()
            count    = self._nav_engine.load_waypoints(csv_text)
            await self._broadcast({"type": "coverage_ready", "count": count})
        except Exception as e:
            logger.error("generate_coverage failed: %s", e)
            await self._broadcast({"type": "coverage_ready", "count": 0, "error": str(e)})

    # ── Broadcast loops ───────────────────────────────────────────────────────

    async def _imu_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(0.05)  # 20 Hz
            if _imu_reader is None:
                continue
            data = _imu_reader.get_data()
            await self._broadcast({
                "type": "imu",
                "ts": data.get("ts"),
                "accel": data.get("accel"),
                "gyro":  data.get("gyro"),
                "compass": data.get("compass"),
            })
            self._nav_engine.on_imu(data)

    async def _rtk_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0)  # 1 Hz
            if _rtk_reader is None:
                continue
            snap = _rtk_reader.get_data()
            await self._broadcast({
                "type":        "rtk",
                "available":   _rtk_reader.is_available,
                "lat":         snap["lat"],
                "lon":         snap["lon"],
                "alt":         snap["alt"],
                "fix_quality": snap["fix_quality"],
                "num_sats":    snap["num_sats"],
                "hdop":        snap["hdop"],
                "speed_knots": snap["speed_knots"],
                "track_deg":   snap["track_deg"],
            })
            self._nav_engine.on_rtk(snap)

    async def _odom_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(0.05)  # 20 Hz
            with _odom_lock:
                snap = dict(_last_odom)
            if not snap.get("ts"):
                continue
            await self._broadcast({
                "type": "odom",
                "v":    snap.get("v", 0.0),
                "w":    snap.get("w", 0.0),
                "state": snap.get("state"),
                "soc":   snap.get("soc"),
                "ts":    snap.get("ts"),
            })

    async def _data_record_loop(self) -> None:
        while True:
            await asyncio.sleep(0.2)  # 5 Hz
            if _data_recorder is None or not _data_recorder.is_recording:
                continue
            imu_snap = _imu_reader.get_data() if _imu_reader else {}
            rtk_snap = _rtk_reader.get_data() if _rtk_reader else {}
            with _vel_lock:
                lin, ang = _last_linear, _last_angular
            with _odom_lock:
                odom_snap = dict(_last_odom)
            _data_recorder.record(imu_snap, rtk_snap, lin, ang, odom_snap)

    async def _watchdog_loop(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            elapsed = time.time() - self._last_heartbeat
            if elapsed > self._watchdog_timeout:
                logger.warning("Watchdog: no heartbeat for %.1fs — emergency stop", elapsed)
                self._send_velocity(0.0, 0.0)
                if self._auto_active:
                    self._send_raw(b"\r")
                self._last_heartbeat = time.time()

    async def _status_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(2.0)
            rtk_ok  = _rtk_reader.is_available  if _rtk_reader  else False
            imu_ok  = _imu_reader.is_available   if _imu_reader  else False
            rec     = _data_recorder.is_recording if _data_recorder else False
            await self._broadcast({
                "type":      "status",
                "serial_ok": self._serial_ok,
                "imu_ok":    imu_ok,
                "rtk_ok":    rtk_ok,
                "recording": rec,
                "message":   "OK" if (self._serial_ok and imu_ok) else "DEGRADED",
            })

    # ── Main serve coroutine ──────────────────────────────────────────────────

    async def serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._last_heartbeat = time.time()
        self._start_serial_reader()

        async with websockets.serve(
            self._ws_handler, "0.0.0.0", self._port,
            ping_interval=20, ping_timeout=10,
        ):
            logger.info("RobotWebSocketServer: running on ws://0.0.0.0:%d", self._port)
            await asyncio.gather(
                self._imu_broadcast_loop(),
                self._rtk_broadcast_loop(),
                self._odom_broadcast_loop(),
                self._data_record_loop(),
                self._watchdog_loop(),
                self._status_broadcast_loop(),
            )


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

class RobotBridge:
    """
    Top-level orchestrator. Assembles and starts all components:
      1. IMUReader         — ESP32 or OAK-D BNO085
      2. RTKReader         — Emlid RS+ GPS (optional)
      3. DataRecorder      — CSV recording
      4. HttpFileServer    — serves web_static/ on HTTP port
      5. NavigationEngine  — autonomous path tracking
      6. RobotWebSocketServer — joystick/nav WS + sensor broadcasts
    """

    def __init__(
        self,
        ws_port: int,
        serial_port: str,
        serial_baud: int,
        serial_timeout: float,
        max_linear: float,
        max_angular: float,
        watchdog_timeout: float,
        imu_source: str,
        esp32_imu_port: str,
        esp32_imu_baud: int,
        rtk_port: str,
        rtk_baud: int,
        rtk_enabled: bool,
        data_log_dir: str,
        compass_min_accuracy: int,
        compass_offset_deg: float,
        nav_kp: float, nav_ki: float, nav_kd: float,
        nav_lookahead: float,
        nav_decel_radius: float,
        nav_arrive_frames: int,
        nav_gps_timeout: float,
        nav_ma_window: int,
        static_dir: Path,
    ) -> None:
        self._http_port   = ws_port
        self._ws_port     = ws_port + 1
        self._static_dir  = static_dir

        self._serial_port    = serial_port
        self._serial_baud    = serial_baud
        self._serial_timeout = serial_timeout
        self._max_linear     = max_linear
        self._max_angular    = max_angular
        self._watchdog_timeout = watchdog_timeout

        self._imu_source          = imu_source
        self._esp32_imu_port      = esp32_imu_port
        self._esp32_imu_baud      = esp32_imu_baud
        self._rtk_port            = rtk_port
        self._rtk_baud            = rtk_baud
        self._rtk_enabled         = rtk_enabled
        self._data_log_dir        = data_log_dir
        self._compass_min_accuracy = compass_min_accuracy
        self._compass_offset_deg  = compass_offset_deg

        self._nav_kp          = nav_kp
        self._nav_ki          = nav_ki
        self._nav_kd          = nav_kd
        self._nav_lookahead   = nav_lookahead
        self._nav_decel_radius = nav_decel_radius
        self._nav_arrive_frames = nav_arrive_frames
        self._nav_gps_timeout = nav_gps_timeout
        self._nav_ma_window   = nav_ma_window

    def run(self) -> None:
        global _imu_reader, _rtk_reader, _data_recorder

        logger.info("=" * 55)
        logger.info("RobotBridge starting")
        logger.info("  HTTP port : %d", self._http_port)
        logger.info("  WS   port : %d", self._ws_port)
        logger.info("  Serial    : %s @ %d", self._serial_port, self._serial_baud)
        logger.info("  IMU       : %s", self._imu_source)
        logger.info("  RTK GPS   : %s", "enabled" if self._rtk_enabled else "disabled")
        logger.info("  Max vel   : linear=%.1f  angular=%.1f", self._max_linear, self._max_angular)
        logger.info("=" * 55)

        # HTTP file server
        if self._static_dir.exists():
            http = HttpFileServer(self._static_dir, self._http_port,
                                  self._max_linear, self._max_angular)
            threading.Thread(target=http.run, daemon=True, name="http-server").start()
        else:
            logger.warning("web_static not found at %s — HTTP server not started", self._static_dir)

        # IMU reader
        if self._imu_source == "esp32":
            _imu_reader = ESP32IMUReader(self._esp32_imu_port, self._esp32_imu_baud,
                                         self._compass_min_accuracy, self._compass_offset_deg)
        else:
            _imu_reader = OakDIMUReader(self._compass_min_accuracy, self._compass_offset_deg)
        _imu_reader.start()

        # RTK reader
        if self._rtk_enabled:
            _rtk_reader = RTKReader(self._rtk_port, self._rtk_baud)
            _rtk_reader.start()
            logger.info("RTKReader started: %s @ %d", self._rtk_port, self._rtk_baud)

        # Data recorder
        _data_recorder = DataRecorder(self._data_log_dir)

        # Open browser
        url = f"http://localhost:{self._http_port}"
        logger.info("Open browser at %s", url)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("RobotBridge: stopped by user.")

    async def _run_async(self) -> None:
        loop = asyncio.get_running_loop()

        nav_engine = NavigationEngine(
            send_velocity_fn   = self._make_velocity_sender(),
            broadcast_fn       = self._make_broadcaster(),
            loop               = loop,
            max_linear         = self._max_linear,
            max_angular        = self._max_angular,
            gps_timeout        = self._nav_gps_timeout,
            arrive_frames      = self._nav_arrive_frames,
            kp=self._nav_kp, ki=self._nav_ki, kd=self._nav_kd,
            decel_radius       = self._nav_decel_radius,
            lookahead          = self._nav_lookahead,
            ma_window          = self._nav_ma_window,
            compass_min_accuracy = self._compass_min_accuracy,
        )

        ws_server = RobotWebSocketServer(
            port             = self._ws_port,
            serial_port      = self._serial_port,
            serial_baud      = self._serial_baud,
            serial_timeout   = self._serial_timeout,
            max_linear       = self._max_linear,
            max_angular      = self._max_angular,
            watchdog_timeout = self._watchdog_timeout,
            nav_engine       = nav_engine,
        )
        ws_server.open_serial()

        try:
            await ws_server.serve()
        finally:
            ws_server.close_serial()

    def _make_velocity_sender(self):
        # Velocity sender placeholder; actual sender set after ws_server is created.
        # NavigationEngine calls this at startup — forwarded to ws_server via closure below.
        _ref: list = []

        def _send(linear: float, angular: float) -> None:
            if _ref:
                _ref[0]._send_velocity(linear, angular)

        # Store reference list so RobotWebSocketServer can inject itself
        self._nav_velocity_ref = _ref
        return _send

    def _make_broadcaster(self):
        _ref: list = []
        self._nav_broadcast_ref = _ref

        async def _broadcast(msg: dict) -> None:
            if _ref:
                await _ref[0]._broadcast(msg)

        return _broadcast


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 (continued) — Simpler orchestration without circular references
# ═════════════════════════════════════════════════════════════════════════════
# Note: RobotBridge._run_async creates NavigationEngine AFTER RobotWebSocketServer
# so we can directly pass ws_server methods to NavigationEngine. The _make_velocity_sender
# / _make_broadcaster helpers above are replaced by the cleaner direct approach below.

# Patch _run_async to use direct method references (avoids circular _ref pattern):

async def _run_async_patched(self: RobotBridge) -> None:
    ws_server = RobotWebSocketServer(
        port             = self._ws_port,
        serial_port      = self._serial_port,
        serial_baud      = self._serial_baud,
        serial_timeout   = self._serial_timeout,
        max_linear       = self._max_linear,
        max_angular      = self._max_angular,
        watchdog_timeout = self._watchdog_timeout,
        nav_engine       = None,  # type: ignore[arg-type]  — injected below
    )
    ws_server.open_serial()

    nav_engine = NavigationEngine(
        send_velocity_fn   = ws_server._send_velocity,
        broadcast_fn       = ws_server._broadcast,
        loop               = asyncio.get_running_loop(),
        max_linear         = self._max_linear,
        max_angular        = self._max_angular,
        gps_timeout        = self._nav_gps_timeout,
        arrive_frames      = self._nav_arrive_frames,
        kp=self._nav_kp, ki=self._nav_ki, kd=self._nav_kd,
        decel_radius       = self._nav_decel_radius,
        lookahead          = self._nav_lookahead,
        ma_window          = self._nav_ma_window,
        compass_min_accuracy = self._compass_min_accuracy,
    )
    ws_server._nav_engine = nav_engine  # inject back

    try:
        await ws_server.serve()
    finally:
        ws_server.close_serial()


# Monkey-patch to avoid circular reference pattern
RobotBridge._run_async = _run_async_patched  # type: ignore[method-assign]


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Farm Robot Web Controller bridge (standalone)"
    )
    parser.add_argument(
        "--ws-port", type=int,
        default=int(os.environ.get("WEB_HTTP_PORT", "8888")),
        help="HTTP port for web UI; WebSocket uses ws-port+1 (default: 8888)",
    )
    parser.add_argument(
        "--serial-port",
        default=os.environ.get("FEATHER_PORT", _default_serial_port()),
        help="Feather M4 serial port",
    )
    parser.add_argument(
        "--serial-baud", type=int,
        default=int(os.environ.get("SERIAL_BAUD", "115200")),
    )
    parser.add_argument(
        "--serial-timeout", type=float,
        default=float(os.environ.get("SERIAL_TIMEOUT", "1.0")),
    )
    parser.add_argument(
        "--max-linear", type=float,
        default=float(os.environ.get("MAX_LINEAR_VEL", "1.0")),
        help="Max linear velocity m/s (default: 1.0)",
    )
    parser.add_argument(
        "--max-angular", type=float,
        default=float(os.environ.get("MAX_ANGULAR_VEL", "1.0")),
        help="Max angular velocity rad/s (default: 1.0)",
    )
    parser.add_argument(
        "--watchdog-timeout", type=float,
        default=float(os.environ.get("WATCHDOG_TIMEOUT", "2.0")),
    )
    parser.add_argument(
        "--imu-source",
        default=os.environ.get("IMU_SOURCE", "esp32"),
        choices=["esp32", "oakd"],
        help="IMU source: esp32 (serial UART) or oakd (depthai)",
    )
    parser.add_argument(
        "--esp32-imu-port",
        default=os.environ.get("ESP32_IMU_PORT", "/dev/ttyUSB0"),
    )
    parser.add_argument(
        "--esp32-imu-baud", type=int,
        default=int(os.environ.get("ESP32_IMU_BAUD", "115200")),
    )
    parser.add_argument(
        "--rtk-port",
        default=os.environ.get("RTK_PORT", "/dev/cu.usbmodem2403"),
    )
    parser.add_argument(
        "--rtk-baud", type=int,
        default=int(os.environ.get("RTK_BAUD", "9600")),
    )
    parser.add_argument(
        "--no-rtk", action="store_true",
        help="Disable RTK GPS reader",
    )
    parser.add_argument(
        "--data-log-dir",
        default=os.environ.get("DATA_LOG_DIR", "data_log"),
    )
    parser.add_argument(
        "--compass-min-accuracy", type=int,
        default=int(os.environ.get("COMPASS_MIN_ACCURACY", "2")),
    )
    parser.add_argument(
        "--compass-offset-deg", type=float,
        default=float(os.environ.get("COMPASS_OFFSET_DEG", "0.0")),
    )
    parser.add_argument("--nav-kp",           type=float, default=float(os.environ.get("NAV_PID_KP",         "0.8")))
    parser.add_argument("--nav-ki",           type=float, default=float(os.environ.get("NAV_PID_KI",         "0.01")))
    parser.add_argument("--nav-kd",           type=float, default=float(os.environ.get("NAV_PID_KD",         "0.05")))
    parser.add_argument("--nav-lookahead",    type=float, default=float(os.environ.get("NAV_LOOKAHEAD_M",    "2.0")))
    parser.add_argument("--nav-decel-radius", type=float, default=float(os.environ.get("NAV_DECEL_RADIUS_M", "3.0")))
    parser.add_argument("--nav-arrive-frames",type=int,   default=int(os.environ.get("NAV_ARRIVE_FRAMES",    "5")))
    parser.add_argument("--nav-gps-timeout",  type=float, default=float(os.environ.get("NAV_GPS_TIMEOUT_S",  "5.0")))
    parser.add_argument("--nav-ma-window",    type=int,   default=int(os.environ.get("NAV_MA_WINDOW",        "10")))
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    RobotBridge(
        ws_port          = args.ws_port,
        serial_port      = args.serial_port,
        serial_baud      = args.serial_baud,
        serial_timeout   = args.serial_timeout,
        max_linear       = args.max_linear,
        max_angular      = args.max_angular,
        watchdog_timeout = args.watchdog_timeout,
        imu_source       = args.imu_source,
        esp32_imu_port   = args.esp32_imu_port,
        esp32_imu_baud   = args.esp32_imu_baud,
        rtk_port         = args.rtk_port,
        rtk_baud         = args.rtk_baud,
        rtk_enabled      = not args.no_rtk,
        data_log_dir     = args.data_log_dir,
        compass_min_accuracy = args.compass_min_accuracy,
        compass_offset_deg   = args.compass_offset_deg,
        nav_kp           = args.nav_kp,
        nav_ki           = args.nav_ki,
        nav_kd           = args.nav_kd,
        nav_lookahead    = args.nav_lookahead,
        nav_decel_radius = args.nav_decel_radius,
        nav_arrive_frames = args.nav_arrive_frames,
        nav_gps_timeout  = args.nav_gps_timeout,
        nav_ma_window    = args.nav_ma_window,
        static_dir       = Path(__file__).parent / "web_static",
    ).run()
