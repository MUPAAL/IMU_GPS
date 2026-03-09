"""
autonav_bridge.py — Autonomous navigation engine: subscribes to IMU, RTK,
and Robot WebSocket feeds, runs GPS-filtered PID/PurePursuit control, and
sends velocity commands back to robot_bridge.

Data flow:
    imu_bridge(WS)   ──→ ImuWsClient   ──→ pipeline.on_imu()   ─┐
    rtk_bridge(WS)   ──→ RtkWsClient   ──→ pipeline.on_rtk()   ─┤── AutoNavPipeline
    robot_bridge(WS)  ──→ RobotWsClient ──→ pipeline.on_odom()  ─┘   (state machine)
                                                                        ↓
                                     AutoNavLoop(20Hz) → pipeline.get_status() → queue
                                                                        ↓
                                         WebSocketServer.broadcast() → Browser
                                                                        ↓
                    pipeline velocity cmd → RobotWsClient.send_command() → robot_bridge

Usage:
    python autonav_bridge.py --ws-port 8805 \
        --imu-ws ws://localhost:8766 --rtk-ws ws://localhost:8776 \
        --robot-ws ws://localhost:8796
    # Browser: http://localhost:8805
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import io
import json
import logging
import math
import socketserver
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass, replace
from enum import Enum, auto
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import numpy as np
import websockets


# ── Logger setup ─────────────────────────────────────────────────────────────

def _setup_logger() -> logging.Logger:
    """Configure module-level logger; output to autonav_bridge.log and stderr."""
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

_EARTH_RADIUS_M = 6_371_000.0


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


@dataclass
class Waypoint:
    """Single navigation waypoint."""
    id:          int
    lat:         float
    lon:         float
    tolerance_m: float
    max_speed:   float


@dataclass
class AutoNavFrame:
    """
    Status payload broadcast to the browser UI each tick.

    Carries navigation state, position, target info, and control output.
    """

    state:          str = "idle"
    nav_mode:       str = "p2p"
    filter_mode:    str = "moving_avg"
    progress:       list = None
    distance_m:     float | None = None
    target_bearing: float | None = None
    bearing_error:  float | None = None
    tolerance_m:    float | None = None
    fix_quality:    int = 0
    position:       dict | None = None
    linear_cmd:     float = 0.0
    angular_cmd:    float = 0.0

    def __post_init__(self):
        if self.progress is None:
            self.progress = [0, 0]

    def to_dict(self) -> dict:
        """Serialize to broadcast-ready dict."""
        return {
            "type": "nav_status",
            "state": self.state,
            "nav_mode": self.nav_mode,
            "filter_mode": self.filter_mode,
            "progress": self.progress,
            "distance_m": round(self.distance_m, 2) if self.distance_m is not None else None,
            "target_bearing": round(self.target_bearing, 1) if self.target_bearing is not None else None,
            "bearing_error": round(self.bearing_error, 1) if self.bearing_error is not None else None,
            "tolerance_m": self.tolerance_m,
            "fix_quality": self.fix_quality,
            "position": self.position,
            "linear_cmd": round(self.linear_cmd, 3),
            "angular_cmd": round(self.angular_cmd, 3),
        }


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE
# ═════════════════════════════════════════════════════════════════════════════

# ── Geo Utilities ────────────────────────────────────────────────────────────

class GeoUtils:
    """Static geodesic calculation methods."""

    @staticmethod
    def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Great-circle distance in metres between two WGS-84 points."""
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(a))

    @staticmethod
    def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """True-north bearing from (lat1,lon1) to (lat2,lon2), degrees [0,360)."""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlam = math.radians(lon2 - lon1)
        x = math.sin(dlam) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
        return math.degrees(math.atan2(x, y)) % 360.0

    @staticmethod
    def normalize_angle(angle_deg: float) -> float:
        """Normalize angle to (-180, 180]."""
        angle = angle_deg % 360.0
        if angle > 180.0:
            angle -= 360.0
        return angle

    @staticmethod
    def project_on_segment(
        p_lat: float, p_lon: float,
        a_lat: float, a_lon: float,
        b_lat: float, b_lon: float,
    ) -> tuple[float, float, float]:
        """Project point P onto segment A->B. Returns (proj_lat, proj_lon, t)."""
        cos_lat = math.cos(math.radians(a_lat))

        def to_local(lat, lon):
            dlat = math.radians(lat - a_lat) * _EARTH_RADIUS_M
            dlon = math.radians(lon - a_lon) * _EARTH_RADIUS_M * cos_lat
            return dlat, dlon

        py, px = to_local(p_lat, p_lon)
        by, bx = to_local(b_lat, b_lon)

        ab_sq = bx * bx + by * by
        if ab_sq < 1e-12:
            return a_lat, a_lon, 0.0

        t = max(0.0, min(1.0, (px * bx + py * by) / ab_sq))
        proj_x = t * bx
        proj_y = t * by

        proj_lat = a_lat + math.degrees(proj_y / _EARTH_RADIUS_M)
        proj_lon = a_lon + math.degrees(proj_x / (_EARTH_RADIUS_M * cos_lat + 1e-12))
        return proj_lat, proj_lon, t


# ── GPS Filters ──────────────────────────────────────────────────────────────

class MovingAverageFilter:
    """Sliding-window GPS position smoother."""

    def __init__(self, window: int = 10) -> None:
        self._window = window
        self._lat_buf: deque[float] = deque(maxlen=window)
        self._lon_buf: deque[float] = deque(maxlen=window)

    def update(self, lat: float, lon: float) -> tuple[float, float]:
        """Push new observation, return current window mean."""
        self._lat_buf.append(lat)
        self._lon_buf.append(lon)
        return sum(self._lat_buf) / len(self._lat_buf), sum(self._lon_buf) / len(self._lon_buf)

    def get_position(self) -> tuple[float, float]:
        """Return current mean position; (0,0) if buffer empty."""
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
    """4D Kalman filter: [delta_lat_m, delta_lon_m, vel_n, vel_e]."""

    def __init__(self, process_noise_std: float = 0.1, gps_noise_std: float = 2.0) -> None:
        self._pn_std = process_noise_std
        self._gps_std = gps_noise_std
        self._origin_lat = 0.0
        self._origin_lon = 0.0
        self._cos_lat = 1.0
        self._initialized = False
        self._x = np.zeros(4)
        self._P = np.eye(4) * 100.0
        self._H_pos = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)

    def init(self, lat: float, lon: float) -> None:
        """Set origin and reset state."""
        self._origin_lat = lat
        self._origin_lon = lon
        self._cos_lat = math.cos(math.radians(lat))
        self._x = np.zeros(4)
        self._P = np.eye(4) * 10.0
        self._initialized = True
        logger.info("KalmanFilter: initialized at (%.7f, %.7f)", lat, lon)

    def predict(self, dt: float, a_north: float = 0.0, a_east: float = 0.0) -> None:
        """IMU-driven prediction step (~20 Hz)."""
        if not self._initialized:
            return
        F = np.array([
            [1, 0, dt, 0], [0, 1, 0, dt],
            [0, 0, 1, 0], [0, 0, 0, 1],
        ], dtype=float)
        B = np.array([
            [0.5 * dt * dt, 0], [0, 0.5 * dt * dt],
            [dt, 0], [0, dt],
        ], dtype=float)
        u = np.array([a_north, a_east])
        q = self._pn_std ** 2
        Q = np.diag([q * dt ** 2, q * dt ** 2, q, q])
        self._x = F @ self._x + B @ u
        self._P = F @ self._P @ F.T + Q

    def update(self, lat: float, lon: float, fix_quality: int = 1) -> tuple[float, float]:
        """GPS observation update (~1 Hz)."""
        if not self._initialized:
            self.init(lat, lon)
            return lat, lon

        if fix_quality == 4:
            r_std = 0.03
        elif fix_quality == 5:
            r_std = 0.5
        elif fix_quality == 2:
            r_std = 1.0
        else:
            r_std = self._gps_std
        R = np.eye(2) * r_std ** 2

        z = np.array([
            math.radians(lat - self._origin_lat) * _EARTH_RADIUS_M,
            math.radians(lon - self._origin_lon) * _EARTH_RADIUS_M * self._cos_lat,
        ])

        S = self._H_pos @ self._P @ self._H_pos.T + R
        K = self._P @ self._H_pos.T @ np.linalg.inv(S)
        self._x = self._x + K @ (z - self._H_pos @ self._x)
        self._P = (np.eye(4) - K @ self._H_pos) @ self._P
        return self.get_position()

    def update_velocity(self, v_north: float, v_east: float) -> None:
        """Odometry velocity observation update (~20 Hz)."""
        if not self._initialized:
            return
        z = np.array([v_north, v_east])
        H = np.array([[0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        R_vel = np.eye(2) * 0.05
        y = z - H @ self._x
        S = H @ self._P @ H.T + R_vel
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ H) @ self._P

    def get_position(self) -> tuple[float, float]:
        """Return current estimated position in decimal degrees."""
        if not self._initialized:
            return 0.0, 0.0
        dlat_m, dlon_m = self._x[0], self._x[1]
        lat = self._origin_lat + math.degrees(dlat_m / _EARTH_RADIUS_M)
        lon = self._origin_lon + math.degrees(dlon_m / (_EARTH_RADIUS_M * self._cos_lat + 1e-12))
        return lat, lon

    def reset(self) -> None:
        self._initialized = False
        self._x = np.zeros(4)
        self._P = np.eye(4) * 100.0

    @property
    def is_ready(self) -> bool:
        return self._initialized


# ── Controllers ──────────────────────────────────────────────────────────────

class PIDController:
    """General-purpose PID controller with output clamping."""

    def __init__(
        self, kp: float = 0.8, ki: float = 0.01, kd: float = 0.05,
        integral_limit: float = 30.0, output_limit: float = 1.0,
    ) -> None:
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._int_lim = integral_limit
        self._out_lim = output_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def compute(self, error: float, dt: float) -> float:
        """Compute PID output from bearing error (degrees)."""
        if dt <= 0:
            return 0.0
        self._integral += error * dt
        self._integral = max(-self._int_lim, min(self._int_lim, self._integral))
        if self._first:
            derivative = 0.0
            self._first = False
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = self._kp * error + self._ki * self._integral + self._kd * derivative
        return max(-self._out_lim, min(self._out_lim, output))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True


class P2PController:
    """Point-to-point controller using PID on bearing error."""

    def __init__(self, pid: PIDController, decel_radius_m: float = 3.0,
                 max_linear_vel: float = 1.0) -> None:
        self._pid = pid
        self._decel_radius = decel_radius_m
        self._max_linear = max_linear_vel

    def compute(
        self, robot_lat: float, robot_lon: float, robot_bearing: float,
        target_wp: Waypoint, dt: float,
    ) -> tuple[float, float]:
        """Return (linear_cmd, angular_cmd)."""
        distance = GeoUtils.haversine(robot_lat, robot_lon, target_wp.lat, target_wp.lon)
        target_bearing = GeoUtils.bearing(robot_lat, robot_lon, target_wp.lat, target_wp.lon)
        bearing_error = GeoUtils.normalize_angle(target_bearing - robot_bearing)

        angular_cmd = self._pid.compute(bearing_error, dt)
        speed_factor = min(1.0, distance / (self._decel_radius + 1e-6))
        heading_factor = max(0.0, 1.0 - abs(bearing_error) / 90.0)
        max_spd = min(target_wp.max_speed, self._max_linear)
        linear_cmd = max_spd * speed_factor * heading_factor

        return linear_cmd, angular_cmd

    def reset(self) -> None:
        self._pid.reset()


class PurePursuitController:
    """Pure pursuit path tracker with lookahead distance."""

    def __init__(self, p2p: P2PController, lookahead_m: float = 2.0) -> None:
        self._p2p = p2p
        self._lookahead = lookahead_m

    def compute(
        self, robot_lat: float, robot_lon: float, robot_bearing: float,
        waypoints: list[Waypoint], current_idx: int, dt: float,
    ) -> tuple[float, float]:
        """Return (linear_cmd, angular_cmd)."""
        if current_idx >= len(waypoints):
            return 0.0, 0.0

        current_wp = waypoints[current_idx]

        # Single waypoint or first segment -> fallback to P2P
        if current_idx == 0 or len(waypoints) < 2:
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        prev_wp = waypoints[current_idx - 1]

        try:
            proj_lat, proj_lon, t = GeoUtils.project_on_segment(
                robot_lat, robot_lon,
                prev_wp.lat, prev_wp.lon,
                current_wp.lat, current_wp.lon,
            )
        except Exception as exc:
            logger.warning("PurePursuitController: projection failed, fallback P2P: %s", exc)
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        seg_bearing = GeoUtils.bearing(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)
        seg_len = GeoUtils.haversine(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)

        if seg_len < 0.1:
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        dist_to_wp = GeoUtils.haversine(proj_lat, proj_lon, current_wp.lat, current_wp.lon)

        if dist_to_wp < self._lookahead:
            lookahead_lat = current_wp.lat
            lookahead_lon = current_wp.lon
        else:
            cos_lat = math.cos(math.radians(proj_lat))
            rad_bearing = math.radians(seg_bearing)
            dlat_m = self._lookahead * math.cos(rad_bearing)
            dlon_m = self._lookahead * math.sin(rad_bearing)
            lookahead_lat = proj_lat + math.degrees(dlat_m / _EARTH_RADIUS_M)
            lookahead_lon = proj_lon + math.degrees(dlon_m / (_EARTH_RADIUS_M * cos_lat + 1e-12))

        lookahead_wp = replace(current_wp, lat=lookahead_lat, lon=lookahead_lon)
        return self._p2p.compute(robot_lat, robot_lon, robot_bearing, lookahead_wp, dt)

    def reset(self) -> None:
        self._p2p.reset()


# ── Waypoint Manager ─────────────────────────────────────────────────────────

class WaypointManager:
    """Manage waypoint sequence with adaptive arrival detection."""

    def __init__(self, arrive_frames: int = 5) -> None:
        self._waypoints: list[Waypoint] = []
        self._idx = 0
        self._arrive_count = 0
        self._arrive_frames = arrive_frames

    def load_csv(self, csv_text: str) -> int:
        """Parse CSV text (header: id,lat,lon,tolerance_m,max_speed). Return count."""
        self._waypoints = []
        self._idx = 0
        self._arrive_count = 0

        lines = csv_text.strip().splitlines()
        if not lines:
            logger.warning("WaypointManager: CSV is empty")
            return 0

        for raw in lines[1:]:  # skip header
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
            except (ValueError, IndexError) as exc:
                logger.warning("WaypointManager: skipping invalid line %r: %s", raw, exc)

        count = len(self._waypoints)
        if count:
            logger.info("WaypointManager: loaded %d waypoints", count)
        else:
            logger.warning("WaypointManager: no valid waypoints loaded")
        return count

    @property
    def current(self) -> Waypoint | None:
        if 0 <= self._idx < len(self._waypoints):
            return self._waypoints[self._idx]
        return None

    @property
    def is_finished(self) -> bool:
        return self._idx >= len(self._waypoints)

    @property
    def progress(self) -> tuple[int, int]:
        total = len(self._waypoints)
        return min(self._idx + 1, total), total

    @property
    def waypoints(self) -> list[Waypoint]:
        return list(self._waypoints)

    @property
    def current_index(self) -> int:
        return self._idx

    def update(self, distance_m: float, fix_quality: int) -> bool:
        """Check arrival. Returns True if waypoint was just reached."""
        wp = self.current
        if wp is None:
            return False

        if fix_quality == 4:
            tolerance = min(wp.tolerance_m, 0.5)
        elif fix_quality == 5:
            tolerance = 2.0
        else:
            tolerance = wp.tolerance_m

        if distance_m < tolerance:
            self._arrive_count += 1
            if self._arrive_count >= self._arrive_frames:
                logger.info(
                    "WaypointManager: reached waypoint %d (dist=%.2fm, tol=%.2fm)",
                    wp.id, distance_m, tolerance,
                )
                self._idx += 1
                self._arrive_count = 0
                return True
        else:
            self._arrive_count = 0
        return False

    def reset(self) -> None:
        self._idx = 0
        self._arrive_count = 0


# ── Coverage Planner ─────────────────────────────────────────────────────────

class CoveragePlanner:
    """Boustrophedon (lawnmower) coverage path generator."""

    def __init__(
        self, boundary: list[tuple[float, float]],
        row_spacing: float = 1.0, direction_deg: float = 0.0,
        overlap: float = 0.0, tolerance_m: float = 1.0, max_speed: float = 0.5,
    ) -> None:
        if len(boundary) < 3:
            raise ValueError("boundary needs at least 3 vertices")
        self._boundary = list(boundary)
        self._row_spacing = max(0.05, row_spacing)
        self._direction = direction_deg % 360.0
        self._overlap = max(0.0, min(0.99, overlap))
        self._tolerance_m = tolerance_m
        self._max_speed = max_speed

        lats = [p[0] for p in boundary]
        lons = [p[1] for p in boundary]
        self._origin_lat = sum(lats) / len(lats)
        self._origin_lon = sum(lons) / len(lons)
        self._cos_lat = math.cos(math.radians(self._origin_lat))

    def _to_enu(self, lat, lon):
        north = math.radians(lat - self._origin_lat) * _EARTH_RADIUS_M
        east = math.radians(lon - self._origin_lon) * _EARTH_RADIUS_M * self._cos_lat
        return east, north

    def _from_enu(self, east, north):
        lat = self._origin_lat + math.degrees(north / _EARTH_RADIUS_M)
        lon = self._origin_lon + math.degrees(east / (_EARTH_RADIUS_M * self._cos_lat + 1e-12))
        return lat, lon

    def _enu_to_scan(self, east, north):
        rad = math.radians(self._direction)
        x = east * math.sin(rad) + north * math.cos(rad)
        y = -east * math.cos(rad) + north * math.sin(rad)
        return x, y

    def _scan_to_enu(self, x, y):
        rad = math.radians(self._direction)
        east = x * math.sin(rad) - y * math.cos(rad)
        north = x * math.cos(rad) + y * math.sin(rad)
        return east, north

    @staticmethod
    def _seg_intersect_y(y, x1, y1, x2, y2):
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1)
            return x1 + t * (x2 - x1)
        return None

    def _clip_scanline(self, y, polygon):
        xs = []
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            xi = self._seg_intersect_y(y, x1, y1, x2, y2)
            if xi is not None:
                xs.append(xi)
        return sorted(xs)

    def generate_csv(self) -> str:
        """Generate coverage waypoints CSV string."""
        scan_pts = [self._enu_to_scan(*self._to_enu(lat, lon)) for lat, lon in self._boundary]
        ys = [p[1] for p in scan_pts]
        y_min, y_max = min(ys), max(ys)

        eff_spacing = self._row_spacing * (1.0 - self._overlap)
        scan_ys = []
        y = y_min + self._row_spacing / 2.0
        while y <= y_max:
            scan_ys.append(y)
            y += eff_spacing

        if not scan_ys:
            logger.warning("CoveragePlanner: cannot generate scan rows (area too small)")
            return "id,lat,lon,tolerance_m,max_speed\n"

        wps_scan = []
        reverse = False
        for sy in scan_ys:
            xs = self._clip_scanline(sy, scan_pts)
            if len(xs) < 2:
                continue
            x_left, x_right = xs[0], xs[-1]
            if reverse:
                wps_scan.append((x_right, sy))
                wps_scan.append((x_left, sy))
            else:
                wps_scan.append((x_left, sy))
                wps_scan.append((x_right, sy))
            reverse = not reverse

        out = io.StringIO()
        writer = csv.writer(out)
        writer.writerow(["id", "lat", "lon", "tolerance_m", "max_speed"])
        for i, (sx, sy) in enumerate(wps_scan):
            east, north = self._scan_to_enu(sx, sy)
            lat, lon = self._from_enu(east, north)
            writer.writerow([i, f"{lat:.8f}", f"{lon:.8f}", self._tolerance_m, self._max_speed])

        logger.info(
            "CoveragePlanner: generated %d waypoints, spacing=%.2fm, direction=%.0f deg",
            len(wps_scan), self._row_spacing, self._direction,
        )
        return out.getvalue()


# ── AutoNav Pipeline (State Machine) ─────────────────────────────────────────

class AutoNavPipeline:
    """
    Main autonomous navigation state machine.

    Pipeline:
        on_imu(data)       → update bearing + Kalman predict + control step
        on_rtk(data)       → update GPS filter
        on_odometry(v, w)  → Kalman velocity observation
        get_status()       → AutoNavFrame
    """

    def __init__(
        self,
        max_linear_vel: float = 1.0,
        max_angular_vel: float = 1.0,
        pid_kp: float = 0.8,
        pid_ki: float = 0.01,
        pid_kd: float = 0.05,
        lookahead_m: float = 2.0,
        decel_radius_m: float = 3.0,
        arrive_frames: int = 5,
        gps_timeout_s: float = 5.0,
        ma_window: int = 10,
        send_velocity_fn=None,
    ) -> None:
        self._max_lin = max_linear_vel
        self._max_ang = max_angular_vel
        self._gps_timeout_s = gps_timeout_s
        self._send_velocity = send_velocity_fn  # callable(speed, ang_rate) or None

        self._lock = threading.Lock()

        # State
        self._state = NavState.IDLE
        self._nav_mode = NavMode.P2P
        self._filter_mode = FilterMode.MOVING_AVG

        # Waypoints
        self._wp_mgr = WaypointManager(arrive_frames)

        # Filters
        self._ma_filter = MovingAverageFilter(ma_window)
        self._kf_filter = KalmanFilter()

        # Controllers
        pid = PIDController(pid_kp, pid_ki, pid_kd, output_limit=max_angular_vel)
        self._p2p_ctrl = P2PController(pid, decel_radius_m, max_linear_vel)
        self._pp_ctrl = PurePursuitController(self._p2p_ctrl, lookahead_m)

        # IMU state
        self._robot_bearing: float | None = None
        self._last_imu_ts = 0.0
        self._last_control_ts = 0.0

        # RTK state
        self._fix_quality = 0
        self._last_gps_ts = 0.0
        self._gps_warning_sent = False

        # Output cache
        self._last_linear_cmd = 0.0
        self._last_angular_cmd = 0.0

        # Throttle diagnostics
        self._last_diag_ts = 0.0
        self._last_status_log_ts = 0.0

    # ── Public API ───────────────────────────────────────────────────────────

    def load_waypoints(self, csv_text: str) -> int:
        """Load waypoints from CSV text."""
        with self._lock:
            return self._wp_mgr.load_csv(csv_text)

    def start(self, force: bool = False) -> bool:
        """Start navigation. Returns True if successful."""
        with self._lock:
            if self._state == NavState.NAVIGATING:
                logger.warning("AutoNavPipeline: already navigating")
                return False
            if self._wp_mgr.current is None:
                logger.warning("AutoNavPipeline: no waypoints loaded")
                return False
            if self._fix_quality < 1 and not force:
                logger.warning("AutoNavPipeline: GPS fix_quality=%d, insufficient", self._fix_quality)
                return False

            self._wp_mgr.reset()
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
            self._gps_warning_sent = False
            self._state = NavState.NAVIGATING
            self._last_control_ts = time.time()

        logger.info(
            "AutoNavPipeline: navigation started, mode=%s, filter=%s, waypoints=%d",
            self._nav_mode.value, self._filter_mode.value, self._wp_mgr.progress[1],
        )
        return True

    def stop(self) -> None:
        """Stop navigation and send zero velocity."""
        with self._lock:
            if self._state == NavState.IDLE:
                return
            self._state = NavState.IDLE
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
            self._last_linear_cmd = 0.0
            self._last_angular_cmd = 0.0

        if self._send_velocity:
            self._send_velocity(0.0, 0.0)
        logger.info("AutoNavPipeline: navigation stopped")

    def set_nav_mode(self, mode_str: str) -> None:
        """Switch navigation mode: 'p2p' or 'pure_pursuit'."""
        try:
            mode = NavMode(mode_str)
        except ValueError:
            logger.warning("AutoNavPipeline: unknown nav mode: %s", mode_str)
            return
        with self._lock:
            self._nav_mode = mode
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
        logger.info("AutoNavPipeline: nav mode -> %s", mode.value)

    def set_filter_mode(self, mode_str: str) -> None:
        """Switch filter mode: 'moving_avg' or 'kalman'."""
        try:
            mode = FilterMode(mode_str)
        except ValueError:
            logger.warning("AutoNavPipeline: unknown filter mode: %s", mode_str)
            return
        with self._lock:
            self._filter_mode = mode
        logger.info("AutoNavPipeline: filter mode -> %s", mode.value)

    def get_status(self) -> AutoNavFrame:
        """Return current navigation status (thread-safe)."""
        with self._lock:
            wp = self._wp_mgr.current
            prog = list(self._wp_mgr.progress)
            distance_m = None
            target_bearing = None
            bearing_error = None
            tolerance_m = None
            position = None

            pos = self._get_filtered_position_unsafe()
            if pos != (0.0, 0.0):
                position = {"lat": round(pos[0], 8), "lon": round(pos[1], 8)}
                if wp is not None:
                    distance_m = GeoUtils.haversine(pos[0], pos[1], wp.lat, wp.lon)
                    target_bearing = GeoUtils.bearing(pos[0], pos[1], wp.lat, wp.lon)
                    tolerance_m = wp.tolerance_m
                    if self._robot_bearing is not None:
                        bearing_error = GeoUtils.normalize_angle(target_bearing - self._robot_bearing)

            return AutoNavFrame(
                state=self._state.name.lower(),
                nav_mode=self._nav_mode.value,
                filter_mode=self._filter_mode.value,
                progress=prog,
                distance_m=distance_m,
                target_bearing=target_bearing,
                bearing_error=bearing_error,
                tolerance_m=tolerance_m,
                fix_quality=self._fix_quality,
                position=position,
                linear_cmd=self._last_linear_cmd,
                angular_cmd=self._last_angular_cmd,
            )

    # ── Sensor Callbacks ─────────────────────────────────────────────────────

    def on_imu(self, imu_data: dict) -> None:
        """IMU callback (~20 Hz). Updates bearing and drives control loop."""
        try:
            euler = imu_data.get("euler", {})
            yaw = euler.get("yaw")
            if yaw is None:
                return

            # heading_deg = (90 - yaw) % 360  (imu_bridge convention)
            bearing = (90 - yaw) % 360.0
            now = time.time()
            dt = now - self._last_imu_ts if self._last_imu_ts > 0 else 0.05
            self._last_imu_ts = now

            with self._lock:
                self._robot_bearing = bearing

                # Kalman prediction with IMU acceleration
                if self._filter_mode == FilterMode.KALMAN and self._kf_filter.is_ready:
                    quat = imu_data.get("quaternion", {})
                    # Use euler-derived bearing to rotate body accel to NED
                    # (simplified: assume flat terrain)
                    rad = math.radians(bearing)
                    accel = imu_data.get("accel", {})
                    ax = accel.get("x", 0.0) if accel else 0.0
                    ay = accel.get("y", 0.0) if accel else 0.0
                    a_north = ax * math.cos(rad) - ay * math.sin(rad)
                    a_east = ax * math.sin(rad) + ay * math.cos(rad)
                    self._kf_filter.predict(dt, a_north, a_east)

                if self._state != NavState.NAVIGATING:
                    return

            self._control_step(now)
        except Exception as exc:
            logger.error("AutoNavPipeline.on_imu: %s", exc)

    def on_rtk(self, rtk_data: dict) -> None:
        """RTK GPS callback (~1 Hz). Updates GPS filter."""
        try:
            lat = rtk_data.get("lat")
            lon = rtk_data.get("lon")
            fq = rtk_data.get("fix_quality", 0)

            with self._lock:
                self._fix_quality = fq

            if lat is None or lon is None or fq < 1:
                return

            with self._lock:
                self._last_gps_ts = time.time()
                self._gps_warning_sent = False

                if self._filter_mode == FilterMode.MOVING_AVG:
                    self._ma_filter.update(lat, lon)
                else:
                    self._kf_filter.update(lat, lon, fq)
        except Exception as exc:
            logger.error("AutoNavPipeline.on_rtk: %s", exc)

    def on_odometry(self, v_linear: float, v_angular: float) -> None:
        """Robot odometry callback (~20 Hz). Kalman velocity observation."""
        try:
            with self._lock:
                if self._filter_mode != FilterMode.KALMAN or not self._kf_filter.is_ready:
                    return
                bearing = self._robot_bearing
                if bearing is None:
                    return
                bearing_rad = math.radians(bearing)
                v_north = v_linear * math.cos(bearing_rad)
                v_east = v_linear * math.sin(bearing_rad)
                self._kf_filter.update_velocity(v_north, v_east)
        except Exception as exc:
            logger.error("AutoNavPipeline.on_odometry: %s", exc)

    # ── Internal Control Step ────────────────────────────────────────────────

    def _control_step(self, now: float) -> None:
        """Core control loop, called by on_imu at ~20 Hz."""
        with self._lock:
            if self._state != NavState.NAVIGATING:
                return

            # GPS timeout detection
            if self._last_gps_ts > 0:
                gps_age = now - self._last_gps_ts
                if gps_age > self._gps_timeout_s and not self._gps_warning_sent:
                    logger.warning("AutoNavPipeline: GPS timeout %.1fs, pausing", gps_age)
                    self._gps_warning_sent = True
                    self._last_linear_cmd = 0.0
                    self._last_angular_cmd = 0.0
                    if self._send_velocity:
                        self._send_velocity(0.0, 0.0)
                    return
                if self._gps_warning_sent:
                    return

            # Get filtered position
            pos = self._get_filtered_position_unsafe()
            if pos == (0.0, 0.0):
                if now - self._last_diag_ts >= 5.0:
                    self._last_diag_ts = now
                    logger.warning("AutoNavPipeline: waiting for GPS filter readiness")
                return

            robot_lat, robot_lon = pos

            if self._robot_bearing is None:
                if now - self._last_diag_ts >= 5.0:
                    self._last_diag_ts = now
                    logger.warning("AutoNavPipeline: compass bearing not available")
                return

            wp = self._wp_mgr.current
            if wp is None:
                return

            dt = now - self._last_control_ts
            self._last_control_ts = now
            if dt <= 0 or dt > 1.0:
                dt = 0.05

            # Distance and arrival
            distance = GeoUtils.haversine(robot_lat, robot_lon, wp.lat, wp.lon)
            switched = self._wp_mgr.update(distance, self._fix_quality)

            if switched:
                if self._wp_mgr.is_finished:
                    self._state = NavState.FINISHED
                    logger.info("AutoNavPipeline: all %d waypoints completed!", self._wp_mgr.progress[1])
                    self._last_linear_cmd = 0.0
                    self._last_angular_cmd = 0.0
                    if self._send_velocity:
                        self._send_velocity(0.0, 0.0)
                    return
                else:
                    self._p2p_ctrl.reset()
                    self._pp_ctrl.reset()
                    wp = self._wp_mgr.current

            # Compute control
            bearing = self._robot_bearing
            if self._nav_mode == NavMode.PURE_PURSUIT:
                linear, angular = self._pp_ctrl.compute(
                    robot_lat, robot_lon, bearing,
                    self._wp_mgr.waypoints, self._wp_mgr.current_index, dt,
                )
            else:
                linear, angular = self._p2p_ctrl.compute(
                    robot_lat, robot_lon, bearing, wp, dt,
                )

            linear = max(-self._max_lin, min(self._max_lin, linear))
            angular = max(-self._max_ang, min(self._max_ang, angular))

            self._last_linear_cmd = linear
            self._last_angular_cmd = angular

            # Periodic status log
            if now - self._last_status_log_ts >= 5.0:
                self._last_status_log_ts = now
                prog = self._wp_mgr.progress
                target_b = GeoUtils.bearing(robot_lat, robot_lon, wp.lat, wp.lon)
                berr = GeoUtils.normalize_angle(target_b - bearing)
                logger.info(
                    "[NAV STATUS] WP#%d/%d dist=%.1fm berr=%+.1f deg "
                    "lin=%.2f ang=%.2f fq=%d",
                    prog[0], prog[1], distance, berr, linear, angular, self._fix_quality,
                )

        # Send velocity (outside lock)
        if self._send_velocity:
            self._send_velocity(linear, angular)

    def _get_filtered_position_unsafe(self) -> tuple[float, float]:
        """Get filtered GPS position. Must be called with lock held."""
        if self._filter_mode == FilterMode.MOVING_AVG:
            if not self._ma_filter.is_ready:
                return 0.0, 0.0
            return self._ma_filter.get_position()
        else:
            if not self._kf_filter.is_ready:
                return 0.0, 0.0
            return self._kf_filter.get_position()


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class _WsClient:
    """Base reconnecting WebSocket client."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, name: str, ws_url: str) -> None:
        self._name = name
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._connected = False

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def connected(self) -> bool:
        return self._connected

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("%s: connected to %s", self._name, self._url)
                    self._connected = True
                    async for msg in ws:
                        try:
                            data = json.loads(msg)
                            with self._lock:
                                self._latest = data
                            self._on_message(data)
                        except json.JSONDecodeError as exc:
                            logger.warning("%s: JSON parse error: %s", self._name, exc)
            except Exception as exc:
                logger.warning(
                    "%s: connection to %s failed: %s — retrying in %.0fs",
                    self._name, self._url, exc, self.RECONNECT_DELAY_S,
                )
            finally:
                self._connected = False
            await asyncio.sleep(self.RECONNECT_DELAY_S)

    def _on_message(self, data: dict) -> None:
        """Override in subclasses to process each incoming message."""


class ImuWsClient(_WsClient):
    """WebSocket client for imu_bridge; calls pipeline.on_imu() per frame."""

    def __init__(self, ws_url: str, pipeline: AutoNavPipeline) -> None:
        super().__init__("ImuWsClient", ws_url)
        self._pipeline = pipeline

    def _on_message(self, data: dict) -> None:
        self._pipeline.on_imu(data)


class RtkWsClient(_WsClient):
    """WebSocket client for rtk_bridge; calls pipeline.on_rtk() per frame."""

    def __init__(self, ws_url: str, pipeline: AutoNavPipeline) -> None:
        super().__init__("RtkWsClient", ws_url)
        self._pipeline = pipeline

    def _on_message(self, data: dict) -> None:
        self._pipeline.on_rtk(data)


class RobotWsClient:
    """
    Bidirectional WebSocket client for robot_bridge.

    Reads odometry data and sends velocity commands.
    """

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str, pipeline: AutoNavPipeline) -> None:
        self._url = ws_url
        self._pipeline = pipeline
        self._ws = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    async def send_command(self, speed: float, ang_rate: float) -> None:
        """Send velocity command to robot_bridge."""
        if self._ws is not None:
            try:
                msg = json.dumps({
                    "velocity": {"speed": round(speed, 3), "ang_rate": round(ang_rate, 3)},
                })
                await self._ws.send(msg)
            except Exception as exc:
                logger.warning("RobotWsClient: send failed: %s", exc)

    async def run(self) -> None:
        """Connect to robot_bridge and read odometry data."""
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("RobotWsClient: connected to %s", self._url)
                    self._ws = ws
                    self._connected = True
                    # ── INPUT ──────────────────────────────────────────────────────────
                    # Robot telemetry frames arrive here from robot_bridge.
                    # If robot data looks wrong, start debugging from this line.
                    async for msg in ws:
                    # ───────────────────────────────────────────────────────────────────
                        try:
                            data = json.loads(msg)
                            v = data.get("meas_speed", 0.0)
                            w = data.get("meas_ang_rate", 0.0)
                            self._pipeline.on_odometry(v, w)
                        except json.JSONDecodeError as exc:
                            logger.warning("RobotWsClient: JSON parse error: %s", exc)
            except Exception as exc:
                logger.warning(
                    "RobotWsClient: connection to %s failed: %s — retrying in %.0fs",
                    self._url, exc, self.RECONNECT_DELAY_S,
                )
            finally:
                self._ws = None
                self._connected = False
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class AutoNavLoop:
    """Asyncio coroutine: sample pipeline status at 20 Hz and enqueue for broadcast."""

    def __init__(self, pipeline: AutoNavPipeline, queue: asyncio.Queue, hz: float = 4.0) -> None:
        self._pipeline = pipeline
        self._queue = queue
        self._period = 1.0 / max(hz, 0.5)

    async def run(self) -> None:
        while True:
            status = self._pipeline.get_status()
            try:
                self._queue.put_nowait(json.dumps(status.to_dict()))
            except asyncio.QueueFull:
                pass
            await asyncio.sleep(self._period)


class WebSocketServer:
    """WebSocket server for browser clients (status broadcast + control)."""

    def __init__(self, port: int, queue: asyncio.Queue, on_client_message=None) -> None:
        self._port = port
        self._queue = queue
        self._clients: set = set()
        self._on_client_message = on_client_message

    async def broadcast(self) -> None:
        while True:
            message = await self._queue.get()
            if not self._clients:
                continue
            dead = set()
            for ws in self._clients.copy():
                try:
                    # ── OUTPUT ─────────────────────────────────────────────────────────
                    # AutoNav status JSON exits the program here to the browser.
                    # If the browser receives wrong data, start debugging here.
                    await ws.send(message)
                    # ───────────────────────────────────────────────────────────────────
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
                except Exception as exc:
                    logger.warning("WebSocketServer: send error: %s", exc)
                    dead.add(ws)
            self._clients.difference_update(dead)

    async def handle_client(self, websocket) -> None:
        addr = websocket.remote_address
        logger.info("WebSocketServer: client connected: %s", addr)
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                if self._on_client_message is not None:
                    try:
                        await self._on_client_message(raw)
                    except Exception as exc:
                        logger.warning("WebSocketServer: message handler error: %s", exc)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as exc:
            logger.warning("WebSocketServer: handler error: %s", exc)
        finally:
            self._clients.discard(websocket)
            logger.info("WebSocketServer: client disconnected: %s", addr)

    async def serve(self) -> None:
        logger.info("WebSocketServer: starting on ws://localhost:%d", self._port)
        asyncio.create_task(self.broadcast())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("WebSocketServer: running.")
            await asyncio.Future()


class HttpFileServer:
    """Serve static files from web_static/ over HTTP."""

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
                logger.info("HttpFileServer: serving %s on port %d", self._static_dir, self._port)
                httpd.serve_forever()
        except Exception as exc:
            logger.error("HttpFileServer: failed to start on port %d: %s", self._port, exc)


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

class AutoNavBridge:
    """
    Top-level orchestrator.  Assembles and starts all components:
      1. AutoNavPipeline   — navigation state machine
      2. ImuWsClient       — reads IMU data, drives control loop
      3. RtkWsClient       — reads GPS data, updates filters
      4. RobotWsClient     — bidirectional: reads odometry, sends velocity
      5. AutoNavLoop       — status sampling coroutine
      6. HttpFileServer    — serves web_static/
      7. WebSocketServer   — broadcasts status to browser
    """

    def __init__(
        self,
        ws_port: int,
        imu_ws_url: str,
        rtk_ws_url: str,
        robot_ws_url: str,
        max_linear_vel: float,
        max_angular_vel: float,
        pid_kp: float,
        pid_ki: float,
        pid_kd: float,
        lookahead_m: float,
        decel_radius_m: float,
        arrive_frames: int,
        gps_timeout_s: float,
        ma_window: int,
        static_dir: Path,
    ) -> None:
        self._http_port = ws_port
        self._ws_port = ws_port + 1
        self._imu_ws_url = imu_ws_url
        self._rtk_ws_url = rtk_ws_url
        self._robot_ws_url = robot_ws_url
        self._static_dir = static_dir

        self._nav_params = dict(
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            pid_kp=pid_kp, pid_ki=pid_ki, pid_kd=pid_kd,
            lookahead_m=lookahead_m,
            decel_radius_m=decel_radius_m,
            arrive_frames=arrive_frames,
            gps_timeout_s=gps_timeout_s,
            ma_window=ma_window,
        )

        self._pipeline: AutoNavPipeline | None = None
        self._robot_client: RobotWsClient | None = None

    def run(self) -> None:
        """Synchronous entry point — blocks until Ctrl-C."""
        logger.info(
            "AutoNavBridge starting | http=:%d ws=:%d imu=%s rtk=%s robot=%s",
            self._http_port, self._ws_port,
            self._imu_ws_url, self._rtk_ws_url, self._robot_ws_url,
        )

        if self._static_dir.exists():
            http_server = HttpFileServer(self._static_dir, self._http_port)
            threading.Thread(target=http_server.run, daemon=True, name="http-server").start()
        else:
            logger.warning("web_static not found at %s", self._static_dir)

        url = f"http://localhost:{self._http_port}"
        logger.info("Open browser at %s", url)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("AutoNavBridge: stopped by user.")

    async def _run_async(self) -> None:
        queue: asyncio.Queue = asyncio.Queue(maxsize=20)

        # Create pipeline with velocity callback that forwards to robot_bridge
        self._robot_client = None  # will be set below

        def _send_velocity(speed: float, ang_rate: float):
            if self._robot_client is not None:
                asyncio.run_coroutine_threadsafe(
                    self._robot_client.send_command(speed, ang_rate),
                    asyncio.get_event_loop(),
                )

        self._pipeline = AutoNavPipeline(
            **self._nav_params,
            send_velocity_fn=_send_velocity,
        )

        # WS clients
        imu_client = ImuWsClient(self._imu_ws_url, self._pipeline)
        rtk_client = RtkWsClient(self._rtk_ws_url, self._pipeline)
        self._robot_client = RobotWsClient(self._robot_ws_url, self._pipeline)

        # Status loop
        nav_loop = AutoNavLoop(self._pipeline, queue, hz=4.0)

        # WS server
        ws_server = WebSocketServer(
            port=self._ws_port, queue=queue,
            on_client_message=self._handle_client_message,
        )

        await asyncio.gather(
            imu_client.run(),
            rtk_client.run(),
            self._robot_client.run(),
            nav_loop.run(),
            ws_server.serve(),
        )

    async def _handle_client_message(self, raw: str) -> None:
        """Parse and dispatch JSON control messages from the browser."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning("AutoNavBridge: invalid JSON: %s", exc)
            return

        pipeline = self._pipeline
        if pipeline is None:
            return

        msg_type = msg.get("type", "")

        if msg_type == "load_waypoints":
            csv_text = msg.get("csv", "")
            pipeline.load_waypoints(csv_text)

        elif msg_type == "start_nav":
            force = msg.get("force", False)
            pipeline.start(force=force)

        elif msg_type == "stop_nav":
            pipeline.stop()

        elif msg_type == "set_nav_mode":
            pipeline.set_nav_mode(msg.get("mode", "p2p"))

        elif msg_type == "set_filter_mode":
            pipeline.set_filter_mode(msg.get("mode", "moving_avg"))

        elif msg_type == "generate_coverage":
            try:
                boundary = msg.get("boundary", [])
                boundary_tuples = [(p["lat"], p["lon"]) for p in boundary]
                planner = CoveragePlanner(
                    boundary=boundary_tuples,
                    row_spacing=msg.get("row_spacing", 1.0),
                    direction_deg=msg.get("direction_deg", 0.0),
                    overlap=msg.get("overlap", 0.0),
                    tolerance_m=msg.get("tolerance_m", 1.0),
                    max_speed=msg.get("max_speed", 0.5),
                )
                csv_text = planner.generate_csv()
                pipeline.load_waypoints(csv_text)
                logger.info("AutoNavBridge: coverage path generated and loaded")
            except Exception as exc:
                logger.error("AutoNavBridge: coverage generation failed: %s", exc)

        else:
            logger.debug("AutoNavBridge: unknown message type: %s", msg_type)


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Autonomous navigation engine: GPS+IMU filtered PID/PurePursuit control"
    )
    parser.add_argument("--ws-port", type=int, default=8805,
                        help="HTTP port; WS uses ws-port+1 (default: 8805)")
    parser.add_argument("--imu-ws", default="ws://localhost:8766",
                        help="IMU bridge WS URL (default: ws://localhost:8766)")
    parser.add_argument("--rtk-ws", default="ws://localhost:8776",
                        help="RTK bridge WS URL (default: ws://localhost:8776)")
    parser.add_argument("--robot-ws", default="ws://localhost:8796",
                        help="Robot bridge WS URL (default: ws://localhost:8796)")
    parser.add_argument("--max-linear-vel", type=float, default=1.0,
                        help="Max linear velocity m/s (default: 1.0)")
    parser.add_argument("--max-angular-vel", type=float, default=1.0,
                        help="Max angular velocity rad/s (default: 1.0)")
    parser.add_argument("--pid-kp", type=float, default=0.8, help="PID Kp (default: 0.8)")
    parser.add_argument("--pid-ki", type=float, default=0.01, help="PID Ki (default: 0.01)")
    parser.add_argument("--pid-kd", type=float, default=0.05, help="PID Kd (default: 0.05)")
    parser.add_argument("--lookahead-m", type=float, default=2.0,
                        help="Pure pursuit lookahead distance m (default: 2.0)")
    parser.add_argument("--decel-radius-m", type=float, default=3.0,
                        help="P2P deceleration radius m (default: 3.0)")
    parser.add_argument("--arrive-frames", type=int, default=5,
                        help="Consecutive frames for arrival (default: 5)")
    parser.add_argument("--gps-timeout-s", type=float, default=5.0,
                        help="GPS timeout seconds (default: 5.0)")
    parser.add_argument("--ma-window", type=int, default=10,
                        help="Moving average filter window (default: 10)")
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    AutoNavBridge(
        ws_port=args.ws_port,
        imu_ws_url=args.imu_ws,
        rtk_ws_url=args.rtk_ws,
        robot_ws_url=args.robot_ws,
        max_linear_vel=args.max_linear_vel,
        max_angular_vel=args.max_angular_vel,
        pid_kp=args.pid_kp,
        pid_ki=args.pid_ki,
        pid_kd=args.pid_kd,
        lookahead_m=args.lookahead_m,
        decel_radius_m=args.decel_radius_m,
        arrive_frames=args.arrive_frames,
        gps_timeout_s=args.gps_timeout_s,
        ma_window=args.ma_window,
        static_dir=Path(__file__).parent / "web_static",
    ).run()
