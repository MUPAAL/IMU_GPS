"""
heading_controller.py — Multi-mode heading and path tracking control.

Supports two navigation modes:
1. Point-to-Point (P2P): Direct heading tracking via PID
2. Pure Pursuit: Multi-waypoint path following with lookahead steering

Reuses PIDController from 00_robot_side/navigation/controller.py and geometric
utilities from geo_utils (bearing, distance, path projection).
"""

import sys
from pathlib import Path
from typing import Tuple, Optional, List
from dataclasses import dataclass

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config

from navigation.controller import PIDController
from navigation.geo_utils import (
    normalize_angle,
    bearing_to_target,
    haversine_distance,
    project_point_on_segment,
)


@dataclass
class Waypoint:
    """Navigation waypoint with target bearing and tolerance."""
    lat: float
    lon: float
    heading_tolerance_deg: float = 5.0
    max_speed: float = config.PATHFOLLOWER_MAX_LINEAR_VEL


class P2PController:
    """
    Point-to-Point (heading-based) controller.

    Tracks a single target heading. Linear velocity can be set independently.
    """

    def __init__(
        self,
        kp: float = config.PATHFOLLOWER_PID_KP,
        ki: float = config.PATHFOLLOWER_PID_KI,
        kd: float = config.PATHFOLLOWER_PID_KD,
        max_angular_vel: float = config.PATHFOLLOWER_MAX_ANGULAR_VEL,
    ):
        self._pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            output_limit=max_angular_vel,
        )
        self._target_heading = 0.0
        self._current_heading = 0.0

    def set_target_heading(self, heading_deg: float) -> None:
        self._target_heading = heading_deg

    def compute(self, current_heading_deg: float, dt: float) -> float:
        """Compute angular velocity to track target heading."""
        self._current_heading = current_heading_deg
        error_deg = normalize_angle(self._target_heading - self._current_heading)
        angular_cmd = self._pid.compute(error_deg, dt)
        return angular_cmd

    def reset(self) -> None:
        self._pid.reset()

    @property
    def target_heading(self) -> float:
        return self._target_heading

    @property
    def current_heading(self) -> float:
        return self._current_heading

    @property
    def heading_error(self) -> float:
        return normalize_angle(self._target_heading - self._current_heading)


class PurePursuitController:
    """
    Pure Pursuit multi-waypoint path follower.

    Projects robot onto current path segment, computes lookahead point at
    NAV_LOOKAHEAD_M distance ahead, and steers toward lookahead instead of
    endpoint for smooth trajectories. Degrades to P2P if needed.
    """

    def __init__(
        self,
        kp: float = config.PATHFOLLOWER_NAV_PID_KP,
        ki: float = config.PATHFOLLOWER_NAV_PID_KI,
        kd: float = config.PATHFOLLOWER_NAV_PID_KD,
        max_angular_vel: float = config.PATHFOLLOWER_MAX_ANGULAR_VEL,
        lookahead_m: float = config.PATHFOLLOWER_NAV_LOOKAHEAD_M,
        decel_radius_m: float = config.PATHFOLLOWER_NAV_DECEL_RADIUS_M,
    ):
        self._pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            output_limit=max_angular_vel,
        )
        self._lookahead_m = lookahead_m
        self._decel_radius_m = decel_radius_m
        self._max_angular_vel = max_angular_vel
        self._max_linear_vel = config.PATHFOLLOWER_MAX_LINEAR_VEL

        # State
        self._waypoints: List[Waypoint] = []
        self._current_idx = 0
        self._current_heading = 0.0
        self._last_target_bearing = 0.0

    def load_waypoints(self, waypoints: List[Waypoint]) -> None:
        """Load waypoint list for path following."""
        self._waypoints = waypoints
        self._current_idx = 0
        self._pid.reset()

    def compute(
        self,
        robot_lat: float,
        robot_lon: float,
        robot_bearing: float,
        dt: float,
    ) -> Tuple[float, float]:
        """
        Compute (linear_velocity, angular_velocity) commands.

        Falls back to P2P if waypoint list empty or single waypoint.

        Args:
            robot_lat, robot_lon: Current GPS position (decimal degrees)
            robot_bearing: Current heading from IMU (degrees, 0=north, CW+)
            dt: Control period (seconds)

        Returns:
            (linear_cmd, angular_cmd) tuple clamped to velocity limits
        """
        self._current_heading = robot_bearing

        # Fallback: no waypoints or single waypoint
        if not self._waypoints or len(self._waypoints) == 1:
            return self._fallback_p2p(dt)

        # Clamp current index
        if self._current_idx >= len(self._waypoints):
            self._current_idx = len(self._waypoints) - 1

        current_wp = self._waypoints[self._current_idx]
        prev_wp = (
            self._waypoints[self._current_idx - 1]
            if self._current_idx > 0
            else None
        )

        # Compute lookahead point
        lookahead_lat, lookahead_lon = self._compute_lookahead(
            robot_lat, robot_lon, prev_wp, current_wp
        )

        # Compute bearing to lookahead point
        target_bearing = bearing_to_target(
            robot_lat, robot_lon, lookahead_lat, lookahead_lon
        )
        self._last_target_bearing = target_bearing

        # Compute steering via PID
        bearing_error = normalize_angle(target_bearing - robot_bearing)
        angular_cmd = self._pid.compute(bearing_error, dt)

        # Compute distance to current waypoint
        dist_to_wp = haversine_distance(robot_lat, robot_lon, current_wp.lat, current_wp.lon)

        # Linear velocity: decelerate when approaching
        speed_factor = min(1.0, dist_to_wp / self._decel_radius_m)
        heading_factor = max(0.0, 1.0 - abs(bearing_error) / 90.0)
        linear_cmd = current_wp.max_speed * speed_factor * heading_factor

        return (
            max(-self._max_linear_vel, min(self._max_linear_vel, linear_cmd)),
            max(-self._max_angular_vel, min(self._max_angular_vel, angular_cmd)),
        )

    def _compute_lookahead(
        self,
        robot_lat: float,
        robot_lon: float,
        prev_wp: Optional[Waypoint],
        current_wp: Waypoint,
    ) -> Tuple[float, float]:
        """
        Compute lookahead point at NAV_LOOKAHEAD_M distance along segment.

        Falls back to current waypoint if projection fails.
        """
        if prev_wp is None:
            return current_wp.lat, current_wp.lon

        try:
            # Project robot onto segment [prev_wp → current_wp]
            proj_lat, proj_lon, t = project_point_on_segment(
                robot_lat,
                robot_lon,
                prev_wp.lat,
                prev_wp.lon,
                current_wp.lat,
                current_wp.lon,
            )

            # Distance from projection to current waypoint
            dist_remaining = haversine_distance(proj_lat, proj_lon, current_wp.lat, current_wp.lon)

            # If remaining < lookahead, return current waypoint
            if dist_remaining < self._lookahead_m:
                return current_wp.lat, current_wp.lon

            # Otherwise, offset lookahead distance along segment toward current_wp
            segment_bearing = bearing_to_target(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)

            # Offset from proj point along bearing
            offset_distance = self._lookahead_m
            lat1_rad = proj_lat * 3.14159 / 180.0
            lon1_rad = proj_lon * 3.14159 / 180.0
            bearing_rad = segment_bearing * 3.14159 / 180.0

            # Haversine inverse: compute new point at offset distance
            earth_radius_m = 6371000.0
            lat2_rad = (
                lat1_rad
                + (offset_distance / earth_radius_m) * 3.14159 / 180.0
            )
            lon2_rad = lon1_rad + (
                (offset_distance / earth_radius_m)
                * (3.14159 / 180.0)
                / __import__("math").cos(lat1_rad)
            )

            return (lat2_rad * 180.0 / 3.14159, lon2_rad * 180.0 / 3.14159)
        except Exception:
            # Projection failed, fallback to current waypoint
            return current_wp.lat, current_wp.lon

    def _fallback_p2p(self, dt: float) -> Tuple[float, float]:
        """Fallback to P2P when waypoints unavailable."""
        bearing_error = normalize_angle(
            self._last_target_bearing - self._current_heading
        )
        angular_cmd = self._pid.compute(bearing_error, dt)
        return (0.0, angular_cmd)

    def advance_waypoint(self) -> bool:
        """Move to next waypoint. Returns True if advanced, False if at end."""
        if self._current_idx < len(self._waypoints) - 1:
            self._current_idx += 1
            return True
        return False

    def reset(self) -> None:
        self._pid.reset()

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if self._waypoints and self._current_idx < len(self._waypoints):
            return self._waypoints[self._current_idx]
        return None

    @property
    def current_index(self) -> int:
        return self._current_idx


class HeadingController:
    """
    Unified heading/path controller supporting both P2P and Pure Pursuit modes.

    Provides a single interface for heading-based (point-to-point) and
    GPS-based (waypoint following) navigation.
    """

    def __init__(
        self,
        kp: float = config.PATHFOLLOWER_PID_KP,
        ki: float = config.PATHFOLLOWER_PID_KI,
        kd: float = config.PATHFOLLOWER_PID_KD,
    ):
        self._p2p = P2PController(kp=kp, ki=ki, kd=kd)
        self._pure_pursuit = PurePursuitController(kp=kp, ki=ki, kd=kd)
        self._mode = "p2p"  # "p2p" or "pure_pursuit"

    def set_mode(self, mode: str) -> None:
        """Set navigation mode: 'p2p' or 'pure_pursuit'."""
        if mode not in ("p2p", "pure_pursuit"):
            raise ValueError(f"Invalid mode: {mode}")
        self._mode = mode
        self._p2p.reset()
        self._pure_pursuit.reset()

    def set_target_heading(self, heading_deg: float) -> None:
        """Set target heading for P2P mode."""
        self._p2p.set_target_heading(heading_deg)

    def load_waypoints(self, waypoints: List[Waypoint]) -> None:
        """Load waypoint list for Pure Pursuit mode."""
        self._pure_pursuit.load_waypoints(waypoints)

    def compute_p2p(self, current_heading_deg: float, dt: float) -> float:
        """Compute angular velocity for P2P mode (heading tracking)."""
        return self._p2p.compute(current_heading_deg, dt)

    def compute_pure_pursuit(
        self,
        robot_lat: float,
        robot_lon: float,
        robot_bearing: float,
        dt: float,
    ) -> Tuple[float, float]:
        """Compute (linear, angular) for Pure Pursuit mode."""
        return self._pure_pursuit.compute(robot_lat, robot_lon, robot_bearing, dt)

    def compute(
        self,
        current_heading_deg: float,
        robot_lat: float = 0.0,
        robot_lon: float = 0.0,
        dt: float = 0.05,
    ) -> Tuple[float, float]:
        """
        Unified compute method (for backward compatibility).

        In P2P mode: returns (0.0, angular_cmd)
        In Pure Pursuit mode: returns (linear_cmd, angular_cmd)
        """
        if self._mode == "p2p":
            angular = self.compute_p2p(current_heading_deg, dt)
            return (0.0, angular)
        else:
            return self.compute_pure_pursuit(robot_lat, robot_lon, current_heading_deg, dt)

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def target_heading(self) -> float:
        return self._p2p.target_heading

    @property
    def current_heading(self) -> float:
        return self._p2p.current_heading

    @property
    def heading_error(self) -> float:
        return self._p2p.heading_error

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        return self._pure_pursuit.current_waypoint
