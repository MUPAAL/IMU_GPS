"""
controller — Navigation controller

PIDController       : Generic PID (angular velocity error → angular velocity output)
P2PController       : Point-to-point control (align current heading to target)
PurePursuitController : Pure Pursuit control (lookahead point tracking, multi-waypoint)
"""

import logging
import math
import sys
from pathlib import Path
from typing import Tuple

# Add parent directories to path to import config
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from config import (
    NAV_PID_KP, NAV_PID_KI, NAV_PID_KD,
    NAV_DECEL_RADIUS_M, NAV_LOOKAHEAD_M,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
    # AUTONAV aliases (for backward compatibility with 00_robot_side)
    AUTONAV_PID_KP, AUTONAV_PID_KI, AUTONAV_PID_KD,
    AUTONAV_DECEL_RADIUS_M, AUTONAV_LOOKAHEAD_M,
    AUTONAV_MAX_LINEAR_VEL, AUTONAV_MAX_ANGULAR_VEL,
)
from navigation.geo_utils import (
    haversine_distance, bearing_to_target, normalize_angle, project_point_on_segment,
)
from navigation.waypoint import Waypoint

logger = logging.getLogger(__name__)


class PIDController:
    """Generic PID controller (output with clamping).

    Args:
        kp, ki, kd     : PID gains
        integral_limit : Integral windup limit (prevent saturation)
        output_limit   : Output clamping limit
    """

    def __init__(
        self,
        kp:             float = NAV_PID_KP,
        ki:             float = NAV_PID_KI,
        kd:             float = NAV_PID_KD,
        integral_limit: float = 30.0,
        output_limit:   float = MAX_ANGULAR_VEL,
    ) -> None:
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._int_lim = integral_limit
        self._out_lim = output_limit

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._first: bool = True

    def compute(self, error: float, dt: float) -> float:
        """Compute PID output.

        Args:
            error : Error (for angular velocity: angle error in degrees)
            dt    : Time step (seconds)

        Returns:
            Control output (clamped)
        """
        if dt <= 0:
            return 0.0

        # Integral term
        self._integral += error * dt
        self._integral = max(-self._int_lim, min(self._int_lim, self._integral))

        # Derivative term (skip first frame)
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
    """Point-to-point (P2P) controller.

    Algorithm:
        1. bearing_error = normalize_angle(target bearing − robot current heading)
        2. angular_cmd   = PID(bearing_error, dt)
           (positive error = target to right = turn right = positive angular velocity)
        3. speed_factor  = clip(distance / NAV_DECEL_RADIUS_M, 0, 1)
           heading_factor = max(0, 1 − |bearing_error| / 90)
           linear_cmd    = max_speed × speed_factor × heading_factor
    """

    def __init__(self) -> None:
        self._pid = PIDController()

    def compute(
        self,
        robot_lat:  float,
        robot_lon:  float,
        robot_bearing: float,
        target_wp:  Waypoint,
        dt:         float,
    ) -> Tuple[float, float]:
        """Compute (linear_cmd, angular_cmd).

        Args:
            robot_lat, robot_lon : Robot current position (decimal degrees)
            robot_bearing        : Robot current heading (degrees, 0=north, clockwise)
            target_wp            : Target waypoint
            dt                   : Control period (seconds)

        Returns:
            (linear_cmd, angular_cmd) in m/s / rad/s
        """
        distance = haversine_distance(robot_lat, robot_lon, target_wp.lat, target_wp.lon)
        target_bearing = bearing_to_target(robot_lat, robot_lon, target_wp.lat, target_wp.lon)
        bearing_error = normalize_angle(target_bearing - robot_bearing)

        angular_cmd = self._pid.compute(bearing_error, dt)

        speed_factor   = min(1.0, distance / (NAV_DECEL_RADIUS_M + 1e-6))
        heading_factor = max(0.0, 1.0 - abs(bearing_error) / 90.0)
        max_spd = min(target_wp.max_speed, MAX_LINEAR_VEL)
        linear_cmd = max_spd * speed_factor * heading_factor

        return linear_cmd, angular_cmd

    def reset(self) -> None:
        self._pid.reset()


class PurePursuitController:
    """Pure Pursuit controller.

    Computes lookahead point L on path segment [prev_wp → current_wp],
    steers towards L instead of endpoint for smooth trajectory.
    Degrades to P2P when single waypoint or projection fails.

    Lookahead distance: NAV_LOOKAHEAD_M
    """

    def __init__(self) -> None:
        self._p2p = P2PController()

    def compute(
        self,
        robot_lat:    float,
        robot_lon:    float,
        robot_bearing: float,
        waypoints:    list,       # list[Waypoint]
        current_idx:  int,
        dt:           float,
    ) -> Tuple[float, float]:
        """Compute (linear_cmd, angular_cmd).

        Args:
            robot_lat, robot_lon : Robot current position
            robot_bearing        : Robot current heading (degrees)
            waypoints            : Complete waypoint list
            current_idx          : Current target waypoint index
            dt                   : Control period (seconds)

        Returns:
            (linear_cmd, angular_cmd)
        """
        if current_idx >= len(waypoints):
            return 0.0, 0.0

        current_wp = waypoints[current_idx]

        # Single waypoint or first segment without previous → fallback to P2P
        if current_idx == 0 or len(waypoints) < 2:
            logger.debug("PurePursuitController: single waypoint, fallback to P2P")
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        prev_wp = waypoints[current_idx - 1]

        # Project robot onto path segment [prev → current]
        try:
            proj_lat, proj_lon, t = project_point_on_segment(
                robot_lat, robot_lon,
                prev_wp.lat, prev_wp.lon,
                current_wp.lat, current_wp.lon,
            )
        except Exception as e:
            logger.warning(f"PurePursuitController: projection calculation failed, fallback to P2P: {e}")
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        # Construct lookahead point: offset from projection point along path direction by NAV_LOOKAHEAD_M
        seg_bearing = bearing_to_target(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)
        seg_len     = haversine_distance(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)

        if seg_len < 0.1:
            logger.warning("PurePursuitController: path segment too short, fallback to P2P")
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        # Remaining path length (from projection point to current waypoint)
        dist_to_wp = haversine_distance(proj_lat, proj_lon, current_wp.lat, current_wp.lon)

        if dist_to_wp < NAV_LOOKAHEAD_M:
            # Lookahead point exceeds current waypoint, use it directly as lookahead target
            lookahead_lat = current_wp.lat
            lookahead_lon = current_wp.lon
        else:
            # Lookahead point = projection point offset along path direction by NAV_LOOKAHEAD_M
            cos_lat = math.cos(math.radians(proj_lat))
            rad_bearing = math.radians(seg_bearing)
            dlat_m = NAV_LOOKAHEAD_M * math.cos(rad_bearing)
            dlon_m = NAV_LOOKAHEAD_M * math.sin(rad_bearing)
            lookahead_lat = proj_lat + math.degrees(dlat_m / 6_371_000.0)
            lookahead_lon = proj_lon + math.degrees(dlon_m / (6_371_000.0 * cos_lat + 1e-12))

        # Construct temporary "virtual" waypoint for P2P computation
        from dataclasses import replace
        lookahead_wp = replace(current_wp, lat=lookahead_lat, lon=lookahead_lon)

        return self._p2p.compute(robot_lat, robot_lon, robot_bearing, lookahead_wp, dt)

    def reset(self) -> None:
        self._p2p.reset()
