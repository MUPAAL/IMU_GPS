"""
nav_engine — main state machine for autonomous navigation

NavigationEngine:
    - Receives IMU data (20 Hz) to drive the control loop
    - Receives RTK GPS data (1 Hz) to update filters
    - Manages waypoint sequence, GPS filters, and controllers
    - Sends velocity commands via send_velocity_fn
    - Broadcasts navigation status via broadcast_fn (async coroutine)

State enum:
    NavState: IDLE | NAVIGATING | FINISHED

Mode enums:
    NavMode:    P2P | PURE_PURSUIT
    FilterMode: MOVING_AVG | KALMAN
"""

import asyncio
import logging
import math
import sys
import threading
import time
from pathlib import Path
from enum import Enum, auto
from typing import Callable, Coroutine

# Add parent directories to path to import config
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from config import (
    NAV_GPS_TIMEOUT_S, MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
    # AUTONAV aliases (for backward compatibility)
    AUTONAV_GPS_TIMEOUT_S, AUTONAV_MAX_LINEAR_VEL, AUTONAV_MAX_ANGULAR_VEL,
)
from navigation.geo_utils import haversine_distance, bearing_to_target
from navigation.waypoint import WaypointManager
from navigation.gps_filter import MovingAverageFilter, KalmanFilter
from navigation.controller import P2PController, PurePursuitController

logger = logging.getLogger(__name__)


# ── Enums ─────────────────────────────────────────────────
class NavState(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    FINISHED   = auto()


class NavMode(Enum):
    P2P           = "p2p"
    PURE_PURSUIT  = "pure_pursuit"


class FilterMode(Enum):
    MOVING_AVG = "moving_avg"
    KALMAN     = "kalman"


# ── Navigation engine ─────────────────────────────────────
class NavigationEngine:
    """GPS + IMU autonomous path navigation engine.

    Args:
        send_velocity_fn : Callable[[float, float], None], sends velocity commands
        broadcast_fn     : async broadcast function async(dict) -> None
        loop             : asyncio event loop (used for cross-thread broadcast scheduling)
    """

    def __init__(
        self,
        send_velocity_fn: Callable[[float, float], None],
        broadcast_fn: Callable[[dict], Coroutine],
        loop: asyncio.AbstractEventLoop,
    ) -> None:
        self._send_velocity = send_velocity_fn
        self._broadcast     = broadcast_fn
        self._loop          = loop

        self._lock = threading.Lock()

        # State
        self._state: NavState   = NavState.IDLE
        self._nav_mode: NavMode       = NavMode.P2P
        self._filter_mode: FilterMode = FilterMode.MOVING_AVG

        # Waypoint management
        self._wp_mgr = WaypointManager()

        # Filters
        self._ma_filter  = MovingAverageFilter()
        self._kf_filter  = KalmanFilter()

        # Controllers
        self._p2p_ctrl = P2PController()
        self._pp_ctrl  = PurePursuitController()

        # IMU state
        self._robot_bearing: float | None = None  # None = not calibrated
        self._last_imu_ts:   float = 0.0
        self._last_control_ts: float = 0.0

        # RTK state
        self._fix_quality: int   = 0
        self._last_gps_ts: float = 0.0
        self._gps_warning_sent: bool = False

        # Broadcast throttling
        self._broadcast_counter: int = 0

        # Diagnostic log throttling (all timestamps in seconds since epoch)
        self._last_uncal_warn_ts:  float = 0.0   # on_imu: compass not calibrated
        self._last_diag_ts:        float = 0.0   # _control_step: filter/bearing not ready
        self._last_status_log_ts:  float = 0.0   # _control_step: periodic NAV STATUS info

    # ── Public API ────────────────────────────────────────
    def load_waypoints(self, csv_text: str) -> int:
        """Load waypoint CSV text and return the number of loaded waypoints."""
        with self._lock:
            count = self._wp_mgr.load_csv(csv_text)
            return count

    def start(self, force: bool = False) -> bool:
        """Start autonomous navigation.

        Requires available waypoints and GPS fix_quality >= 1
        (force=True can bypass the fix check).

        Args:
            force: When True, ignore fix_quality and start directly
                (for debugging / no-GPS testing).

        Returns:
            True if started successfully; False if prerequisites are not met.
        """
        with self._lock:
            if self._state == NavState.NAVIGATING:
                logger.warning("NavigationEngine: already navigating, ignoring duplicate start()")
                return False
            if self._wp_mgr.current is None:
                logger.warning("NavigationEngine: no valid waypoints, cannot start")
                return False
            if self._fix_quality < 1:
                if force:
                    logger.warning("NavigationEngine: GPS fix_quality=0, forcing start (force=True)")
                else:
                    logger.warning(f"NavigationEngine: GPS fix_quality={self._fix_quality}, insufficient for navigation")
                    return False

            self._wp_mgr.reset()
            # Do not reset filters—keep existing GPS history for immediate reuse
            # Reset controller PID states to avoid integral windup residue
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
            self._gps_warning_sent = False
            self._state = NavState.NAVIGATING
            self._last_control_ts = time.time()

        logger.info(
            f"NavigationEngine: navigation started, mode={self._nav_mode.value}, "
            f"filter={self._filter_mode.value}, waypoints={self._wp_mgr.progress[1]}"
        )
        self._schedule_broadcast()
        return True

    def stop(self) -> None:
        """Stop navigation and send a zero-velocity command."""
        with self._lock:
            if self._state == NavState.IDLE:
                return
            self._state = NavState.IDLE
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()

        self._send_velocity(0.0, 0.0)
        logger.info("NavigationEngine: navigation stopped")
        self._schedule_broadcast()

    def set_nav_mode(self, mode: NavMode) -> None:
        with self._lock:
            self._nav_mode = mode
            self._p2p_ctrl.reset()
            self._pp_ctrl.reset()
        logger.info(f"NavigationEngine: navigation mode switched -> {mode.value}")

    def set_filter_mode(self, mode: FilterMode) -> None:
        with self._lock:
            self._filter_mode = mode
        logger.info(f"NavigationEngine: filter switched -> {mode.value}")

    def get_status(self) -> dict:
        """Return the current navigation status dict (thread-safe)."""
        with self._lock:
            wp = self._wp_mgr.current
            prog = self._wp_mgr.progress
            distance_m = None
            target_bearing = None
            tolerance_m    = None

            if wp is not None and self._filter_mode == FilterMode.MOVING_AVG:
                pos = self._ma_filter.get_position()
                if pos != (0.0, 0.0):
                    distance_m     = haversine_distance(pos[0], pos[1], wp.lat, wp.lon)
                    target_bearing = bearing_to_target(pos[0], pos[1], wp.lat, wp.lon)
                    tolerance_m    = wp.tolerance_m
            elif wp is not None:
                pos = self._kf_filter.get_position()
                if pos != (0.0, 0.0):
                    distance_m     = haversine_distance(pos[0], pos[1], wp.lat, wp.lon)
                    target_bearing = bearing_to_target(pos[0], pos[1], wp.lat, wp.lon)
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

    # ── Sensor callbacks ─────────────────────────────────
    def on_imu(self, imu_data: dict) -> None:
        """IMU callback (20 Hz).

        Updates robot heading, runs the Kalman prediction step,
        and triggers the control step.
        """
        try:
            compass = imu_data.get("compass", {})
            calibrated = compass.get("calibrated", False)
            if not calibrated:
                now_check = time.time()
                with self._lock:
                    if (self._state == NavState.NAVIGATING
                            and now_check - self._last_uncal_warn_ts >= 5.0):
                        self._last_uncal_warn_ts = now_check
                        accuracy = compass.get("accuracy", 0)
                        logger.warning(
                            "NavigationEngine: navigating but compass accuracy is insufficient (accuracy=%d/3). "
                            "Robot motion is stopped. You can set COMPASS_MIN_ACCURACY=0 to accept "
                            "uncalibrated data, or complete figure-8 calibration and restart navigation.",
                            accuracy,
                        )
                return

            bearing = compass.get("bearing", 0.0)
            now = time.time()
            dt = now - self._last_imu_ts if self._last_imu_ts > 0 else 0.05
            self._last_imu_ts = now

            with self._lock:
                self._robot_bearing = bearing

                # Kalman prediction (use IMU acceleration as control input)
                if self._filter_mode == FilterMode.KALMAN and self._kf_filter.is_ready:
                    accel = imu_data.get("accel", {})
                    ax, ay = accel.get("x", 0.0), accel.get("y", 0.0)
                    # Rotate body-frame acceleration (forward=x, lateral=y) into NED
                    rad = math.radians(bearing)
                    a_north = ax * math.cos(rad) - ay * math.sin(rad)
                    a_east  = ax * math.sin(rad) + ay * math.cos(rad)
                    self._kf_filter.predict(dt, a_north, a_east)

                if self._state != NavState.NAVIGATING:
                    return

            # Control step (run outside the lock to reduce lock hold time)
            self._control_step(now)

        except Exception as e:
            logger.error(f"NavigationEngine.on_imu: {e}")

    def on_rtk(self, rtk_data: dict) -> None:
        """RTK GPS callback (1 Hz).

        Pushes GPS into the active filter and checks GPS timeout.
        """
        try:
            lat = rtk_data.get("lat")
            lon = rtk_data.get("lon")
            fq  = rtk_data.get("fix_quality", 0)

            with self._lock:
                self._fix_quality = fq

            if lat is None or lon is None or fq < 1:
                return

            with self._lock:
                self._last_gps_ts  = time.time()
                self._gps_warning_sent = False

                if self._filter_mode == FilterMode.MOVING_AVG:
                    self._ma_filter.update(lat, lon)
                else:
                    self._kf_filter.update(lat, lon, fq)

        except Exception as e:
            logger.error(f"NavigationEngine.on_rtk: {e}")

    def on_odometry(self, v_linear: float, v_angular: float) -> None:
        """Amiga TPDO1 里程计回调（约 20 Hz）。

        将机体坐标系线速度旋转到 NED 坐标系，作为速度观测传给卡尔曼滤波器，
        收紧速度状态估计，减少 GPS 漂移。

        Args:
            v_linear  : 实测线速度（m/s），正值前进
            v_angular : 实测角速度（rad/s），左转为正
        """
        try:
            with self._lock:
                if self._filter_mode != FilterMode.KALMAN:
                    return  # MA 滤波器无速度状态，跳过
                if not self._kf_filter.is_ready:
                    return  # 卡尔曼尚未初始化
                bearing = self._robot_bearing
                if bearing is None:
                    return  # 无航向无法旋转到 NED

                bearing_rad = math.radians(bearing)
                v_north = v_linear * math.cos(bearing_rad)
                v_east  = v_linear * math.sin(bearing_rad)
                self._kf_filter.update_velocity(v_north, v_east)
        except Exception as e:
            logger.error(f"NavigationEngine.on_odometry: {e}")

    # ── Control step (internal, every 50 ms) ─────────────
    def _control_step(self, now: float) -> None:
        """Core control loop, called by on_imu at about 20 Hz."""
        with self._lock:
            if self._state != NavState.NAVIGATING:
                return

            # GPS timeout detection
            if self._last_gps_ts > 0:
                gps_age = now - self._last_gps_ts
                if gps_age > NAV_GPS_TIMEOUT_S and not self._gps_warning_sent:
                    logger.warning(f"NavigationEngine: GPS timeout {gps_age:.1f}s, pausing navigation")
                    self._gps_warning_sent = True
                    self._send_velocity(0.0, 0.0)
                    self._schedule_broadcast_unsafe({"type": "nav_warning", "msg": "GPS timeout"})
                    return
                if self._gps_warning_sent:
                    return  # In timeout state; wait for GPS recovery (on_rtk clears _gps_warning_sent)

            # Get filtered position
            if self._filter_mode == FilterMode.MOVING_AVG:
                if not self._ma_filter.is_ready:
                    if now - self._last_diag_ts >= 5.0:
                        self._last_diag_ts = now
                        logger.warning(
                            "NavigationEngine: waiting for GPS filter readiness "
                            "(MovingAvg window not full), navigation paused"
                        )
                    return
                pos = self._ma_filter.get_position()
            else:
                if not self._kf_filter.is_ready:
                    if now - self._last_diag_ts >= 5.0:
                        self._last_diag_ts = now
                        logger.warning(
                            "NavigationEngine: waiting for GPS filter readiness "
                            "(Kalman not initialized), navigation paused"
                        )
                    return
                pos = self._kf_filter.get_position()

            robot_lat, robot_lon = pos

            if self._robot_bearing is None:
                if now - self._last_diag_ts >= 5.0:
                    self._last_diag_ts = now
                    logger.warning(
                        "NavigationEngine: compass bearing is not set, navigation paused. "
                        "Please verify IMU is connected or set COMPASS_MIN_ACCURACY=0"
                    )
                return

            wp = self._wp_mgr.current
            if wp is None:
                return

            dt = now - self._last_control_ts
            self._last_control_ts = now
            if dt <= 0 or dt > 1.0:
                dt = 0.05

            # Distance and arrival check
            distance = haversine_distance(robot_lat, robot_lon, wp.lat, wp.lon)
            switched = self._wp_mgr.update(distance, self._fix_quality)

            if switched:
                if self._wp_mgr.is_finished:
                    # All waypoints completed
                    self._state = NavState.FINISHED
                    total = self._wp_mgr.progress[1]
                    logger.info(f"NavigationEngine: all {total} waypoints completed!")
                    self._send_velocity(0.0, 0.0)
                    self._schedule_broadcast_unsafe({"type": "nav_complete", "total_wp": total})
                    self._schedule_broadcast_unsafe(self._get_status_unsafe())
                    return
                else:
                    # Switch to next waypoint
                    self._p2p_ctrl.reset()
                    self._pp_ctrl.reset()
                    wp = self._wp_mgr.current

            # Compute control output
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

            # Clamp to configured limits
            linear  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  linear))
            angular = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular))

            # Capture status snapshot for periodic terminal log (every 5 s)
            _status_snap = None
            if now - self._last_status_log_ts >= 5.0:
                self._last_status_log_ts = now
                prog = self._wp_mgr.progress
                target_b = bearing_to_target(robot_lat, robot_lon, wp.lat, wp.lon)
                bearing_err = ((target_b - self._robot_bearing + 180) % 360) - 180
                filt_name = (
                    "MA(ready)" if self._filter_mode == FilterMode.MOVING_AVG
                    else "Kalman(ready)"
                )
                _status_snap = (
                    prog[0], prog[1], wp.lat, wp.lon,
                    distance, bearing_err, linear, angular,
                    filt_name, self._fix_quality,
                )

        self._send_velocity(linear, angular)

        if _status_snap is not None:
            idx, total, wlat, wlon, dist, berr, lin, ang, filt_str, fq = _status_snap
            logger.info(
                "[NAV STATUS] Target: WP#%d/%d (%.6f,%.6f), Distance: %.1fm, "
                "Bearing error: %+.1f°, Linear vel: %.2f, Angular vel: %.2f, Filter: %s, GPS quality: %d",
                idx, total, wlat, wlon, dist, berr, lin, ang, filt_str, fq,
            )

        # Broadcast status at 4 Hz
        self._broadcast_counter += 1
        if self._broadcast_counter % 5 == 0:
            self._schedule_broadcast()

    # ── Broadcast helpers ────────────────────────────────
    def _schedule_broadcast(self) -> None:
        """Thread-safely schedule async broadcast from worker thread (without lock)."""
        status = self.get_status()
        asyncio.run_coroutine_threadsafe(self._broadcast(status), self._loop)

    def _schedule_broadcast_unsafe(self, msg: dict) -> None:
        """Schedule broadcast while already holding lock (called from lock scope)."""
        asyncio.run_coroutine_threadsafe(self._broadcast(msg), self._loop)

    def _get_status_unsafe(self) -> dict:
        """Internal unlocked get_status version (call only when lock is already held)."""
        wp = self._wp_mgr.current
        prog = self._wp_mgr.progress
        return {
            "type":           "nav_status",
            "state":          self._state.name.lower(),
            "progress":       list(prog),
            "distance_m":     None,
            "target_bearing": None,
            "nav_mode":       self._nav_mode.value,
            "filter_mode":    self._filter_mode.value,
            "tolerance_m":    wp.tolerance_m if wp else None,
            "fix_quality":    self._fix_quality,
        }
