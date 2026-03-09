"""
controller — 导航控制器

PIDController       : 通用 PID（角速度误差 → 角速度输出）
P2PController       : 点对点控制（用当前朝向对准目标）
PurePursuitController : Pure Pursuit 控制（前视点跟踪，跨多个航点）
"""

import logging
import math
from typing import Tuple

from config import (
    NAV_PID_KP, NAV_PID_KI, NAV_PID_KD,
    NAV_DECEL_RADIUS_M, NAV_LOOKAHEAD_M,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
)
from navigation.geo_utils import (
    haversine_distance, bearing_to_target, normalize_angle, project_point_on_segment,
)
from navigation.waypoint import Waypoint

logger = logging.getLogger(__name__)


class PIDController:
    """通用 PID 控制器（输出带限幅）。

    Args:
        kp, ki, kd     : PID 增益
        integral_limit : 积分限幅（防积分饱和）
        output_limit   : 输出限幅
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
        """计算 PID 输出。

        Args:
            error : 误差（角速度用角度误差，单位度）
            dt    : 时间步长（秒）

        Returns:
            控制输出（已限幅）
        """
        if dt <= 0:
            return 0.0

        # 积分
        self._integral += error * dt
        self._integral = max(-self._int_lim, min(self._int_lim, self._integral))

        # 微分（首帧跳过）
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
    """点对点（P2P）控制器。

    算法：
        1. bearing_error = normalize_angle(目标方位 − 机器人当前朝向)
        2. angular_cmd   = PID(bearing_error, dt)
           （正误差 = 目标在右 = 右转 = angular 为正/负视协议）
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
        """计算 (linear_cmd, angular_cmd)。

        Args:
            robot_lat, robot_lon : 机器人当前位置（十进制度）
            robot_bearing        : 机器人当前朝向（度，0=北，顺时针）
            target_wp            : 目标航点
            dt                   : 控制周期（秒）

        Returns:
            (linear_cmd, angular_cmd)，单位 m/s / rad/s
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
    """Pure Pursuit 控制器。

    在路径段 [prev_wp → current_wp] 上求前视点 L，转向 L 而不是终点，
    使轨迹平滑。退化（单航点 / 投影失败）时回落到 P2P。

    前视距离：NAV_LOOKAHEAD_M
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
        """计算 (linear_cmd, angular_cmd)。

        Args:
            robot_lat, robot_lon : 机器人当前位置
            robot_bearing        : 机器人当前朝向（度）
            waypoints            : 完整航点列表
            current_idx          : 当前目标航点索引
            dt                   : 控制周期（秒）

        Returns:
            (linear_cmd, angular_cmd)
        """
        if current_idx >= len(waypoints):
            return 0.0, 0.0

        current_wp = waypoints[current_idx]

        # 单航点或首段无前段 → fallback P2P
        if current_idx == 0 or len(waypoints) < 2:
            logger.debug("PurePursuitController: 单航点，回落 P2P")
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        prev_wp = waypoints[current_idx - 1]

        # 求机器人在路径段 [prev → current] 上的投影
        try:
            proj_lat, proj_lon, t = project_point_on_segment(
                robot_lat, robot_lon,
                prev_wp.lat, prev_wp.lon,
                current_wp.lat, current_wp.lon,
            )
        except Exception as e:
            logger.warning(f"PurePursuitController: 投影计算失败，回落 P2P: {e}")
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        # 构建前视点：从投影点沿路径方向偏移 NAV_LOOKAHEAD_M
        seg_bearing = bearing_to_target(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)
        seg_len     = haversine_distance(prev_wp.lat, prev_wp.lon, current_wp.lat, current_wp.lon)

        if seg_len < 0.1:
            logger.warning("PurePursuitController: 路径段过短，回落 P2P")
            return self._p2p.compute(robot_lat, robot_lon, robot_bearing, current_wp, dt)

        # 剩余路径长度（从投影点到当前航点）
        dist_to_wp = haversine_distance(proj_lat, proj_lon, current_wp.lat, current_wp.lon)

        if dist_to_wp < NAV_LOOKAHEAD_M:
            # 前视点超过当前航点，直接用当前航点作为前视目标
            lookahead_lat = current_wp.lat
            lookahead_lon = current_wp.lon
        else:
            # 前视点 = 投影点沿路径方向偏移 NAV_LOOKAHEAD_M
            cos_lat = math.cos(math.radians(proj_lat))
            rad_bearing = math.radians(seg_bearing)
            dlat_m = NAV_LOOKAHEAD_M * math.cos(rad_bearing)
            dlon_m = NAV_LOOKAHEAD_M * math.sin(rad_bearing)
            lookahead_lat = proj_lat + math.degrees(dlat_m / 6_371_000.0)
            lookahead_lon = proj_lon + math.degrees(dlon_m / (6_371_000.0 * cos_lat + 1e-12))

        # 构造一个临时"虚拟"航点用于 P2P 计算
        from dataclasses import replace
        lookahead_wp = replace(current_wp, lat=lookahead_lat, lon=lookahead_lon)

        return self._p2p.compute(robot_lat, robot_lon, robot_bearing, lookahead_wp, dt)

    def reset(self) -> None:
        self._p2p.reset()
