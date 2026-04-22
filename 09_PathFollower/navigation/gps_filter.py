"""
gps_filter — GPS 平滑滤波器

MovingAverageFilter : 滑动窗口均值
KalmanFilter        : 4 维卡尔曼滤波（位置 + 速度），支持 IMU 加速度输入
"""

import logging
import math
from collections import deque
from typing import Tuple

import numpy as np

from config import NAV_MA_WINDOW

logger = logging.getLogger(__name__)

_EARTH_RADIUS_M = 6_371_000.0


class MovingAverageFilter:
    """滑动窗口平均滤波器（经纬度）。"""

    def __init__(self, window: int = NAV_MA_WINDOW) -> None:
        self._window = window
        self._lat_buf: deque[float] = deque(maxlen=window)
        self._lon_buf: deque[float] = deque(maxlen=window)

    def update(self, lat: float, lon: float) -> Tuple[float, float]:
        """推入新观测，返回当前窗口均值。"""
        self._lat_buf.append(lat)
        self._lon_buf.append(lon)
        return sum(self._lat_buf) / len(self._lat_buf), sum(self._lon_buf) / len(self._lon_buf)

    def get_position(self) -> Tuple[float, float]:
        """返回当前均值位置；缓冲区为空时返回 (0.0, 0.0)。"""
        if not self._lat_buf:
            return 0.0, 0.0
        return sum(self._lat_buf) / len(self._lat_buf), sum(self._lon_buf) / len(self._lon_buf)

    def reset(self) -> None:
        self._lat_buf.clear()
        self._lon_buf.clear()

    @property
    def is_ready(self) -> bool:
        """缓冲区已达到窗口大小时为 True。"""
        return len(self._lat_buf) >= self._window


class KalmanFilter:
    """4 维卡尔曼滤波器（位置 + 速度）。

    状态向量：x = [Δlat_m, Δlon_m, vel_lat_m/s, vel_lon_m/s]
    （相对初始化点的局部平面偏移，避免大数精度丢失）

    参数：
        process_noise_std   : 过程噪声标准差（米），默认 0.1
        gps_noise_std       : GPS 观测噪声（米），默认 2.0（根据 fix_quality 动态调整）
    """

    def __init__(
        self,
        process_noise_std: float = 0.1,
        gps_noise_std:     float = 2.0,
    ) -> None:
        self._pn_std  = process_noise_std
        self._gps_std = gps_noise_std

        # 初始化点（世界坐标，用于 local↔world 转换）
        self._origin_lat: float = 0.0
        self._origin_lon: float = 0.0
        self._cos_lat:    float = 1.0
        self._initialized = False

        # 状态向量 x = [dlat_m, dlon_m, vn, ve]
        self._x = np.zeros(4)
        # 协方差矩阵
        self._P = np.eye(4) * 100.0

        # 观测矩阵（只观测位置）
        self._H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

    # ── 初始化 ────────────────────────────────────────────
    def init(self, lat: float, lon: float) -> None:
        """设置原点，重置状态。"""
        self._origin_lat = lat
        self._origin_lon = lon
        self._cos_lat    = math.cos(math.radians(lat))
        self._x = np.zeros(4)
        self._P = np.eye(4) * 10.0
        self._initialized = True
        logger.info(f"KalmanFilter: 初始化原点 ({lat:.7f}, {lon:.7f})")

    # ── 预测（IMU 驱动，20 Hz）────────────────────────────
    def predict(self, dt: float, a_north: float = 0.0, a_east: float = 0.0) -> None:
        """运动模型预测步骤。

        Args:
            dt      : 时间步长（秒）
            a_north : 北向加速度（m/s²，机体系旋转后的 NED）
            a_east  : 东向加速度（m/s²）
        """
        if not self._initialized:
            return

        # 状态转移矩阵 F（匀加速运动）
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # 控制输入：加速度 → 位置/速度增量
        B = np.array([
            [0.5 * dt * dt, 0            ],
            [0,             0.5 * dt * dt],
            [dt,            0            ],
            [0,             dt           ],
        ], dtype=float)
        u = np.array([a_north, a_east])

        # 过程噪声矩阵 Q
        q = self._pn_std ** 2
        Q = np.diag([q * dt**2, q * dt**2, q, q])

        self._x = F @ self._x + B @ u
        self._P = F @ self._P @ F.T + Q

    # ── 更新（GPS 到时，1 Hz）────────────────────────────
    def update(self, lat: float, lon: float, fix_quality: int = 1) -> Tuple[float, float]:
        """GPS 观测更新步骤。

        Args:
            lat, lon    : 新 GPS 观测（十进制度）
            fix_quality : 决定观测噪声大小

        Returns:
            (filtered_lat, filtered_lon)
        """
        if not self._initialized:
            self.init(lat, lon)
            return lat, lon

        # 动态调整观测噪声
        if fix_quality == 4:       # RTK fixed
            r_std = 0.03
        elif fix_quality == 5:     # RTK float
            r_std = 0.5
        elif fix_quality == 2:     # DGPS
            r_std = 1.0
        else:                      # GPS / 无信号
            r_std = self._gps_std
        R = np.eye(2) * r_std ** 2

        # 将 GPS 观测转换为本地坐标（米）
        z = np.array([
            math.radians(lat - self._origin_lat) * _EARTH_RADIUS_M,
            math.radians(lon - self._origin_lon) * _EARTH_RADIUS_M * self._cos_lat,
        ])

        # 卡尔曼增益
        S = self._H @ self._P @ self._H.T + R
        K = self._P @ self._H.T @ np.linalg.inv(S)

        # 更新状态
        innov = z - self._H @ self._x
        self._x = self._x + K @ innov
        self._P = (np.eye(4) - K @ self._H) @ self._P

        return self.get_position()

    # ── 读取当前估计 ──────────────────────────────────────
    def get_position(self) -> Tuple[float, float]:
        """返回当前估计位置（十进制度）。"""
        if not self._initialized:
            return 0.0, 0.0
        dlat_m = self._x[0]
        dlon_m = self._x[1]
        lat = self._origin_lat + math.degrees(dlat_m / _EARTH_RADIUS_M)
        lon = self._origin_lon + math.degrees(
            dlon_m / (_EARTH_RADIUS_M * self._cos_lat + 1e-12)
        )
        return lat, lon

    # ── 速度观测更新（里程计，~20 Hz）──────────────────────
    def update_velocity(self, v_north: float, v_east: float) -> None:
        """轮速里程计速度观测更新步骤（约 20 Hz）。

        Amiga TPDO1 提供实测线速度，经 on_odometry() 旋转到 NED 后调用。
        仅观测速度状态，不更新位置状态。

        Args:
            v_north : 北向速度（m/s）
            v_east  : 东向速度（m/s）

        状态向量：x = [Δlat_m, Δlon_m, vel_lat, vel_lon]
        观测矩阵：H = [[0,0,1,0],[0,0,0,1]]（只观测速度）
        """
        if not self._initialized:
            return

        z = np.array([v_north, v_east])
        H = np.array([[0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        # 里程计速度噪声：σ ≈ 0.22 m/s（实测偏差约 0.05 m²/s²）
        R_vel = np.eye(2) * 0.05

        y = z - H @ self._x
        S = H @ self._P @ H.T + R_vel
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ H) @ self._P

    def reset(self) -> None:
        self._initialized = False
        self._x = np.zeros(4)
        self._P = np.eye(4) * 100.0

    @property
    def is_ready(self) -> bool:
        return self._initialized
