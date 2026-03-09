"""
coverage_planner — Boustrophedon（蛇形往返）覆盖路径生成器

输入一块田地的 GPS 边界多边形，自动生成等间距往返扫描航点 CSV。
生成的 CSV 可直接通过 NavigationEngine.load_waypoints() 加载。

算法流程（Boustrophedon / 割草机式）：
    1. 将多边形顶点投影到本地 ENU 平面（以重心为原点）
    2. 将 ENU (east, north) 转换到扫描坐标系 (x', y')
       x' = 机器人行进方向（沿 direction_deg 方向）
       y' = 行与行之间的间距方向（垂直于 x'）
    3. 按 row_spacing × (1 - overlap) 在 y' 方向生成等间距扫描线
    4. 每条扫描线与多边形求交，取最左最右端点
    5. S 形连接（奇数行翻转）
    6. 反投影回 WGS-84 经纬度，生成 Waypoint CSV

ENU↔扫描坐标系变换（direction_deg = θ，为罗盘方位角，0=North）：
    x' =  east * sin(θ) + north * cos(θ)
    y' = -east * cos(θ) + north * sin(θ)
    east  = x' * sin(θ) - y' * cos(θ)
    north = x' * cos(θ) + y' * sin(θ)
"""

import csv
import io
import logging
import math
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

_EARTH_RADIUS_M = 6_371_000.0

# 默认参数
DEFAULT_TOLERANCE_M = 1.0
DEFAULT_MAX_SPEED    = 0.5


class CoveragePlanner:
    """Boustrophedon（蛇形往返）覆盖路径生成器。

    Args:
        boundary      : GPS 边界多边形 [(lat, lon), ...]，至少 3 个顶点。
        row_spacing   : 行间距（米），默认 1.0
        direction_deg : 机器人行进方向（罗盘角，0=N-S 往返，90=E-W 往返），默认 0
        overlap       : 行重叠率（0~1），默认 0
        tolerance_m   : 航点到达容差（米），默认 1.0
        max_speed     : 各航点最大速度（m/s），默认 0.5
    """

    def __init__(
        self,
        boundary:      List[Tuple[float, float]],
        row_spacing:   float = 1.0,
        direction_deg: float = 0.0,
        overlap:       float = 0.0,
        tolerance_m:   float = DEFAULT_TOLERANCE_M,
        max_speed:     float = DEFAULT_MAX_SPEED,
    ) -> None:
        if len(boundary) < 3:
            raise ValueError("boundary 至少需要 3 个顶点")
        self._boundary    = list(boundary)
        self._row_spacing = max(0.05, row_spacing)
        self._direction   = direction_deg % 360.0
        self._overlap     = max(0.0, min(0.99, overlap))
        self._tolerance_m = tolerance_m
        self._max_speed   = max_speed

        # 重心作为 ENU 投影原点
        lats = [p[0] for p in boundary]
        lons = [p[1] for p in boundary]
        self._origin_lat = sum(lats) / len(lats)
        self._origin_lon = sum(lons) / len(lons)
        self._cos_lat    = math.cos(math.radians(self._origin_lat))

    # ── ENU 投影 ──────────────────────────────────────────────

    def _to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        """WGS-84 → 本地 ENU（米），返回 (east_m, north_m)。"""
        north = math.radians(lat - self._origin_lat) * _EARTH_RADIUS_M
        east  = math.radians(lon - self._origin_lon) * _EARTH_RADIUS_M * self._cos_lat
        return east, north

    def _from_enu(self, east_m: float, north_m: float) -> Tuple[float, float]:
        """本地 ENU（米）→ WGS-84，返回 (lat, lon)。"""
        lat = self._origin_lat + math.degrees(north_m / _EARTH_RADIUS_M)
        lon = self._origin_lon + math.degrees(
            east_m / (_EARTH_RADIUS_M * self._cos_lat + 1e-12)
        )
        return lat, lon

    # ── 坐标系变换 ─────────────────────────────────────────────

    def _enu_to_scan(self, east: float, north: float) -> Tuple[float, float]:
        """ENU → 扫描坐标系 (x', y')，x' 沿 direction_deg 方向。"""
        rad = math.radians(self._direction)
        x =  east * math.sin(rad) + north * math.cos(rad)
        y = -east * math.cos(rad) + north * math.sin(rad)
        return x, y

    def _scan_to_enu(self, x: float, y: float) -> Tuple[float, float]:
        """扫描坐标系 (x', y') → ENU。"""
        rad = math.radians(self._direction)
        east  = x * math.sin(rad) - y * math.cos(rad)
        north = x * math.cos(rad) + y * math.sin(rad)
        return east, north

    # ── 扫描线与多边形求交 ────────────────────────────────────

    @staticmethod
    def _seg_intersect_y(
        y: float,
        x1: float, y1: float,
        x2: float, y2: float,
    ) -> Optional[float]:
        """水平扫描线 y=const 与线段 (x1,y1)→(x2,y2) 的交点 x 坐标。

        使用半开区间 [y_min, y_max) 避免顶点被重复计数。
        无交返回 None。
        """
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1)
            return x1 + t * (x2 - x1)
        return None

    def _clip_scanline(
        self,
        y: float,
        polygon: List[Tuple[float, float]],
    ) -> List[float]:
        """取扫描线 y 与多边形所有边的交点 x 列表（已升序排列）。"""
        xs = []
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            xi = self._seg_intersect_y(y, x1, y1, x2, y2)
            if xi is not None:
                xs.append(xi)
        return sorted(xs)

    # ── 主生成函数 ─────────────────────────────────────────────

    def generate(self) -> List[Tuple[float, float]]:
        """生成覆盖路径，返回 GPS 坐标列表 [(lat, lon), ...]。"""
        # 1. GPS → ENU → 扫描坐标系
        scan_pts = [
            self._enu_to_scan(*self._to_enu(lat, lon))
            for lat, lon in self._boundary
        ]

        # 2. 确定 y' 扫描范围
        ys = [p[1] for p in scan_pts]
        y_min, y_max = min(ys), max(ys)

        effective_spacing = self._row_spacing * (1.0 - self._overlap)
        scan_ys: List[float] = []
        y = y_min + self._row_spacing / 2.0
        while y <= y_max:
            scan_ys.append(y)
            y += effective_spacing

        if not scan_ys:
            logger.warning(
                "CoveragePlanner: 无法生成扫描行（区域太小或 row_spacing 太大）"
            )
            return []

        # 3. 每行与多边形求交，生成 S 形路径
        waypoints_scan: List[Tuple[float, float]] = []
        reverse = False
        for sy in scan_ys:
            xs = self._clip_scanline(sy, scan_pts)
            if len(xs) < 2:
                continue  # 扫描线未穿过多边形内部，跳过

            x_left, x_right = xs[0], xs[-1]
            if reverse:
                waypoints_scan.append((x_right, sy))
                waypoints_scan.append((x_left,  sy))
            else:
                waypoints_scan.append((x_left,  sy))
                waypoints_scan.append((x_right, sy))
            reverse = not reverse

        if not waypoints_scan:
            logger.warning(
                "CoveragePlanner: 扫描线与多边形无有效交叉，请检查边界点顺序"
            )
            return []

        # 4. 扫描坐标系 → ENU → GPS
        gps_wps: List[Tuple[float, float]] = []
        for sx, sy in waypoints_scan:
            east, north = self._scan_to_enu(sx, sy)
            lat, lon    = self._from_enu(east, north)
            gps_wps.append((lat, lon))

        logger.info(
            "CoveragePlanner: 生成 %d 个航点，"
            "row_spacing=%.2fm，overlap=%.0f%%，方向=%.0f°",
            len(gps_wps), self._row_spacing, self._overlap * 100, self._direction,
        )
        return gps_wps

    def generate_csv(self) -> str:
        """生成 CSV 格式字符串，可直接传给 NavigationEngine.load_waypoints()。

        格式：id,lat,lon,tolerance_m,max_speed
        """
        gps_wps = self.generate()
        if not gps_wps:
            return "id,lat,lon,tolerance_m,max_speed\n"

        out = io.StringIO()
        writer = csv.writer(out)
        writer.writerow(["id", "lat", "lon", "tolerance_m", "max_speed"])
        for i, (lat, lon) in enumerate(gps_wps):
            writer.writerow([
                i,
                f"{lat:.8f}",
                f"{lon:.8f}",
                self._tolerance_m,
                self._max_speed,
            ])
        return out.getvalue()
