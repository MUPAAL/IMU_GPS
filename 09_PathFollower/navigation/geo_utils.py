"""
geo_utils — 地理计算纯函数（无副作用）

提供 WGS-84 球面距离、方位角、角度归一化、路径投影等工具函数。
"""

import math
from typing import Tuple

# ── WGS-84 参数 ────────────────────────────────────────────
_EARTH_RADIUS_M = 6_371_000.0  # 平均地球半径（米）


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两点间的大圆距离（Haversine 公式）。

    Args:
        lat1, lon1: 起点（十进制度）
        lat2, lon2: 终点（十进制度）

    Returns:
        距离（米）
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_to_target(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算从 (lat1, lon1) 指向 (lat2, lon2) 的真北方位角。

    Args:
        lat1, lon1: 出发点（十进制度）
        lat2, lon2: 目标点（十进制度）

    Returns:
        方位角（度），0 = 正北，顺时针为正，范围 [0, 360)
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y))) % 360.0


def normalize_angle(angle_deg: float) -> float:
    """将角度归一化到 (-180, 180] 区间。

    用于计算方位角误差时保证最短旋转方向。
    """
    angle = angle_deg % 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle


def project_point_on_segment(
    p_lat: float, p_lon: float,
    a_lat: float, a_lon: float,
    b_lat: float, b_lon: float,
) -> Tuple[float, float, float]:
    """将点 P 投影到线段 A→B 上，返回投影坐标和参数 t。

    采用局部平面近似（小范围有效）。

    Args:
        p_lat, p_lon: 待投影点
        a_lat, a_lon: 线段起点
        b_lat, b_lon: 线段终点

    Returns:
        (proj_lat, proj_lon, t)
        t ∈ [0, 1]：0 表示在 A，1 表示在 B，< 0 或 > 1 表示超出线段范围
    """
    # 以 A 为原点做局部平面近似
    cos_lat = math.cos(math.radians(a_lat))

    # 将经纬度差转换为近似米
    def _to_local(lat: float, lon: float) -> Tuple[float, float]:
        dlat = math.radians(lat - a_lat) * _EARTH_RADIUS_M
        dlon = math.radians(lon - a_lon) * _EARTH_RADIUS_M * cos_lat
        return dlat, dlon

    py, px = _to_local(p_lat, p_lon)
    ay, ax = 0.0, 0.0
    by, bx = _to_local(b_lat, b_lon)

    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay

    ab_sq = abx * abx + aby * aby
    if ab_sq < 1e-12:
        # A 和 B 重合，直接返回 A
        return a_lat, a_lon, 0.0

    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))

    # 投影点本地坐标
    proj_x = ax + t * abx
    proj_y = ay + t * aby

    # 反算经纬度
    proj_lat = a_lat + math.degrees(proj_y / _EARTH_RADIUS_M)
    proj_lon = a_lon + math.degrees(proj_x / (_EARTH_RADIUS_M * cos_lat + 1e-12))

    return proj_lat, proj_lon, t
