"""
waypoint — Waypoint 数据类 + WaypointManager

CSV 格式（QGIS 导出）：
    id,lat,lon,tolerance_m,max_speed
    0,30.12345,120.98765,1.0,0.5
    1,30.12400,120.98800,1.0,0.5
"""

import logging
from dataclasses import dataclass
from typing import Optional

from config import NAV_ARRIVE_FRAMES

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """单个导航航点。"""
    id:          int
    lat:         float   # 十进制度，+北/-南
    lon:         float   # 十进制度，+东/-西
    tolerance_m: float   # 到达判定半径（米）
    max_speed:   float   # 该段最大线速度（m/s）


class WaypointManager:
    """管理航点序列，提供到达判定逻辑。"""

    def __init__(self) -> None:
        self._waypoints: list[Waypoint] = []
        self._idx:       int = 0
        self._arrive_count: int = 0

    # ── 加载 ──────────────────────────────────────────────
    def load_csv(self, csv_text: str) -> int:
        """解析 CSV 文本，返回成功加载的航点数。

        首行视为标题行（包含 'lat' / 'id' 等字段名），失败行 warning 跳过。
        """
        self._waypoints = []
        self._idx = 0
        self._arrive_count = 0

        lines = csv_text.strip().splitlines()
        if not lines:
            logger.warning("WaypointManager: CSV is empty")
            return 0

        # 跳过标题行（首行）
        data_lines = lines[1:]
        for raw in data_lines:
            raw = raw.strip()
            if not raw:
                continue
            try:
                parts = [p.strip() for p in raw.split(",")]
                if len(parts) < 5:
                    raise ValueError(f"期望 5 列，实际 {len(parts)} 列")
                wp = Waypoint(
                    id          = int(parts[0]),
                    lat         = float(parts[1]),
                    lon         = float(parts[2]),
                    tolerance_m = float(parts[3]),
                    max_speed   = float(parts[4]),
                )
                self._waypoints.append(wp)
            except (ValueError, IndexError) as e:
                logger.warning(f"WaypointManager: 跳过无效行 {raw!r}: {e}")

        count = len(self._waypoints)
        if count:
            logger.info(f"WaypointManager: 加载 {count} 个航点")
        else:
            logger.warning("WaypointManager: 未能加载任何有效航点")
        return count

    # ── 属性 ──────────────────────────────────────────────
    @property
    def current(self) -> Optional[Waypoint]:
        """当前目标航点，若已完成返回 None。"""
        if 0 <= self._idx < len(self._waypoints):
            return self._waypoints[self._idx]
        return None

    @property
    def is_finished(self) -> bool:
        """所有航点均已到达。"""
        return self._idx >= len(self._waypoints)

    @property
    def progress(self) -> tuple[int, int]:
        """(已完成数+1, 总数)，即当前正在前往第几个（从1计数）。"""
        total = len(self._waypoints)
        current_no = min(self._idx + 1, total)
        return current_no, total

    @property
    def waypoints(self) -> list[Waypoint]:
        """返回完整航点列表（只读用途）。"""
        return list(self._waypoints)

    @property
    def current_index(self) -> int:
        return self._idx

    # ── 到达判定 ──────────────────────────────────────────
    def update(self, distance_m: float, fix_quality: int) -> bool:
        """根据当前距离判断是否到达，到达后切换下一航点。

        自适应容差：
            fix_quality == 4 (RTK 固定) → min(wp.tolerance_m, 0.5m)
            fix_quality == 5 (RTK 浮点) → 2.0m
            其他                        → wp.tolerance_m

        连续 NAV_ARRIVE_FRAMES 帧距离 < tolerance → 切换，返回 True。
        """
        wp = self.current
        if wp is None:
            return False

        # 自适应容差
        if fix_quality == 4:
            tolerance = min(wp.tolerance_m, 0.5)
        elif fix_quality == 5:
            tolerance = 2.0
        else:
            tolerance = wp.tolerance_m

        if distance_m < tolerance:
            self._arrive_count += 1
            if self._arrive_count >= NAV_ARRIVE_FRAMES:
                logger.info(
                    f"WaypointManager: 到达航点 {wp.id} "
                    f"(dist={distance_m:.2f}m, tol={tolerance:.2f}m)"
                )
                self._idx += 1
                self._arrive_count = 0
                return True
        else:
            self._arrive_count = 0
        return False

    def reset(self) -> None:
        """重置指针回 0，重新开始。"""
        self._idx = 0
        self._arrive_count = 0
        logger.info("WaypointManager: 重置至第一个航点")
