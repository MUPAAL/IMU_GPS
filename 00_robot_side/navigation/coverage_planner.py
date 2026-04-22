"""
coverage_planner — Boustrophedon (snake sweep) coverage path generator

Takes GPS boundary polygon of a field and auto-generates equally-spaced
back-and-forth scanning waypoint CSV. Generated CSV can be directly loaded
via NavigationEngine.load_waypoints().

Algorithm flow (Boustrophedon / lawn mower pattern):
    1. Project polygon vertices to local ENU plane (with centroid as origin)
    2. Transform ENU (east, north) to scan coordinate system (x', y')
       x' = robot travel direction (along direction_deg)
       y' = inter-row spacing direction (perpendicular to x')
    3. Generate equally-spaced scan lines in y' direction by row_spacing × (1 - overlap)
    4. Find intersections of each scan line with polygon, extract left/right endpoints
    5. Connect in S pattern (reverse odd rows)
    6. Back-project to WGS-84 lat/lon, generate Waypoint CSV

ENU↔Scan coordinate transform (direction_deg = θ, compass bearing, 0=North):
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
    """Boustrophedon (snake sweep) coverage path generator.

    Args:
        boundary      : GPS boundary polygon [(lat, lon), ...], min 3 vertices
        row_spacing   : Row spacing (meters), default 1.0
        direction_deg : Robot travel direction (compass bearing, 0=N-S sweep, 90=E-W sweep), default 0
        overlap       : Row overlap ratio (0~1), default 0
        tolerance_m   : Waypoint arrival tolerance (meters), default 1.0
        max_speed     : Max speed at each waypoint (m/s), default 0.5
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

        # Centroid as ENU projection origin
        lats = [p[0] for p in boundary]
        lons = [p[1] for p in boundary]
        self._origin_lat = sum(lats) / len(lats)
        self._origin_lon = sum(lons) / len(lons)
        self._cos_lat    = math.cos(math.radians(self._origin_lat))

    # ── ENU Projection ──────────────────────────────────────────────

    def _to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        """WGS-84 → Local ENU (meters), return (east_m, north_m)."""
        north = math.radians(lat - self._origin_lat) * _EARTH_RADIUS_M
        east  = math.radians(lon - self._origin_lon) * _EARTH_RADIUS_M * self._cos_lat
        return east, north

    def _from_enu(self, east_m: float, north_m: float) -> Tuple[float, float]:
        """Local ENU (meters) → WGS-84, return (lat, lon)."""
        lat = self._origin_lat + math.degrees(north_m / _EARTH_RADIUS_M)
        lon = self._origin_lon + math.degrees(
            east_m / (_EARTH_RADIUS_M * self._cos_lat + 1e-12)
        )
        return lat, lon

    # ── Coordinate System Transform ─────────────────────────────────────────────

    def _enu_to_scan(self, east: float, north: float) -> Tuple[float, float]:
        """ENU → Scan coordinate system (x', y'), x' along direction_deg."""
        rad = math.radians(self._direction)
        x =  east * math.sin(rad) + north * math.cos(rad)
        y = -east * math.cos(rad) + north * math.sin(rad)
        return x, y

    def _scan_to_enu(self, x: float, y: float) -> Tuple[float, float]:
        """Scan coordinate system (x', y') → ENU."""
        rad = math.radians(self._direction)
        east  = x * math.sin(rad) - y * math.cos(rad)
        north = x * math.cos(rad) + y * math.sin(rad)
        return east, north

    # ── Scan Line - Polygon Intersection ────────────────────────────────────

    @staticmethod
    def _seg_intersect_y(
        y: float,
        x1: float, y1: float,
        x2: float, y2: float,
    ) -> Optional[float]:
        """Find x coordinate of intersection between horizontal scan line y=const
        and line segment (x1,y1)→(x2,y2).

        Use half-open interval [y_min, y_max) to avoid duplicate vertex counting.
        Return None if no intersection.
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
        """Get list of x coordinates where scan line y intersects all polygon edges (sorted ascending)."""
        xs = []
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            xi = self._seg_intersect_y(y, x1, y1, x2, y2)
            if xi is not None:
                xs.append(xi)
        return sorted(xs)

    # ── Main Generation Function ─────────────────────────────────────────────

    def generate(self) -> List[Tuple[float, float]]:
        """Generate coverage path, return GPS coordinate list [(lat, lon), ...]."""
        # 1. GPS → ENU → Scan coordinate system
        scan_pts = [
            self._enu_to_scan(*self._to_enu(lat, lon))
            for lat, lon in self._boundary
        ]

        # 2. Determine y' scan range
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
                "CoveragePlanner: Unable to generate scan lines (region too small or row_spacing too large)"
            )
            return []

        # 3. Find intersections of each row with polygon, generate S-shaped path
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
                "CoveragePlanner: No valid intersection between scan lines and polygon, check boundary point order"
            )
            return []

        # 4. Scan coordinate system → ENU → GPS
        gps_wps: List[Tuple[float, float]] = []
        for sx, sy in waypoints_scan:
            east, north = self._scan_to_enu(sx, sy)
            lat, lon    = self._from_enu(east, north)
            gps_wps.append((lat, lon))

        logger.info(
            "CoveragePlanner: Generated %d waypoints, "
            "row_spacing=%.2fm, overlap=%.0f%%, direction=%.0f°",
            len(gps_wps), self._row_spacing, self._overlap * 100, self._direction,
        )
        return gps_wps

    def generate_csv(self) -> str:
        """Generate CSV format string, can be directly passed to NavigationEngine.load_waypoints().

        Format: id,lat,lon,tolerance_m,max_speed
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
