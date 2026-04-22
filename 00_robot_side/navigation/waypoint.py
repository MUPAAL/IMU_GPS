"""
waypoint — Waypoint dataclass + WaypointManager

CSV format (QGIS export):
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
    """Single navigation waypoint."""
    id:          int
    lat:         float   # Decimal degrees, +north/-south
    lon:         float   # Decimal degrees, +east/-west
    tolerance_m: float   # Arrival determination radius (meters)
    max_speed:   float   # Maximum linear speed for this segment (m/s)


class WaypointManager:
    """Manage waypoint sequence, provide arrival determination logic."""

    def __init__(self) -> None:
        self._waypoints: list[Waypoint] = []
        self._idx:       int = 0
        self._arrive_count: int = 0

    # ── Loading ──────────────────────────────────────────────
    def load_csv(self, csv_text: str) -> int:
        """Parse CSV text, return count of successfully loaded waypoints.

        First row is treated as header row (containing 'lat' / 'id' fields), failed rows warning skip.
        """
        self._waypoints = []
        self._idx = 0
        self._arrive_count = 0

        lines = csv_text.strip().splitlines()
        if not lines:
            logger.warning("WaypointManager: CSV is empty")
            return 0

        # Skip header row (first line)
        data_lines = lines[1:]
        for raw in data_lines:
            raw = raw.strip()
            if not raw:
                continue
            try:
                parts = [p.strip() for p in raw.split(",")]
                if len(parts) < 5:
                    raise ValueError(f"Expected 5 columns, got {len(parts)}")
                wp = Waypoint(
                    id          = int(parts[0]),
                    lat         = float(parts[1]),
                    lon         = float(parts[2]),
                    tolerance_m = float(parts[3]),
                    max_speed   = float(parts[4]),
                )
                self._waypoints.append(wp)
            except (ValueError, IndexError) as e:
                logger.warning(f"WaypointManager: Skip invalid row {raw!r}: {e}")

        count = len(self._waypoints)
        if count:
            logger.info(f"WaypointManager: Loaded {count} waypoints")
        else:
            logger.warning("WaypointManager: Failed to load any valid waypoints")
        return count

    # ── Properties ──────────────────────────────────────────────
    @property
    def current(self) -> Optional[Waypoint]:
        """Current target waypoint, return None if completed."""
        if 0 <= self._idx < len(self._waypoints):
            return self._waypoints[self._idx]
        return None

    @property
    def is_finished(self) -> bool:
        """All waypoints have been reached."""
        return self._idx >= len(self._waypoints)

    @property
    def progress(self) -> tuple[int, int]:
        """(completed+1, total), i.e., currently heading to which waypoint (1-indexed)."""
        total = len(self._waypoints)
        current_no = min(self._idx + 1, total)
        return current_no, total

    @property
    def waypoints(self) -> list[Waypoint]:
        """Return complete waypoint list (for read-only usage)."""
        return list(self._waypoints)

    @property
    def current_index(self) -> int:
        return self._idx

    # ── Arrival Determination ──────────────────────────────────────────────
    def update(self, distance_m: float, fix_quality: int) -> bool:
        """Determine arrival based on current distance, switch to next waypoint if reached.

        Adaptive tolerance:
            fix_quality == 4 (RTK fixed) → min(wp.tolerance_m, 0.5m)
            fix_quality == 5 (RTK float) → 2.0m
            other                        → wp.tolerance_m

        NAV_ARRIVE_FRAMES consecutive frames with distance < tolerance → switch, return True.
        """
        wp = self.current
        if wp is None:
            return False

        # Adaptive tolerance
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
                    f"WaypointManager: Reached waypoint {wp.id} "
                    f"(dist={distance_m:.2f}m, tol={tolerance:.2f}m)"
                )
                self._idx += 1
                self._arrive_count = 0
                return True
        else:
            self._arrive_count = 0
        return False

    def reset(self) -> None:
        """Reset pointer to 0, start over."""
        self._idx = 0
        self._arrive_count = 0
        logger.info("WaypointManager: Reset to first waypoint")
