"""
ESP32 IMU Reader — BNO085 via serial UART

Serial protocol (ESP32 firmware output):
    Format : "roll,pitch,yaw,accuracy\\n"
    Example: "12.34,-5.67,135.20,3\\n"
    Fields :
        roll     float  degrees, around X-axis
        pitch    float  degrees, around Y-axis
        yaw      float  degrees, around Z-axis
                        BNO085 ARVR_STABILIZED_RV uses ENU frame:
                            yaw=0 → East, counter-clockwise positive
                        bearing (compass, clockwise from North) = (90 - yaw) % 360
        accuracy int    0-3, BNO085 magnetometer calibration accuracy
                        3 fields (no accuracy) also accepted; accuracy defaults to -1

Public API (aligned with sensors/imu_reader.py):
    ESP32IMUReader(threading.Thread, daemon=True)
        .get_data()    -> dict  — thread-safe latest IMU snapshot
        .is_available  -> bool  — True once first valid line is parsed
"""

import logging
import threading
import time

import serial

from config import (
    COMPASS_MIN_ACCURACY,
    COMPASS_OFFSET_DEG,
    ESP32_IMU_PORT,
    ESP32_IMU_BAUD,
)

logger = logging.getLogger(__name__)

# ── Default snapshot (mirrors imu_reader.py structure) ───────────────────────
_DEFAULT_DATA: dict = {
    "accel":   {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro":    {"x": 0.0, "y": 0.0, "z": 0.0},
    "compass": {
        "bearing":    0.0,
        "cardinal":   "N",
        "calibrated": False,
        "accuracy":   0,
        # no quaternion from serial; fill with identity so callers never KeyError
        "quat":       {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    },
    "ts": 0.0,
}

_CARDINALS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


def _bearing_to_cardinal(bearing: float) -> str:
    return _CARDINALS[int((bearing + 22.5) / 45.0) % 8]


def _yaw_to_bearing_enu(yaw: float) -> float:
    """Convert BNO085 ARVR ENU yaw to compass bearing [0, 360).

    ENU frame: yaw=0 → East, counter-clockwise positive.
    Compass bearing: 0 → North, clockwise positive.
    Formula: bearing = (90 - yaw) % 360
    """
    return (90.0 - yaw) % 360.0


def _parse_line(raw: str) -> tuple[float, float, float, int] | None:
    """Parse one serial line.  Returns (roll, pitch, yaw, accuracy) or None."""
    parts = raw.strip().split(",")
    try:
        if len(parts) == 4:
            return float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3])
        if len(parts) == 3:
            return float(parts[0]), float(parts[1]), float(parts[2]), -1
    except (ValueError, IndexError):
        pass
    return None


class ESP32IMUReader(threading.Thread):
    """Daemon thread: reads BNO085 data from ESP32 over serial and exposes
    the latest snapshot via get_data() / is_available.

    Drop-in replacement for sensors.imu_reader.IMUReader.
    """

    def __init__(self) -> None:
        super().__init__(name="ESP32IMUReader", daemon=True)
        self._lock = threading.Lock()
        self._data: dict = dict(_DEFAULT_DATA)
        self._available = False

    @property
    def is_available(self) -> bool:
        """True once the first valid IMU packet has been received."""
        return self._available

    def get_data(self) -> dict:
        """Return a shallow copy of the latest IMU snapshot (thread-safe)."""
        with self._lock:
            return dict(self._data)

    # ── Thread entry ──────────────────────────────────────────────────────────
    def run(self) -> None:
        logger.info(f"ESP32IMUReader: opening {ESP32_IMU_PORT} @ {ESP32_IMU_BAUD} baud")
        try:
            ser = serial.Serial(ESP32_IMU_PORT, ESP32_IMU_BAUD, timeout=2.0)
        except serial.SerialException as e:
            logger.error(f"ESP32IMUReader: failed to open serial port [{ESP32_IMU_PORT}]: {e}")
            return  # is_available stays False → graceful degradation

        logger.info(f"ESP32IMUReader: serial port opened: {ser.name}")
        buf = b""

        while True:
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException as e:
                logger.error(f"ESP32IMUReader: serial read error: {e}")
                time.sleep(0.1)
                continue

            if not chunk:
                continue

            buf += chunk
            while b"\n" in buf:
                raw_bytes, buf = buf.split(b"\n", 1)
                raw_str = raw_bytes.decode("ascii", errors="replace").strip()
                if not raw_str or raw_str.startswith("#"):
                    # ignore empty lines and firmware diagnostic lines
                    if raw_str.startswith("#"):
                        logger.debug(f"ESP32IMUReader: [fw] {raw_str}")
                    continue
                self._process_line(raw_str)

    def _process_line(self, raw: str) -> None:
        result = None
        try:
            result = _parse_line(raw)
        except Exception as e:
            logger.warning(f"ESP32IMUReader: parse error [{e}] raw={raw!r}")
            return

        if result is None:
            logger.warning(f"ESP32IMUReader: unexpected format (not 3 or 4 fields): {raw!r}")
            return

        roll, pitch, yaw, accuracy = result

        # Convert ENU yaw → compass bearing
        bearing = _yaw_to_bearing_enu(yaw)
        # Apply static magnetic-north alignment offset
        bearing = (bearing + COMPASS_OFFSET_DEG) % 360.0
        cardinal = _bearing_to_cardinal(bearing)

        # accuracy = -1 means firmware does not report it; treat as uncalibrated
        eff_accuracy = accuracy if accuracy >= 0 else 0
        calibrated = eff_accuracy >= COMPASS_MIN_ACCURACY

        snap: dict = {
            "accel":   {"x": 0.0, "y": 0.0, "z": 0.0},   # ESP32 serial does not send accel
            "gyro":    {"x": roll,  "y": pitch, "z": yaw}, # repurpose for raw angles (deg)
            "compass": {
                "bearing":    bearing,
                "cardinal":   cardinal,
                "calibrated": calibrated,
                "accuracy":   eff_accuracy,
                "quat":       {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            },
            "ts": time.time(),
        }

        with self._lock:
            self._data = snap
            if not self._available:
                self._available = True
                logger.info(
                    f"ESP32IMUReader: first packet received — "
                    f"bearing={bearing:.1f}°  accuracy={accuracy}"
                )
