"""
Data Recorder — CSV writer for IMU + RTK + command data

CSV columns:
  timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
  compass_bearing, lat, lon, alt, fix_quality, num_sats, hdop,
  linear_cmd, angular_cmd,
  odom_speed, odom_angrate,  -- actual measured speed/angular rate from Amiga TPDO1
  amiga_soc                  -- battery state-of-charge (%) from Amiga TPDO1

Usage:
    recorder = DataRecorder("data_log")
    fname = recorder.start()          # creates robot_data_YYYYMMDD_HHMMSS.csv
    recorder.record(imu_snap, rtk_snap, 0.5, 0.0)
    recorder.stop()                   # closes file

Thread safety:
    Internal threading.Lock protects all file writes.
    record() is safe to call from asyncio loop (fast, non-blocking I/O).
"""

import csv
import logging
import threading
from datetime import datetime
from pathlib import Path

# ── Logging ────────────────────────────────────────────────
_py_name = Path(__file__).stem
Path("log").mkdir(exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(f"log/{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

_CSV_HEADER = [
    "timestamp",
    "accel_x", "accel_y", "accel_z",
    "gyro_x",  "gyro_y",  "gyro_z",
    "compass_bearing",
    "lat", "lon", "alt",
    "fix_quality", "num_sats", "hdop",
    "linear_cmd", "angular_cmd",
    "odom_speed", "odom_angrate",  # actual measured speed from Amiga TPDO1
    "amiga_soc",                   # battery state-of-charge (%) from Amiga TPDO1
]


class DataRecorder:
    """CSV recorder for robot sensor data. Not a thread itself; called from asyncio loop."""

    def __init__(self, log_dir: str) -> None:
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        self._recording = False
        self._current_filename = ""

    # ── Public API ────────────────────────────────────────
    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def current_filename(self) -> str:
        return self._current_filename

    def start(self) -> str:
        """
        Create a new CSV file and write the header row.
        Returns the filename (relative path string).
        Raises: IOError on file creation failure.
        """
        with self._lock:
            if self._recording:
                logger.warning("DataRecorder: start() called while already recording — stopping previous file first")
                self._close_file()

            ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_data_{ts_str}.csv"
            filepath = self._log_dir / filename

            try:
                self._file = open(filepath, "w", newline="", encoding="utf-8")
                self._writer = csv.writer(self._file)
                self._writer.writerow(_CSV_HEADER)
                self._file.flush()
                self._recording = True
                self._current_filename = filename
                logger.info(f"DataRecorder: recording started → {filepath}")
                return filename
            except OSError as e:
                logger.error(f"DataRecorder: failed to create CSV file [{filepath}]: {e}")
                self._file = None
                self._writer = None
                self._recording = False
                raise

    def stop(self) -> None:
        """Close the current CSV file and stop recording."""
        with self._lock:
            self._close_file()

    def record(
        self,
        imu_snap: dict,
        rtk_snap: dict,
        linear: float,
        angular: float,
        odom_snap: dict | None = None,  # optional odometry snapshot {v, w, state, soc, ts}
    ) -> None:
        """
        Write one row to the CSV.
        Safe to call at high frequency; silently skips if not recording.
        """
        if not self._recording:
            return

        ts = datetime.now().isoformat(timespec="milliseconds")

        accel = imu_snap.get("accel", {})
        gyro  = imu_snap.get("gyro",  {})
        comp  = imu_snap.get("compass", {})
        odom  = odom_snap or {}

        row = [
            ts,
            _fmt(accel.get("x")),
            _fmt(accel.get("y")),
            _fmt(accel.get("z")),
            _fmt(gyro.get("x")),
            _fmt(gyro.get("y")),
            _fmt(gyro.get("z")),
            _fmt(comp.get("bearing")),
            _fmt(rtk_snap.get("lat"), 8),
            _fmt(rtk_snap.get("lon"), 8),
            _fmt(rtk_snap.get("alt"), 3),
            rtk_snap.get("fix_quality", ""),
            rtk_snap.get("num_sats", ""),
            _fmt(rtk_snap.get("hdop"), 2),
            _fmt(linear, 4),
            _fmt(angular, 4),
            _fmt(odom.get("v"),   4),
            _fmt(odom.get("w"),   4),
            odom.get("soc", ""),
        ]

        with self._lock:
            if not self._recording or self._writer is None:
                return
            try:
                self._writer.writerow(row)
                self._file.flush()
            except OSError as e:
                logger.error(f"DataRecorder: failed to write CSV row: {e}")
                self._recording = False
                self._close_file()

    # ── Internal helpers ──────────────────────────────────
    def _close_file(self) -> None:
        """Close file handle. Must be called with self._lock held."""
        if self._file is not None:
            try:
                self._file.close()
                logger.info(f"DataRecorder: recording stopped, file closed: {self._current_filename}")
            except OSError as e:
                logger.warning(f"DataRecorder: error closing file: {e}")
            finally:
                self._file = None
                self._writer = None
        self._recording = False
        self._current_filename = ""


def _fmt(value, decimals: int = 6) -> str:
    """Format a numeric value to string, or empty string if None."""
    if value is None:
        return ""
    try:
        return f"{float(value):.{decimals}f}"
    except (TypeError, ValueError):
        return str(value)
