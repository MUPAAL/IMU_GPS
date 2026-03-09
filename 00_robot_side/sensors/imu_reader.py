"""
IMU Reader — OAK-D BNO085 via depthai

Public API:
    IMUReader(threading.Thread, daemon=True)
        .get_data() -> dict   — thread-safe access to the latest IMU snapshot (aligned with RTKReader API)
        .is_available -> bool  — set to True once the depthai pipeline starts

    quaternion_to_compass(real, i, j, k) -> (bearing, cardinal)

    # Backward-compatible module-level globals (deprecated; prefer IMUReader.get_data())
    imu_lock, imu_data, imu_available
"""

import logging
import math
import os
import threading
import time

from config import COMPASS_MIN_ACCURACY, COMPASS_OFFSET_DEG

logger = logging.getLogger(__name__)

# ── Module-level global IMU data (thread-safe) ───────────
imu_lock = threading.Lock()
imu_data: dict = {
    "accel":   {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro":    {"x": 0.0, "y": 0.0, "z": 0.0},
    "compass": {
        "bearing":    0.0,
        "cardinal":   "N",
        "calibrated": False,
        "accuracy":   0,
        "quat":       {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    },
    "ts": 0.0,
}
imu_available: bool = False  # True once depthai pipeline is running


# ── Quaternion → compass bearing ──────────────────────────
def quaternion_to_compass(real: float, i: float, j: float, k: float) -> tuple[float, str]:
    """Convert BNO085 ROTATION_VECTOR quaternion to compass bearing [0, 360).

    0 = magnetic north, clockwise positive.
    Coordinate system selectable via COORD_SYSTEM env var (default: NED).
    """
    coord = os.environ.get("COORD_SYSTEM", "NED").upper()
    yaw_rad = math.atan2(2 * (real * k + i * j), 1 - 2 * (j * j + k * k))
    if coord == "ENU":
        # ENU: yaw is measured counter-clockwise from East; convert to clockwise-from-North bearing
        bearing = (90.0 - math.degrees(yaw_rad)) % 360.0
    else:
        # NED: yaw is already a clockwise-from-North bearing
        bearing = math.degrees(yaw_rad) % 360.0
    cardinals = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    cardinal  = cardinals[int((bearing + 22.5) / 45.0) % 8]
    return bearing, cardinal


# ── IMU reader thread (depthai OAK-D) ────────────────────
class IMUReader(threading.Thread):
    """Daemon thread: continuously reads IMU packets from OAK-D and updates module-level imu_data.

    Prefer accessing data via get_data() / is_available instead of reading module globals directly.
    """

    def __init__(self) -> None:
        super().__init__(name="IMUReader", daemon=True)

    @property
    def is_available(self) -> bool:
        """Whether the depthai pipeline started successfully."""
        return imu_available

    def get_data(self) -> dict:
        """Return the latest IMU snapshot in a thread-safe way (aligned with RTKReader.get_data())."""
        with imu_lock:
            return dict(imu_data)

    def run(self) -> None:
        global imu_available
        try:
            import depthai as dai
        except ImportError:
            logger.warning("depthai not installed — IMU unavailable, HUD will show zeros")
            return

        try:
            with dai.Pipeline() as pipeline:
                imu_node = pipeline.create(dai.node.IMU)
                imu_node.enableIMUSensor(dai.IMUSensor.LINEAR_ACCELERATION, 400)
                imu_node.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
                imu_node.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
                imu_node.setBatchReportThreshold(1)
                imu_node.setMaxBatchReports(10)

                imu_queue = imu_node.out.createOutputQueue(maxSize=50, blocking=False)
                pipeline.start()
                imu_available = True
                logger.info("IMUReader: depthai IMU pipeline started")

                while pipeline.isRunning():
                    try:
                        imu_data_pkt = imu_queue.get()
                        if imu_data_pkt is None:
                            continue
                        for pkt in imu_data_pkt.packets:
                            self._process_packet(pkt)
                    except Exception as e:
                        logger.error(f"IMUReader: failed to read packet: {e}")

        except Exception as e:
            logger.error(f"IMUReader: depthai pipeline failed to start: {e}")
            imu_available = False

    def _process_packet(self, pkt) -> None:
        global imu_data
        try:
            accel = pkt.acceleroMeter
            gyro  = pkt.gyroscope
            rot   = pkt.rotationVector

            w, xi, yj, zk = rot.real, rot.i, rot.j, rot.k
            try:
                accuracy = int(rot.accuracy)  # 0-3: BNO085 calibration accuracy
            except (AttributeError, TypeError, ValueError):
                # Fallback: infer from all-zero quaternion check
                accuracy = 0 if (w == 0.0 and xi == 0.0 and yj == 0.0 and zk == 0.0) else 3
            calibrated = accuracy >= COMPASS_MIN_ACCURACY
            bearing, cardinal = quaternion_to_compass(w, xi, yj, zk)
            # Apply static compass offset (manual north-alignment compensation)
            bearing = (bearing + COMPASS_OFFSET_DEG) % 360.0
            cardinals_list = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
            cardinal = cardinals_list[int((bearing + 22.5) / 45.0) % 8]

            with imu_lock:
                imu_data = {
                    "accel": {"x": accel.x, "y": accel.y, "z": accel.z},
                    "gyro":  {"x": gyro.x,  "y": gyro.y,  "z": gyro.z},
                    "compass": {
                        "bearing":    bearing,
                        "cardinal":   cardinal,
                        "calibrated": calibrated,
                        "accuracy":   accuracy,
                        "quat":       {"w": w, "x": xi, "y": yj, "z": zk},
                    },
                    "ts": time.time(),
                }
        except Exception as e:
            logger.error(f"IMUReader: packet processing error: {e}")
