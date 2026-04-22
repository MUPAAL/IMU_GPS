"""
sensor_threads.py — Daemon threads for IMU and RTK data collection.

Imports IMUPipeline and NMEAPipeline from the bridge modules and runs serial
readers in daemon threads. Exposes thread-safe snapshot methods for the web
controller to poll latest sensor data.

Architecture:
  - IMUReader daemon thread → IMUPipeline → snapshot storage
  - RTKReader daemon thread → NMEAPipeline → snapshot storage
"""

import sys
import logging
import threading
import time
from pathlib import Path
from typing import Optional

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config

# Import pipeline classes from existing bridge modules
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "01_IMU"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "02_RTK"))

from imu_bridge import IMUPipeline, FrameRateTracker, IMUFrame
from rtk_bridge import NMEAPipeline, RTKFrame

logger = logging.getLogger(__name__)


class IMUReader(threading.Thread):
    """
    Daemon thread for BNO085 IMU serial reading and pipeline processing.

    Continuously reads JSON from ESP32-C3 serial port, processes through IMUPipeline,
    and stores the latest IMUFrame in thread-safe snapshot.
    """

    def __init__(
        self,
        port: str = config.IMU_SERIAL_PORT,
        baud: int = config.IMU_BAUD,
        north_offset_deg: float = config.IMU_NORTH_OFFSET,
    ):
        super().__init__(daemon=True, name="IMUReader")
        self._port = port
        self._baud = baud
        self._north_offset_deg = north_offset_deg

        # Pipeline and snapshot storage (protected by lock)
        self._pipeline = IMUPipeline(
            FrameRateTracker(window=50),
            north_offset_deg=north_offset_deg,
        )
        self._snapshot: Optional[IMUFrame] = None
        self._snapshot_lock = threading.Lock()
        self._running = False

    def run(self) -> None:
        """Entry point for the IMU reader thread."""
        logger.info(f"IMUReader: starting on {self._port}@{self._baud}")
        self._running = True

        while self._running:
            try:
                with serial.Serial(self._port, self._baud, timeout=1.0) as ser:
                    logger.info(f"IMUReader: opened {self._port}")
                    self._read_loop(ser)
            except serial.SerialException as e:
                logger.error(f"IMUReader: serial error: {e}. Retrying in 3s...")
                time.sleep(3.0)
            except Exception as e:
                logger.error(f"IMUReader: unexpected error: {e}. Retrying in 3s...")
                time.sleep(3.0)

    def _read_loop(self, ser: serial.Serial) -> None:
        """Inner read loop; exits on serial error to trigger reconnect."""
        while self._running:
            try:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if line.startswith("#") or not line.startswith("{"):
                    continue

                # Process through pipeline and update snapshot
                result_json = self._pipeline.process(line)
                if result_json is not None:
                    frame = self._pipeline.snapshot()
                    if frame is not None:
                        with self._snapshot_lock:
                            self._snapshot = frame
                            logger.debug(
                                f"IMUReader: snapshot updated. heading={frame.heading:.1f}°"
                            )

            except serial.SerialException:
                break
            except Exception as e:
                logger.warning(f"IMUReader: read error: {e}")
                break

    def snapshot(self) -> Optional[IMUFrame]:
        """Return the latest IMU frame snapshot (thread-safe)."""
        with self._snapshot_lock:
            return self._snapshot

    def stop(self) -> None:
        """Request the thread to stop gracefully."""
        self._running = False


class RTKReader(threading.Thread):
    """
    Daemon thread for RTK GPS serial reading and NMEA pipeline processing.

    Continuously reads NMEA sentences from RTK receiver, processes through NMEAPipeline,
    and stores the latest RTKFrame in thread-safe snapshot.
    """

    def __init__(
        self,
        port: str = config.RTK_SERIAL_PORT,
        baud: int = config.RTK_BAUD,
    ):
        super().__init__(daemon=True, name="RTKReader")
        self._port = port
        self._baud = baud

        # Pipeline and snapshot storage
        self._pipeline = NMEAPipeline()
        self._snapshot: Optional[RTKFrame] = None
        self._snapshot_lock = threading.Lock()
        self._running = False

    def run(self) -> None:
        """Entry point for the RTK reader thread."""
        logger.info(f"RTKReader: starting on {self._port}@{self._baud}")
        self._running = True

        while self._running:
            try:
                with serial.Serial(self._port, self._baud, timeout=1.0) as ser:
                    logger.info(f"RTKReader: opened {self._port}")
                    self._read_loop(ser)
            except serial.SerialException as e:
                logger.error(f"RTKReader: serial error: {e}. Retrying in 3s...")
                time.sleep(3.0)
            except Exception as e:
                logger.error(f"RTKReader: unexpected error: {e}. Retrying in 3s...")
                time.sleep(3.0)

    def _read_loop(self, ser: serial.Serial) -> None:
        """Inner read loop; exits on serial error to trigger reconnect."""
        while self._running:
            try:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line or line.startswith("#"):
                    continue

                # Process through NMEA pipeline
                self._pipeline.process(line)

                # Periodically snapshot the current state
                frame = self._pipeline.snapshot()
                with self._snapshot_lock:
                    self._snapshot = frame
                    if frame.lat is not None and frame.lon is not None:
                        logger.debug(
                            f"RTKReader: snapshot updated. lat={frame.lat:.6f} lon={frame.lon:.6f} sats={frame.num_sats}"
                        )

            except serial.SerialException:
                break
            except Exception as e:
                logger.warning(f"RTKReader: read error: {e}")
                break

    def snapshot(self) -> Optional[RTKFrame]:
        """Return the latest RTK frame snapshot (thread-safe)."""
        with self._snapshot_lock:
            return self._snapshot

    def stop(self) -> None:
        """Request the thread to stop gracefully."""
        self._running = False
