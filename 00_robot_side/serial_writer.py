"""
Thread-safe serial port write wrapper.
"""

import logging
import threading
from pathlib import Path

import serial

from config import FEATHER_PORT, SERIAL_BAUD, SERIAL_TIMEOUT, ALLOWED_COMMANDS

# ── Logging configuration ──────────────────────────────────
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


class SerialWriter:
    """Thread-safe serial port write wrapper."""

    def __init__(self, port: str = FEATHER_PORT, baud: int = SERIAL_BAUD) -> None:
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()
        self._ser: serial.Serial | None = None

    def open(self) -> None:
        """Open the serial port; raises on failure."""
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=SERIAL_TIMEOUT)
            logger.info(f"Serial port opened: {self._port} @ {self._baud} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port [{self._port}]: {e}")
            raise

    def close(self) -> None:
        """Close the serial port."""
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
                logger.info("Serial port closed")

    def write_command(self, char: str) -> None:
        """Write a control character to serial (whitelist-filtered; illegal chars are discarded)."""
        if char not in ALLOWED_COMMANDS:
            logger.warning(f"Illegal command character intercepted: {repr(char)}")
            return

        self._write_raw(char.encode())

    def emergency_stop(self) -> None:
        """Send an emergency stop (space) to the serial port; called on watchdog timeout."""
        logger.warning("Emergency stop triggered! Sending space to serial port")
        self._write_raw(b" ")

    def _write_raw(self, data: bytes) -> None:
        with self._lock:
            if self._ser is None or not self._ser.is_open:
                logger.error("Serial port not open, cannot write")
                return
            try:
                self._ser.write(data)
                logger.debug(f"Serial write: {repr(data)}")
            except serial.SerialException as e:
                logger.error(f"Serial write failed: {e}")
                raise

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open
