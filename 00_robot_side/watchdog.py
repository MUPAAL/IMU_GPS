"""
Watchdog timer.
Call start() on client connect, reset() on every message, stop() on disconnect.
If no reset() is received within the timeout, the emergency-stop callback fires automatically.
"""

import logging
import threading
from pathlib import Path

from config import WATCHDOG_TIMEOUT

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


class Watchdog:
    """
    Timeout watchdog.
    If reset() is not called within `timeout` seconds, on_timeout callback is invoked (emergency stop).
    """

    def __init__(self, timeout: float = WATCHDOG_TIMEOUT, on_timeout=None) -> None:
        self._timeout = timeout
        self._on_timeout = on_timeout or self._default_timeout_handler
        self._timer: threading.Timer | None = None
        self._lock = threading.Lock()
        self._active = False

    def start(self) -> None:
        """Start the watchdog."""
        with self._lock:
            self._active = True
            self._schedule()
        logger.info(f"Watchdog started, timeout: {self._timeout}s")

    def reset(self) -> None:
        """Reset the timeout timer."""
        with self._lock:
            if not self._active:
                return
            self._cancel()
            self._schedule()

    def stop(self) -> None:
        """Stop the watchdog (does NOT trigger emergency stop)."""
        with self._lock:
            self._active = False
            self._cancel()
        logger.info("Watchdog stopped")

    def _schedule(self) -> None:
        self._timer = threading.Timer(self._timeout, self._trigger)
        self._timer.daemon = True
        self._timer.start()

    def _cancel(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _trigger(self) -> None:
        logger.warning(f"Watchdog timeout! No message received for {self._timeout}s, triggering emergency stop")
        try:
            self._on_timeout()
        except Exception as e:
            logger.error(f"Emergency stop callback raised an exception: {e}")

    @staticmethod
    def _default_timeout_handler() -> None:
        logger.error("Watchdog timeout: no emergency stop callback configured!")
