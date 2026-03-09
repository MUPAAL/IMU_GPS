"""
Robot-side main program.
TCP server: single-connection mode, receives keyboard commands + heartbeats,
forwards to Feather M4 CAN via serial port.

Usage:
    export FEATHER_PORT=/dev/cu.usbmodem14201   # macOS, optional
    python robot_receiver.py
"""

import logging
import signal
import socket
import sys
from pathlib import Path

from config import (
    FEATHER_PORT,
    HEARTBEAT_CHAR,
    TCP_HOST,
    TCP_PORT,
    WATCHDOG_TIMEOUT,
)
from core.serial_writer import SerialWriter
from core.watchdog import Watchdog

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


class RobotReceiver:
    """TCP server: receives remote keyboard commands and forwards them to the serial port."""

    def __init__(self) -> None:
        self._serial = SerialWriter()
        self._watchdog = Watchdog(
            timeout=WATCHDOG_TIMEOUT,
            on_timeout=self._on_watchdog_timeout,
        )
        self._server_sock: socket.socket | None = None
        self._running = False

    def setup(self) -> None:
        """Open the serial port and create the TCP server socket."""
        self._serial.open()
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((TCP_HOST, TCP_PORT))
        self._server_sock.listen(1)   # single-connection mode
        logger.info(f"TCP server started, listening on {TCP_HOST}:{TCP_PORT}")
        logger.info(f"Serial port: {FEATHER_PORT}, watchdog timeout: {WATCHDOG_TIMEOUT}s")

    def run(self) -> None:
        """Main loop: wait for client connections; restart after each disconnect."""
        self._running = True
        while self._running:
            logger.info("Waiting for remote client connection...")
            try:
                client_sock, addr = self._server_sock.accept()
            except OSError as e:
                if self._running:
                    logger.error(f"accept() failed: {e}")
                break

            logger.info(f"Remote client connected: {addr}")
            self._watchdog.start()
            try:
                self._handle_client(client_sock)
            finally:
                self._watchdog.stop()
                self._serial.emergency_stop()
                client_sock.close()
                logger.info(f"Remote client disconnected: {addr}, emergency stop sent")

    def _handle_client(self, sock: socket.socket) -> None:
        """Handle a single client connection; read one byte at a time and dispatch."""
        while True:
            try:
                data = sock.recv(1)
            except OSError as e:
                logger.warning(f"recv() error, connection lost: {e}")
                break

            if not data:
                # TCP graceful close (recv returns empty bytes)
                logger.info("Remote client closed connection gracefully")
                break

            char = data.decode("utf-8", errors="ignore")
            self._dispatch(char)

    def _dispatch(self, char: str) -> None:
        self._watchdog.reset()
        if char == HEARTBEAT_CHAR:
            logger.debug("Heartbeat received")
        else:
            self._serial.write_command(char)
            logger.info(f"Command: {repr(char)}")

    def _on_watchdog_timeout(self) -> None:
        """Emergency stop on watchdog timeout (runs in timer thread)."""
        self._serial.emergency_stop()

    def shutdown(self) -> None:
        """Stop the main loop, close socket and serial port."""
        logger.info("Shutting down robot receiver...")
        self._running = False
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError as e:
                logger.warning(f"Error closing server socket: {e}")
        self._watchdog.stop()
        self._serial.close()
        logger.info("Robot receiver shut down")


def main() -> None:
    receiver = RobotReceiver()

    def _signal_handler(signum, frame):
        logger.info(f"Signal {signum} received, starting graceful shutdown...")
        receiver.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        receiver.setup()
        receiver.run()
    except Exception as e:
        logger.error(f"Robot receiver encountered an error: {e}")
        raise
    finally:
        receiver.shutdown()


if __name__ == "__main__":
    main()