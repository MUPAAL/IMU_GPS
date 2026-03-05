"""
robot_serial.py - Robot serial communication module.

Sends V velocity commands and receives O odometry feedback.

Protocol:
  TX: V{speed:.2f},{ang_rate:.3f}\n
  RX: O:{meas_speed:.3f},{meas_ang_rate:.3f},{state:d},{soc:d}\n

Robot states:
  2 = STATE_AUTO_READY  (standby, ignores speed commands)
  3 = STATE_AUTO_ACTIVE (active, executes speed commands)

Safety rule: V commands are only sent when state == STATE_AUTO_ACTIVE.
Otherwise a stop command V0.00,0.000 is sent automatically.
"""

import logging
import threading
import time

import serial

logger = logging.getLogger(__name__)

STATE_AUTO_READY = 2
STATE_AUTO_ACTIVE = 3

CMD_STOP = "V0.00,0.000\n"


class RobotSerial:
    """
    Thread-safe serial interface for the robot.

    Start with RobotSerial.start(), send commands with send_velocity(),
    read feedback with get_feedback().
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout

        self._lock = threading.Lock()
        self._feedback: dict = {
            "meas_speed_ms": 0.0,
            "meas_ang_rate_rads": 0.0,
            "state": 0,
            "state_name": "UNKNOWN",
            "soc": 0,
            "ts": 0.0,
            "connected": False,
        }

        self._pending_cmd: str | None = None
        self._cmd_lock = threading.Lock()

        self._ser: serial.Serial | None = None
        self._reader_thread: threading.Thread | None = None
        self._writer_thread: threading.Thread | None = None
        self._running = False

    # ── Public API ────────────────────────────────────────

    def start(self) -> bool:
        """
        Open serial port and start reader/writer threads.
        Returns True if port opened successfully.
        """
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
            logger.info("RobotSerial: opened port %s @ %d baud", self._port, self._baud)
        except serial.SerialException as e:
            logger.error("RobotSerial: failed to open port %s: %s", self._port, e)
            with self._lock:
                self._feedback["connected"] = False
            return False

        self._running = True
        with self._lock:
            self._feedback["connected"] = True

        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="robot-reader", daemon=True
        )
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="robot-writer", daemon=True
        )
        self._reader_thread.start()
        self._writer_thread.start()
        return True

    def stop(self) -> None:
        """Stop threads and close serial port."""
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception as e:
                logger.warning("RobotSerial: error closing port: %s", e)
        with self._lock:
            self._feedback["connected"] = False
        logger.info("RobotSerial: stopped")

    def send_velocity(self, speed_ms: float, ang_rate_rads: float) -> None:
        """
        Queue a velocity command. The writer thread sends it on the next cycle.
        Safety: if robot state != STATE_AUTO_ACTIVE, stop command is sent instead.
        """
        cmd = f"V{speed_ms:.2f},{ang_rate_rads:.3f}\n"
        with self._cmd_lock:
            self._pending_cmd = cmd

    def get_feedback(self) -> dict:
        """Return a thread-safe shallow copy of the latest O feedback."""
        with self._lock:
            return dict(self._feedback)

    # ── Internal threads ──────────────────────────────────

    def _reader_loop(self) -> None:
        """Read O feedback lines from serial and update internal state."""
        logger.info("RobotSerial: reader thread started")
        while self._running:
            try:
                if self._ser is None or not self._ser.is_open:
                    time.sleep(0.1)
                    continue
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if line:
                    self._parse_feedback(line)
            except serial.SerialException as e:
                logger.error("RobotSerial: serial read error: %s", e)
                with self._lock:
                    self._feedback["connected"] = False
                self._running = False
            except Exception as e:
                logger.error("RobotSerial: unexpected reader error: %s", e)
        logger.info("RobotSerial: reader thread exiting")

    def _writer_loop(self) -> None:
        """
        Send queued velocity commands at ~20 Hz.
        Applies safety check: only send V commands in STATE_AUTO_ACTIVE.
        """
        logger.info("RobotSerial: writer thread started")
        while self._running:
            try:
                with self._cmd_lock:
                    cmd = self._pending_cmd
                    self._pending_cmd = None

                if cmd is None:
                    time.sleep(0.05)
                    continue

                # Safety check: override with stop if not in active state
                with self._lock:
                    robot_state = self._feedback.get("state", 0)

                if robot_state != STATE_AUTO_ACTIVE:
                    if cmd != CMD_STOP:
                        logger.debug(
                            "RobotSerial: state=%d (not ACTIVE), sending stop instead of: %s",
                            robot_state,
                            cmd.strip(),
                        )
                    cmd = CMD_STOP

                if self._ser and self._ser.is_open:
                    self._ser.write(cmd.encode("ascii"))
                    self._ser.flush()

            except serial.SerialException as e:
                logger.error("RobotSerial: serial write error: %s", e)
                with self._lock:
                    self._feedback["connected"] = False
                self._running = False
            except Exception as e:
                logger.error("RobotSerial: unexpected writer error: %s", e)

            time.sleep(0.05)  # ~20 Hz write rate
        logger.info("RobotSerial: writer thread exiting")

    def _parse_feedback(self, line: str) -> None:
        """
        Parse O feedback line:
          O:{meas_speed:.3f},{meas_ang_rate:.3f},{state:d},{soc:d}
        """
        if not line.startswith("O:"):
            return
        try:
            payload = line[2:]  # strip 'O:'
            parts = payload.split(",")
            if len(parts) < 4:
                logger.warning("RobotSerial: short O feedback, skipping: %r", line)
                return

            meas_speed = float(parts[0])
            meas_ang_rate = float(parts[1])
            state = int(parts[2])
            soc = int(parts[3])

            state_names = {
                STATE_AUTO_READY: "AUTO_READY",
                STATE_AUTO_ACTIVE: "AUTO_ACTIVE",
            }
            state_name = state_names.get(state, f"STATE_{state}")

            with self._lock:
                self._feedback["meas_speed_ms"] = meas_speed
                self._feedback["meas_ang_rate_rads"] = meas_ang_rate
                self._feedback["state"] = state
                self._feedback["state_name"] = state_name
                self._feedback["soc"] = soc
                self._feedback["ts"] = time.time()
                self._feedback["connected"] = True

        except (ValueError, IndexError) as e:
            logger.warning("RobotSerial: failed to parse O feedback: %s — line=%r", e, line)
