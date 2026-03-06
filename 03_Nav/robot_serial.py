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

# **************** IMPORTS ****************

import logging
import threading
import time

import serial

# **************** CONSTANTS ****************

logger = logging.getLogger(__name__)

STATE_AUTO_READY  = 2   # robot is powered but not accepting motion commands
STATE_AUTO_ACTIVE = 3   # robot is ready and will execute V commands

CMD_STOP = "V0.00,0.000\n"   # safe stop command sent whenever state != ACTIVE


# **************** CLASS: RobotSerial ****************

class RobotSerial:
    """
    Thread-safe serial interface for the robot.

    Data flow:
      nav_bridge  ──send_velocity()──►  _pending_cmd  ──_writer_loop()──►  serial TX
      serial RX   ──_reader_loop()──►  _parse_feedback()  ──►  _feedback{}
      nav_bridge  ──get_feedback()──►  _feedback{}

    Start with RobotSerial.start(), send commands with send_velocity(),
    read feedback with get_feedback().
    """

    # **************** INIT ****************

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0) -> None:
        self._port    = port
        self._baud    = baud
        self._timeout = timeout

        # Lock for _feedback dict (reader thread writes, nav_bridge reads)
        self._lock = threading.Lock()

        # Internal feedback store — updated by _parse_feedback(), read by get_feedback()
        self._feedback: dict = {
            "meas_speed_ms":      0.0,
            "meas_ang_rate_rads": 0.0,
            "state":              0,
            "state_name":         "UNKNOWN",
            "soc":                0,
            "ts":                 0.0,
            "connected":          False,
        }

        # Pending command slot — written by send_velocity(), consumed by _writer_loop()
        self._pending_cmd: str | None = None
        self._cmd_lock = threading.Lock()

        self._ser: serial.Serial | None = None
        self._reader_thread: threading.Thread | None = None
        self._writer_thread: threading.Thread | None = None
        self._running = False

    # **************** PUBLIC API ****************

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

    # ****************
    # **************** DATA INPUT: nav_bridge calls this to inject a velocity command
    # ****************

    def send_velocity(self, speed_ms: float, ang_rate_rads: float) -> None:
        """
        Queue a velocity command. The writer thread picks it up on the next cycle.

        Caller (nav_bridge) sets desired speed/ang_rate;
        _writer_loop() applies the safety check and writes to serial.

        Safety: if robot state != STATE_AUTO_ACTIVE, writer overrides with CMD_STOP.
        """
        cmd = f"V{speed_ms:.2f},{ang_rate_rads:.3f}\n"
        with self._cmd_lock:
            self._pending_cmd = cmd   # overwrite any unsent previous command

    # ****************
    # **************** DATA OUTPUT: nav_bridge calls this to read back robot state
    # ****************

    def get_feedback(self) -> dict:
        """
        Return a thread-safe shallow copy of the latest O feedback snapshot.

        Keys:
          meas_speed_ms      — measured wheel speed [m/s]
          meas_ang_rate_rads — measured angular rate [rad/s]
          state              — robot FSM state integer (2=READY, 3=ACTIVE)
          state_name         — human-readable state string
          soc                — battery state of charge [%]
          ts                 — time.time() of last received O line
          connected          — True if serial port is open and responding
        """
        with self._lock:
            return dict(self._feedback)

    # **************** INTERNAL THREADS ****************

    # ****************
    # **************** DATA INPUT: serial RX → _feedback (robot → this module)
    # ****************

    def _reader_loop(self) -> None:
        """
        Daemon thread: continuously reads lines from serial RX.

        Each line is dispatched to _parse_feedback().
        On serial error: sets connected=False and exits thread.
        """
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
                    self._parse_feedback(line)   # ← INJECT into _feedback
            except serial.SerialException as e:
                logger.error("RobotSerial: serial read error: %s", e)
                with self._lock:
                    self._feedback["connected"] = False
                self._running = False
            except Exception as e:
                logger.error("RobotSerial: unexpected reader error: %s", e)
        logger.info("RobotSerial: reader thread exiting")

    # ****************
    # **************** DATA OUTPUT: _pending_cmd → serial TX (this module → robot)
    # ****************

    def _writer_loop(self) -> None:
        """
        Daemon thread: sends queued velocity commands at ~20 Hz.

        Safety gate: checks robot FSM state before every write.
          - STATE_AUTO_ACTIVE  → send the queued V command as-is
          - Any other state    → override with CMD_STOP ("V0.00,0.000")

        This ensures the robot cannot be driven accidentally if it drops
        out of active mode (e.g. E-stop, fault, or during bootup).
        """
        logger.info("RobotSerial: writer thread started")
        while self._running:
            try:
                # ── Consume pending command ───────────────
                with self._cmd_lock:
                    cmd = self._pending_cmd
                    self._pending_cmd = None

                if cmd is None:
                    time.sleep(0.05)
                    continue

                # ── Safety gate: state check ───────────────
                with self._lock:
                    robot_state = self._feedback.get("state", 0)

                if robot_state != STATE_AUTO_ACTIVE:
                    if cmd != CMD_STOP:
                        logger.debug(
                            "RobotSerial: state=%d (not ACTIVE), sending stop instead of: %s",
                            robot_state,
                            cmd.strip(),
                        )
                    cmd = CMD_STOP   # override with safe stop

                # ── Write to serial TX ────────────────────
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

    # **************** FEEDBACK PARSER ****************

    def _parse_feedback(self, line: str) -> None:
        """
        Parse one O feedback line and update _feedback under lock.

        Expected format:
          O:{meas_speed:.3f},{meas_ang_rate:.3f},{state:d},{soc:d}

        Example:
          O:0.480,0.115,3,85

        Invalid or non-O lines are silently skipped (not our protocol).
        """
        if not line.startswith("O:"):
            return   # not a feedback line (could be debug print from robot)
        try:
            payload = line[2:]   # strip leading "O:"
            parts   = payload.split(",")
            if len(parts) < 4:
                logger.warning("RobotSerial: short O feedback, skipping: %r", line)
                return

            meas_speed    = float(parts[0])
            meas_ang_rate = float(parts[1])
            state         = int(parts[2])
            soc           = int(parts[3])

            state_names = {
                STATE_AUTO_READY:  "AUTO_READY",
                STATE_AUTO_ACTIVE: "AUTO_ACTIVE",
            }
            state_name = state_names.get(state, f"STATE_{state}")

            # ── Store parsed values (thread-safe) ─────────
            with self._lock:
                self._feedback["meas_speed_ms"]      = meas_speed
                self._feedback["meas_ang_rate_rads"] = meas_ang_rate
                self._feedback["state"]              = state
                self._feedback["state_name"]         = state_name
                self._feedback["soc"]                = soc
                self._feedback["ts"]                 = time.time()
                self._feedback["connected"]          = True

        except (ValueError, IndexError) as e:
            logger.warning("RobotSerial: failed to parse O feedback: %s — line=%r", e, line)
