"""
RTK GPS Reader — Emlid RS+ NMEA parser (GGA / RMC)

Architecture:
  RTKReader(threading.Thread, daemon=True)
    ├─ Opens serial port (RTK_PORT, RTK_BAUD)
    ├─ Loops readline() → _dispatch()
    │    ├─ _parse_gga(): lat/lon/alt/fix_quality/num_sats/hdop
    │    └─ _parse_rmc(): speed_knots/track_deg (only when status=="A")
    └─ get_data() → thread-safe shallow copy snapshot

Graceful degradation:
  - Serial open failure → is_available=False, thread exits cleanly
  - Malformed NMEA sentence → logger.warning, skip line
  - Checksum mismatch → logger.warning, skip line

Usage:
    reader = RTKReader()
    reader.start()
    snap = reader.get_data()   # {"lat": ..., "lon": ..., ...}
"""

# **************** IMPORTS ****************

import logging
import os
import threading
import time

import serial

# **************** SERIAL PORT CONFIG (from environment variables) ****************
#
# Override at runtime without editing code:
#   export RTK_PORT=/dev/ttyUSB0
#   export RTK_BAUD=115200

RTK_PORT:    str   = os.environ.get("RTK_PORT",    "/dev/cu.usbmodem2403")
RTK_BAUD:    int   = int(os.environ.get("RTK_BAUD",    "9600"))
RTK_TIMEOUT: float = float(os.environ.get("RTK_TIMEOUT", "1.0"))

logger = logging.getLogger(__name__)


# **************** CLASS: RTKReader ****************

class RTKReader(threading.Thread):
    """
    Daemon thread: reads NMEA sentences from the Emlid RS+ GPS receiver
    and maintains a thread-safe internal data snapshot.

    Data flow:
      serial RX  ──run()──►  _dispatch()  ──►  _parse_gga() / _parse_rmc()
                                                   └──►  _data{}  (under _lock)
      rtk_bridge ──get_data()──►  shallow copy of _data{}
    """

    # **************** INIT ****************

    def __init__(self) -> None:
        super().__init__(name="RTKReader", daemon=True)
        self._lock = threading.Lock()

        # Internal data store — written by parser methods, read by get_data()
        self._data: dict = {
            "lat":         None,   # float, decimal degrees (+N / -S)
            "lon":         None,   # float, decimal degrees (+E / -W)
            "alt":         None,   # float, metres above MSL
            "fix_quality": 0,      # int: 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
            "num_sats":    0,      # int, number of satellites in use
            "hdop":        None,   # float, horizontal dilution of precision
            "speed_knots": None,   # float, ground speed in knots (from RMC)
            "track_deg":   None,   # float, true course over ground in degrees (from RMC)
            "ts":          0.0,    # float, time.time() of last successful GGA update
            "raw_gga":     "",     # str, last raw GGA sentence (for debugging)
        }
        self._available = False    # True once serial port opens successfully

    # **************** PUBLIC API ****************

    @property
    def is_available(self) -> bool:
        """True if the serial port opened successfully."""
        return self._available

    # ****************
    # **************** DATA OUTPUT: rtk_bridge reads latest snapshot via this method
    # ****************

    def get_data(self) -> dict:
        """
        Return a thread-safe shallow copy of the latest RTK data snapshot.

        Callers (rtk_bridge) poll this at broadcast_hz; no blocking occurs because
        the copy operation is protected by a lock rather than waiting for a new frame.

        Returned dict keys:
          lat, lon, alt        — WGS-84 position (None until first GGA)
          fix_quality          — 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
          num_sats, hdop       — quality indicators
          speed_knots          — ground speed [knots], None until first valid RMC
          track_deg            — true course [°], None until first valid RMC
          ts                   — Unix timestamp of last GGA update
          raw_gga              — last raw GGA sentence string
        """
        with self._lock:
            return dict(self._data)

    # ****************
    # **************** DATA INPUT: serial RX → _data (GPS hardware → this thread)
    # ****************

    def run(self) -> None:
        """
        Thread entry point.  Opens the serial port, then loops calling readline()
        and dispatching each NMEA sentence to _dispatch().

        On serial open failure: logs error, sets is_available=False, exits thread.
        On runtime serial error: logs error, breaks inner loop, closes port, exits.
        Always closes the serial port in the finally block.
        """
        try:
            ser = serial.Serial(RTK_PORT, RTK_BAUD, timeout=RTK_TIMEOUT)
        except serial.SerialException as e:
            logger.error(f"RTKReader: failed to open serial port [{RTK_PORT}]: {e}")
            self._available = False
            return

        self._available = True
        logger.info(f"RTKReader: serial port opened: {RTK_PORT} @ {RTK_BAUD} baud")

        try:
            while True:
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line:
                        self._dispatch(line)   # ← route to GGA/RMC parser
                except serial.SerialException as e:
                    logger.error(f"RTKReader: serial read error: {e}")
                    break
                except Exception as e:
                    logger.error(f"RTKReader: unexpected error during readline: {e}")
        finally:
            try:
                ser.close()
            except Exception as e:
                logger.warning(f"RTKReader: error closing serial port: {e}")
            self._available = False
            logger.info("RTKReader: thread exiting")

    # **************** NMEA DISPATCH ****************

    def _dispatch(self, line: str) -> None:
        """
        Verify NMEA checksum, identify sentence type, route to the correct parser.

        Only GGA and RMC sentences are processed; all others are silently ignored.
        Talker prefix (GP / GN / GL / GA …) is stripped before type comparison,
        so both $GPGGA and $GNGGA are matched as "GGA".
        """
        if not line.startswith("$"):
            return   # not an NMEA sentence

        if not self._verify_checksum(line):
            logger.warning(f"RTKReader: checksum mismatch, skipping: {line!r}")
            return

        # Strip '$' prefix and trailing '*XX' checksum to get the field body
        body  = line[1:].split("*")[0]
        parts = body.split(",")
        if not parts:
            return

        # Sentence type = talker_msg without the 2-char talker prefix (GP/GN/GL…)
        sentence_type = parts[0][2:]

        if sentence_type == "GGA":
            self._parse_gga(parts)
        elif sentence_type == "RMC":
            self._parse_rmc(parts)

    # ****************
    # **************** DATA INPUT: GGA sentence → _data position fields
    # ****************

    def _parse_gga(self, parts: list[str]) -> None:
        """
        Parse a GGA sentence and update position + quality fields in _data.

        GGA sentence structure:
          $GPGGA,HHMMSS.ss,LLLL.LL,a,YYYYY.YY,a,x,xx,x.x,x.x,M,...*hh
          index:    0        1      2  3        4  5  6   7   8   9

          [1]  UTC time
          [2]  Latitude  DDMM.MMMMM
          [3]  N/S indicator
          [4]  Longitude DDDMM.MMMMM
          [5]  E/W indicator
          [6]  Fix quality (0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float)
          [7]  Number of satellites in use
          [8]  HDOP
          [9]  Altitude MSL (metres)

        Updates: lat, lon, alt, fix_quality, num_sats, hdop, ts, raw_gga
        """
        try:
            if len(parts) < 11:
                logger.warning(f"RTKReader: GGA too short: {parts}")
                return

            fix_quality = int(parts[6])   if parts[6] else 0
            num_sats    = int(parts[7])   if parts[7] else 0
            hdop        = float(parts[8]) if parts[8] else None
            alt         = float(parts[9]) if parts[9] else None

            lat = self._nmea_to_decimal(parts[2], parts[3]) if (parts[2] and parts[3]) else None
            lon = self._nmea_to_decimal(parts[4], parts[5]) if (parts[4] and parts[5]) else None

            # ── INJECT parsed GGA fields into shared data store ───
            with self._lock:
                self._data["lat"]         = lat
                self._data["lon"]         = lon
                self._data["alt"]         = alt
                self._data["fix_quality"] = fix_quality
                self._data["num_sats"]    = num_sats
                self._data["hdop"]        = hdop
                self._data["ts"]          = time.time()
                self._data["raw_gga"]     = ",".join(parts)

        except (ValueError, IndexError) as e:
            logger.warning(f"RTKReader: failed to parse GGA: {e} — parts={parts}")

    # ****************
    # **************** DATA INPUT: RMC sentence → _data speed/course fields
    # ****************

    def _parse_rmc(self, parts: list[str]) -> None:
        """
        Parse a RMC sentence and update speed / track fields in _data.

        RMC sentence structure:
          $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,...*hh
          index:    0        1 2       3  4        5  6   7   8

          [2]  Status: 'A'=active (valid), 'V'=void (skip)
          [7]  Speed over ground [knots]
          [8]  True course over ground [degrees]

        Updates: speed_knots, track_deg
        Only processed when status == 'A' (valid fix).
        """
        try:
            if len(parts) < 9:
                return
            status = parts[2]
            if status != "A":
                return   # void fix — position data not reliable

            speed_knots = float(parts[7]) if parts[7] else None
            track_deg   = float(parts[8]) if parts[8] else None

            # ── INJECT parsed RMC fields into shared data store ───
            with self._lock:
                self._data["speed_knots"] = speed_knots
                self._data["track_deg"]   = track_deg

        except (ValueError, IndexError) as e:
            logger.warning(f"RTKReader: failed to parse RMC: {e} — parts={parts}")

    # **************** STATIC HELPERS ****************

    @staticmethod
    def _verify_checksum(sentence: str) -> bool:
        """
        Validate NMEA XOR checksum.

        The checksum is the XOR of all bytes between '$' and '*' (exclusive).
        The two hex digits after '*' are the expected checksum value.

        Returns False if '*' is absent, hex digits are malformed, or XOR
        of payload bytes does not match the declared checksum.
        """
        try:
            star_idx = sentence.rindex("*")
        except ValueError:
            return False   # no checksum field present

        payload      = sentence[1:star_idx]
        expected_hex = sentence[star_idx + 1:star_idx + 3]
        if len(expected_hex) != 2:
            return False

        try:
            expected = int(expected_hex, 16)
        except ValueError:
            return False

        computed = 0
        for ch in payload:
            computed ^= ord(ch)
        return computed == expected

    @staticmethod
    def _nmea_to_decimal(raw: str, direction: str) -> float:
        """
        Convert NMEA coordinate string to decimal degrees.

        NMEA format:
          Latitude:  DDMM.MMMMM  (2-digit degree prefix)
          Longitude: DDDMM.MMMMM (3-digit degree prefix)

        Algorithm:
          degrees = integer part before last 2 digits before the decimal point
          minutes = last 2 digits before decimal + decimal fraction
          decimal_degrees = degrees + minutes / 60.0

        direction: 'N' / 'E' → positive,  'S' / 'W' → negative
        """
        dot_idx = raw.index(".")
        deg_str = raw[:dot_idx - 2]    # everything before the last 2 pre-dot digits
        min_str = raw[dot_idx - 2:]    # last 2 pre-dot digits + decimal fraction
        degrees = float(deg_str) + float(min_str) / 60.0
        if direction in ("S", "W"):
            degrees = -degrees
        return degrees
