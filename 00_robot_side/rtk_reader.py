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

import logging
import threading
import time
from pathlib import Path

import serial

from config import RTK_PORT, RTK_BAUD, RTK_TIMEOUT

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


class RTKReader(threading.Thread):
    """Daemon thread: reads NMEA sentences from Emlid RS+ and updates internal data store."""

    def __init__(self) -> None:
        super().__init__(name="RTKReader", daemon=True)
        self._lock = threading.Lock()
        self._data: dict = {
            "lat":         None,   # float, decimal degrees (+N/-S)
            "lon":         None,   # float, decimal degrees (+E/-W)
            "alt":         None,   # float, metres (MSL)
            "fix_quality": 0,      # int: 0=no fix,1=GPS,2=DGPS,4=RTK fixed,5=RTK float
            "num_sats":    0,      # int
            "hdop":        None,   # float
            "speed_knots": None,   # float
            "track_deg":   None,   # float, true heading degrees
            "ts":          0.0,    # float, time.time() of last update
            "raw_gga":     "",     # str, last raw GGA sentence
        }
        self._available = False   # True once serial port opens successfully

    # ── Public API ────────────────────────────────────────
    @property
    def is_available(self) -> bool:
        """True if the serial port opened successfully."""
        return self._available

    def get_data(self) -> dict:
        """Return a thread-safe shallow copy of the latest RTK data snapshot."""
        with self._lock:
            return dict(self._data)

    # ── Thread entry ──────────────────────────────────────
    def run(self) -> None:
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
                        self._dispatch(line)
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

    # ── NMEA dispatch ─────────────────────────────────────
    def _dispatch(self, line: str) -> None:
        """Verify checksum and route to GGA/RMC parser."""
        if not line.startswith("$"):
            return
        if not self._verify_checksum(line):
            logger.warning(f"RTKReader: checksum mismatch, skipping: {line!r}")
            return

        # Strip leading '$' and trailing checksum '*XX'
        body = line[1:].split("*")[0]
        parts = body.split(",")
        if not parts:
            return

        # talker_msg e.g. GPGGA / GNGGA / GPRMC / GNRMC
        sentence_type = parts[0][2:]  # slice off 2-char talker prefix (GP/GN/GL…)

        if sentence_type == "GGA":
            self._parse_gga(parts)
        elif sentence_type == "RMC":
            self._parse_rmc(parts)

    # ── GGA parser ────────────────────────────────────────
    def _parse_gga(self, parts: list[str]) -> None:
        """
        $GPGGA,HHMMSS.ss,LLLL.LL,a,YYYYY.YY,a,x,xx,x.x,x.x,M,...*hh
        Index:    0        1      2  3        4  5  6   7   8   9
        """
        try:
            if len(parts) < 10:
                logger.warning(f"RTKReader: GGA too short: {parts}")
                return

            fix_quality = int(parts[6]) if parts[6] else 0
            num_sats    = int(parts[7]) if parts[7] else 0
            hdop        = float(parts[8]) if parts[8] else None
            alt         = float(parts[9]) if parts[9] else None

            lat = self._nmea_to_decimal(parts[1], parts[2]) if (parts[1] and parts[2]) else None
            lon = self._nmea_to_decimal(parts[3], parts[4]) if (parts[3] and parts[4]) else None

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

    # ── RMC parser ────────────────────────────────────────
    def _parse_rmc(self, parts: list[str]) -> None:
        """
        $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,...*hh
        Index:    0        1 2       3  4        5  6   7   8
        parts[2] = status ('A'=active/'V'=void)
        """
        try:
            if len(parts) < 9:
                return
            status = parts[2]
            if status != "A":
                return  # void fix — skip

            speed_knots = float(parts[7]) if parts[7] else None
            track_deg   = float(parts[8]) if parts[8] else None

            with self._lock:
                self._data["speed_knots"] = speed_knots
                self._data["track_deg"]   = track_deg

        except (ValueError, IndexError) as e:
            logger.warning(f"RTKReader: failed to parse RMC: {e} — parts={parts}")

    # ── Static helpers ────────────────────────────────────
    @staticmethod
    def _verify_checksum(sentence: str) -> bool:
        """XOR checksum: characters between '$' and '*' must equal hex after '*'."""
        try:
            star_idx = sentence.rindex("*")
        except ValueError:
            return False  # no checksum field
        payload = sentence[1:star_idx]
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
        Convert NMEA coordinate to decimal degrees.
        Latitude format:  DDMM.MMMMM  (2-digit degrees)
        Longitude format: DDDMM.MMMMM (3-digit degrees)
        direction: 'N'/'S' for latitude, 'E'/'W' for longitude
        """
        dot_idx = raw.index(".")
        # degrees: all digits before the last 2 digits before the dot
        deg_str = raw[:dot_idx - 2]
        min_str = raw[dot_idx - 2:]
        degrees = float(deg_str) + float(min_str) / 60.0
        if direction in ("S", "W"):
            degrees = -degrees
        return degrees
