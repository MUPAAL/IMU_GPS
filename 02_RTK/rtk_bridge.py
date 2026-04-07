"""
rtk_bridge.py — RTK serial reader to WebSocket bridge + static web map UI.

Data flow:
    Serial Port A/B → SerialReader(s) → NMEAPipeline(s) (accumulate GGA + RMC state)
                                               ↓  snapshot() on active source at N Hz
                                        BroadcastLoop → WebSocketServer → Browser

NMEAPipeline stages:
    raw NMEA line
      → _verify_checksum()   → validated line | None
      → _dispatch()          → routes by sentence type
      → _parse_gga()         → updates internal RTKFrame (lat/lon/alt/fix/sats/hdop)
      → _parse_rmc()         → updates internal RTKFrame (speed/track)

Runs:
    - SerialReader daemon thread(s) (NMEA parser, auto-reconnect on failure)
  - HttpFileServer daemon thread (serves web_static/)
  - BroadcastLoop asyncio coroutine (polls pipeline, pushes to WS clients)
  - WebSocketServer asyncio server

Port convention (same as 01_IMU):
    HTTP  = --ws-port        (default 8775)
    WebSocket = --ws-port+1  (default 8776)

Multi-RTK mode:
    - pass one or two --port values
    - the browser can switch the active source when two receivers are present
"""

# ── IMPORTS ────────────────────────────────────────────────────────────────────

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

import argparse
import asyncio
import copy
import json
import logging
import math
import serial
import socketserver
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from http.server import SimpleHTTPRequestHandler
from pathlib import Path
from typing import Optional

import websockets

# ── LOGGING ────────────────────────────────────────────────────────────────────

_py_name = Path(__file__).stem
OUTPUT_PATH = Path(__file__).parent
log_file_name = OUTPUT_PATH / f"{_py_name}.log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_file_name, encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

# ── DEFAULT POSITION (fallback when no GPS fix) ────────────────────────────────
#
# Used by RTKFrame.to_dict() when lat/lon are still None.
# Set to the known field-test site so the map opens in the right place.

DEFAULT_LAT: float = 38.9412928598587
DEFAULT_LON: float = -92.31884600793728

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 1 · DATA MODEL
# ══════════════════════════════════════════════════════════════════════════════


@dataclass
class RTKFrame:
    """
    Typed data carrier for one RTK position snapshot.

    Fields are accumulated incrementally across multiple NMEA sentences:
      - GGA fills: lat, lon, alt, fix_quality, num_sats, hdop, rtk_ts
      - RMC fills: speed_knots, track_deg
    """

    lat:         Optional[float] = None   # decimal degrees, +N/-S
    lon:         Optional[float] = None   # decimal degrees, +E/-W
    alt:         Optional[float] = None   # metres above MSL
    fix_quality: int             = 0      # 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    num_sats:    int             = 0      # satellites in use
    hdop:        Optional[float] = None   # horizontal dilution of precision
    speed_knots: Optional[float] = None   # ground speed from RMC
    track_deg:   Optional[float] = None   # true course over ground from RMC
    rtk_ts:      Optional[float] = None   # Unix timestamp of last GGA update

    def to_dict(
        self,
        *,
        server_ts: float,
        default_lat: float,
        default_lon: float,
    ) -> dict:
        """
        Serialize to broadcast-ready dict.

        When lat/lon are None (no fix yet), substitutes default_lat/default_lon
        and sets source='default' so the frontend can show a placeholder marker.

        Output keys:
          lat, lon, alt           — position
          fix_quality             — GPS fix type
          num_sats, hdop          — quality indicators
          speed_knots, track_deg  — motion (may be None)
          rtk_ts                  — timestamp of last GGA (None until first fix)
          server_ts               — time.time() at serialization
          source                  — "rtk" | "default"
        """
        use_default = self.lat is None or self.lon is None
        return {
            "lat":         default_lat if use_default else self.lat,
            "lon":         default_lon if use_default else self.lon,
            "alt":         self.alt,
            "fix_quality": self.fix_quality,
            "num_sats":    self.num_sats,
            "hdop":        self.hdop,
            "speed_knots": self.speed_knots,
            "track_deg":   self.track_deg,
            "rtk_ts":      self.rtk_ts if self.rtk_ts is not None else 0.0,
            "server_ts":   server_ts,
            "source":      "default" if use_default else "rtk",
        }


@dataclass
class RTKSourceInfo:
    """Runtime status for one configured RTK serial source."""

    source_id: str
    port: str
    label: str
    connected: bool = False
    last_seen_ts: Optional[float] = None
    last_error: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "source_id": self.source_id,
            "port": self.port,
            "label": self.label,
            "connected": self.connected,
            "last_seen_ts": self.last_seen_ts,
            "last_error": self.last_error,
        }


class RTKSourceManager:
    """Manage multiple RTK pipelines and expose one active source at a time."""

    def __init__(
        self,
        sources: list[tuple[str, str]],
        default_lat: float,
        default_lon: float,
        heading_source_a: str,
        heading_source_b: str,
        heading_offset_deg: float,
        heading_min_baseline_m: float,
    ) -> None:
        if not sources:
            raise ValueError("RTKSourceManager requires at least one source")

        self._pipelines = {source_id: NMEAPipeline() for source_id, _ in sources}
        self._sources = {
            source_id: RTKSourceInfo(
                source_id=source_id,
                port=port,
                label=f"RTK {index + 1} ({port})",
            )
            for index, (source_id, port) in enumerate(sources)
        }
        self._active_source = sources[0][0]
        self._default_lat = default_lat
        self._default_lon = default_lon
        self._heading_source_a = heading_source_a
        self._heading_source_b = heading_source_b
        self._heading_offset_deg = heading_offset_deg
        self._heading_min_baseline_m = heading_min_baseline_m
        self._lock = threading.Lock()

    def source_options(self) -> list[dict]:
        with self._lock:
            return [info.to_dict() for info in self._sources.values()]

    def active_source(self) -> str:
        with self._lock:
            return self._active_source

    def active_source_label(self) -> str:
        with self._lock:
            info = self._sources.get(self._active_source)
            return info.label if info else self._active_source

    def set_active_source(self, source_id: str) -> None:
        with self._lock:
            if source_id not in self._sources:
                raise KeyError(source_id)
            if source_id != self._active_source:
                logger.info("RTKSourceManager: active source changed to %s", source_id)
            self._active_source = source_id

    def mark_connected(self, source_id: str, connected: bool) -> None:
        with self._lock:
            info = self._sources.get(source_id)
            if info is None:
                return
            info.connected = connected
            if connected:
                info.last_error = None

    def mark_seen(self, source_id: str) -> None:
        with self._lock:
            info = self._sources.get(source_id)
            if info is not None:
                info.last_seen_ts = time.time()

    def mark_error(self, source_id: str, error_text: str) -> None:
        with self._lock:
            info = self._sources.get(source_id)
            if info is not None:
                info.last_error = error_text
                info.connected = False

    def process_line(self, source_id: str, raw_line: str) -> None:
        pipeline = self._pipelines.get(source_id)
        if pipeline is None:
            logger.warning("RTKSourceManager: unknown source %s, dropping line", source_id)
            return

        self.mark_seen(source_id)
        pipeline.process(raw_line)

    def snapshot(self) -> dict:
        with self._lock:
            active_source = self._active_source
            active_info = self._sources[active_source]
            sources = [info.to_dict() for info in self._sources.values()]

        frame = self._pipelines[active_source].snapshot()
        source_frames = self._collect_source_frames(sources)
        heading_info = self._compute_dual_rtk_heading()
        payload = frame.to_dict(
            server_ts=time.time(),
            default_lat=self._default_lat,
            default_lon=self._default_lon,
        )
        payload.update(
            {
                "rtk_source": active_source,
                "rtk_source_label": active_info.label,
                "rtk_sources": sources,
                "rtk_source_frames": source_frames,
                "rtk_active_source": active_source,
            }
        )
        payload.update(heading_info)
        return payload

    def _collect_source_frames(self, sources: list[dict]) -> list[dict]:
        """Return latest frame for each configured RTK source."""
        frames: list[dict] = []
        for source in sources:
            source_id = source["source_id"]
            pipeline = self._pipelines.get(source_id)
            frame = pipeline.snapshot() if pipeline is not None else RTKFrame()
            frames.append(
                {
                    "source_id": source_id,
                    "label": source.get("label", source_id),
                    "connected": bool(source.get("connected", False)),
                    "lat": frame.lat,
                    "lon": frame.lon,
                    "alt": frame.alt,
                    "fix_quality": frame.fix_quality,
                    "num_sats": frame.num_sats,
                    "hdop": frame.hdop,
                    "speed_knots": frame.speed_knots,
                    "track_deg": frame.track_deg,
                    "rtk_ts": frame.rtk_ts,
                    "has_fix": frame.lat is not None and frame.lon is not None,
                }
            )
        return frames

    def _compute_dual_rtk_heading(self) -> dict:
        """
        Compute heading from two RTK antennas.

        Heading is the bearing from source A to source B in ENU frame:
            heading = atan2(delta_east, delta_north) [deg]
        """
        pipeline_a = self._pipelines.get(self._heading_source_a)
        pipeline_b = self._pipelines.get(self._heading_source_b)
        frame_a = pipeline_a.snapshot() if pipeline_a is not None else None
        frame_b = pipeline_b.snapshot() if pipeline_b is not None else None
        if frame_a is None or frame_b is None:
            return {
                "heading_deg": None,
                "heading_dir": None,
                "heading_valid": False,
                "heading_mode": "dual_rtk",
                "heading_baseline_m": None,
                "heading_source_a": self._heading_source_a,
                "heading_source_b": self._heading_source_b,
                "heading_error": "missing_source",
            }

        if frame_a.lat is None or frame_a.lon is None or frame_b.lat is None or frame_b.lon is None:
            return {
                "heading_deg": None,
                "heading_dir": None,
                "heading_valid": False,
                "heading_mode": "dual_rtk",
                "heading_baseline_m": None,
                "heading_source_a": self._heading_source_a,
                "heading_source_b": self._heading_source_b,
                "heading_error": "missing_fix",
            }

        # Prefer RTK-grade fixes for heading stability.
        if frame_a.fix_quality < 4 or frame_b.fix_quality < 4:
            return {
                "heading_deg": None,
                "heading_dir": None,
                "heading_valid": False,
                "heading_mode": "dual_rtk",
                "heading_baseline_m": None,
                "heading_source_a": self._heading_source_a,
                "heading_source_b": self._heading_source_b,
                "heading_error": "insufficient_fix_quality",
            }

        d_north_m, d_east_m = self._delta_ne_m(
            frame_a.lat,
            frame_a.lon,
            frame_b.lat,
            frame_b.lon,
        )
        baseline_m = math.hypot(d_north_m, d_east_m)
        if baseline_m < self._heading_min_baseline_m:
            return {
                "heading_deg": None,
                "heading_dir": None,
                "heading_valid": False,
                "heading_mode": "dual_rtk",
                "heading_baseline_m": baseline_m,
                "heading_source_a": self._heading_source_a,
                "heading_source_b": self._heading_source_b,
                "heading_error": "baseline_too_short",
            }

        heading_raw = (math.degrees(math.atan2(d_east_m, d_north_m)) + 360.0) % 360.0
        heading_deg = (heading_raw + self._heading_offset_deg) % 360.0
        return {
            "heading_deg": round(heading_deg, 3),
            "heading_dir": self._heading_dir(heading_deg),
            "heading_valid": True,
            "heading_mode": "dual_rtk",
            "heading_baseline_m": round(baseline_m, 3),
            "heading_source_a": self._heading_source_a,
            "heading_source_b": self._heading_source_b,
            "heading_error": None,
        }

    @staticmethod
    def _delta_ne_m(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
        """Approximate local north/east deltas in metres between two WGS84 points."""
        earth_r = 6_371_000.0
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        d_lat = lat2_rad - lat1_rad
        d_lon = math.radians(lon2 - lon1)
        mean_lat = 0.5 * (lat1_rad + lat2_rad)
        d_north = d_lat * earth_r
        d_east = d_lon * earth_r * math.cos(mean_lat)
        return d_north, d_east

    @staticmethod
    def _heading_dir(heading_deg: float) -> str:
        labels = [
            "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
        ]
        idx = int((heading_deg + 11.25) // 22.5) % 16
        return labels[idx]


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 2 · PIPELINE
# ══════════════════════════════════════════════════════════════════════════════


class NMEAPipeline:
    """
    Stateful accumulator pipeline for NMEA sentences.

    Each call to process() feeds one raw line through the pipeline and updates
    the internal RTKFrame.  Callers retrieve the current state via snapshot().

    Pipeline stages:
        raw NMEA line
          → _verify_checksum()   → validated line | None (invalid lines dropped)
          → _dispatch()          → routes by sentence type (GGA / RMC / ignore)
          → _parse_gga()         → updates self._frame (lat/lon/alt/fix/sats/hdop/rtk_ts)
          → _parse_rmc()         → updates self._frame (speed_knots/track_deg)

    Thread safety:
        process() is called from the SerialReader thread.
        snapshot() is called from the BroadcastLoop coroutine (event loop thread).
        A threading.Lock protects all accesses to self._frame.
    """

    def __init__(self) -> None:
        self._frame = RTKFrame()
        self._lock  = threading.Lock()

    # ── Public API ─────────────────────────────────────────────────────────────

    def process(self, raw_line: str) -> None:
        """
        CORE — Feed one raw NMEA line through the pipeline; update internal state.

        Receives validated text from SerialReader (INPUT) and updates self._frame,
        which BroadcastLoop will snapshot and serialize toward WebSocketServer (OUTPUT).
        """
        validated = self._verify_checksum(raw_line)
        if validated is not None:
            self._dispatch(validated)

    def snapshot(self) -> RTKFrame:
        """Return a thread-safe deep copy of the current accumulated frame."""
        with self._lock:
            return copy.copy(self._frame)

    # ── Stage 1: checksum verification ─────────────────────────────────────────

    def _verify_checksum(self, line: str) -> Optional[str]:
        """
        Validate NMEA XOR checksum.

        Returns the original line if valid, None otherwise.
        Checksum = XOR of all bytes between '$' and '*' (exclusive).
        """
        if not line.startswith("$"):
            return None
        try:
            star_idx = line.rindex("*")
        except ValueError:
            return None   # no checksum field

        payload      = line[1:star_idx]
        expected_hex = line[star_idx + 1 : star_idx + 3]
        if len(expected_hex) != 2:
            return None

        try:
            expected = int(expected_hex, 16)
        except ValueError:
            return None

        computed = 0
        for ch in payload:
            computed ^= ord(ch)

        if computed != expected:
            logger.warning("NMEAPipeline: checksum mismatch, skipping: %r", line)
            return None
        return line

    # ── Stage 2: dispatch ──────────────────────────────────────────────────────

    def _dispatch(self, line: str) -> None:
        """
        Route a validated NMEA sentence to the appropriate parser.

        Strips the talker prefix (GP/GN/GL/GA…) before comparing sentence type,
        so both $GPGGA and $GNGGA match as "GGA".
        """
        body   = line[1:].split("*")[0]
        fields = body.split(",")
        if not fields or len(fields[0]) < 3:
            return

        sentence_type = fields[0][2:]   # strip 2-char talker prefix

        if sentence_type == "GGA":
            self._parse_gga(fields)
        elif sentence_type == "RMC":
            self._parse_rmc(fields)

    # ── Stage 3a: GGA parser ───────────────────────────────────────────────────

    def _parse_gga(self, fields: list) -> None:
        """
        Parse a GGA sentence and update position + quality fields.

        GGA field indices:
          [1] UTC time
          [2] Latitude  DDMM.MMMMM
          [3] N/S indicator
          [4] Longitude DDDMM.MMMMM
          [5] E/W indicator
          [6] Fix quality  (0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float)
          [7] Number of satellites in use
          [8] HDOP
          [9] Altitude MSL (metres)
        """
        try:
            if len(fields) < 11:
                logger.warning("NMEAPipeline: GGA sentence too short: %s", fields)
                return

            fix_quality = int(fields[6])   if fields[6] else 0
            num_sats    = int(fields[7])   if fields[7] else 0
            hdop        = float(fields[8]) if fields[8] else None
            alt         = float(fields[9]) if fields[9] else None
            lat = self._nmea_to_decimal(fields[2], fields[3]) \
                  if (fields[2] and fields[3]) else None
            lon = self._nmea_to_decimal(fields[4], fields[5]) \
                  if (fields[4] and fields[5]) else None

            with self._lock:
                self._frame.lat         = lat
                self._frame.lon         = lon
                self._frame.alt         = alt
                self._frame.fix_quality = fix_quality
                self._frame.num_sats    = num_sats
                self._frame.hdop        = hdop
                self._frame.rtk_ts      = time.time()

        except (ValueError, IndexError) as e:
            logger.warning("NMEAPipeline: failed to parse GGA: %s — fields=%s", e, fields)

    # ── Stage 3b: RMC parser ───────────────────────────────────────────────────

    def _parse_rmc(self, fields: list) -> None:
        """
        Parse a RMC sentence and update speed / track fields.

        RMC field indices:
          [2] Status: 'A'=active (valid), 'V'=void (skip)
          [7] Speed over ground [knots]
          [8] True course over ground [degrees]

        Only processed when status == 'A'.
        """
        try:
            if len(fields) < 9:
                return
            if fields[2] != "A":
                return   # void fix — data not reliable

            speed_knots = float(fields[7]) if fields[7] else None
            track_deg   = float(fields[8]) if fields[8] else None

            with self._lock:
                self._frame.speed_knots = speed_knots
                self._frame.track_deg   = track_deg

        except (ValueError, IndexError) as e:
            logger.warning("NMEAPipeline: failed to parse RMC: %s — fields=%s", e, fields)

    # ── Helper ─────────────────────────────────────────────────────────────────

    @staticmethod
    def _nmea_to_decimal(raw: str, direction: str) -> Optional[float]:
        """
        Convert NMEA coordinate string to decimal degrees.

        NMEA format: DDMM.MMMMM (lat) or DDDMM.MMMMM (lon).
        Algorithm: degrees = integer prefix before last 2 pre-dot digits;
                   minutes = last 2 pre-dot digits + decimal fraction.
        direction 'S' or 'W' negates the result.
        """
        try:
            dot_idx = raw.index(".")
            deg_str = raw[: dot_idx - 2]
            min_str = raw[dot_idx - 2 :]
            degrees = float(deg_str) + float(min_str) / 60.0
            if direction in ("S", "W"):
                degrees = -degrees
            return degrees
        except (ValueError, IndexError) as e:
            logger.warning("NMEAPipeline: _nmea_to_decimal failed for %r %r: %s", raw, direction, e)
            return None


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 3 · I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════════


class SerialReader:
    """
    Daemon thread: read NMEA lines from one GPS receiver and feed its pipeline.

    Auto-reconnects every 3 s on serial failure so the bridge survives
    cable disconnects or device resets.
    """

    RECONNECT_DELAY = 3.0   # seconds between reconnect attempts

    def __init__(
        self,
        source_id: str,
        port: str,
        baud: int,
        manager: RTKSourceManager,
        timeout: float = 1.0,
    ) -> None:
        self._source_id = source_id
        self._port     = port
        self._baud     = baud
        self._manager  = manager
        self._timeout  = timeout
        self._thread   = threading.Thread(
            target=self._run,
            name=f"rtk-serial-reader-{source_id}",
            daemon=True,
        )

    def start(self) -> None:
        """Start the daemon reader thread."""
        self._thread.start()

    def _run(self) -> None:
        """Thread body: open port → read loop → reconnect on error."""
        while True:
            try:
                ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
            except serial.SerialException as e:
                self._manager.mark_error(self._source_id, str(e))
                logger.error("SerialReader: failed to open %s: %s — retrying in %.0fs",
                             self._port, e, self.RECONNECT_DELAY)
                time.sleep(self.RECONNECT_DELAY)
                continue

            logger.info("SerialReader: port opened: %s @ %d baud", self._port, self._baud)
            self._manager.mark_connected(self._source_id, True)
            try:
                while True:
                    try:
                        # ── INPUT ──────────────────────────────────────────────────────────
                        # Raw NMEA bytes arrive here from the GPS receiver.
                        # If data looks wrong, start debugging from this line.
                        raw = ser.readline()
                        # ───────────────────────────────────────────────────────────────────
                        if not raw:
                            continue
                        line = raw.decode("ascii", errors="ignore").strip()
                        if line:
                            self._manager.process_line(self._source_id, line)
                    except serial.SerialException as e:
                        self._manager.mark_error(self._source_id, str(e))
                        logger.error("SerialReader: read error: %s — reconnecting", e)
                        break
                    except Exception as e:
                        self._manager.mark_error(self._source_id, str(e))
                        logger.error("SerialReader: unexpected error: %s", e)
            finally:
                try:
                    ser.close()
                except Exception as e:
                    logger.warning("SerialReader: error closing port: %s", e)
                self._manager.mark_connected(self._source_id, False)
                logger.info("SerialReader: port closed, reconnecting in %.0fs",
                            self.RECONNECT_DELAY)
                time.sleep(self.RECONNECT_DELAY)


class BroadcastLoop:
    """
    Asyncio coroutine: poll NMEAPipeline.snapshot() at N Hz and push serialized
    frames into an asyncio.Queue for WebSocketServer to consume.

    Decouples the snapshot/serialize step from the per-client send step.
    """

    def __init__(
        self,
        manager: RTKSourceManager,
        queue: asyncio.Queue,
        hz: float,
    ) -> None:
        self._manager     = manager
        self._queue       = queue
        self._period      = 1.0 / max(hz, 0.5)

    async def run(self) -> None:
        """Coroutine entry: sample pipeline and enqueue JSON messages."""
        while True:
            msg = json.dumps(self._manager.snapshot())
            # Non-blocking put; drop frame if queue is full to avoid backlog
            try:
                self._queue.put_nowait(msg)   # → WebSocketServer
            except asyncio.QueueFull:
                logger.debug("BroadcastLoop: queue full, dropping frame")
            await asyncio.sleep(self._period)


class WebSocketServer:
    """
    Manage browser WebSocket connections and broadcast queued messages.

    All connected clients receive every message dequeued from the shared queue.
    Dead connections are silently pruned.
    """

    def __init__(self, port: int, queue: asyncio.Queue, manager: RTKSourceManager) -> None:
        self._port    = port
        self._queue   = queue
        self._manager = manager
        self._clients: set = set()
        self._lock    = asyncio.Lock()

    async def handle_client(self, websocket) -> None:
        """Accept and track a new browser connection until it closes."""
        addr = websocket.remote_address
        logger.info("WebSocket client connected: %s", addr)
        async with self._lock:
            self._clients.add(websocket)
        try:
            async for message in websocket:
                await self._handle_message(message)
        finally:
            async with self._lock:
                self._clients.discard(websocket)
            logger.info("WebSocket client disconnected: %s", addr)

    async def _handle_message(self, message: str) -> None:
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            logger.warning("WebSocketServer: ignored non-JSON control message: %r", message)
            return

        if not isinstance(payload, dict):
            return

        if payload.get("type") == "set_active_source":
            source_id = str(payload.get("source", "")).strip()
            if not source_id:
                return
            try:
                self._manager.set_active_source(source_id)
            except KeyError:
                logger.warning("WebSocketServer: unknown RTK source requested: %s", source_id)

    async def broadcast(self) -> None:
        """Dequeue messages and forward to all connected clients."""
        while True:
            msg  = await self._queue.get()
            async with self._lock:
                clients = set(self._clients)

            if not clients:
                continue

            dead = set()
            for ws in clients:
                try:
                    # ── OUTPUT ─────────────────────────────────────────────────────────
                    # Processed JSON frame exits the program here to the browser.
                    # If the browser receives wrong data, start debugging here.
                    await ws.send(msg)
                    # ───────────────────────────────────────────────────────────────────
                except Exception:
                    dead.add(ws)

            if dead:
                async with self._lock:
                    self._clients.difference_update(dead)

    async def serve(self) -> None:
        """Start the WebSocket server and run broadcast + accept loops forever."""
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("WebSocket server listening on ws://localhost:%d", self._port)
            await self.broadcast()


class HttpFileServer:
    """
    Serve web_static/ over HTTP in a daemon thread.

    Uses Python's built-in SimpleHTTPRequestHandler; HTTP access logs are
    forwarded to logger.debug to avoid cluttering INFO output.
    """

    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port       = port
        self._thread     = threading.Thread(
            target=self._run,
            name="rtk-http-server",
            daemon=True,
        )

    def start(self) -> None:
        """Start the daemon HTTP server thread."""
        self._thread.start()

    def _run(self) -> None:
        """Thread body: create TCPServer and serve_forever."""
        static_dir = self._static_dir   # captured for closure below

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def log_message(self, format, *args):
                logger.debug("HTTP %s - %s", self.address_string(), format % args)

        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self._port), _Handler) as httpd:
                logger.info("HTTP server serving %s on port %d", self._static_dir, self._port)
                httpd.serve_forever()
        except Exception as e:
            logger.error("HttpFileServer: failed to start on port %d: %s", self._port, e)


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 4 · APPLICATION
# ══════════════════════════════════════════════════════════════════════════════


class RTKBridge:
    """
    Top-level orchestrator.  Assembles and starts all components:
      1. NMEAPipeline      — stateful NMEA accumulator
      2. SerialReader      — daemon thread feeding the pipeline
      3. HttpFileServer    — daemon thread serving web_static/
      4. BroadcastLoop     — asyncio coroutine polling pipeline → queue
      5. WebSocketServer   — asyncio server delivering queue to browsers
    """

    def __init__(
        self,
        serial_ports: list[str],
        baud: int,
        ws_port: int,
        hz: float,
        static_dir: Path,
        default_lat: float,
        default_lon: float,
        open_browser: bool,
        heading_source_a: str,
        heading_source_b: str,
        heading_offset_deg: float,
        heading_min_baseline_m: float,
    ) -> None:
        self._serial_ports = serial_ports
        self._baud         = baud
        self._http_port    = ws_port
        self._ws_port      = ws_port + 1
        self._hz           = hz
        self._static_dir   = static_dir
        self._default_lat  = default_lat
        self._default_lon  = default_lon
        self._open_browser = open_browser
        self._heading_source_a = heading_source_a
        self._heading_source_b = heading_source_b
        self._heading_offset_deg = heading_offset_deg
        self._heading_min_baseline_m = heading_min_baseline_m

    def run(self) -> None:
        """Synchronous entry point: start daemon threads then hand off to asyncio."""
        # ── 1. Source manager and per-source NMEA pipelines ───────────────────
        sources = [
            (f"rtk{index + 1}", port)
            for index, port in enumerate(self._serial_ports)
        ]
        manager = RTKSourceManager(
            sources,
            self._default_lat,
            self._default_lon,
            heading_source_a=self._heading_source_a,
            heading_source_b=self._heading_source_b,
            heading_offset_deg=self._heading_offset_deg,
            heading_min_baseline_m=self._heading_min_baseline_m,
        )

        # ── 2. Serial reader daemon thread(s) ──────────────────────────────────
        readers = []
        for source_id, port in sources:
            reader = SerialReader(source_id, port, self._baud, manager)
            reader.start()
            readers.append(reader)

        # ── 3. HTTP file server daemon thread ──────────────────────────────────
        if not self._static_dir.exists():
            logger.warning("web_static directory not found at %s", self._static_dir)
        else:
            HttpFileServer(self._static_dir, self._http_port).start()

        logger.info(
            "RTK bridge | sources=%s | heading=%s->%s (offset=%.2f°, min_baseline=%.2fm) | HTTP=:%d  WebSocket=:%d  broadcast=%.1f Hz",
            ", ".join(self._serial_ports),
            self._heading_source_a,
            self._heading_source_b,
            self._heading_offset_deg,
            self._heading_min_baseline_m,
            self._http_port,
            self._ws_port,
            self._hz,
        )
        logger.info("Open browser at http://localhost:%d", self._http_port)

        if self._open_browser:
            url = f"http://localhost:{self._http_port}"
            threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        # ── 4 + 5. Asyncio: broadcast loop + WS server ─────────────────────────
        try:
            asyncio.run(self._run_async(manager))
        except KeyboardInterrupt:
            logger.info("RTKBridge: stopped by user.")

    async def _run_async(self, manager: RTKSourceManager) -> None:
        """Asyncio coroutines: BroadcastLoop feeds the queue; WebSocketServer drains it."""
        queue      = asyncio.Queue(maxsize=10)
        ws_server  = WebSocketServer(self._ws_port, queue, manager)
        broadcast  = BroadcastLoop(manager, queue, self._hz)

        await asyncio.gather(
            broadcast.run(),
            ws_server.serve(),
        )


# ── Entry Point ────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RTK serial-to-WebSocket bridge")
    parser.add_argument(
        "--port",
        nargs="+",
        default=_default_rtk_ports(),
        help="Serial port(s) for RTK receiver(s); pass one or two values",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=_cfg.RTK_BAUD if _cfg else 9600,
        help="Serial baud rate",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=_cfg.RTK_WS_PORT if _cfg else 8775,
        help="HTTP port for web UI; WebSocket uses ws-port+1",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=_cfg.RTK_HZ if _cfg else 5.0,
        help="Broadcast rate in Hz",
    )
    parser.add_argument(
        "--open-browser",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Auto-open browser after startup (default: true)",
    )
    parser.add_argument(
        "--heading-source-a",
        default=_cfg.RTK_HEADING_SOURCE_A if _cfg and hasattr(_cfg, "RTK_HEADING_SOURCE_A") else "rtk1",
        help="Dual-RTK heading baseline start source id (default: rtk1)",
    )
    parser.add_argument(
        "--heading-source-b",
        default=_cfg.RTK_HEADING_SOURCE_B if _cfg and hasattr(_cfg, "RTK_HEADING_SOURCE_B") else "rtk2",
        help="Dual-RTK heading baseline end source id (default: rtk2)",
    )
    parser.add_argument(
        "--heading-offset-deg",
        type=float,
        default=_cfg.RTK_HEADING_OFFSET_DEG if _cfg and hasattr(_cfg, "RTK_HEADING_OFFSET_DEG") else 0.0,
        help="Heading calibration offset in degrees after dual-RTK computation",
    )
    parser.add_argument(
        "--heading-min-baseline-m",
        type=float,
        default=_cfg.RTK_HEADING_MIN_BASELINE_M if _cfg and hasattr(_cfg, "RTK_HEADING_MIN_BASELINE_M") else 0.3,
        help="Minimum baseline length in metres required to accept dual-RTK heading",
    )
    return parser.parse_args()


def _default_rtk_ports() -> list[str]:
    if _cfg is not None and hasattr(_cfg, "RTK_SERIAL_PORTS"):
        return list(getattr(_cfg, "RTK_SERIAL_PORTS"))
    if _cfg is not None and hasattr(_cfg, "RTK_SERIAL_PORT"):
        return [getattr(_cfg, "RTK_SERIAL_PORT")]
    return ["/dev/cu.usbmodem11203"]


if __name__ == "__main__":
    args = _parse_args()
    RTKBridge(
        serial_ports=args.port,
        baud=args.baud,
        ws_port=args.ws_port,
        hz=args.hz,
        static_dir=Path(__file__).parent / "web_static",
        default_lat=DEFAULT_LAT,
        default_lon=DEFAULT_LON,
        open_browser=args.open_browser,
        heading_source_a=args.heading_source_a,
        heading_source_b=args.heading_source_b,
        heading_offset_deg=args.heading_offset_deg,
        heading_min_baseline_m=args.heading_min_baseline_m,
    ).run()
