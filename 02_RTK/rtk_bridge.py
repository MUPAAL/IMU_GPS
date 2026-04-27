"""
rtk_bridge.py — RTK serial reader to WebSocket bridge + static web map UI.

Data flow:
    Serial Port → SerialReader → NMEAPipeline (accumulate GGA + RMC state)
                                        ↓  snapshot() at N Hz
                                 BroadcastLoop → WebSocketServer → Browser

NMEAPipeline stages:
    raw NMEA line
      → _verify_checksum()   → validated line | None
      → _dispatch()          → routes by sentence type
      → _parse_gga()         → updates internal RTKFrame (lat/lon/alt/fix/sats/hdop)
      → _parse_rmc()         → updates internal RTKFrame (speed/track)

Port convention:
    HTTP      = ws_port      (default 8775)
    WebSocket = ws_port + 1  (default 8776)
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

import asyncio
import copy
import json
import logging
import serial
import socketserver
import threading
import time
import webbrowser
from dataclasses import dataclass
from http.server import SimpleHTTPRequestHandler
from pathlib import Path
from typing import Optional

import websockets

WS_MSG_TYPE_RTK_FRAME = "rtk_frame"
WS_MSG_VERSION = 1
SCRIPT_SERIAL_PORT = None
SCRIPT_BAUD = None
SCRIPT_WS_PORT = None
SCRIPT_HZ = None
SCRIPT_OPEN_BROWSER = None

# ── LOGGING ────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

DEFAULT_LAT: float = 38.9412928598587
DEFAULT_LON: float = -92.31884600793728

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 1 · DATA MODEL
# ══════════════════════════════════════════════════════════════════════════════


@dataclass
class RTKFrame:
    """
    Typed data carrier for one RTK position snapshot.

    Fields accumulated across NMEA sentences:
      - GGA: lat, lon, alt, fix_quality, num_sats, hdop, rtk_ts
      - RMC: speed_knots, track_deg
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

    def to_dict(self, *, server_ts: float, default_lat: float, default_lon: float) -> dict:
        use_default = self.lat is None or self.lon is None
        return {
            "type":        WS_MSG_TYPE_RTK_FRAME,
            "version":     WS_MSG_VERSION,
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


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 2 · PIPELINE
# ══════════════════════════════════════════════════════════════════════════════


class NMEAPipeline:
    """Stateful accumulator for NMEA sentences. Thread-safe via internal lock."""

    def __init__(self) -> None:
        self._frame = RTKFrame()
        self._lock  = threading.Lock()

    def process(self, raw_line: str) -> None:
        validated = self._verify_checksum(raw_line)
        if validated is not None:
            self._dispatch(validated)

    def snapshot(self) -> RTKFrame:
        with self._lock:
            return copy.copy(self._frame)

    def _verify_checksum(self, line: str) -> Optional[str]:
        if not line.startswith("$"):
            return None
        try:
            star_idx = line.rindex("*")
        except ValueError:
            return None
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

    def _dispatch(self, line: str) -> None:
        body   = line[1:].split("*")[0]
        fields = body.split(",")
        if not fields or len(fields[0]) < 3:
            return
        sentence_type = fields[0][2:]
        if sentence_type == "GGA":
            self._parse_gga(fields)
        elif sentence_type == "RMC":
            self._parse_rmc(fields)

    def _parse_gga(self, fields: list) -> None:
        try:
            if len(fields) < 11:
                logger.warning("NMEAPipeline: GGA sentence too short: %s", fields)
                return
            fix_quality = int(fields[6])   if fields[6] else 0
            num_sats    = int(fields[7])   if fields[7] else 0
            hdop        = float(fields[8]) if fields[8] else None
            alt         = float(fields[9]) if fields[9] else None
            lat = self._nmea_to_decimal(fields[2], fields[3]) if (fields[2] and fields[3]) else None
            lon = self._nmea_to_decimal(fields[4], fields[5]) if (fields[4] and fields[5]) else None
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

    def _parse_rmc(self, fields: list) -> None:
        try:
            if len(fields) < 9 or fields[2] != "A":
                return
            speed_knots = float(fields[7]) if fields[7] else None
            track_deg   = float(fields[8]) if fields[8] else None
            with self._lock:
                self._frame.speed_knots = speed_knots
                self._frame.track_deg   = track_deg
        except (ValueError, IndexError) as e:
            logger.warning("NMEAPipeline: failed to parse RMC: %s — fields=%s", e, fields)

    @staticmethod
    def _nmea_to_decimal(raw: str, direction: str) -> Optional[float]:
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
    """Daemon thread: read NMEA lines from the GPS receiver and feed the pipeline."""

    RECONNECT_DELAY = 3.0

    def __init__(self, port: str, baud: int, pipeline: NMEAPipeline, timeout: float = 1.0) -> None:
        self._port     = port
        self._baud     = baud
        self._pipeline = pipeline
        self._timeout  = timeout
        self._thread   = threading.Thread(target=self._run, name="rtk-serial-reader", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while True:
            try:
                ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
            except serial.SerialException as e:
                logger.error("SerialReader: failed to open %s: %s — retrying in %.0fs",
                             self._port, e, self.RECONNECT_DELAY)
                time.sleep(self.RECONNECT_DELAY)
                continue
            logger.info("SerialReader: port opened: %s @ %d baud", self._port, self._baud)
            try:
                while True:
                    try:
                        # ── INPUT ──────────────────────────────────────────────
                        raw = ser.readline()
                        # ───────────────────────────────────────────────────────
                        if not raw:
                            continue
                        line = raw.decode("ascii", errors="ignore").strip()
                        if line:
                            self._pipeline.process(line)
                    except serial.SerialException as e:
                        logger.error("SerialReader: read error: %s — reconnecting", e)
                        break
                    except Exception as e:
                        logger.error("SerialReader: unexpected error: %s", e)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(self.RECONNECT_DELAY)


class BroadcastLoop:
    """Asyncio coroutine: poll NMEAPipeline.snapshot() at N Hz and push to queue."""

    def __init__(self, pipeline: NMEAPipeline, queue: asyncio.Queue, hz: float,
                 default_lat: float, default_lon: float) -> None:
        self._pipeline    = pipeline
        self._queue       = queue
        self._period      = 1.0 / max(hz, 0.5)
        self._default_lat = default_lat
        self._default_lon = default_lon

    async def run(self) -> None:
        while True:
            frame = self._pipeline.snapshot()
            msg = json.dumps(frame.to_dict(
                server_ts=time.time(),
                default_lat=self._default_lat,
                default_lon=self._default_lon,
            ))
            try:
                self._queue.put_nowait(msg)
            except asyncio.QueueFull:
                logger.warning("BroadcastLoop: queue full, dropping frame")
            await asyncio.sleep(self._period)


class WebSocketServer:
    """Manage browser WebSocket connections and broadcast queued messages."""

    def __init__(self, port: int, queue: asyncio.Queue) -> None:
        self._port    = port
        self._queue   = queue
        self._clients: set = set()
        self._lock    = asyncio.Lock()

    async def handle_client(self, websocket) -> None:
        async with self._lock:
            self._clients.add(websocket)
        try:
            async for _ in websocket:
                pass
        finally:
            async with self._lock:
                self._clients.discard(websocket)

    async def broadcast(self) -> None:
        while True:
            msg = await self._queue.get()
            async with self._lock:
                clients = set(self._clients)
            if not clients:
                continue
            dead = set()
            for ws in clients:
                try:
                    # ── OUTPUT ─────────────────────────────────────────────────
                    await ws.send(msg)
                    # ───────────────────────────────────────────────────────────
                except Exception:
                    dead.add(ws)
            if dead:
                async with self._lock:
                    self._clients.difference_update(dead)

    async def serve(self) -> None:
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            await self.broadcast()


class HttpFileServer:
    """Serve web_static/ over HTTP in a daemon thread."""

    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port       = port
        self._thread     = threading.Thread(target=self._run, name="rtk-http-server", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        static_dir = self._static_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def log_message(self, format, *args):
                pass

        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self._port), _Handler) as httpd:
                httpd.serve_forever()
        except Exception as e:
            logger.error("HttpFileServer: failed to start on port %d: %s", self._port, e)


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 4 · APPLICATION
# ══════════════════════════════════════════════════════════════════════════════


class RTKBridge:
    """Top-level orchestrator."""

    def __init__(self, serial_port: str, baud: int, ws_port: int, hz: float,
                 static_dir: Path, default_lat: float, default_lon: float, open_browser: bool) -> None:
        self._serial_port  = serial_port
        self._baud         = baud
        self._http_port    = ws_port
        self._ws_port      = ws_port + 1
        self._hz           = hz
        self._static_dir   = static_dir
        self._default_lat  = default_lat
        self._default_lon  = default_lon
        self._open_browser = open_browser

    def run(self) -> None:
        pipeline = NMEAPipeline()
        SerialReader(self._serial_port, self._baud, pipeline).start()

        if not self._static_dir.exists():
            logger.warning("web_static directory not found at %s", self._static_dir)
        else:
            HttpFileServer(self._static_dir, self._http_port).start()

        if self._open_browser:
            url = f"http://localhost:{self._http_port}"
            threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        try:
            asyncio.run(self._run_async(pipeline))
        except KeyboardInterrupt:
            logger.warning("RTKBridge: interrupted, shutting down")

    async def _run_async(self, pipeline: NMEAPipeline) -> None:
        queue     = asyncio.Queue(maxsize=10)
        ws_server = WebSocketServer(self._ws_port, queue)
        broadcast = BroadcastLoop(pipeline, queue, self._hz, self._default_lat, self._default_lon)
        await asyncio.gather(broadcast.run(), ws_server.serve())


if __name__ == "__main__":
    serial_port  = SCRIPT_SERIAL_PORT  if SCRIPT_SERIAL_PORT  is not None else (_cfg.RTK_SERIAL_PORT if _cfg else "/dev/cu.usbmodem1103")
    baud         = SCRIPT_BAUD         if SCRIPT_BAUD         is not None else (_cfg.RTK_BAUD        if _cfg else 9600)
    ws_port      = SCRIPT_WS_PORT      if SCRIPT_WS_PORT      is not None else (_cfg.RTK_WS_PORT     if _cfg else 8775)
    hz           = SCRIPT_HZ           if SCRIPT_HZ           is not None else (_cfg.RTK_HZ          if _cfg else 5.0)
    open_browser = SCRIPT_OPEN_BROWSER if SCRIPT_OPEN_BROWSER is not None else True

    RTKBridge(
        serial_port=serial_port,
        baud=baud,
        ws_port=ws_port,
        hz=hz,
        static_dir=Path(__file__).parent / "web_static",
        default_lat=DEFAULT_LAT,
        default_lon=DEFAULT_LON,
        open_browser=open_browser,
    ).run()
