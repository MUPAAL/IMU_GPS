"""
rtk_bridge.py — RTK serial reader to WebSocket bridge + static web map UI.

Data flow:
    Serial Port → SerialReader → NMEAPipeline (accumulates GGA + RMC state)
                                        ↓  snapshot() at N Hz
                                 BroadcastLoop → WebSocketServer → Browser

NMEAPipeline stages:
    raw NMEA line
      → _verify_checksum()   → validated line | None
      → _dispatch()          → routes by sentence type
      → _parse_gga()         → updates internal RTKFrame (lat/lon/alt/fix/sats/hdop)
      → _parse_rmc()         → updates internal RTKFrame (speed/track)

Runs:
  - SerialReader daemon thread  (NMEA parser, auto-reconnect on failure)
  - HttpFileServer daemon thread (serves web_static/)
  - BroadcastLoop asyncio coroutine (polls pipeline, pushes to WS clients)
  - WebSocketServer asyncio server

Port convention (same as 01_IMU):
  HTTP  = --ws-port        (default 8775)
  WebSocket = --ws-port+1  (default 8776)
"""

# ── IMPORTS ────────────────────────────────────────────────────────────────────

import argparse
import asyncio
import copy
import json
import logging
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
    Daemon thread: read NMEA lines from the GPS receiver and feed into NMEAPipeline.

    Auto-reconnects every 3 s on serial failure so the bridge survives
    cable disconnects or device resets.
    """

    RECONNECT_DELAY = 3.0   # seconds between reconnect attempts

    def __init__(
        self,
        port: str,
        baud: int,
        pipeline: NMEAPipeline,
        timeout: float = 1.0,
    ) -> None:
        self._port     = port
        self._baud     = baud
        self._pipeline = pipeline
        self._timeout  = timeout
        self._thread   = threading.Thread(
            target=self._run,
            name="rtk-serial-reader",
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
                logger.error("SerialReader: failed to open %s: %s — retrying in %.0fs",
                             self._port, e, self.RECONNECT_DELAY)
                time.sleep(self.RECONNECT_DELAY)
                continue

            logger.info("SerialReader: port opened: %s @ %d baud", self._port, self._baud)
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
                            self._pipeline.process(line)
                    except serial.SerialException as e:
                        logger.error("SerialReader: read error: %s — reconnecting", e)
                        break
                    except Exception as e:
                        logger.error("SerialReader: unexpected error: %s", e)
            finally:
                try:
                    ser.close()
                except Exception as e:
                    logger.warning("SerialReader: error closing port: %s", e)
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
        pipeline: NMEAPipeline,
        queue: asyncio.Queue,
        hz: float,
        default_lat: float,
        default_lon: float,
    ) -> None:
        self._pipeline    = pipeline
        self._queue       = queue
        self._period      = 1.0 / max(hz, 0.5)
        self._default_lat = default_lat
        self._default_lon = default_lon

    async def run(self) -> None:
        """Coroutine entry: sample pipeline and enqueue JSON messages."""
        while True:
            frame = self._pipeline.snapshot()
            msg   = json.dumps(
                frame.to_dict(
                    server_ts=time.time(),
                    default_lat=self._default_lat,
                    default_lon=self._default_lon,
                )
            )
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

    def __init__(self, port: int, queue: asyncio.Queue) -> None:
        self._port    = port
        self._queue   = queue
        self._clients: set = set()
        self._lock    = asyncio.Lock()

    async def handle_client(self, websocket) -> None:
        """Accept and track a new browser connection until it closes."""
        addr = websocket.remote_address
        logger.info("WebSocket client connected: %s", addr)
        async with self._lock:
            self._clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            async with self._lock:
                self._clients.discard(websocket)
            logger.info("WebSocket client disconnected: %s", addr)

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

            def log_message(self, fmt, *args):
                logger.debug("HTTP %s - %s", self.address_string(), fmt % args)

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
        serial_port: str,
        baud: int,
        ws_port: int,
        hz: float,
        static_dir: Path,
        default_lat: float,
        default_lon: float,
        open_browser: bool,
    ) -> None:
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
        """Synchronous entry point: start daemon threads then hand off to asyncio."""
        # ── 1. NMEA pipeline (shared state, no thread of its own) ──────────────
        pipeline = NMEAPipeline()

        # ── 2. Serial reader daemon thread ─────────────────────────────────────
        serial_reader = SerialReader(self._serial_port, self._baud, pipeline)
        serial_reader.start()

        # ── 3. HTTP file server daemon thread ──────────────────────────────────
        if not self._static_dir.exists():
            logger.warning("web_static directory not found at %s", self._static_dir)
        else:
            HttpFileServer(self._static_dir, self._http_port).start()

        logger.info(
            "RTK bridge | HTTP=:%d  WebSocket=:%d  broadcast=%.1f Hz",
            self._http_port, self._ws_port, self._hz,
        )
        logger.info("Open browser at http://localhost:%d", self._http_port)

        if self._open_browser:
            url = f"http://localhost:{self._http_port}"
            threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        # ── 4 + 5. Asyncio: broadcast loop + WS server ─────────────────────────
        try:
            asyncio.run(self._run_async(pipeline))
        except KeyboardInterrupt:
            logger.info("RTKBridge: stopped by user.")

    async def _run_async(self, pipeline: NMEAPipeline) -> None:
        """Asyncio coroutines: BroadcastLoop feeds the queue; WebSocketServer drains it."""
        queue      = asyncio.Queue(maxsize=10)
        ws_server  = WebSocketServer(self._ws_port, queue)
        broadcast  = BroadcastLoop(
            pipeline, queue, self._hz, self._default_lat, self._default_lon
        )

        await asyncio.gather(
            broadcast.run(),
            ws_server.serve(),
        )


# ── Entry Point ────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RTK serial-to-WebSocket bridge")
    parser.add_argument(
        "--port",
        default="/dev/cu.usbmodem1203",
        help="Serial port for RTK receiver (default: /dev/ttyACM1)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=9600,
        help="Serial baud rate (default: 9600)",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8775,
        help="HTTP port for web UI; WebSocket uses ws-port+1 (default: 8775)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="Broadcast rate in Hz (default: 5)",
    )
    parser.add_argument(
        "--open-browser",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Auto-open browser after startup (default: true)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    RTKBridge(
        serial_port=args.port,
        baud=args.baud,
        ws_port=args.ws_port,
        hz=args.hz,
        static_dir=Path(__file__).parent / "web_static",
        default_lat=DEFAULT_LAT,
        default_lon=DEFAULT_LON,
        open_browser=args.open_browser,
    ).run()
