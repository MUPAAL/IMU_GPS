"""
rtk_bridge.py - RTK serial reader to WebSocket bridge + static web map UI.

Runs:
  - RTKReader serial thread (NMEA parser in rtk_reader.py)
  - HTTP server for web_static/
  - WebSocket broadcast of latest RTK snapshot

If RTK has no valid lat/lon, it falls back to DEFAULT_LAT/LON.
"""

# **************** IMPORTS ****************

import argparse
import asyncio
import json
import logging
import socketserver
import threading
import time
import webbrowser
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import websockets

from rtk_reader import RTKReader

# **************** LOGGING SETUP ****************

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

# **************** DEFAULT POSITION (fallback when no GPS fix) ****************
#
# Used by normalize_frame() when RTKReader has not yet produced a valid lat/lon.
# Set to the known field-test site so the map opens in the right place.

DEFAULT_LAT = 38.9412928598587
DEFAULT_LON = -92.31884600793728

# **************** GLOBAL STATE ****************

connected_clients: set = set()   # all live WebSocket connections from browsers


# **************** FRAME NORMALIZER ****************

def normalize_frame(raw: dict) -> dict:
    """
    Convert the raw RTKReader snapshot into a broadcast-ready dict.

    If lat/lon are None (no fix or serial unavailable), substitutes DEFAULT
    position values and marks source="default" so the frontend can indicate
    that the displayed position is a placeholder.

    Input (from RTKReader.get_data()):
      lat, lon, alt, fix_quality, num_sats, hdop,
      speed_knots, track_deg, ts, raw_gga

    Output (sent to all WebSocket clients):
      lat, lon, alt           — position (DEFAULT_LAT/LON if no fix)
      fix_quality             — 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
      num_sats, hdop          — quality indicators
      speed_knots, track_deg  — motion (may be None)
      rtk_ts                  — timestamp from RTKReader (0.0 if never updated)
      server_ts               — time.time() at moment of normalization
      source                  — "rtk" | "default"
    """
    lat = raw.get("lat")
    lon = raw.get("lon")
    use_default = lat is None or lon is None
    if use_default:
        lat = DEFAULT_LAT
        lon = DEFAULT_LON

    return {
        "lat":         lat,
        "lon":         lon,
        "alt":         raw.get("alt"),
        "fix_quality": raw.get("fix_quality", 0),
        "num_sats":    raw.get("num_sats", 0),
        "hdop":        raw.get("hdop"),
        "speed_knots": raw.get("speed_knots"),
        "track_deg":   raw.get("track_deg"),
        "rtk_ts":      raw.get("ts", 0.0),
        "server_ts":   time.time(),
        "source":      "default" if use_default else "rtk",
    }


# **************** WEBSOCKET CONNECTION HANDLER ****************

async def ws_handler(websocket) -> None:
    """
    Accept a new browser WebSocket connection.

    Registers the socket in connected_clients so broadcast_loop() can push frames.
    The connection is kept open until the client disconnects.
    No inbound messages are expected (read-only map visualizer).
    """
    addr = websocket.remote_address
    logger.info(f"WebSocket client connected: {addr}")
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        logger.info(f"WebSocket client disconnected: {addr}")


# ****************
# **************** DATA OUTPUT: RTKReader → all web clients (this process → browser)
# ****************

async def broadcast_loop(reader: RTKReader, hz: float) -> None:
    """
    Asyncio coroutine: polls RTKReader at `hz` Hz and pushes the normalized
    snapshot to every connected browser.

    RTKReader is a daemon thread that continuously updates its internal snapshot;
    this coroutine simply samples that snapshot at the configured broadcast rate.
    There is no queue here — the latest snapshot overwrites any previous one.

    Dead connections (any send exception) are silently removed.

    Broadcast JSON structure (see normalize_frame() for full field list):
      lat / lon / alt         — position
      fix_quality             — GPS fix type
      num_sats / hdop         — quality
      speed_knots / track_deg — motion
      rtk_ts / server_ts      — timestamps
      source                  — "rtk" | "default"
    """
    period = 1.0 / max(hz, 0.5)
    while True:
        frame = normalize_frame(reader.get_data())   # ← READ from RTKReader
        if connected_clients:
            msg  = json.dumps(frame)
            dead = set()
            for ws in connected_clients.copy():
                try:
                    await ws.send(msg)               # ← PUSH to browser
                except Exception:
                    dead.add(ws)
            connected_clients.difference_update(dead)
        await asyncio.sleep(period)


# **************** HTTP SERVER (static web UI) ****************

def start_http_server(static_dir: Path, http_port: int) -> None:
    """
    Serve the RTK map visualizer from web_static/ on http_port.
    Runs in a dedicated daemon thread (not the asyncio loop).
    """
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def log_message(self, fmt, *args):
            logger.debug("HTTP %s - %s", self.address_string(), fmt % args)

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", http_port), Handler) as httpd:
        logger.info("HTTP server serving %s on port %d", static_dir, http_port)
        httpd.serve_forever()


# **************** MAIN ENTRY POINT ****************

async def main_async(args: argparse.Namespace) -> None:
    """
    Bootstrap sequence:
      1. Start RTKReader serial thread (begins reading NMEA immediately)
      2. Start HTTP server thread
      3. Launch broadcast_loop coroutine (polls RTKReader, pushes to browsers)
      4. Start WebSocket server and await forever
    """
    # ── 1. RTKReader serial thread ─────────────────────────
    reader = RTKReader()
    reader.start()

    # ── 2. HTTP server thread ──────────────────────────────
    static_dir = Path(__file__).parent / "web_static"
    if not static_dir.exists():
        logger.warning("web_static directory not found at %s", static_dir)
    else:
        http_thread = threading.Thread(
            target=start_http_server,
            args=(static_dir, args.ws_port),
            daemon=True,
            name="rtk-http-server",
        )
        http_thread.start()

    ws_actual_port = args.ws_port + 1
    logger.info("Starting WebSocket server on ws://localhost:%d", ws_actual_port)
    logger.info("Open browser at http://localhost:%d", args.ws_port)

    if args.open_browser:
        url = f"http://localhost:{args.ws_port}"
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    # ── 3. Broadcast coroutine ─────────────────────────────
    asyncio.create_task(broadcast_loop(reader, args.hz))

    # ── 4. WebSocket server ────────────────────────────────
    async with websockets.serve(ws_handler, "0.0.0.0", ws_actual_port):
        logger.info("RTK bridge running. Press Ctrl+C to stop.")
        await asyncio.Future()   # run forever


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RTK serial-to-WebSocket bridge")
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8775,
        help="HTTP port for web UI (WebSocket uses ws-port+1)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="Broadcast rate in Hz (default: 5)",
    )
    parser.add_argument(
        "--open-browser",
        default=True,
        help="Auto-open browser after startup",
    )
    return parser.parse_args()


# **************** SCRIPT ENTRY ****************

if __name__ == "__main__":
    args = parse_args()
    logger.info(
        "Starting RTK bridge | http=:%d | ws=:%d | broadcast_hz=%.2f",
        args.ws_port,
        args.ws_port + 1,
        args.hz,
    )
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("Bridge stopped by user.")
