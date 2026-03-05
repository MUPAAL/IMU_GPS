"""
serial_bridge.py - Serial port to WebSocket bridge for BNO085 IMU data.

Reads JSON frames from the ESP32-C3 serial port, computes Euler angles,
broadcasts to all WebSocket clients, and serves static web files over HTTP.

Usage:
    python serial_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765
"""

import asyncio
import json
import logging
import math
import argparse
import time
import webbrowser
from pathlib import Path
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler
import threading
import socketserver

import serial
import websockets

# ---------- Logging setup ----------
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

# ---------- Globals ----------
connected_clients: set = set()
data_queue: asyncio.Queue = None  # initialized in main
_loop: asyncio.AbstractEventLoop = None

# Stats for Hz calculation
_frame_times: list = []
_FRAME_WINDOW = 50  # rolling window for Hz calculation


# ---------- Euler angle computation ----------
def quaternion_to_euler(qi: float, qj: float, qk: float, qr: float) -> dict:
    """
    Convert quaternion (i, j, k, r) to Euler angles in degrees.
    Returns dict with roll, pitch, yaw in degrees.

    Convention: ZYX intrinsic (yaw-pitch-roll), right-hand coordinate system.
    """
    # Roll (rotation around X-axis)
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around Y-axis)
    sinp = 2.0 * (qr * qj - qk * qi)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # clamp to ±90°
    else:
        pitch = math.asin(sinp)

    # Yaw (rotation around Z-axis)
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {
        "roll":  round(math.degrees(roll),  2),
        "pitch": round(math.degrees(pitch), 2),
        "yaw":   round(math.degrees(yaw),   2),
    }


def compute_hz() -> float:
    """Return rolling-average Hz over last _FRAME_WINDOW frames."""
    global _frame_times
    now = time.monotonic()
    _frame_times.append(now)
    if len(_frame_times) > _FRAME_WINDOW:
        _frame_times = _frame_times[-_FRAME_WINDOW:]
    if len(_frame_times) < 2:
        return 0.0
    elapsed = _frame_times[-1] - _frame_times[0]
    if elapsed <= 0:
        return 0.0
    return round((len(_frame_times) - 1) / elapsed, 1)


# ---------- Serial reader thread ----------
def serial_reader_thread(port: str, baud: int, loop: asyncio.AbstractEventLoop,
                          queue: asyncio.Queue) -> None:
    """
    Blocking thread: reads lines from serial port, validates JSON,
    enriches with Euler angles + Hz, then puts to asyncio queue.
    """
    logger.info(f"Opening serial port {port} at {baud} baud.")
    while True:
        try:
            with serial.Serial(port, baud, timeout=1.0) as ser:
                logger.info(f"Serial port {port} opened successfully.")
                while True:
                    try:
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode("utf-8", errors="replace").strip()

                        # Skip comment lines from firmware
                        if line.startswith("#") or not line.startswith("{"):
                            continue

                        try:
                            frame = json.loads(line)
                        except json.JSONDecodeError as e:
                            logger.warning(f"JSON parse error: {e} | raw: {line[:80]}")
                            continue

                        # Validate required fields
                        if "rot" not in frame:
                            logger.warning("Frame missing 'rot' field, skipping.")
                            continue

                        # Compute Euler angles from rotation vector quaternion
                        rot = frame["rot"]
                        euler = quaternion_to_euler(
                            rot.get("qi", 0.0),
                            rot.get("qj", 0.0),
                            rot.get("qk", 0.0),
                            rot.get("qr", 1.0),
                        )
                        frame["euler"] = euler
                        frame["hz"] = compute_hz()

                        # Put into asyncio queue (thread-safe)
                        asyncio.run_coroutine_threadsafe(queue.put(frame), loop)

                    except serial.SerialException as e:
                        logger.error(f"Serial read error: {e}")
                        break
                    except Exception as e:
                        logger.error(f"Unexpected error in serial read loop: {e}")
                        break

        except serial.SerialException as e:
            logger.error(f"Cannot open serial port {port}: {e}. Retrying in 3s.")
        except Exception as e:
            logger.error(f"Serial thread unexpected error: {e}. Retrying in 3s.")

        time.sleep(3.0)


# ---------- WebSocket broadcaster ----------
async def broadcaster(queue: asyncio.Queue) -> None:
    """Consume queue and broadcast JSON to all connected WebSocket clients."""
    while True:
        frame = await queue.get()
        if not connected_clients:
            continue
        msg = json.dumps(frame)
        dead_clients = set()
        for ws in connected_clients.copy():
            try:
                await ws.send(msg)
            except websockets.exceptions.ConnectionClosed:
                dead_clients.add(ws)
            except Exception as e:
                logger.warning(f"Error sending to client: {e}")
                dead_clients.add(ws)
        connected_clients.difference_update(dead_clients)


# ---------- WebSocket handler ----------
async def ws_handler(websocket) -> None:
    """Handle a new WebSocket connection."""
    addr = websocket.remote_address
    logger.info(f"WebSocket client connected: {addr}")
    connected_clients.add(websocket)
    try:
        # Keep connection alive until client disconnects
        await websocket.wait_closed()
    except Exception as e:
        logger.warning(f"WebSocket handler error for {addr}: {e}")
    finally:
        connected_clients.discard(websocket)
        logger.info(f"WebSocket client disconnected: {addr}")


# ---------- HTTP static file server ----------
def start_http_server(static_dir: Path, http_port: int) -> None:
    """Start a simple HTTP server to serve web_static/ files."""

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def log_message(self, format, *args):
            # Route HTTP access logs through our logger
            logger.debug(f"HTTP {self.address_string()} - " + format % args)

    # Allow port reuse
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", http_port), Handler) as httpd:
        logger.info(f"HTTP server serving {static_dir} on port {http_port}")
        httpd.serve_forever()


# ---------- Main ----------
async def main_async(args: argparse.Namespace) -> None:
    global data_queue, _loop
    _loop = asyncio.get_running_loop()
    data_queue = asyncio.Queue(maxsize=200)

    # Start serial reader in background thread
    t = threading.Thread(
        target=serial_reader_thread,
        args=(args.port, args.baud, _loop, data_queue),
        daemon=True,
        name="serial-reader",
    )
    t.start()

    # Start HTTP server in background thread
    static_dir = Path(__file__).parent / "web_static"
    if not static_dir.exists():
        logger.warning(f"web_static directory not found at {static_dir}")
    else:
        http_thread = threading.Thread(
            target=start_http_server,
            args=(static_dir, args.ws_port),
            daemon=True,
            name="http-server",
        )
        http_thread.start()

    # Start broadcaster coroutine
    asyncio.create_task(broadcaster(data_queue))

    # Start WebSocket server on a different port (ws_port + 1)
    ws_actual_port = args.ws_port + 1
    logger.info(f"Starting WebSocket server on ws://localhost:{ws_actual_port}")
    logger.info(f"Open browser at http://localhost:{args.ws_port}")

    # Open browser automatically
    url = f"http://localhost:{args.ws_port}"
    threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    async with websockets.serve(ws_handler, "0.0.0.0", ws_actual_port):
        logger.info("Bridge running. Press Ctrl+C to stop.")
        await asyncio.Future()  # run forever


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="BNO085 serial-to-WebSocket bridge"
    )
    parser.add_argument(
        "--port", default="/dev/ttyACM0",
        help="Serial port device (default: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baud", type=int, default=921600,
        help="Serial baud rate (default: 921600)"
    )
    parser.add_argument(
        "--ws-port", type=int, default=8765,
        help="HTTP port for web UI (WebSocket uses ws-port+1, default: 8765)"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    logger.info(
        f"Starting BNO085 bridge | serial={args.port}@{args.baud} "
        f"| http=:{args.ws_port} | ws=:{args.ws_port + 1}"
    )
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("Bridge stopped by user.")
