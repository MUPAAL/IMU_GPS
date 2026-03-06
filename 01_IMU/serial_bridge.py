"""
serial_bridge.py - Serial port to WebSocket bridge for BNO085 IMU data.

Reads JSON frames from the ESP32-C3 serial port, computes Euler angles,
broadcasts to all WebSocket clients, and serves static web files over HTTP.

Usage:
    python serial_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765
"""

# **************** IMPORTS ****************

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

# **************** GLOBAL STATE ****************

connected_clients: set = set()          # all live WebSocket connections from browsers
data_queue: asyncio.Queue = None        # inter-thread pipe: serial thread → asyncio loop
_loop: asyncio.AbstractEventLoop = None # reference held by serial thread for thread-safe put

# Rolling window for frame-rate calculation
_frame_times: list = []
_FRAME_WINDOW = 50   # keep the last N timestamps to compute average Hz

# **************** EULER ANGLE COMPUTATION ****************

def quaternion_to_euler(qi: float, qj: float, qk: float, qr: float) -> dict:
    """
    Convert BNO085 rotation-vector quaternion (i, j, k, r) to Euler angles.

    Convention: ZYX intrinsic (yaw → pitch → roll), right-hand coordinate system.
    Returns a dict with roll / pitch / yaw in degrees, rounded to 2 decimal places.

    BNO085 quaternion field mapping:
      qi → x component
      qj → y component
      qk → z component
      qr → w (scalar) component
    """
    # Roll — rotation around X-axis
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch — rotation around Y-axis (clamped at ±90° to avoid gimbal lock singularity)
    sinp = 2.0 * (qr * qj - qk * qi)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw — rotation around Z-axis
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {
        "roll":  round(math.degrees(roll),  2),
        "pitch": round(math.degrees(pitch), 2),
        "yaw":   round(math.degrees(yaw),   2),
    }


# **************** HZ CALCULATION ****************

def compute_hz() -> float:
    """
    Return rolling-average frame rate over the last _FRAME_WINDOW frames.
    Called once per received serial frame; updates _frame_times in-place.
    """
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


# ****************
# **************** DATA INPUT: ESP32 serial RX → data_queue (hardware → this process)
# ****************

def serial_reader_thread(port: str, baud: int, loop: asyncio.AbstractEventLoop,
                          queue: asyncio.Queue) -> None:
    """
    Blocking daemon thread: reads JSON lines from ESP32-C3 serial port.

    Per-frame processing pipeline:
      1. readline() from serial port
      2. Skip comment lines (start with '#') and non-JSON lines
      3. json.loads() — discard malformed lines with a warning
      4. Validate 'rot' field presence (mandatory for orientation)
      5. quaternion_to_euler() — compute roll/pitch/yaw and attach as frame["euler"]
      6. compute_hz()          — attach live frame rate as frame["hz"]
      7. asyncio.run_coroutine_threadsafe(queue.put(frame), loop)
         → hands enriched frame to the asyncio event loop

    On serial open failure or read error: logs error, waits 3 s, retries forever.

    Expected JSON frame fields from ESP32 firmware:
      rot.qi / qj / qk / qr  — rotation vector quaternion
      lin_accel.x / y / z    — linear acceleration [m/s²]
      gyro.x / y / z         — angular rate [rad/s]
      mag.x / y / z          — magnetometer [µT]
      cal                     — calibration status 0–3
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

                        # Skip firmware comment lines and non-JSON output
                        if line.startswith("#") or not line.startswith("{"):
                            continue

                        try:
                            frame = json.loads(line)
                        except json.JSONDecodeError as e:
                            logger.warning(f"JSON parse error: {e} | raw: {line[:80]}")
                            continue

                        # Require rotation vector to be present
                        if "rot" not in frame:
                            logger.warning("Frame missing 'rot' field, skipping.")
                            continue

                        # Enrich frame with derived fields
                        rot = frame["rot"]
                        euler = quaternion_to_euler(
                            rot.get("qi", 0.0),
                            rot.get("qj", 0.0),
                            rot.get("qk", 0.0),
                            rot.get("qr", 1.0),
                        )
                        frame["euler"] = euler          # attach Euler angles
                        frame["hz"]    = compute_hz()   # attach live frame rate

                        # ── INJECT enriched frame into asyncio queue ──────────
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


# ****************
# **************** DATA OUTPUT: data_queue → all web clients (this process → browser)
# ****************

async def broadcaster(queue: asyncio.Queue) -> None:
    """
    Asyncio coroutine: drains data_queue and pushes each frame to every
    connected browser WebSocket client.

    Dead connections (ConnectionClosed or send error) are collected and
    removed from connected_clients after each broadcast round.

    Broadcast JSON frame structure (assembled by serial_reader_thread):
      rot        — raw quaternion: {qi, qj, qk, qr}
      lin_accel  — linear acceleration: {x, y, z}  [m/s²]
      gyro       — angular rate: {x, y, z}          [rad/s]
      mag        — magnetometer: {x, y, z}           [µT]
      cal        — calibration status (0–3)
      euler      — computed Euler angles: {roll, pitch, yaw} [°]
      hz         — measured frame rate [Hz]
    """
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


# **************** WEBSOCKET CONNECTION HANDLER ****************

async def ws_handler(websocket) -> None:
    """
    Accept a new browser WebSocket connection.

    Registers the socket in connected_clients so broadcaster() can push frames.
    The connection is kept open until the client closes it or drops.
    No inbound messages are expected from the browser (read-only visualizer).
    """
    addr = websocket.remote_address
    logger.info(f"WebSocket client connected: {addr}")
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    except Exception as e:
        logger.warning(f"WebSocket handler error for {addr}: {e}")
    finally:
        connected_clients.discard(websocket)
        logger.info(f"WebSocket client disconnected: {addr}")


# **************** HTTP SERVER (static web UI) ****************

def start_http_server(static_dir: Path, http_port: int) -> None:
    """
    Serve the IMU 3D visualizer from web_static/ on http_port.
    Runs in a dedicated daemon thread (not the asyncio loop).
    """
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def log_message(self, format, *args):
            logger.debug(f"HTTP {self.address_string()} - " + format % args)

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", http_port), Handler) as httpd:
        logger.info(f"HTTP server serving {static_dir} on port {http_port}")
        httpd.serve_forever()


# **************** MAIN ENTRY POINT ****************

async def main_async(args: argparse.Namespace) -> None:
    """
    Bootstrap sequence:
      1. Create asyncio Queue (serial thread → broadcaster coroutine pipe)
      2. Start serial reader thread (daemon)
      3. Start HTTP server thread (daemon)
      4. Launch broadcaster coroutine
      5. Start WebSocket server and await forever
    """
    global data_queue, _loop
    _loop = asyncio.get_running_loop()
    data_queue = asyncio.Queue(maxsize=200)   # cap prevents unbounded backlog

    # ── 1+2. Serial reader thread ──────────────────────────
    t = threading.Thread(
        target=serial_reader_thread,
        args=(args.port, args.baud, _loop, data_queue),
        daemon=True,
        name="serial-reader",
    )
    t.start()

    # ── 3. HTTP server thread ──────────────────────────────
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

    # ── 4. Broadcaster coroutine ───────────────────────────
    asyncio.create_task(broadcaster(data_queue))

    # ── 5. WebSocket server ────────────────────────────────
    ws_actual_port = args.ws_port + 1
    logger.info(f"Starting WebSocket server on ws://localhost:{ws_actual_port}")
    logger.info(f"Open browser at http://localhost:{args.ws_port}")

    url = f"http://localhost:{args.ws_port}"
    threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    async with websockets.serve(ws_handler, "0.0.0.0", ws_actual_port):
        logger.info("Bridge running. Press Ctrl+C to stop.")
        await asyncio.Future()   # run forever


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="BNO085 serial-to-WebSocket bridge"
    )
    parser.add_argument(
        "--port", default="/dev/ttyACM0",
        help="Serial port device (linux: /dev/ttyACM0, find: ls /dev/cu.*)"
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


# **************** SCRIPT ENTRY ****************

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
