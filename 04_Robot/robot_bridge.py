"""
robot_bridge.py — Farm Robot serial bridge + nav_bridge proxy (standalone).

Data flow:
    03_Nav/nav_bridge.py WS :8786
        └─ NavBridgeClient (WS client) ──→ re-broadcast {imu, rtk, nav_status} to browsers

    Feather M4 serial ← _send_velocity()  ← joystick commands
                      → O: odometry lines → _odom_broadcast_loop() 20Hz → browsers
                      → S:ACTIVE/S:READY  → state_status broadcast → browsers

    asyncio.gather():
        ├─ _odom_broadcast_loop()   20 Hz
        ├─ _watchdog_loop()          0.5 Hz
        ├─ NavBridgeClient.run()     (proxy from nav_bridge)
        └─ websockets.serve() :ws_port+1
               ↕ joystick / toggle_state / heartbeat

    HttpFileServer :ws_port → web_static/ (index.html, app.js, style.css)

Usage:
    python robot_bridge.py [--ws-port 8888] [--serial-port /dev/ttyACM0]
    # Browser:   http://localhost:8888
    # WebSocket: ws://localhost:8889
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

import argparse
import asyncio
import json
import logging
import os
import platform
import threading
import time
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
import websockets
import subprocess as _subprocess
import sys as _sys
from pathlib import Path as _Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("robot_bridge")

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ══════════════════════════════════════════════════════════════════════════════

_last_odom:   dict  = {}
_odom_lock          = threading.Lock()
_vel_lock           = threading.Lock()
_last_linear:  float = 0.0
_last_angular: float = 0.0

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE (delegated to 03_Nav/nav_bridge.py)
# ══════════════════════════════════════════════════════════════════════════════
# IMU, RTK, Navigation, Coverage planning are handled by nav_bridge.
# This bridge only manages the serial link to the Feather M4.

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════════

# ── HTTP file server ──────────────────────────────────────────────────────────

class _StaticHandler(SimpleHTTPRequestHandler):
    """Serve web_static/ and inject speed limits into index.html."""

    def __init__(self, *args, directory: str, max_linear: float, max_angular: float, **kwargs):
        self._max_linear  = max_linear
        self._max_angular = max_angular
        super().__init__(*args, directory=directory, **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            src = Path(self.directory) / "index.html"
            html = src.read_text(encoding="utf-8")
            html = html.replace(
                "<html",
                f'<html data-max-linear="{self._max_linear}" '
                f'data-max-angular="{self._max_angular}"',
                1,
            )
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            super().do_GET()

    def log_message(self, format, *args):
        pass


class HttpFileServer:
    def __init__(self, port: int, static_dir: str, max_linear: float, max_angular: float):
        self._port        = port
        self._static_dir  = static_dir
        self._max_linear  = max_linear
        self._max_angular = max_angular

    def start(self) -> None:
        def make_handler(*args, **kwargs):
            return _StaticHandler(
                *args,
                directory=self._static_dir,
                max_linear=self._max_linear,
                max_angular=self._max_angular,
                **kwargs,
            )

        server = ThreadingHTTPServer(("0.0.0.0", self._port), make_handler)
        t = threading.Thread(target=server.serve_forever, name="HttpFileServer", daemon=True)
        t.start()
        logger.info("HttpFileServer: http://0.0.0.0:%d", self._port)


# ── nav_bridge WebSocket client ───────────────────────────────────────────────

class NavBridgeClient:
    """Connects to nav_bridge WS and re-broadcasts data to robot_bridge clients."""

    def __init__(self, url: str, broadcast_fn):
        self._url          = url
        self._broadcast_fn = broadcast_fn

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("NavBridgeClient: connected to %s", self._url)
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        if "imu" in msg:
                            await self._broadcast_fn({"type": "imu", **msg["imu"]})
                        if "rtk" in msg:
                            rtk = msg["rtk"]
                            available = bool(
                                rtk.get("source") == "rtk"
                                or (rtk.get("fix_quality") or 0) > 0
                            )
                            await self._broadcast_fn({"type": "rtk", "available": available, **rtk})
                        if "nav" in msg:
                            nav = msg["nav"]
                            await self._broadcast_fn({
                                "type":           "nav_status",
                                "state":          nav.get("state", "idle"),
                                "progress":       [nav.get("reached_count", 0),
                                                   nav.get("total_waypoints", 0)],
                                "distance_m":     nav.get("target_distance_m"),
                                "heading_deg":    nav.get("heading_deg"),
                                "heading_dir":    nav.get("heading_dir"),
                                "target_bearing": nav.get("heading_deg"),
                                "nav_mode":       nav.get("nav_mode", "--"),
                                "filter_mode":    nav.get("filter_mode", "--"),
                                "tolerance_m":    nav.get("tolerance_m"),
                            })
            except Exception as exc:
                logger.warning("NavBridgeClient: %s — retry in 3s", exc)
                await asyncio.sleep(3)

# ── CameraBridgeClient (for testing) ─────────────────────────────────────────
class CamBridgeClient:
    """Connects to camera_bridge WS, reads path_target, drives robot."""

    def __init__(self, url: str, send_velocity_fn):
        self._url             = url
        self._send_velocity   = send_velocity_fn

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("CamBridgeClient: connected to %s", self._url)
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        target = msg.get("path_target")
                        if target is not None:
                            x_norm, y_norm = target
                            # PID-style steering: scale angular by x offset
                            linear  =  0.3                    # constant forward speed
                            angular = -x_norm * 1.0           # negative = turn towards target
                            direction = "CENTER"
                            if x_norm < -0.1:
                                direction = "LEFT"
                            elif x_norm > 0.1:
                                direction = "RIGHT"
                            print(f"[PATH] x={x_norm:+.3f} y={y_norm:.3f} → {direction}  cmd=({linear:.2f}, {angular:.2f})")
                            self._send_velocity(linear, angular)
                        else:
                            print("[PATH] no target → stop")
                            self._send_velocity(0.0, 0.0)
            except Exception as exc:
                logger.warning("CamBridgeClient: %s — retry in 3s", exc)
                await asyncio.sleep(3)

# ── Robot WebSocket server ────────────────────────────────────────────────────

class RobotWebSocketServer:
    """
    WebSocket server for the browser.

    Receives: joystick, toggle_state, heartbeat
    Broadcasts: odom (20 Hz), state_status (on change),
                imu / rtk / nav_status (proxied from NavBridgeClient)
    Serial:   Feather M4 via pyserial
    """

    def __init__(
        self,
        port:             int,
        serial_port:      str,
        serial_baud:      int,
        serial_timeout:   float,
        max_linear:       float,
        max_angular:      float,
        watchdog_timeout: float,
        nav_ws_url:       str,
    ):
        self._port             = port
        self._serial_port      = serial_port
        self._serial_baud      = serial_baud
        self._serial_timeout   = serial_timeout
        self._max_linear       = max_linear
        self._max_angular      = max_angular
        self._watchdog_timeout = watchdog_timeout
        self._nav_ws_url       = nav_ws_url

        self._ser:         serial.Serial | None = None
        self._ser_lock     = threading.Lock()
        self._serial_ok    = False

        self._auto_active  = False
        self._clients:     set = set()
        self._clients_lock = asyncio.Lock()
        self._loop:        asyncio.AbstractEventLoop | None = None
        self._last_heartbeat = time.time()

    # ── Serial ───────────────────────────────────────────────────────────────

    def open_serial(self) -> None:
        try:
            self._ser = serial.Serial(
                self._serial_port, self._serial_baud, timeout=self._serial_timeout
            )
            self._serial_ok = True
            logger.info("Serial: opened %s @ %d", self._serial_port, self._serial_baud)
        except serial.SerialException as exc:
            logger.error("Serial: cannot open %s: %s", self._serial_port, exc)
            self._serial_ok = False

    def close_serial(self) -> None:
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
                logger.info("Serial: closed")

    def _send_velocity(self, linear: float, angular: float) -> None:
        global _last_linear, _last_angular
        cmd = f"V{linear:.2f},{angular:.2f}\n".encode()
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write(cmd)
            except serial.SerialException as exc:
                logger.error("Serial: write error: %s", exc)
                self._serial_ok = False
                return
        with _vel_lock:
            _last_linear  = linear
            _last_angular = angular

    def _send_raw(self, data: bytes) -> None:
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write(data)
            except serial.SerialException as exc:
                logger.error("Serial: raw write error: %s", exc)
                self._serial_ok = False

    def _start_serial_reader(self) -> None:
        t = threading.Thread(
            target=self._serial_reader_thread, name="SerialReader", daemon=True
        )
        t.start()

    def _serial_reader_thread(self) -> None:
        buf = b""
        while True:
            try:
                with self._ser_lock:
                    if self._ser is None or not self._ser.is_open:
                        buf = b""
                        time.sleep(0.1)
                        continue
                    n     = self._ser.in_waiting
                    chunk = self._ser.read(n) if n > 0 else b""
            except serial.SerialException as exc:
                logger.error("SerialReader: error: %s", exc)
                buf = b""
                time.sleep(0.1)
                continue
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._handle_serial_line(line.strip())
            else:
                time.sleep(0.01)

    def _handle_serial_line(self, line: bytes) -> None:
        if line == b"S:ACTIVE":
            new_state = True
        elif line == b"S:READY":
            new_state = False
        elif line.startswith(b"O:"):
            try:
                parts     = line[2:].decode().split(",")
                v         = float(parts[0])
                w         = float(parts[1])
                state_int = int(parts[2]) if len(parts) > 2 else None
                soc       = int(parts[3]) if len(parts) > 3 else None
                with _odom_lock:
                    _last_odom.update({"v": v, "w": w, "state": state_int,
                                       "soc": soc, "ts": time.time()})
            except (ValueError, IndexError):
                pass
            return
        else:
            return
        if self._auto_active == new_state:
            return
        self._auto_active = new_state
        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"type": "state_status", "active": new_state}),
                self._loop,
            )

    # ── Broadcast ─────────────────────────────────────────────────────────────

    async def _broadcast(self, obj: dict) -> None:
        msg = json.dumps(obj)
        async with self._clients_lock:
            clients = set(self._clients)
        dead = set()
        for ws in clients:
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        if dead:
            async with self._clients_lock:
                self._clients -= dead

    # ── Broadcast loops ───────────────────────────────────────────────────────

    async def _odom_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(0.05)  # 20 Hz
            with _odom_lock:
                odom = dict(_last_odom)
            if odom:
                await self._broadcast({"type": "odom", **odom})

    async def _watchdog_loop(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            elapsed = time.time() - self._last_heartbeat
            if elapsed > self._watchdog_timeout:
                logger.warning("Watchdog: no heartbeat for %.1fs — emergency stop", elapsed)
                self._send_velocity(0.0, 0.0)
                if self._auto_active:
                    self._send_raw(b"\r")
                self._last_heartbeat = time.time()

    # ── WebSocket handler ──────────────────────────────────────────────────────

    async def _ws_handler(self, websocket) -> None:
        async with self._clients_lock:
            self._clients.add(websocket)
        logger.info("WS: client connected: %s", websocket.remote_address)
        try:
            await websocket.send(
                json.dumps({"type": "state_status", "active": self._auto_active})
            )
        except Exception:
            pass
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                msg_type = msg.get("type")

                if msg_type in ("heartbeat", "joystick"):
                    self._last_heartbeat = time.time()

                if msg_type == "joystick":
                    try:
                        lin = max(-self._max_linear,
                                  min(self._max_linear,  float(msg.get("linear",  0.0))))
                        ang = max(-self._max_angular,
                                  min(self._max_angular, float(msg.get("angular", 0.0))))
                        self._send_velocity(lin, ang)
                    except (TypeError, ValueError):
                        pass

                elif msg_type == "toggle_state":
                    self._last_heartbeat = time.time()
                    self._send_raw(b"\r")
                    logger.info("WS: toggle_state → serial \\r")

        except Exception:
            pass
        finally:
            async with self._clients_lock:
                self._clients.discard(websocket)
            logger.info("WS: client disconnected: %s", websocket.remote_address)

    # ── Main serve ────────────────────────────────────────────────────────────

    async def serve(self) -> None:
        self._loop           = asyncio.get_running_loop()
        self._last_heartbeat = time.time()
        self._start_serial_reader()

        nav_client = NavBridgeClient(
            url          = self._nav_ws_url,
            broadcast_fn = self._broadcast,
        )

        async with websockets.serve(
            self._ws_handler, "0.0.0.0", self._port,
            ping_interval=20, ping_timeout=10,
        ):
            logger.info("RobotWebSocketServer: ws://0.0.0.0:%d", self._port)
            cam_client = CamBridgeClient(
            url              = "ws://localhost:8816",
            send_velocity_fn = self._send_velocity,
            )

            await asyncio.gather(
                self._odom_broadcast_loop(),
                self._watchdog_loop(),
                nav_client.run(),
                cam_client.run(),
            )


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════════

class RobotBridge:
    """Top-level orchestrator for the robot serial bridge."""

    def __init__(
        self,
        ws_port:          int,
        serial_port:      str,
        serial_baud:      int,
        serial_timeout:   float,
        max_linear:       float,
        max_angular:      float,
        watchdog_timeout: float,
        nav_ws_url:       str,
    ):
        self._ws_port          = ws_port
        self._serial_port      = serial_port
        self._serial_baud      = serial_baud
        self._serial_timeout   = serial_timeout
        self._max_linear       = max_linear
        self._max_angular      = max_angular
        self._watchdog_timeout = watchdog_timeout
        self._nav_ws_url       = nav_ws_url

    def run(self) -> None:
        static_dir = str(Path(__file__).parent / "web_static")

        http_server = HttpFileServer(
            port        = self._ws_port,
            static_dir  = static_dir,
            max_linear  = self._max_linear,
            max_angular = self._max_angular,
        )
        #EDITS
        # Start yellow detector as a subprocess
        _detector_path = _Path(__file__).parent / "yellow_detector.py"
        if _detector_path.exists():
            _subprocess.Popen(
                [_sys.executable, str(_detector_path)],
                stdout=_subprocess.DEVNULL,
                stderr=_subprocess.DEVNULL,
            )
            logger.info("Started yellow_detector.py on port 8082")
        else:
            logger.warning("yellow_detector.py not found — camera detection disabled")
        http_server.start()

        ws_server = RobotWebSocketServer(
            port             = self._ws_port + 1,
            serial_port      = self._serial_port,
            serial_baud      = self._serial_baud,
            serial_timeout   = self._serial_timeout,
            max_linear       = self._max_linear,
            max_angular      = self._max_angular,
            watchdog_timeout = self._watchdog_timeout,
            nav_ws_url       = self._nav_ws_url,
        )
        ws_server.open_serial()

        url = f"http://localhost:{self._ws_port}"
        logger.info("RobotBridge: http://localhost:%d  ws://localhost:%d  nav→%s",
                    self._ws_port, self._ws_port + 1, self._nav_ws_url)
        logger.info("Open browser at %s", url)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
        asyncio.run(ws_server.serve())


def _default_serial_port() -> str:
    return "/dev/cu.usbmodem1301" if platform.system() == "Darwin" else "/dev/ttyACM0"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robot Bridge — serial + nav proxy")
    parser.add_argument(
        "--ws-port", type=int,
        default=int(os.environ.get("WEB_HTTP_PORT", str(_cfg.ROBOT_WS_PORT if _cfg else 8888))),
        help="HTTP port; WebSocket = ws-port+1",
    )
    parser.add_argument(
        "--serial-port",
        default=os.environ.get("FEATHER_PORT", _cfg.ROBOT_SERIAL_PORT if _cfg else _default_serial_port()),
        help="Feather M4 serial port",
    )
    parser.add_argument(
        "--serial-baud", type=int,
        default=int(os.environ.get("SERIAL_BAUD", str(_cfg.ROBOT_SERIAL_BAUD if _cfg else 115200))),
    )
    parser.add_argument(
        "--serial-timeout", type=float,
        default=float(os.environ.get("SERIAL_TIMEOUT", str(_cfg.ROBOT_SERIAL_TIMEOUT if _cfg else 1.0))),
    )
    parser.add_argument(
        "--max-linear", type=float,
        default=float(os.environ.get("MAX_LINEAR_VEL", str(_cfg.ROBOT_MAX_LINEAR if _cfg else 1.0))),
        help="Max linear velocity m/s",
    )
    parser.add_argument(
        "--max-angular", type=float,
        default=float(os.environ.get("MAX_ANGULAR_VEL", str(_cfg.ROBOT_MAX_ANGULAR if _cfg else 1.0))),
        help="Max angular velocity rad/s",
    )
    parser.add_argument(
        "--watchdog-timeout", type=float,
        default=float(os.environ.get("WATCHDOG_TIMEOUT", str(_cfg.ROBOT_WATCHDOG_TIMEOUT if _cfg else 2.0))),
    )
    parser.add_argument(
        "--nav-ws-url",
        default=os.environ.get("NAV_WS_URL", _cfg.ROBOT_NAV_WS if _cfg else "ws://localhost:8786"),
        help="nav_bridge WebSocket URL",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    RobotBridge(
        ws_port          = args.ws_port,
        serial_port      = args.serial_port,
        serial_baud      = args.serial_baud,
        serial_timeout   = args.serial_timeout,
        max_linear       = args.max_linear,
        max_angular      = args.max_angular,
        watchdog_timeout = args.watchdog_timeout,
        nav_ws_url       = args.nav_ws_url,
    ).run()
