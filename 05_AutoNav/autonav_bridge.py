"""
autonav_bridge.py — Autonomous Navigation bridge (camera + joystick control).

Data flow:
    robot_bridge WS :8796
        └─ RobotBridgeClient (WS client) ──→ re-broadcast {imu, rtk, nav_status, odom} to browsers

    Joystick commands from browser ──→ RobotBridgeClient ──→ robot_bridge :8796

    asyncio.gather():
        ├─ RobotBridgeClient.run()     (proxy from robot_bridge)
        └─ websockets.serve() :ws_port+1
               ↕ joystick / heartbeat / cam_switch

    HttpFileServer :ws_port → web_static/ (index.html, app.js, style.css)

Usage:
    python autonav_bridge.py [--ws-port 8805] [--robot-ws-url ws://localhost:8796]
    # Browser:   http://localhost:8805
    # WebSocket: ws://localhost:8806
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
import threading
import time
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import websockets

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("autonav_bridge")

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — HTTP FILE SERVER
# ══════════════════════════════════════════════════════════════════════════════

class _StaticHandler(SimpleHTTPRequestHandler):
    """Serve web_static/ and inject config into index.html."""

    def __init__(self, *args, directory: str, config: dict, **kwargs):
        self._config = config
        super().__init__(*args, directory=directory, **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            src  = Path(self.directory) / "index.html"
            html = src.read_text(encoding="utf-8")
            # Inject config as data attributes on <html>
            attrs = " ".join(f'data-{k}="{v}"' for k, v in self._config.items())
            html = html.replace("<html", f"<html {attrs}", 1)
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
    def __init__(self, port: int, static_dir: str, config: dict):
        self._port       = port
        self._static_dir = static_dir
        self._config     = config

    def start(self) -> None:
        def make_handler(*args, **kwargs):
            return _StaticHandler(*args, directory=self._static_dir, config=self._config, **kwargs)

        server = ThreadingHTTPServer(("0.0.0.0", self._port), make_handler)
        t = threading.Thread(target=server.serve_forever, name="HttpFileServer", daemon=True)
        t.start()
        logger.info("HttpFileServer: http://0.0.0.0:%d", self._port)


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — ROBOT BRIDGE CLIENT (proxy)
# ══════════════════════════════════════════════════════════════════════════════

class RobotBridgeClient:
    """
    Connects to robot_bridge WS and:
    - re-broadcasts incoming data (odom, imu, rtk, nav_status, state_status) to browsers
    - forwards joystick/toggle commands from browsers to robot_bridge
    """

    def __init__(self, url: str, broadcast_fn):
        self._url          = url
        self._broadcast_fn = broadcast_fn
        self._ws           = None
        self._ws_lock      = asyncio.Lock()

    async def send(self, msg: dict) -> None:
        async with self._ws_lock:
            ws = self._ws
        if ws is not None:
            try:
                await ws.send(json.dumps(msg))
            except Exception as exc:
                logger.warning("RobotBridgeClient.send: %s", exc)

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    async with self._ws_lock:
                        self._ws = ws
                    logger.info("RobotBridgeClient: connected to %s", self._url)
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        await self._broadcast_fn(msg)
            except Exception as exc:
                logger.warning("RobotBridgeClient: %s — retry in 3s", exc)
            finally:
                async with self._ws_lock:
                    self._ws = None
            await asyncio.sleep(3)


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — AUTONAV WEBSOCKET SERVER
# ══════════════════════════════════════════════════════════════════════════════

class AutoNavWebSocketServer:
    """
    WebSocket server for the browser UI.

    Receives:  joystick, heartbeat, cam_switch
    Broadcasts: odom, imu, rtk, nav_status, state_status (proxied from robot_bridge)
    """

    def __init__(
        self,
        port:             int,
        max_linear:       float,
        max_angular:      float,
        watchdog_timeout: float,
        robot_ws_url:     str,
    ):
        self._port             = port
        self._max_linear       = max_linear
        self._max_angular      = max_angular
        self._watchdog_timeout = watchdog_timeout
        self._robot_ws_url     = robot_ws_url

        self._clients:       set  = set()
        self._clients_lock         = asyncio.Lock()
        self._loop:          asyncio.AbstractEventLoop | None = None
        self._last_heartbeat = time.time()
        self._robot_client:  RobotBridgeClient | None = None

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

    # ── Watchdog ──────────────────────────────────────────────────────────────

    async def _watchdog_loop(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            elapsed = time.time() - self._last_heartbeat
            if elapsed > self._watchdog_timeout:
                logger.warning("Watchdog: no heartbeat for %.1fs — sending stop", elapsed)
                if self._robot_client:
                    await self._robot_client.send(
                        {"type": "joystick", "linear": 0.0, "angular": 0.0, "force": 0.0}
                    )
                self._last_heartbeat = time.time()

    # ── WebSocket handler ──────────────────────────────────────────────────────

    async def _ws_handler(self, websocket) -> None:
        async with self._clients_lock:
            self._clients.add(websocket)
        logger.info("WS: client connected: %s", websocket.remote_address)
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
                        if self._robot_client:
                            await self._robot_client.send(
                                {"type": "joystick", "linear": lin, "angular": ang, "force": 1.0}
                            )
                    except (TypeError, ValueError):
                        pass

                elif msg_type == "toggle_state":
                    self._last_heartbeat = time.time()
                    if self._robot_client:
                        await self._robot_client.send({"type": "toggle_state"})
                    logger.info("WS: toggle_state forwarded to robot_bridge")

                elif msg_type == "heartbeat":
                    if self._robot_client:
                        await self._robot_client.send({"type": "heartbeat"})

        except Exception:
            pass
        finally:
            async with self._clients_lock:
                self._clients.discard(websocket)
            # Stop robot on disconnect
            if self._robot_client:
                await self._robot_client.send(
                    {"type": "joystick", "linear": 0.0, "angular": 0.0, "force": 0.0}
                )
            logger.info("WS: client disconnected: %s", websocket.remote_address)

    # ── Main serve ────────────────────────────────────────────────────────────

    async def serve(self) -> None:
        self._loop           = asyncio.get_running_loop()
        self._last_heartbeat = time.time()

        self._robot_client = RobotBridgeClient(
            url          = self._robot_ws_url,
            broadcast_fn = self._broadcast,
        )

        async with websockets.serve(
            self._ws_handler, "0.0.0.0", self._port,
            ping_interval=20, ping_timeout=10,
        ):
            logger.info("AutoNavWebSocketServer: ws://0.0.0.0:%d", self._port)
            await asyncio.gather(
                self._watchdog_loop(),
                self._robot_client.run(),
            )


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════════

class AutoNavBridge:
    """Top-level orchestrator for the AutoNav bridge."""

    def __init__(
        self,
        ws_port:          int,
        max_linear:       float,
        max_angular:      float,
        watchdog_timeout: float,
        robot_ws_url:     str,
        cam1_ip:          str,
        cam2_ip:          str,
        cam1_port:        int,
        cam2_port:        int,
    ):
        self._ws_port          = ws_port
        self._max_linear       = max_linear
        self._max_angular      = max_angular
        self._watchdog_timeout = watchdog_timeout
        self._robot_ws_url     = robot_ws_url
        self._cam1_ip          = cam1_ip
        self._cam2_ip          = cam2_ip
        self._cam1_port        = cam1_port
        self._cam2_port        = cam2_port

    def run(self) -> None:
        static_dir = str(Path(__file__).parent / "web_static")

        config = {
            "max-linear":  self._max_linear,
            "max-angular": self._max_angular,
            "cam1-url":    f"http://{self._cam1_ip}:{self._cam1_port}",
            "cam2-url":    f"http://{self._cam2_ip}:{self._cam2_port}",
            "ws-port":     self._ws_port + 1,
        }

        http_server = HttpFileServer(
            port       = self._ws_port,
            static_dir = static_dir,
            config     = config,
        )
        http_server.start()

        ws_server = AutoNavWebSocketServer(
            port             = self._ws_port + 1,
            max_linear       = self._max_linear,
            max_angular      = self._max_angular,
            watchdog_timeout = self._watchdog_timeout,
            robot_ws_url     = self._robot_ws_url,
        )

        url = f"http://localhost:{self._ws_port}"
        logger.info(
            "AutoNavBridge: http://localhost:%d  ws://localhost:%d  robot→%s",
            self._ws_port, self._ws_port + 1, self._robot_ws_url,
        )
        logger.info("Open browser at %s", url)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
        asyncio.run(ws_server.serve())


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 5 — CLI
# ══════════════════════════════════════════════════════════════════════════════

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AutoNav Bridge — camera + robot control")
    parser.add_argument(
        "--ws-port", type=int,
        default=int(os.environ.get("AUTONAV_WS_PORT", str(_cfg.AUTONAV_WS_PORT if _cfg else 8805))),
        help="HTTP port; WebSocket = ws-port+1",
    )
    parser.add_argument(
        "--max-linear", type=float,
        default=float(os.environ.get("AUTONAV_MAX_LINEAR_VEL", str(_cfg.AUTONAV_MAX_LINEAR_VEL if _cfg else 1.0))),
    )
    parser.add_argument(
        "--max-angular", type=float,
        default=float(os.environ.get("AUTONAV_MAX_ANGULAR_VEL", str(_cfg.AUTONAV_MAX_ANGULAR_VEL if _cfg else 1.0))),
    )
    parser.add_argument(
        "--watchdog-timeout", type=float,
        default=float(os.environ.get("AUTONAV_WATCHDOG_TIMEOUT", "2.0")),
    )
    parser.add_argument(
        "--robot-ws-url",
        default=os.environ.get("AUTONAV_ROBOT_WS", _cfg.AUTONAV_ROBOT_WS if _cfg else "ws://localhost:8796"),
        help="robot_bridge WebSocket URL",
    )
    parser.add_argument(
        "--cam1-ip",
        default=os.environ.get("CAM1_IP", _cfg.CAM1_IP if _cfg else "10.95.76.11"),
    )
    parser.add_argument(
        "--cam2-ip",
        default=os.environ.get("CAM2_IP", _cfg.CAM2_IP if _cfg else "10.95.76.10"),
    )
    parser.add_argument(
        "--cam1-port", type=int,
        default=int(os.environ.get("CAM1_STREAM_PORT", str(_cfg.CAM1_STREAM_PORT if _cfg else 8080))),
    )
    parser.add_argument(
        "--cam2-port", type=int,
        default=int(os.environ.get("CAM2_STREAM_PORT", str(_cfg.CAM2_STREAM_PORT if _cfg else 8081))),
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    AutoNavBridge(
        ws_port          = args.ws_port,
        max_linear       = args.max_linear,
        max_angular      = args.max_angular,
        watchdog_timeout = args.watchdog_timeout,
        robot_ws_url     = args.robot_ws_url,
        cam1_ip          = args.cam1_ip,
        cam2_ip          = args.cam2_ip,
        cam1_port        = args.cam1_port,
        cam2_port        = args.cam2_port,
    ).run()