"""
nav_bridge.py — IMU + RTK viewer: merges both streams and serves the Nav dashboard.

Data flow:
    imu_bridge(WS :8766) ──→ ImuWsClient.latest ─┐
                                                   ├─ BroadcastLoop(10Hz) → queue → NavWebSocketServer → browser
    rtk_bridge(WS :8776) ──→ RtkWsClient.latest ─┘

Usage:
    python nav_bridge.py
    # Browser: http://localhost:8785
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

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

WS_MSG_VERSION = 1
DEFAULT_NAV_PORT = _cfg.NAV_WS_PORT if _cfg else 8785
DEFAULT_IMU_WS   = _cfg.NAV_IMU_WS  if _cfg else "ws://localhost:8766"
DEFAULT_RTK_WS   = _cfg.NAV_RTK_WS  if _cfg else "ws://localhost:8776"
DEFAULT_HZ       = _cfg.NAV_HZ      if _cfg else 10.0


def _setup_logger() -> logging.Logger:
    py_name = Path(__file__).stem
    log_file = Path(__file__).parent / f"{py_name}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[
            logging.FileHandler(log_file, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    return logging.getLogger(__name__)


logger = _setup_logger()


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════════

class ImuWsClient:
    """Subscribe to imu_bridge WS and store the latest frame."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url    = ws_url
        self._latest: dict = {}
        self._lock   = threading.Lock()
        self._ws     = None

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    async def send(self, msg: dict) -> None:
        if self._ws is not None:
            try:
                await self._ws.send(json.dumps(msg))
            except Exception as exc:
                logger.warning("ImuWsClient: send failed: %s", exc)

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("ImuWsClient: connected to %s", self._url)
                    self._ws = ws
                    # ── INPUT ──────────────────────────────────────────────
                    async for msg in ws:
                    # ───────────────────────────────────────────────────────
                        try:
                            with self._lock:
                                self._latest = json.loads(msg)
                        except json.JSONDecodeError as exc:
                            logger.warning("ImuWsClient: JSON parse error: %s", exc)
            except Exception as exc:
                logger.warning("ImuWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S)
            finally:
                self._ws = None
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class RtkWsClient:
    """Subscribe to rtk_bridge WS and store the latest frame."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url    = ws_url
        self._latest: dict = {}
        self._lock   = threading.Lock()

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("RtkWsClient: connected to %s", self._url)
                    # ── INPUT ──────────────────────────────────────────────
                    async for msg in ws:
                    # ───────────────────────────────────────────────────────
                        try:
                            with self._lock:
                                self._latest = json.loads(msg)
                        except json.JSONDecodeError as exc:
                            logger.warning("RtkWsClient: JSON parse error: %s", exc)
            except Exception as exc:
                logger.warning("RtkWsClient: %s — retrying in %.0fs", exc, self.RECONNECT_DELAY_S)
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class BroadcastLoop:
    """Merge latest IMU + RTK frames and push to queue at N Hz."""

    def __init__(self, imu: ImuWsClient, rtk: RtkWsClient, queue: asyncio.Queue, hz: float) -> None:
        self._imu    = imu
        self._rtk    = rtk
        self._queue  = queue
        self._period = 1.0 / max(hz, 0.5)

    async def run(self) -> None:
        while True:
            payload = json.dumps({
                "type":    "nav_frame",
                "version": WS_MSG_VERSION,
                "imu":     self._imu.latest,
                "rtk":     self._rtk.latest,
            })
            try:
                self._queue.put_nowait(payload)
            except asyncio.QueueFull:
                logger.debug("BroadcastLoop: queue full, dropping frame")
            await asyncio.sleep(self._period)


class NavWebSocketServer:
    """Broadcast nav_frame to browser clients."""

    def __init__(self, port: int, queue: asyncio.Queue, imu_client: ImuWsClient) -> None:
        self._port       = port
        self._queue      = queue
        self._imu_client = imu_client
        self._clients: set = set()

    async def broadcast(self) -> None:
        while True:
            message = await self._queue.get()
            if not self._clients:
                continue
            dead: set = set()
            for ws in self._clients.copy():
                try:
                    # ── OUTPUT ─────────────────────────────────────────────
                    await ws.send(message)
                    # ───────────────────────────────────────────────────────
                except Exception:
                    dead.add(ws)
            self._clients.difference_update(dead)

    async def handle_client(self, websocket) -> None:
        addr = websocket.remote_address
        logger.info("NavWebSocketServer: client connected: %s", addr)
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                    if "set_north_offset" in msg:
                        await self._imu_client.send({"set_north_offset": msg["set_north_offset"]})
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            self._clients.discard(websocket)
            logger.info("NavWebSocketServer: client disconnected: %s", addr)

    async def serve(self) -> None:
        asyncio.create_task(self.broadcast())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("NavWebSocketServer: ws://localhost:%d", self._port)
            await asyncio.Future()


class HttpFileServer:
    """Serve web_static/ over HTTP in a daemon thread."""

    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port       = port

    def run(self) -> None:
        static_dir = self._static_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)
            def log_message(self, format, *args):
                logger.debug("HTTP %s - %s", self.address_string(), format % args)

        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self._port), _Handler) as httpd:
                logger.info("HttpFileServer: serving %s on port %d", self._static_dir, self._port)
                httpd.serve_forever()
        except Exception as exc:
            logger.error("HttpFileServer: failed on port %d: %s", self._port, exc)


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════════

class NavBridge:
    """Top-level orchestrator: subscribe to IMU + RTK, serve the nav dashboard."""

    def __init__(self, nav_port: int, imu_ws_url: str, rtk_ws_url: str, hz: float, static_dir: Path) -> None:
        self._http_port  = nav_port
        self._ws_port    = nav_port + 1
        self._imu_ws_url = imu_ws_url
        self._rtk_ws_url = rtk_ws_url
        self._hz         = hz
        self._static_dir = static_dir

    def run(self) -> None:
        logger.info("NavBridge | http=:%d  ws=:%d  hz=%.1f  imu=%s  rtk=%s",
                    self._http_port, self._ws_port, self._hz, self._imu_ws_url, self._rtk_ws_url)

        if self._static_dir.exists():
            threading.Thread(
                target=HttpFileServer(self._static_dir, self._http_port).run,
                daemon=True, name="http-server",
            ).start()

        logger.info("Open browser at http://localhost:%d", self._http_port)
        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("NavBridge: stopped by user.")

    async def _run_async(self) -> None:
        queue      = asyncio.Queue(maxsize=20)
        imu_client = ImuWsClient(self._imu_ws_url)
        rtk_client = RtkWsClient(self._rtk_ws_url)
        broadcast  = BroadcastLoop(imu_client, rtk_client, queue, self._hz)
        ws_server  = NavWebSocketServer(self._ws_port, queue, imu_client)

        await asyncio.gather(
            imu_client.run(),
            rtk_client.run(),
            broadcast.run(),
            ws_server.serve(),
        )


if __name__ == "__main__":
    NavBridge(
        nav_port    = DEFAULT_NAV_PORT,
        imu_ws_url  = DEFAULT_IMU_WS,
        rtk_ws_url  = DEFAULT_RTK_WS,
        hz          = DEFAULT_HZ,
        static_dir  = Path(__file__).parent / "web_static",
    ).run()
