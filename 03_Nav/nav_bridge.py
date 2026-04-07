"""
nav_bridge.py — Navigation aggregator: merges IMU + RTK streams into a single
WebSocket feed for the Nav dashboard.

Data flow:
    imu_bridge(WS) ──→ ImuWsClient.latest ─┐
                                             ├─ NavLoop(10Hz) → NavController → NavPayload → queue
    rtk_bridge(WS) ──→ RtkWsClient.latest ─┘                                                  │
                                                                                                ↓
                                                                        NavWebSocketServer.broadcast() → browser

Usage:
    python nav_bridge.py --nav-port 8785 --imu-ws ws://localhost:8766 --rtk-ws ws://localhost:8776
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

import argparse
import asyncio
import json
import logging
import math
import socketserver
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import websockets


# ── Logger setup ─────────────────────────────────────────────────────────────

def _setup_logger() -> logging.Logger:
    """Configure module-level logger; output to nav_bridge.log and stderr."""
    py_name = Path(__file__).stem
    output_path = Path(__file__).parent
    log_file = output_path / f"{py_name}.log"

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


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class NavPayload:
    """
    Aggregated payload sent to the browser each tick.

    Carries the latest IMU snapshot, RTK snapshot, and computed navigation state.
    """

    imu: dict = field(default_factory=dict)
    rtk: dict = field(default_factory=dict)
    nav: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Serialize to broadcast-ready dict."""
        return {
            "imu": self.imu,
            "rtk": self.rtk,
            "nav": self.nav,
        }


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE
# ═════════════════════════════════════════════════════════════════════════════

class GeoMath:
    """Static helper methods for geodesic calculations."""

    EARTH_RADIUS_M = 6_371_000

    @staticmethod
    def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Return great-circle distance in metres between two WGS-84 points."""
        to_rad = math.radians
        d_lat = to_rad(lat2 - lat1)
        d_lon = to_rad(lon2 - lon1)
        a = (
            math.sin(d_lat / 2) ** 2
            + math.cos(to_rad(lat1)) * math.cos(to_rad(lat2))
            * math.sin(d_lon / 2) ** 2
        )
        return 2 * GeoMath.EARTH_RADIUS_M * math.asin(math.sqrt(a))


class NavController:
    """
    Navigation state machine: manages waypoints, heading computation, and
    target-reaching logic.

        Waypoints are loaded from the browser; heading is resolved with priority:
            1) dual-RTK heading from rtk_bridge
            2) IMU heading payload (fallback)
            3) IMU yaw conversion fallback (legacy)
    """

    REACH_TOLERANCE_M = 0.5  # default distance threshold

    def __init__(self) -> None:
        self._waypoints: list[dict] = []
        self._state: str = "idle"  # "idle" | "navigating"
        self._reached_count: int = 0

    # ── Public API ─────────────────────────────────────────────────────────

    def load_waypoints(self, wps: list[dict]) -> None:
        """Replace the current waypoint list with a new one from the browser."""
        self._waypoints = []
        for wp in wps:
            self._waypoints.append({
                "id": wp.get("id", ""),
                "lat": float(wp.get("lat", 0)),
                "lon": float(wp.get("lon", 0)),
                "tolerance_m": float(wp.get("tolerance_m", self.REACH_TOLERANCE_M)),
                "reached": False,
            })
        self._reached_count = 0
        self._state = "idle"
        logger.info("NavController: loaded %d waypoints", len(self._waypoints))

    def start_nav(self) -> None:
        """Begin navigation toward waypoints."""
        if not self._waypoints:
            logger.warning("NavController: start_nav called with no waypoints")
            return
        self._state = "navigating"
        logger.info("NavController: navigation started")

    def stop_nav(self) -> None:
        """Pause navigation."""
        self._state = "idle"
        logger.info("NavController: navigation stopped")

    def compute_nav(self, imu: dict, rtk: dict) -> dict:
        """
        CORE — Compute navigation state from latest IMU and RTK data.

        Receives latest sensor snapshots from NavLoop and returns a dict
        with heading, state, target info, and progress — ready for
        NavPayload assembly and broadcast to the browser (OUTPUT).

        Steps:
          1. Read heading from IMU payload (imu.heading.deg/dir) when available
          2. If navigating, find current target and check reach distance
          3. Return nav dict
        """
        heading_deg = None
        heading_dir = None
        heading_source = None

        # 1) Prefer dual-RTK heading when available.
        if rtk.get("heading_valid") and rtk.get("heading_deg") is not None:
            heading_deg = float(rtk.get("heading_deg"))
            heading_dir = rtk.get("heading_dir")
            heading_source = "rtk"

        # 2) Fallback to IMU heading payload.
        if heading_deg is None:
            heading = imu.get("heading", {})
            if heading.get("deg") is not None:
                heading_deg = float(heading.get("deg"))
                heading_dir = heading.get("dir")
                heading_source = "imu"

        # 3) Legacy fallback for older IMU payloads without heading block.
        if heading_deg is None:
            euler = imu.get("euler", {})
            yaw = euler.get("yaw")
            if yaw is not None:
                heading_deg = round((90 - yaw) % 360, 2)
                heading_source = "imu_yaw"

        if heading_deg is not None and not heading_dir:
            heading_dir = self._heading_dir(heading_deg)

        # Target logic
        current_target = None
        target_distance_m = None

        if self._state == "navigating":
            lat = rtk.get("lat")
            lon = rtk.get("lon")
            if lat is not None and lon is not None:
                for wp in self._waypoints:
                    if wp["reached"]:
                        continue
                    d = GeoMath.haversine(lat, lon, wp["lat"], wp["lon"])
                    current_target = wp
                    target_distance_m = round(d, 2)
                    if d <= wp["tolerance_m"]:
                        wp["reached"] = True
                        self._reached_count += 1
                        logger.info(
                            "NavController: waypoint %s reached (%.2f m)",
                            wp["id"], d,
                        )
                        # Move to next unreached
                        continue
                    break  # stop at first unreached

        return {
            "heading_deg": heading_deg,
            "heading_dir": heading_dir,
            "heading_source": heading_source,
            "state": self._state,
            "current_target": current_target,
            "target_distance_m": target_distance_m,
            "reached_count": self._reached_count,
            "total_waypoints": len(self._waypoints),
        }

    @staticmethod
    def _heading_dir(heading_deg: float) -> str:
        labels = [
            "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
        ]
        idx = int((heading_deg + 11.25) // 22.5) % 16
        return labels[idx]


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class ImuWsClient:
    """
    WebSocket client that connects to imu_bridge and stores the latest frame.

    Also supports sending control messages (e.g. set_north_offset) back
    to imu_bridge.
    """

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._ws = None

    @property
    def latest(self) -> dict:
        """Thread-safe access to the most recent IMU frame."""
        with self._lock:
            return dict(self._latest)

    async def send(self, msg: dict) -> None:
        """Send a JSON message to imu_bridge (e.g. set_north_offset)."""
        if self._ws is not None:
            try:
                await self._ws.send(json.dumps(msg))
            except Exception as exc:
                logger.warning("ImuWsClient: send failed: %s", exc)

    async def run(self) -> None:
        """Connect to imu_bridge WS and continuously read frames."""
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("ImuWsClient: connected to %s", self._url)
                    self._ws = ws
                    # ── INPUT ──────────────────────────────────────────────────────────
                    # IMU JSON frames arrive here from imu_bridge via WebSocket.
                    # If IMU data looks wrong, start debugging from this line.
                    async for msg in ws:
                    # ───────────────────────────────────────────────────────────────────
                        try:
                            data = json.loads(msg)
                            with self._lock:
                                self._latest = data
                        except json.JSONDecodeError as exc:
                            logger.warning("ImuWsClient: JSON parse error: %s", exc)
            except Exception as exc:
                logger.warning(
                    "ImuWsClient: connection to %s failed: %s — retrying in %.0fs",
                    self._url, exc, self.RECONNECT_DELAY_S,
                )
            finally:
                self._ws = None
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class NullImuClient:
    """No-op IMU client used when running NavBridge in RTK-only mode."""

    @property
    def latest(self) -> dict:
        return {}

    async def send(self, msg: dict) -> None:
        _ = msg


class RtkWsClient:
    """
    WebSocket client that connects to rtk_bridge and stores the latest frame.
    """

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()

    @property
    def latest(self) -> dict:
        """Thread-safe access to the most recent RTK frame."""
        with self._lock:
            return dict(self._latest)

    async def run(self) -> None:
        """Connect to rtk_bridge WS and continuously read frames."""
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("RtkWsClient: connected to %s", self._url)
                    # ── INPUT ──────────────────────────────────────────────────────────
                    # RTK JSON frames arrive here from rtk_bridge via WebSocket.
                    # If RTK data looks wrong, start debugging from this line.
                    async for msg in ws:
                    # ───────────────────────────────────────────────────────────────────
                        try:
                            data = json.loads(msg)
                            with self._lock:
                                self._latest = data
                        except json.JSONDecodeError as exc:
                            logger.warning("RtkWsClient: JSON parse error: %s", exc)
            except Exception as exc:
                logger.warning(
                    "RtkWsClient: connection to %s failed: %s — retrying in %.0fs",
                    self._url, exc, self.RECONNECT_DELAY_S,
                )
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class NavLoop:
    """
    Asyncio coroutine: aggregate latest IMU + RTK data at N Hz, run
    NavController, and push NavPayload into the broadcast queue.
    """

    def __init__(
        self,
        imu_client: ImuWsClient,
        rtk_client: RtkWsClient,
        nav_controller: NavController,
        queue: asyncio.Queue,
        hz: float,
    ) -> None:
        self._imu = imu_client
        self._rtk = rtk_client
        self._nav = nav_controller
        self._queue = queue
        self._period = 1.0 / max(hz, 0.5)

    async def run(self) -> None:
        """Main loop: sample, compute, enqueue."""
        while True:
            imu = self._imu.latest
            rtk = self._rtk.latest
            nav = self._nav.compute_nav(imu, rtk)

            payload = NavPayload(imu=imu, rtk=rtk, nav=nav)
            try:
                self._queue.put_nowait(json.dumps(payload.to_dict()))
            except asyncio.QueueFull:
                logger.debug("NavLoop: queue full, dropping frame")

            await asyncio.sleep(self._period)


class NavWebSocketServer:
    """
    WebSocket server for browser clients.

    Broadcasts aggregated NavPayload messages from the queue and accepts
    control commands (set_north_offset, load_waypoints, start/stop_nav).
    """

    def __init__(
        self,
        port: int,
        queue: asyncio.Queue,
        on_client_message=None,
    ) -> None:
        self._port = port
        self._queue = queue
        self._clients: set = set()
        self._on_client_message = on_client_message

    async def broadcast(self) -> None:
        """Continuously dequeue messages and forward to all connected clients."""
        while True:
            message: str = await self._queue.get()
            if not self._clients:
                continue

            dead: set = set()
            for ws in self._clients.copy():
                try:
                    # ── OUTPUT ─────────────────────────────────────────────────────────
                    # Aggregated JSON payload exits the program here to the browser.
                    # If the browser receives wrong data, start debugging here.
                    await ws.send(message)
                    # ───────────────────────────────────────────────────────────────────
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
                except Exception as exc:
                    logger.warning("NavWebSocketServer: error sending to client: %s", exc)
                    dead.add(ws)

            self._clients.difference_update(dead)

    async def handle_client(self, websocket) -> None:
        """Handle a single browser WebSocket connection lifecycle."""
        addr = websocket.remote_address
        logger.info("NavWebSocketServer: client connected: %s", addr)
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                if self._on_client_message is not None:
                    try:
                        await self._on_client_message(raw)
                    except Exception as exc:
                        logger.warning(
                            "NavWebSocketServer: error handling message from %s: %s",
                            addr, exc,
                        )
        except websockets.exceptions.ConnectionClosed:
            logger.debug("NavWebSocketServer: connection closed normally for %s", addr)
        except Exception as exc:
            logger.warning("NavWebSocketServer: handler error for %s: %s", addr, exc)
        finally:
            self._clients.discard(websocket)
            logger.info("NavWebSocketServer: client disconnected: %s", addr)

    async def serve(self) -> None:
        """Start the WebSocket server and the broadcast coroutine."""
        logger.info("NavWebSocketServer: starting on ws://localhost:%d", self._port)
        asyncio.create_task(self.broadcast())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("NavWebSocketServer: running.")
            await asyncio.Future()  # run forever


class HttpFileServer:
    """
    Serve static files from a directory over HTTP.

    Runs in a dedicated daemon thread.
    """

    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port = port

    def run(self) -> None:
        """Entry point for the HTTP server thread."""
        static_dir = self._static_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def log_message(self, format, *args):
                logger.debug("HTTP %s - %s", self.address_string(), format % args)

        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self._port), _Handler) as httpd:
                logger.info(
                    "HttpFileServer: serving %s on port %d",
                    self._static_dir, self._port,
                )
                httpd.serve_forever()
        except Exception as exc:
            logger.error("HttpFileServer: failed to start on port %d: %s", self._port, exc)


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

class NavBridge:
    """
    Top-level orchestrator.  Assembles and starts all components:
      1. ImuWsClient     — reads from imu_bridge WS
      2. RtkWsClient     — reads from rtk_bridge WS
      3. NavController   — navigation state machine
      4. NavLoop          — aggregation coroutine (10 Hz)
      5. HttpFileServer   — serves web_static/
      6. NavWebSocketServer — broadcasts to browser clients
    """

    def __init__(
        self,
        nav_port: int,
        imu_ws_url: str | None,
        rtk_ws_url: str,
        hz: float,
        static_dir: Path,
    ) -> None:
        self._http_port = nav_port
        self._ws_port = nav_port + 1
        self._imu_ws_url = imu_ws_url
        self._rtk_ws_url = rtk_ws_url
        self._hz = hz
        self._static_dir = static_dir
        self._imu_client: ImuWsClient | None = None
        self._nav_controller: NavController | None = None

    def run(self) -> None:
        """Synchronous entry point — blocks until Ctrl-C."""
        logger.info(
            "NavBridge starting | http=:%d  ws=:%d  hz=%.1f  imu=%s  rtk=%s",
            self._http_port, self._ws_port, self._hz,
            self._imu_ws_url if self._imu_ws_url else "disabled",
            self._rtk_ws_url,
        )

        # HTTP file server
        if self._static_dir.exists():
            http_server = HttpFileServer(self._static_dir, self._http_port)
            threading.Thread(
                target=http_server.run, daemon=True, name="http-server"
            ).start()
        else:
            logger.warning(
                "web_static directory not found at %s; HTTP server not started.",
                self._static_dir,
            )

        url = f"http://localhost:{self._http_port}"
        logger.info("Open browser at %s", url)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("NavBridge: stopped by user.")

    async def _run_async(self) -> None:
        """Start all async components."""
        queue: asyncio.Queue = asyncio.Queue(maxsize=20)

        # Sensor clients
        if self._imu_ws_url:
            self._imu_client = ImuWsClient(self._imu_ws_url)
            imu_run_task = self._imu_client.run()
        else:
            self._imu_client = NullImuClient()
            imu_run_task = asyncio.sleep(3600 * 24 * 365 * 100)
        rtk_client = RtkWsClient(self._rtk_ws_url)

        # Navigation controller
        self._nav_controller = NavController()

        # Aggregation loop
        nav_loop = NavLoop(
            imu_client=self._imu_client,
            rtk_client=rtk_client,
            nav_controller=self._nav_controller,
            queue=queue,
            hz=self._hz,
        )

        # WebSocket server
        ws_server = NavWebSocketServer(
            port=self._ws_port,
            queue=queue,
            on_client_message=self._handle_client_message,
        )

        await asyncio.gather(
            imu_run_task,
            rtk_client.run(),
            nav_loop.run(),
            ws_server.serve(),
        )

    async def _handle_client_message(self, raw: str) -> None:
        """Parse and dispatch JSON control messages from the browser."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning("NavBridge: invalid JSON from browser: %s | raw: %s", exc, raw[:120])
            return

        if "set_north_offset" in msg:
            if self._imu_client is not None:
                await self._imu_client.send({"set_north_offset": msg["set_north_offset"]})
                logger.info("NavBridge: forwarded set_north_offset=%s to imu_bridge", msg["set_north_offset"])

        if "load_waypoints" in msg:
            if self._nav_controller is not None:
                self._nav_controller.load_waypoints(msg["load_waypoints"])

        if "start_nav" in msg:
            if self._nav_controller is not None:
                self._nav_controller.start_nav()

        if "stop_nav" in msg:
            if self._nav_controller is not None:
                self._nav_controller.stop_nav()


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Navigation aggregator: merges IMU + RTK into a single WS feed"
    )
    parser.add_argument(
        "--nav-port",
        type=int,
        default=_cfg.NAV_WS_PORT if _cfg else 8785,
        help="HTTP port for web UI; WebSocket uses nav-port+1",
    )
    parser.add_argument(
        "--imu-ws",
        default=_cfg.NAV_IMU_WS if _cfg else "ws://localhost:8766",
        help="WebSocket URL for imu_bridge",
    )
    parser.add_argument(
        "--no-imu",
        action="store_true",
        help="Disable IMU WebSocket and run NavBridge in RTK-only mode",
    )
    parser.add_argument(
        "--rtk-ws",
        default=_cfg.NAV_RTK_WS if _cfg else "ws://localhost:8776",
        help="WebSocket URL for rtk_bridge",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=_cfg.NAV_HZ if _cfg else 10.0,
        help="Broadcast rate in Hz",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    NavBridge(
        nav_port=args.nav_port,
        imu_ws_url=None if args.no_imu else args.imu_ws,
        rtk_ws_url=args.rtk_ws,
        hz=args.hz,
        static_dir=Path(__file__).parent / "web_static",
    ).run()
