"""
camera_bridge.py — OAK-D camera MJPEG streaming bridge with WebSocket control.

Data flow:
    OAK-D Camera → FrameSource.get_frame() → MJPEGServer (HTTP multipart) → Browser <img>
                                                    ↑
    Browser → WebSocketServer → _handle_client_message → switch camera / toggle stream
                                                    ↓
              WebSocketServer.broadcast(1Hz) ← CameraPipeline.get_status() → Browser (status)

Usage:
    python camera_bridge.py --ws-port 8815 \
        --cam1-ip 10.95.76.11 --cam2-ip 10.95.76.10 \
        --cam1-stream-port 8080 --cam2-stream-port 8081
    # Browser:     http://localhost:8815  (control panel)
    # MJPEG:       http://localhost:8080  (camera 1 stream)
"""

from __future__ import annotations

import argparse
import asyncio
import http.server
import json
import logging
import socketserver
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import cv2
import numpy as np
import websockets

from plugins import FrameSource, get_plugin, list_plugins


# ── Logger setup ─────────────────────────────────────────────────────────────

def _setup_logger() -> logging.Logger:
    """Configure module-level logger; output to camera_bridge.log and stderr."""
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
class CameraFrame:
    """
    Status payload broadcast to the browser UI at 1 Hz.

    Carries current camera selection, streaming state, and MJPEG endpoints.
    """

    cam_selection: int = 1
    streaming: bool = False
    fps: float = 0.0
    width: int = 0
    height: int = 0
    mjpeg_url_cam1: str = ""
    mjpeg_url_cam2: str = ""
    # ── Plugin fields ──
    active_plugin: str = ""
    active_plugin_config: dict = field(default_factory=dict)
    available_plugins: list = field(default_factory=list)

    def to_dict(self) -> dict:
        """Serialize to broadcast-ready dict."""
        return {
            "cam_selection": self.cam_selection,
            "streaming": self.streaming,
            "fps": round(self.fps, 1),
            "width": self.width,
            "height": self.height,
            "mjpeg_url_cam1": self.mjpeg_url_cam1,
            "mjpeg_url_cam2": self.mjpeg_url_cam2,
            "active_plugin": self.active_plugin,
            "active_plugin_config": self.active_plugin_config,
            "available_plugins": self.available_plugins,
        }


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE
# ═════════════════════════════════════════════════════════════════════════════

class MJPEGServer:
    """
    HTTP MJPEG streaming server.

    Captures frames from a FrameSource in a background thread and serves
    them as a multipart/x-mixed-replace HTTP stream.
    """

    def __init__(
        self, source: FrameSource, port: int, quality: int = 80,
    ) -> None:
        self._source = source
        self._port = port
        self._quality = quality
        self._latest_jpeg: bytes = b""
        self._frame_lock = threading.Lock()
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._httpd: socketserver.TCPServer | None = None
        self._fps_tracker_times: list[float] = []
        self._current_fps: float = 0.0

    @property
    def fps(self) -> float:
        return self._current_fps

    @property
    def streaming(self) -> bool:
        return self._running

    def start(self) -> None:
        """Open the frame source and start capture + HTTP server threads."""
        if self._running:
            return

        try:
            self._source.open()
        except Exception as exc:
            logger.error("MJPEGServer: cannot open source: %s", exc)
            return

        self._running = True

        # Capture thread
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name=f"mjpeg-capture-{self._port}",
        )
        self._capture_thread.start()

        # HTTP server thread
        server_ref = self
        quality = self._quality

        class _StreamHandler(http.server.BaseHTTPRequestHandler):
            def do_GET(self_h):
                self_h.send_response(200)
                self_h.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self_h.send_header("Cache-Control", "no-cache")
                self_h.end_headers()
                try:
                    while server_ref._running:
                        with server_ref._frame_lock:
                            jpeg = server_ref._latest_jpeg
                        if not jpeg:
                            time.sleep(0.01)
                            continue
                        try:
                            self_h.wfile.write(b"--frame\r\n")
                            self_h.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                            self_h.wfile.write(jpeg)
                            self_h.wfile.write(b"\r\n")
                        except (BrokenPipeError, ConnectionResetError):
                            break
                        time.sleep(0.01)
                except Exception:
                    pass

            def log_message(self_h, fmt, *args):
                logger.debug("MJPEG %s - %s", self_h.address_string(), fmt % args)

        def _run_http():
            socketserver.TCPServer.allow_reuse_address = True
            try:
                self._httpd = socketserver.TCPServer(("", self._port), _StreamHandler)
                logger.info("MJPEGServer: MJPEG stream on http://0.0.0.0:%d", self._port)
                self._httpd.serve_forever()
            except Exception as exc:
                logger.error("MJPEGServer: HTTP server error on port %d: %s", self._port, exc)

        threading.Thread(target=_run_http, daemon=True, name=f"mjpeg-http-{self._port}").start()

    def stop(self) -> None:
        """Stop capture and HTTP server, release frame source."""
        self._running = False
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception as exc:
                logger.warning("MJPEGServer: error shutting down HTTP: %s", exc)
            self._httpd = None
        try:
            self._source.close()
        except Exception as exc:
            logger.warning("MJPEGServer: error closing source: %s", exc)
        self._latest_jpeg = b""
        self._current_fps = 0.0
        logger.info("MJPEGServer: stopped on port %d", self._port)

    def _capture_loop(self) -> None:
        """Background thread: continuously grab frames and encode to JPEG."""
        while self._running:
            frame = self._source.get_frame()
            if frame is not None:
                ok, buf = cv2.imencode(
                    ".jpg", frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self._quality],
                )
                if ok:
                    with self._frame_lock:
                        self._latest_jpeg = buf.tobytes()
                    self._tick_fps()
            else:
                time.sleep(0.005)

    def _tick_fps(self) -> None:
        """Track capture frame rate with a rolling window."""
        now = time.monotonic()
        self._fps_tracker_times.append(now)
        if len(self._fps_tracker_times) > 60:
            self._fps_tracker_times = self._fps_tracker_times[-60:]
        if len(self._fps_tracker_times) >= 2:
            elapsed = self._fps_tracker_times[-1] - self._fps_tracker_times[0]
            if elapsed > 0:
                self._current_fps = (len(self._fps_tracker_times) - 1) / elapsed


class CameraPipeline:
    """
    Manages MJPEG server lifecycle and produces status snapshots.

    Wraps one or two MJPEGServer instances and provides start/stop/switch
    control, plus a get_status() method for periodic broadcasts.
    Supports dynamic plugin switching via the plugin registry.
    """

    def __init__(
        self,
        stream_ports: dict[int, int],
        quality: int,
        default_plugin: str = "simple_color",
        default_config: dict | None = None,
        cam_configs: dict[int, dict] | None = None,
    ) -> None:
        self._stream_ports = stream_ports
        self._quality = quality
        self._cam_selection = 1
        self._active_plugin_name = default_plugin
        self._active_plugin_config = default_config or {}
        self._cam_configs = cam_configs or {}
        self._servers: dict[int, MJPEGServer] = {}
        self._lock = threading.Lock()

    @property
    def cam_selection(self) -> int:
        return self._cam_selection

    def start_stream(self, cam_id: int | None = None) -> None:
        """Start the MJPEG stream for the given camera (or current selection)."""
        cam_id = cam_id or self._cam_selection
        with self._lock:
            if cam_id in self._servers:
                return  # already running
            plugin_cls = get_plugin(self._active_plugin_name)
            # Merge per-cam base config with active plugin config
            config = {**self._cam_configs.get(cam_id, {}), **self._active_plugin_config}
            source = plugin_cls(**config)
            port = self._stream_ports.get(cam_id, 8080)
            server = MJPEGServer(source, port, self._quality)
            server.start()
            self._servers[cam_id] = server
            logger.info("CameraPipeline: started camera %d stream on port %d (plugin=%s)",
                        cam_id, port, self._active_plugin_name)

    def stop_stream(self, cam_id: int | None = None) -> None:
        """Stop the MJPEG stream for the given camera (or current selection)."""
        cam_id = cam_id or self._cam_selection
        with self._lock:
            server = self._servers.pop(cam_id, None)
            if server:
                server.stop()
                logger.info("CameraPipeline: stopped camera %d stream", cam_id)

    def switch_camera(self, cam_id: int) -> None:
        """Switch active camera selection."""
        if cam_id not in (1, 2):
            logger.warning("CameraPipeline: invalid cam_id=%d", cam_id)
            return
        self._cam_selection = cam_id
        logger.info("CameraPipeline: switched to camera %d", cam_id)

    def switch_plugin(self, plugin_name: str, config: dict | None = None) -> None:
        """Stop current streams, switch plugin, restart previously active streams."""
        with self._lock:
            was_streaming = list(self._servers.keys())
            for cid in was_streaming:
                self._servers.pop(cid).stop()
            self._active_plugin_name = plugin_name
            if config is not None:
                self._active_plugin_config = config
        # Restart outside lock
        for cid in was_streaming:
            self.start_stream(cid)
        logger.info("CameraPipeline: switched to plugin '%s'", plugin_name)

    def get_status(self) -> CameraFrame:
        """Return current status as a CameraFrame dataclass."""
        with self._lock:
            server = self._servers.get(self._cam_selection)
            streaming = server.streaming if server else False
            fps = server.fps if server else 0.0

        ports = self._stream_ports
        return CameraFrame(
            cam_selection=self._cam_selection,
            streaming=streaming,
            fps=fps,
            width=self._active_plugin_config.get("width", 0),
            height=self._active_plugin_config.get("height", 0),
            mjpeg_url_cam1=f"http://{{host}}:{ports.get(1, 8080)}/",
            mjpeg_url_cam2=f"http://{{host}}:{ports.get(2, 8081)}/",
            active_plugin=self._active_plugin_name,
            active_plugin_config=self._active_plugin_config,
            available_plugins=list_plugins(),
        )

    def shutdown(self) -> None:
        """Stop all running streams."""
        for cam_id in list(self._servers.keys()):
            self.stop_stream(cam_id)


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class WebSocketServer:
    """
    WebSocket server for browser clients.

    Broadcasts CameraFrame status at 1 Hz and accepts control commands
    (start_stream, stop_stream, switch_camera).
    """

    def __init__(
        self,
        port: int,
        pipeline: CameraPipeline,
        on_client_message=None,
    ) -> None:
        self._port = port
        self._pipeline = pipeline
        self._clients: set = set()
        self._on_client_message = on_client_message

    async def broadcast_loop(self) -> None:
        """Broadcast status to all clients at 1 Hz."""
        while True:
            if self._clients:
                status = self._pipeline.get_status()
                message = json.dumps(status.to_dict())
                dead: set = set()
                for ws in self._clients.copy():
                    try:
                        # ── OUTPUT ─────────────────────────────────────────────────────────
                        # Camera status JSON exits the program here to the browser.
                        # If the browser receives wrong data, start debugging here.
                        await ws.send(message)
                        # ───────────────────────────────────────────────────────────────────
                    except websockets.exceptions.ConnectionClosed:
                        dead.add(ws)
                    except Exception as exc:
                        logger.warning("WebSocketServer: error sending to client: %s", exc)
                        dead.add(ws)
                self._clients.difference_update(dead)
            await asyncio.sleep(1.0)

    async def handle_client(self, websocket) -> None:
        """Handle a single browser WebSocket connection lifecycle."""
        addr = websocket.remote_address
        logger.info("WebSocketServer: client connected: %s", addr)
        self._clients.add(websocket)
        try:
            async for raw in websocket:
                if self._on_client_message is not None:
                    try:
                        self._on_client_message(raw)
                    except Exception as exc:
                        logger.warning(
                            "WebSocketServer: error handling message from %s: %s",
                            addr, exc,
                        )
        except websockets.exceptions.ConnectionClosed:
            logger.debug("WebSocketServer: connection closed normally for %s", addr)
        except Exception as exc:
            logger.warning("WebSocketServer: handler error for %s: %s", addr, exc)
        finally:
            self._clients.discard(websocket)
            logger.info("WebSocketServer: client disconnected: %s", addr)

    async def serve(self) -> None:
        """Start the WebSocket server and the broadcast coroutine."""
        logger.info("WebSocketServer: starting on ws://localhost:%d", self._port)
        asyncio.create_task(self.broadcast_loop())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("WebSocketServer: running.")
            await asyncio.Future()  # run forever


class HttpFileServer:
    """
    Serve static files from web_static/ over HTTP.

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

            def log_message(self, fmt, *args):
                logger.debug("HTTP %s - %s", self.address_string(), fmt % args)

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

class CameraBridge:
    """
    Top-level orchestrator.  Assembles and starts all components:
      1. CameraPipeline   — manages MJPEG servers + camera lifecycle
      2. HttpFileServer    — serves web_static/ on HTTP port
      3. WebSocketServer   — control/status WS on HTTP port + 1
    """

    def __init__(
        self,
        ws_port: int,
        cam1_ip: str | None,
        cam2_ip: str | None,
        cam_selection: int,
        cam1_stream_port: int,
        cam2_stream_port: int,
        fps: int,
        width: int,
        height: int,
        quality: int,
        static_dir: Path,
        plugin: str = "simple_color",
    ) -> None:
        self._http_port = ws_port
        self._ws_port = ws_port + 1
        self._cam_selection = cam_selection
        self._static_dir = static_dir

        # Build per-camera base configs (device_ip per cam)
        cam_configs: dict[int, dict] = {}
        if cam1_ip:
            cam_configs[1] = {"device_ip": cam1_ip}
        if cam2_ip:
            cam_configs[2] = {"device_ip": cam2_ip}

        self._pipeline = CameraPipeline(
            stream_ports={1: cam1_stream_port, 2: cam2_stream_port},
            quality=quality,
            default_plugin=plugin,
            default_config={"fps": fps, "width": width, "height": height},
            cam_configs=cam_configs,
        )

    def run(self) -> None:
        """Synchronous entry point — blocks until Ctrl-C."""
        logger.info(
            "CameraBridge starting | http=:%d ws=:%d",
            self._http_port, self._ws_port,
        )

        # HTTP file server
        if self._static_dir.exists():
            http_server = HttpFileServer(self._static_dir, self._http_port)
            threading.Thread(
                target=http_server.run, daemon=True, name="http-server",
            ).start()
        else:
            logger.warning(
                "web_static directory not found at %s; HTTP server not started.",
                self._static_dir,
            )

        # Auto-start selected camera stream
        self._pipeline.switch_camera(self._cam_selection)
        self._pipeline.start_stream(self._cam_selection)

        url = f"http://localhost:{self._http_port}"
        logger.info("Open browser at %s", url)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("CameraBridge: stopped by user.")
        finally:
            self._pipeline.shutdown()

    async def _run_async(self) -> None:
        """Start all async components."""
        ws_server = WebSocketServer(
            port=self._ws_port,
            pipeline=self._pipeline,
            on_client_message=self._handle_client_message,
        )
        await ws_server.serve()

    def _handle_client_message(self, raw: str) -> None:
        """Parse and dispatch JSON control messages from the browser."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning(
                "CameraBridge: invalid JSON from browser: %s | raw: %s",
                exc, raw[:120],
            )
            return

        msg_type = msg.get("type", "")

        if msg_type == "start_stream":
            cam_id = msg.get("cam_id", self._pipeline.cam_selection)
            self._pipeline.start_stream(cam_id)

        elif msg_type == "stop_stream":
            cam_id = msg.get("cam_id", self._pipeline.cam_selection)
            self._pipeline.stop_stream(cam_id)

        elif msg_type == "switch_camera":
            cam_id = msg.get("cam_id", 1)
            self._pipeline.switch_camera(cam_id)

        elif msg_type == "switch_plugin":
            plugin_name = msg.get("plugin_name", "")
            config = msg.get("config")
            try:
                self._pipeline.switch_plugin(plugin_name, config)
            except KeyError as exc:
                logger.warning("CameraBridge: switch_plugin failed: %s", exc)

        elif msg_type == "update_plugin_config":
            config = msg.get("config", {})
            self._pipeline.switch_plugin(
                self._pipeline._active_plugin_name, config,
            )

        else:
            logger.debug("CameraBridge: unknown message type: %s", msg_type)


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OAK-D camera MJPEG streaming bridge with WebSocket control"
    )
    parser.add_argument(
        "--ws-port", type=int, default=8815,
        help="HTTP port for web UI; WebSocket uses ws-port+1 (default: 8815)",
    )
    parser.add_argument(
        "--cam1-ip", default=None,
        help="OAK-D camera 1 IP address (default: None = USB auto-detect)",
    )
    parser.add_argument(
        "--cam2-ip", default=None,
        help="OAK-D camera 2 IP address (default: None)",
    )
    parser.add_argument(
        "--cam-selection", type=int, default=1, choices=[1, 2],
        help="Initial camera selection (default: 1)",
    )
    parser.add_argument(
        "--cam1-stream-port", type=int, default=8080,
        help="MJPEG stream port for camera 1 (default: 8080)",
    )
    parser.add_argument(
        "--cam2-stream-port", type=int, default=8081,
        help="MJPEG stream port for camera 2 (default: 8081)",
    )
    parser.add_argument("--fps", type=int, default=30, help="Camera FPS (default: 30)")
    parser.add_argument("--width", type=int, default=1280, help="Frame width (default: 1280)")
    parser.add_argument("--height", type=int, default=720, help="Frame height (default: 720)")
    parser.add_argument(
        "--mjpeg-quality", type=int, default=80,
        help="JPEG encoding quality 1-100 (default: 80)",
    )
    parser.add_argument(
        "--plugin", default="simple_color",
        help="Initial plugin name (default: simple_color)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    CameraBridge(
        ws_port=args.ws_port,
        cam1_ip=args.cam1_ip,
        cam2_ip=args.cam2_ip,
        cam_selection=args.cam_selection,
        cam1_stream_port=args.cam1_stream_port,
        cam2_stream_port=args.cam2_stream_port,
        fps=args.fps,
        width=args.width,
        height=args.height,
        quality=args.mjpeg_quality,
        static_dir=Path(__file__).parent / "web_static",
        plugin=args.plugin,
    ).run()
