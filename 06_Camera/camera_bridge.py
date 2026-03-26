"""
camera_bridge.py — OAK-D camera MJPEG streaming bridge with WebSocket control.

Architecture:
    CameraDevice  — opens one depthai pipeline at startup, holds ALL camera
                    nodes (RGB, Left, Right, StereoDepth).  Never restarted
                    except from Advanced Settings (fps/resolution change).

    MJPEGServer   — capture loop calls device.get_frames() then proc.process().
                    Supports atomic processor swap (set_processor) with zero
                    stream downtime.

    CameraPipeline — owns one CameraDevice + one MJPEGServer per camera.
                     Plugin switch = processor swap only.  Camera never
                     restarted on plugin change.

Data flow:
    OAK-D → CameraDevice.get_frames() → proc.process(frames) → MJPEGServer
    → HTTP multipart stream → Browser <img>

    Browser → WebSocket → _handle_client_message → switch_plugin (instant)
                                                  → restart_cameras (rare)
    WebSocket ← 1 Hz broadcast ← CameraPipeline.get_status()

Usage:
    python camera_bridge.py --cam1-ip 10.95.76.11
    # Browser:  http://localhost:8815
    # MJPEG:    http://localhost:8080
"""

from __future__ import annotations

import argparse
import asyncio
import http.server
import importlib
import json
import logging
import os
import socketserver
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import cv2
import numpy as np
import websockets

try:
    _plugins_mod = importlib.import_module("plugins")
except ModuleNotFoundError:
    _plugins_mod = importlib.import_module(f"{__package__}.plugins")

get_processor = _plugins_mod.get_processor
list_plugins  = _plugins_mod.list_plugins

try:
    import depthai as dai
except ImportError:
    dai = None


class _ThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


# ── Logger setup ─────────────────────────────────────────────────────────────

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


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class CameraFrame:
    """Status payload broadcast to the browser UI at 1 Hz."""

    cam_selection: int = 1
    streaming: bool = False
    fps: float = 0.0
    width: int = 0
    height: int = 0
    mjpeg_url_cam1: str = ""
    mjpeg_url_cam2: str = ""
    cam1_clients: int = 0
    cam2_clients: int = 0
    cam1_streaming: bool = False
    cam2_streaming: bool = False
    cam1_fps: float = 0.0
    cam2_fps: float = 0.0
    active_plugin: str = ""
    active_plugin_config: dict = field(default_factory=dict)
    available_plugins: list = field(default_factory=list)
    available_streams: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "cam_selection": self.cam_selection,
            "streaming": self.streaming,
            "fps": round(self.fps, 1),
            "width": self.width,
            "height": self.height,
            "mjpeg_url_cam1": self.mjpeg_url_cam1,
            "mjpeg_url_cam2": self.mjpeg_url_cam2,
            "cam1_clients": self.cam1_clients,
            "cam2_clients": self.cam2_clients,
            "cam1_streaming": self.cam1_streaming,
            "cam2_streaming": self.cam2_streaming,
            "cam1_fps": round(self.cam1_fps, 1),
            "cam2_fps": round(self.cam2_fps, 1),
            "active_plugin": self.active_plugin,
            "active_plugin_config": self.active_plugin_config,
            "available_plugins": self.available_plugins,
            "available_streams": self.available_streams,
        }


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — CAMERA DEVICE
# ═════════════════════════════════════════════════════════════════════════════

# Camera socket constants (depthai v3)
_RGB_SOCKET   = "CAM_A"
_LEFT_SOCKET  = "CAM_B"
_RIGHT_SOCKET = "CAM_C"


class CameraDevice:
    """
    One depthai pipeline per physical OAK-D camera.

    Opened once at startup.  Holds all camera nodes (RGB, Left, Right,
    StereoDepth) so that plugins can access any stream without restarting.
    Only close()/open() again when fps or resolution changes.
    """

    def __init__(
        self,
        device_ip: str | None,
        fps: int,
        width: int,
        height: int,
        enable_stereo: bool = False,
    ) -> None:
        self._device_ip = device_ip
        self._fps = fps
        self._width = width
        self._height = height
        self._enable_stereo = enable_stereo

        self._device = None
        self._pipeline = None
        self._queues: dict[str, object] = {}

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def open(self) -> None:
        """Create depthai device, build pipeline with all nodes, start."""
        if dai is None:
            raise RuntimeError("depthai library not installed")

        try:
            if self._device_ip:
                dev_info = dai.DeviceInfo(self._device_ip)
                self._device = dai.Device(dev_info)
            else:
                self._device = dai.Device()

            self._pipeline = dai.Pipeline(self._device)
            self._queues = {}

            # ── RGB camera ────────────────────────────────────────────────
            cam_rgb = self._pipeline.create(dai.node.Camera).build(
                boardSocket=getattr(dai.CameraBoardSocket, _RGB_SOCKET)
            )
            rgb_out = cam_rgb.requestOutput(
                size=(self._width, self._height),
                fps=float(self._fps),
            )
            self._queues["rgb"] = rgb_out.createOutputQueue(maxSize=1, blocking=False)

            # ── Stereo cameras (optional) ─────────────────────────────────
            if self._enable_stereo:
                cam_left = self._pipeline.create(dai.node.Camera).build(
                    boardSocket=getattr(dai.CameraBoardSocket, _LEFT_SOCKET)
                )
                cam_right = self._pipeline.create(dai.node.Camera).build(
                    boardSocket=getattr(dai.CameraBoardSocket, _RIGHT_SOCKET)
                )

                stereo = self._pipeline.create(dai.node.StereoDepth)
                stereo.setExtendedDisparity(True)

                left_out = cam_left.requestOutput(size=(640, 400), fps=float(self._fps))
                right_out = cam_right.requestOutput(size=(640, 400), fps=float(self._fps))
                left_out.link(stereo.left)
                right_out.link(stereo.right)

                self._queues["depth"] = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
                self._queues["disparity"] = stereo.disparity.createOutputQueue(maxSize=1, blocking=False)

            self._pipeline.start()
            logger.info(
                "CameraDevice opened: ip=%s %dx%d@%dfps stereo=%s streams=%s",
                self._device_ip, self._width, self._height, self._fps,
                self._enable_stereo, list(self._queues.keys()),
            )
        except Exception as exc:
            logger.error("CameraDevice: failed to open: %s", exc)
            self.close()
            raise

    def close(self) -> None:
        """Stop pipeline and release device."""
        self._queues = {}
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as exc:
                logger.warning("CameraDevice: error stopping pipeline: %s", exc)
            self._pipeline = None
        if self._device is not None:
            try:
                self._device.close()
            except Exception as exc:
                logger.warning("CameraDevice: error closing device: %s", exc)
            self._device = None
        logger.info("CameraDevice closed: ip=%s", self._device_ip)

    # ── Frame access ──────────────────────────────────────────────────────────

    def get_frames(self, stream_names: list[str]) -> dict[str, np.ndarray | None]:
        """Non-blocking grab for requested streams. Returns dict of BGR arrays."""
        result: dict[str, np.ndarray | None] = {}
        for name in stream_names:
            q = self._queues.get(name)
            if q is None:
                result[name] = None
                continue
            try:
                pkt = q.tryGet()
                if pkt is not None:
                    frame = pkt.getCvFrame()
                    # depth/disparity frames are single-channel; convert for display
                    if name in ("depth", "disparity") and frame.ndim == 2:
                        frame = cv2.applyColorMap(
                            cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U),
                            cv2.COLORMAP_JET,
                        )
                    result[name] = frame
                else:
                    result[name] = None
            except Exception as exc:
                logger.warning("CameraDevice: get_frames('%s') error: %s", name, exc)
                result[name] = None
        return result

    def available_streams(self) -> list[str]:
        return list(self._queues.keys())

    @property
    def is_open(self) -> bool:
        return self._pipeline is not None


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — MJPEG SERVER
# ═════════════════════════════════════════════════════════════════════════════

class MJPEGServer:
    """
    HTTP MJPEG streaming server backed by a CameraDevice.

    The capture loop runs as long as _running is True.  Plugin switches
    are zero-downtime: set_processor() atomically swaps the processor.
    The HTTP server (start_http/stop_http) can be toggled independently.
    """

    def __init__(self, device: CameraDevice, port: int, quality: int = 80) -> None:
        self._device = device
        self._port = port
        self._quality = quality
        self._latest_jpeg: bytes = b""
        self._frame_lock = threading.Lock()
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._httpd: _ThreadingTCPServer | None = None
        self._fps_tracker_times: list[float] = []
        self._current_fps: float = 0.0
        self._client_count = 0
        self._client_lock = threading.Lock()
        self._processor = None
        self._processor_lock = threading.Lock()

    @property
    def fps(self) -> float:
        return self._current_fps

    @property
    def streaming(self) -> bool:
        return self._running

    @property
    def client_count(self) -> int:
        with self._client_lock:
            return self._client_count

    def start_http(self) -> None:
        """Start capture thread and HTTP streaming server."""
        if self._running:
            return
        self._running = True

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"mjpeg-capture-{self._port}",
        )
        self._capture_thread.start()

        server_ref = self
        quality = self._quality

        class _StreamHandler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                with server_ref._client_lock:
                    server_ref._client_count += 1
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while server_ref._running:
                        with server_ref._frame_lock:
                            jpeg = server_ref._latest_jpeg
                        if not jpeg:
                            time.sleep(0.01)
                            continue
                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        except (BrokenPipeError, ConnectionResetError):
                            break
                        time.sleep(0.01)
                except Exception:
                    pass
                finally:
                    with server_ref._client_lock:
                        server_ref._client_count = max(0, server_ref._client_count - 1)

            def log_message(self, format, *args):
                logger.debug("MJPEG %s - %s", self.address_string(), format % args)

        def _run_http():
            try:
                self._httpd = _ThreadingTCPServer(("", self._port), _StreamHandler)
                logger.info("MJPEGServer: stream on http://0.0.0.0:%d", self._port)
                self._httpd.serve_forever()
            except Exception as exc:
                logger.error("MJPEGServer: HTTP error on port %d: %s", self._port, exc)

        threading.Thread(
            target=_run_http, daemon=True, name=f"mjpeg-http-{self._port}"
        ).start()

    def stop_http(self) -> None:
        """Stop capture loop and HTTP server (does not close the camera device)."""
        self._running = False
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception as exc:
                logger.warning("MJPEGServer: error shutting down HTTP: %s", exc)
            self._httpd = None
        self._latest_jpeg = b""
        self._current_fps = 0.0
        with self._client_lock:
            self._client_count = 0
        logger.info("MJPEGServer: stopped on port %d", self._port)

    def set_processor(self, processor) -> None:
        """Atomically swap the active FrameProcessor (zero downtime)."""
        with self._processor_lock:
            self._processor = processor

    def reconfigure_processor(self, **kwargs) -> None:
        with self._processor_lock:
            proc = self._processor
        if proc is None:
            return
        try:
            proc.reconfigure(**kwargs)
        except Exception as exc:
            logger.warning("MJPEGServer: processor reconfigure error: %s", exc)

    def _capture_loop(self) -> None:
        """Background thread: get frames from device, process, encode to JPEG."""
        while self._running:
            with self._processor_lock:
                proc = self._processor
            if proc is None:
                time.sleep(0.01)
                continue

            frames = self._device.get_frames(proc.required_streams())
            try:
                output = proc.process(frames)
            except Exception as exc:
                logger.warning("MJPEGServer: processor error: %s", exc)
                output = None

            if output is not None:
                ok, buf = cv2.imencode(
                    ".jpg", output,
                    [cv2.IMWRITE_JPEG_QUALITY, self._quality],
                )
                if ok:
                    with self._frame_lock:
                        self._latest_jpeg = buf.tobytes()
                    self._tick_fps()
            else:
                time.sleep(0.005)

    def _tick_fps(self) -> None:
        now = time.monotonic()
        self._fps_tracker_times.append(now)
        if len(self._fps_tracker_times) > 60:
            self._fps_tracker_times = self._fps_tracker_times[-60:]
        if len(self._fps_tracker_times) >= 2:
            elapsed = self._fps_tracker_times[-1] - self._fps_tracker_times[0]
            if elapsed > 0:
                self._current_fps = (len(self._fps_tracker_times) - 1) / elapsed


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — PIPELINE
# ═════════════════════════════════════════════════════════════════════════════

class CameraPipeline:
    """
    Manages CameraDevice + MJPEGServer lifecycle and status snapshots.

    All camera resources are opened at __init__ time.  Plugin switching
    is always instant (processor swap only).  Only restart_cameras() ever
    closes and reopens the device — and only when called explicitly from
    Advanced Settings.
    """

    def __init__(
        self,
        cam_configs: dict[int, dict],
        stream_ports: dict[int, int],
        quality: int,
        fps: int,
        width: int,
        height: int,
        default_plugin: str = "simple_color",
        default_config: dict | None = None,
        enable_stereo: bool = False,
    ) -> None:
        self._stream_ports = stream_ports
        self._quality = quality
        self._fps = fps
        self._width = width
        self._height = height
        self._cam_selection = 1
        self._active_plugin_name = default_plugin
        self._active_plugin_config = default_config or {}
        self._enable_stereo = enable_stereo
        self._lock = threading.Lock()

        # Build devices and servers for each configured camera
        self._devices: dict[int, CameraDevice] = {}
        self._servers: dict[int, MJPEGServer] = {}
        for cam_id, cfg in cam_configs.items():
            dev = CameraDevice(
                device_ip=cfg.get("device_ip"),
                fps=fps,
                width=width,
                height=height,
                enable_stereo=enable_stereo,
            )
            port = stream_ports.get(cam_id, 8080)
            srv = MJPEGServer(dev, port, quality)
            self._devices[cam_id] = dev
            self._servers[cam_id] = srv

        # Open all devices and start HTTP servers
        self._open_all()

    def _open_all(self) -> None:
        """Open all camera devices and start MJPEG HTTP servers."""
        proc_cls = get_processor(self._active_plugin_name)
        processor = proc_cls(**self._active_plugin_config)

        for cam_id, dev in self._devices.items():
            try:
                dev.open()
            except Exception as exc:
                logger.error("CameraPipeline: failed to open camera %d: %s", cam_id, exc)

            srv = self._servers[cam_id]
            srv.set_processor(processor)
            srv.start_http()

    @property
    def cam_selection(self) -> int:
        return self._cam_selection

    def switch_camera(self, cam_id: int) -> None:
        if cam_id not in (1, 2):
            logger.warning("CameraPipeline: invalid cam_id=%d", cam_id)
            return
        self._cam_selection = cam_id
        logger.info("CameraPipeline: switched to camera %d", cam_id)

    def switch_plugin(self, plugin_name: str, config: dict | None = None) -> None:
        """
        Swap the active processor across all running servers.

        Always instant — no camera restart, no stream downtime.
        """
        proc_cls = get_processor(plugin_name)  # raises KeyError if unknown
        # Use only the config provided for this plugin — do NOT carry over
        # the previous plugin's config (different plugins have different keys).
        new_config = config or {}
        with self._lock:
            self._active_plugin_name = plugin_name
            self._active_plugin_config = new_config
            servers = list(self._servers.values())

        processor = proc_cls(**new_config)
        for srv in servers:
            srv.set_processor(processor)
        logger.info(
            "CameraPipeline: processor switched to '%s' (no restart)", plugin_name
        )

    def update_plugin_config(self, config: dict) -> None:
        """Update active plugin config in-place via reconfigure()."""
        if not isinstance(config, dict):
            return
        with self._lock:
            self._active_plugin_config = {**self._active_plugin_config, **config}
            servers = list(self._servers.values())
        for srv in servers:
            srv.reconfigure_processor(**config)
        logger.info("CameraPipeline: plugin config updated in-place")

    def restart_cameras(self, fps: int, width: int, height: int) -> None:
        """
        Close all devices and reopen with new fps/resolution.

        Only called from Advanced Settings.  Stream downtime expected.
        """
        logger.info(
            "CameraPipeline: restarting cameras — %dx%d @ %d fps", width, height, fps
        )
        self._fps = fps
        self._width = width
        self._height = height

        for cam_id, srv in self._servers.items():
            srv.stop_http()

        for cam_id, dev in self._devices.items():
            dev._fps = fps
            dev._width = width
            dev._height = height
            try:
                dev.close()
                dev.open()
            except Exception as exc:
                logger.error(
                    "CameraPipeline: failed to restart camera %d: %s", cam_id, exc
                )

        proc_cls = get_processor(self._active_plugin_name)
        processor = proc_cls(**self._active_plugin_config)
        for srv in self._servers.values():
            srv.set_processor(processor)
            srv.start_http()

        logger.info("CameraPipeline: all cameras restarted")

    def start_http(self, cam_id: int | None = None) -> None:
        """Start (or resume) HTTP streaming for a camera."""
        cams = [cam_id] if cam_id is not None else list(self._servers.keys())
        for cid in cams:
            srv = self._servers.get(cid)
            if srv and not srv.streaming:
                srv.start_http()

    def stop_http(self, cam_id: int | None = None) -> None:
        """Stop HTTP streaming for a camera (device stays open)."""
        cams = [cam_id] if cam_id is not None else list(self._servers.keys())
        for cid in cams:
            srv = self._servers.get(cid)
            if srv and srv.streaming:
                srv.stop_http()

    def get_status(self) -> CameraFrame:
        """Return current status as a CameraFrame dataclass."""
        with self._lock:
            server = self._servers.get(self._cam_selection)
            streaming = server.streaming if server else False
            fps = server.fps if server else 0.0
            srv1 = self._servers.get(1)
            srv2 = self._servers.get(2)
            dev1 = self._devices.get(1)

        ports = self._stream_ports
        return CameraFrame(
            cam_selection=self._cam_selection,
            streaming=streaming,
            fps=fps,
            width=self._width,
            height=self._height,
            mjpeg_url_cam1=f"http://{{host}}:{ports.get(1, 8080)}/",
            mjpeg_url_cam2=f"http://{{host}}:{ports.get(2, 8081)}/",
            cam1_clients=srv1.client_count if srv1 else 0,
            cam2_clients=srv2.client_count if srv2 else 0,
            cam1_streaming=srv1.streaming if srv1 else False,
            cam2_streaming=srv2.streaming if srv2 else False,
            cam1_fps=srv1.fps if srv1 else 0.0,
            cam2_fps=srv2.fps if srv2 else 0.0,
            active_plugin=self._active_plugin_name,
            active_plugin_config=self._active_plugin_config,
            available_plugins=list_plugins(),
            available_streams=dev1.available_streams() if dev1 else [],
        )

    def shutdown(self) -> None:
        """Stop all HTTP servers and close all camera devices."""
        for srv in self._servers.values():
            try:
                srv.stop_http()
            except Exception:
                pass
        for dev in self._devices.values():
            try:
                dev.close()
            except Exception:
                pass


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 5 — I/O ADAPTERS
# ═════════════════════════════════════════════════════════════════════════════

class WebSocketServer:
    """WebSocket server: broadcasts CameraFrame status at 1 Hz, accepts commands."""

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
        while True:
            if self._clients:
                status = self._pipeline.get_status()
                message = json.dumps(status.to_dict())
                dead: set = set()
                for ws in self._clients.copy():
                    try:
                        await ws.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        dead.add(ws)
                    except Exception as exc:
                        logger.warning("WebSocketServer: send error: %s", exc)
                        dead.add(ws)
                self._clients.difference_update(dead)
            await asyncio.sleep(1.0)

    async def handle_client(self, websocket) -> None:
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
            logger.debug("WebSocketServer: connection closed for %s", addr)
        except Exception as exc:
            logger.warning("WebSocketServer: handler error for %s: %s", addr, exc)
        finally:
            self._clients.discard(websocket)
            logger.info("WebSocketServer: client disconnected: %s", addr)

    async def serve(self) -> None:
        logger.info("WebSocketServer: starting on ws://localhost:%d", self._port)
        asyncio.create_task(self.broadcast_loop())
        async with websockets.serve(self.handle_client, "0.0.0.0", self._port):
            logger.info("WebSocketServer: running.")
            await asyncio.Future()


class HttpFileServer:
    """Serve static files from web_static/ over HTTP."""

    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port = port

    def run(self) -> None:
        static_dir = self._static_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def log_message(self, format, *args):
                logger.debug("HTTP %s - %s", self.address_string(), format % args)

        try:
            with _ThreadingTCPServer(("", self._port), _Handler) as httpd:
                logger.info(
                    "HttpFileServer: serving %s on port %d",
                    self._static_dir, self._port,
                )
                httpd.serve_forever()
        except Exception as exc:
            logger.error(
                "HttpFileServer: failed on port %d: %s", self._port, exc
            )


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 6 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

class CameraBridge:
    """
    Top-level orchestrator.

    1. CameraPipeline   — opens all camera devices at startup
    2. HttpFileServer    — serves web_static/
    3. WebSocketServer   — control/status WS
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
        enable_stereo: bool = False,
    ) -> None:
        self._http_port = ws_port
        self._ws_port = ws_port + 1
        self._cam_selection = cam_selection
        self._static_dir = static_dir
        # Store original argv for process-restart (Advanced Settings)
        self._original_argv = sys.argv[:]

        cam_configs: dict[int, dict] = {}
        if cam1_ip:
            cam_configs[1] = {"device_ip": cam1_ip}
        if cam2_ip:
            cam_configs[2] = {"device_ip": cam2_ip}

        self._pipeline = CameraPipeline(
            cam_configs=cam_configs,
            stream_ports={1: cam1_stream_port, 2: cam2_stream_port},
            quality=quality,
            fps=fps,
            width=width,
            height=height,
            default_plugin=plugin,
            default_config={},
            enable_stereo=enable_stereo,
        )

    def run(self) -> None:
        logger.info(
            "CameraBridge starting | http=:%d ws=:%d",
            self._http_port, self._ws_port,
        )

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

        self._pipeline.switch_camera(self._cam_selection)

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
        ws_server = WebSocketServer(
            port=self._ws_port,
            pipeline=self._pipeline,
            on_client_message=self._handle_client_message,
        )
        await ws_server.serve()

    def _handle_client_message(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning(
                "CameraBridge: invalid JSON: %s | raw: %s", exc, raw[:120]
            )
            return

        msg_type = msg.get("type", "")

        if msg_type == "start_stream":
            cam_id = msg.get("cam_id")
            self._pipeline.start_http(cam_id)

        elif msg_type == "stop_stream":
            cam_id = msg.get("cam_id")
            self._pipeline.stop_http(cam_id)

        elif msg_type == "switch_camera":
            cam_id = msg.get("cam_id", 1)
            self._pipeline.switch_camera(cam_id)

        elif msg_type == "switch_plugin":
            plugin_name = msg.get("plugin_name", "")
            config = msg.get("config") or {}
            try:
                self._pipeline.switch_plugin(plugin_name, config)
            except KeyError as exc:
                logger.warning("CameraBridge: switch_plugin failed: %s", exc)

        elif msg_type == "update_plugin_config":
            config = msg.get("config", {})
            self._pipeline.update_plugin_config(config)

        elif msg_type == "restart_camera":
            fps    = int(msg.get("fps", 30))
            width  = int(msg.get("width", 1280))
            height = int(msg.get("height", 720))
            # depthai does not support closing/reopening a device in the same
            # process.  Re-exec the whole process with updated fps/width/height.
            threading.Thread(
                target=self._restart_process,
                args=(fps, width, height),
                daemon=True,
                name="camera-restart",
            ).start()

        else:
            logger.debug("CameraBridge: unknown message type: %s", msg_type)

    def _restart_process(self, fps: int, width: int, height: int) -> None:
        """
        Re-exec the current Python process with updated fps/width/height args.

        depthai does not support closing and reopening a device within the
        same process, so the only reliable way to change resolution or FPS
        is to restart the entire process via os.execv.
        """
        logger.info(
            "CameraBridge: restarting process — %dx%d @ %d fps", width, height, fps
        )
        # Build new argv: start from original, patch --fps/--width/--height
        argv = self._original_argv[:]

        def _replace_or_append(args: list[str], flag: str, value: str) -> None:
            for i, arg in enumerate(args):
                if arg == flag and i + 1 < len(args):
                    args[i + 1] = value
                    return
                if arg.startswith(f"{flag}="):
                    args[i] = f"{flag}={value}"
                    return
            args.extend([flag, value])

        _replace_or_append(argv, "--fps",    str(fps))
        _replace_or_append(argv, "--width",  str(width))
        _replace_or_append(argv, "--height", str(height))

        self._pipeline.shutdown()
        time.sleep(0.5)  # let depthai release resources before exec
        # argv[0] is the script path; keep it so Python knows what to run
        os.execv(sys.executable, [sys.executable] + argv)


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
        "--cam1-ip", default="10.95.76.11",
        help="OAK-D camera 1 IP address",
    )
    parser.add_argument(
        "--cam2-ip", default="10.95.76.10",
        help="OAK-D camera 2 IP address (default: 10.95.76.10)",
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
    parser.add_argument(
        "--stereo", default=True,
        help="Enable stereo depth nodes (Left/Right/StereoDepth)",
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
        enable_stereo=args.stereo,
    ).run()
