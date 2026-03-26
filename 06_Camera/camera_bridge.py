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
import base64
import datetime
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

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

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
        enable_disparity: bool = False,
    ) -> None:
        self._device_ip = device_ip
        self._fps = fps
        self._width = width
        self._height = height
        self._enable_stereo = enable_stereo
        self._enable_disparity = enable_disparity

        self._device = None
        self._pipeline = None
        self._queues: dict[str, object] = {}
        self._last_frames: dict[str, np.ndarray] = {}      # colorised frames for display
        self._last_raw_frames: dict[str, np.ndarray] = {}  # original frames before colorisation

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
                if self._enable_disparity:
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
        self._last_frames = {}
        self._last_raw_frames = {}
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

    def get_frames(self, stream_names: list[str]) -> tuple[dict[str, np.ndarray | None], bool]:
        """Non-blocking grab for requested streams.

        Returns:
            (frames_dict, has_new_frame)

        When tryGet() returns None (no new frame yet), the last good frame for
        that stream is returned instead.  This prevents flickering when streams
        run at different rates (e.g. RGB at 30fps, depth at 25fps).
        """
        result: dict[str, np.ndarray | None] = {}
        has_new_frame = False
        for name in stream_names:
            q = self._queues.get(name)
            if q is None:
                result[name] = None
                continue
            try:
                pkt = q.tryGet()
                if pkt is not None:
                    has_new_frame = True
                    raw = pkt.getCvFrame()
                    self._last_raw_frames[name] = raw  # store before colorisation
                    frame = raw
                    # depth/disparity frames are single-channel; convert for display
                    if name in ("depth", "disparity") and frame.ndim == 2:
                        frame = cv2.applyColorMap(
                            cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U),
                            cv2.COLORMAP_JET,
                        )
                    self._last_frames[name] = frame
                # Return last good frame if no new one arrived
                result[name] = self._last_frames.get(name)
            except Exception as exc:
                logger.warning("CameraDevice: get_frames('%s') error: %s", name, exc)
                result[name] = self._last_frames.get(name)
        return result, has_new_frame

    def available_streams(self) -> list[str]:
        return list(self._queues.keys())

    def get_raw_snapshot(self) -> dict[str, np.ndarray | None]:
        """Return copies of the most recent raw (pre-colorisation) frames."""
        return {
            k: v.copy() if v is not None else None
            for k, v in self._last_raw_frames.items()
        }

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
        self._latest_output_frame: np.ndarray | None = None
        self._output_lock = threading.Lock()

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

    def get_latest_output_frame(self) -> np.ndarray | None:
        with self._output_lock:
            f = self._latest_output_frame
            return f.copy() if f is not None else None

    def _capture_loop(self) -> None:
        """Background thread: get frames from device, process, encode to JPEG."""
        while self._running:
            with self._processor_lock:
                proc = self._processor
            if proc is None:
                time.sleep(0.01)
                continue

            frames, has_new_frame = self._device.get_frames(proc.required_streams())
            try:
                output = proc.process(frames)
            except Exception as exc:
                logger.warning("MJPEGServer: processor error: %s", exc)
                output = None

            if output is not None:
                with self._output_lock:
                    self._latest_output_frame = output
                ok, buf = cv2.imencode(
                    ".jpg", output,
                    [cv2.IMWRITE_JPEG_QUALITY, self._quality],
                )
                if ok:
                    with self._frame_lock:
                        self._latest_jpeg = buf.tobytes()
                if has_new_frame:
                    self._tick_fps()
            else:
                time.sleep(0.005)

    def _tick_fps(self) -> None:
        now = time.monotonic()
        self._fps_tracker_times.append(now)
        # 只保留最近 2 秒内的时间戳
        cutoff = now - 2.0
        self._fps_tracker_times = [t for t in self._fps_tracker_times if t >= cutoff]
        if len(self._fps_tracker_times) >= 2:
            elapsed = self._fps_tracker_times[-1] - self._fps_tracker_times[0]
            if elapsed >= 0.5:  # 至少积累 0.5 秒再输出，避免启动瞬间数值虚高
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
        enable_disparity: bool = False,
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
                enable_disparity=enable_disparity,
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

    def take_snapshot(self, cam_id: int, snapshots_dir: Path) -> str:
        """Capture a frame from cam_id and save an interactive HTML inspector.

        Returns the relative URL path (e.g. '/snapshots/snap_xxx.html').
        """
        srv = self._servers.get(cam_id)
        dev = self._devices.get(cam_id)
        if srv is None or dev is None:
            raise RuntimeError(f"Camera {cam_id} not available")
        output_frame = srv.get_latest_output_frame()
        if output_frame is None:
            raise RuntimeError("No frame available — is the stream running?")
        raw_frames = dev.get_raw_snapshot()
        snapshots_dir.mkdir(parents=True, exist_ok=True)
        ts_file = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"snap_{ts_file}.html"
        html = _generate_snapshot_html(output_frame, raw_frames, self._active_plugin_name)
        (snapshots_dir / filename).write_text(html, encoding="utf-8")
        logger.info("Snapshot saved: %s/%s", snapshots_dir, filename)
        return f"/snapshots/{filename}"

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


# ── Snapshot HTML generator ───────────────────────────────────────────────────

def _generate_snapshot_html(
    output_frame: np.ndarray,
    raw_frames: dict[str, np.ndarray | None],
    plugin_name: str,
) -> str:
    """Return a self-contained HTML page for interactive pixel inspection."""
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    h, w = output_frame.shape[:2]

    _, png_buf = cv2.imencode(".png", output_frame)
    img_b64 = base64.b64encode(png_buf.tobytes()).decode()

    raw_data: dict = {}
    for name, arr in raw_frames.items():
        if arr is None:
            continue
        if arr.dtype == np.uint16:
            dtype_str = "uint16"
        elif arr.dtype == np.float32:
            dtype_str = "float32"
        else:
            dtype_str = "uint8"
            arr = arr.astype(np.uint8)
        raw_data[name] = {
            "dtype": dtype_str,
            "shape": list(arr.shape),
            "b64": base64.b64encode(arr.tobytes()).decode(),
        }

    raw_json_str = json.dumps(raw_data)

    return (
        _SNAPSHOT_HTML
        .replace("__TS__", ts)
        .replace("__PLUGIN__", plugin_name)
        .replace("__W__", str(w))
        .replace("__H__", str(h))
        .replace("__IMG_B64__", img_b64)
        .replace("__RAW_JSON__", raw_json_str)
    )


_SNAPSHOT_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Snapshot __TS__</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: #111; color: #ddd; font-family: monospace; padding: 14px; }
h2 { font-size: 12px; color: #777; margin-bottom: 10px; letter-spacing: .03em; }
#wrap { display: inline-block; cursor: crosshair; }
canvas { display: block; max-width: 100%; }
#tip {
  position: fixed; display: none; pointer-events: none; z-index: 99;
  background: rgba(10,10,10,.92); border: 1px solid #333; border-radius: 5px;
  padding: 8px 12px; font-size: 12px; line-height: 1.7; min-width: 150px;
}
.c0 { color: #888; }
.c1 { color: #9cf; }
.c2 { color: #9f9; }
.cl { color: #fb9; }
</style>
</head>
<body>
<h2>__TS__ &nbsp;|&nbsp; plugin: __PLUGIN__ &nbsp;|&nbsp; __W__&times;__H__</h2>
<div id="wrap"><canvas id="c"></canvas></div>
<div id="tip"></div>
<script>
const IMG_B64 = "__IMG_B64__";
const RAW = __RAW_JSON__;

// decode raw streams into typed arrays
const streams = {};
for (const [name, info] of Object.entries(RAW)) {
  const bytes = Uint8Array.from(atob(info.b64), c => c.charCodeAt(0));
  let arr;
  if (info.dtype === "uint16") arr = new Uint16Array(bytes.buffer);
  else if (info.dtype === "float32") arr = new Float32Array(bytes.buffer);
  else arr = bytes;
  arr._shape = info.shape;
  streams[name] = arr;
}

const canvas = document.getElementById("c");
const ctx = canvas.getContext("2d");
const img = new Image();
img.onload = () => { canvas.width = img.width; canvas.height = img.height; ctx.drawImage(img, 0, 0); };
img.src = "data:image/png;base64," + IMG_B64;

const tip = document.getElementById("tip");

function rawVal(name, x, y) {
  const arr = streams[name];
  if (!arr) return null;
  const [rh, rw, ...rest] = arr._shape;
  if (x < 0 || x >= rw || y < 0 || y >= rh) return null;
  if (rest.length === 1) {
    const C = rest[0];  // e.g. rgb: HxWx3 stored as BGR
    const b = arr[y * rw * C + x * C + 0];
    const g = arr[y * rw * C + x * C + 1];
    const r = arr[y * rw * C + x * C + 2];
    return `R=${r} G=${g} B=${b}`;
  }
  return String(arr[y * rw + x]);
}

const UNITS = { depth: " mm", disparity: " px" };

canvas.addEventListener("mousemove", e => {
  const r = canvas.getBoundingClientRect();
  const x = Math.floor((e.clientX - r.left) * canvas.width  / r.width);
  const y = Math.floor((e.clientY - r.top)  * canvas.height / r.height);
  if (x < 0 || x >= canvas.width || y < 0 || y >= canvas.height) return;

  const px = ctx.getImageData(x, y, 1, 1).data;
  let html = `<div class="c0">(${x}, ${y})</div>`;
  html += `<div class="c1">canvas &nbsp;R=${px[0]} G=${px[1]} B=${px[2]}</div>`;
  for (const name of Object.keys(streams)) {
    const v = rawVal(name, x, y);
    if (v !== null)
      html += `<div><span class="cl">${name}:</span> <span class="c2">${v}${UNITS[name]||""}</span></div>`;
  }

  tip.innerHTML = html;
  tip.style.display = "block";
  const tx = e.clientX + 16, ty = e.clientY + 16;
  tip.style.left = (tx + tip.offsetWidth  > window.innerWidth  ? e.clientX - tip.offsetWidth  - 8 : tx) + "px";
  tip.style.top  = (ty + tip.offsetHeight > window.innerHeight ? e.clientY - tip.offsetHeight - 8 : ty) + "px";
});
canvas.addEventListener("mouseleave", () => tip.style.display = "none");
</script>
</body>
</html>"""


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
                        reply = self._on_client_message(raw)
                        if reply:
                            await websocket.send(reply)
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
        enable_disparity: bool = False,
    ) -> None:
        self._http_port = ws_port
        self._ws_port = ws_port + 1
        self._cam_selection = cam_selection
        self._static_dir = static_dir

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
            enable_disparity=enable_disparity,
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

    def _handle_client_message(self, raw: str) -> str | None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning(
                "CameraBridge: invalid JSON: %s | raw: %s", exc, raw[:120]
            )
            return None

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

        elif msg_type == "snapshot":
            cam_id = msg.get("cam_id", self._pipeline.cam_selection)
            snapshots_dir = self._static_dir / "snapshots"
            try:
                url = self._pipeline.take_snapshot(cam_id, snapshots_dir)
                return json.dumps({"type": "snapshot_ready", "url": url})
            except Exception as exc:
                logger.warning("CameraBridge: snapshot error: %s", exc)
                return json.dumps({"type": "snapshot_error", "error": str(exc)})

        else:
            logger.debug("CameraBridge: unknown message type: %s", msg_type)
        return None


# ── Entry Point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OAK-D camera MJPEG streaming bridge with WebSocket control"
    )
    _c = _cfg
    parser.add_argument(
        "--ws-port", type=int, default=_c.CAM_WS_PORT if _c else 8815,
        help="HTTP port for web UI; WebSocket uses ws-port+1",
    )
    parser.add_argument(
        "--cam1-ip", default=_c.CAM1_IP if _c else "10.95.76.11",
        help="OAK-D camera 1 IP address",
    )
    parser.add_argument(
        "--cam2-ip", default=_c.CAM2_IP if _c else "10.95.76.10",
        help="OAK-D camera 2 IP address",
    )
    parser.add_argument(
        "--cam-selection", type=int, default=1, choices=[1, 2],
        help="Initial camera selection (default: 1)",
    )
    parser.add_argument(
        "--cam1-stream-port", type=int, default=_c.CAM1_STREAM_PORT if _c else 8080,
        help="MJPEG stream port for camera 1",
    )
    parser.add_argument(
        "--cam2-stream-port", type=int, default=_c.CAM2_STREAM_PORT if _c else 8081,
        help="MJPEG stream port for camera 2",
    )
    parser.add_argument("--fps", type=int, default=_c.CAM_FPS if _c else 25,
                        help="Camera FPS")
    parser.add_argument("--width", type=int, default=_c.CAM_WIDTH if _c else 640,
                        help="Frame width (pixels)")
    parser.add_argument("--height", type=int, default=_c.CAM_HEIGHT if _c else 480,
                        help="Frame height (pixels)")
    parser.add_argument(
        "--mjpeg-quality", type=int, default=_c.CAM_MJPEG_QUALITY if _c else 80,
        help="JPEG encoding quality 1-100",
    )
    parser.add_argument(
        "--plugin", default=_c.CAM_DEFAULT_PLUGIN if _c else "simple_color",
        help="Initial plugin name",
    )
    parser.add_argument(
        "--stereo", action=argparse.BooleanOptionalAction,
        default=_c.CAM_ENABLE_STEREO if _c else False,
        help="Enable stereo depth nodes (Left/Right/StereoDepth)",
    )
    parser.add_argument(
        "--disparity", action=argparse.BooleanOptionalAction,
        default=_c.CAM_ENABLE_DISPARITY if _c else False,
        help="Enable raw disparity stream (extra CPU/bandwidth; off by default)",
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
        enable_disparity=args.disparity,
    ).run()
