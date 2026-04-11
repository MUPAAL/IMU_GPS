#!/usr/bin/env python3
"""
09_PathFollow/path_follow_bridge.py

Single-layer path-follow bridge:
- INPUT:  camera MJPEG frame stream
- CORE:   yellow-line observation + simple center-follow control
- OUTPUT: websocket status for UI + optional command send to 04_Robot
"""

from __future__ import annotations

import asyncio
import json
import logging
import socketserver
import threading
import time
from dataclasses import asdict
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import cv2
import websockets

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

from core_policy import CmdVel, PathObservation, compute_command, extract_observation, safety_guard

WS_MSG_TYPE_STATUS = "path_follow_status"
WS_MSG_VERSION = 1

DEFAULT_HTTP_PORT = 8895
DEFAULT_WS_PORT = DEFAULT_HTTP_PORT + 1
DEFAULT_CAMERA_URL = f"http://localhost:{_cfg.CAM1_STREAM_PORT if _cfg else 8080}/"
DEFAULT_ROBOT_WS = "ws://localhost:8889"

ENABLE_SEND_COMMANDS = False
CONTROL_HZ = 10.0

LOG_FILE = Path(__file__).with_suffix(".log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.FileHandler(LOG_FILE, encoding="utf-8"), logging.StreamHandler()],
)
logger = logging.getLogger(__name__)


class FrameSource:
    def __init__(self, mjpeg_url: str) -> None:
        self._url = mjpeg_url
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = False

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._run, daemon=True, name="pathfollow-frame-source").start()

    def stop(self) -> None:
        self._running = False

    def latest(self) -> np.ndarray | None:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def _run(self) -> None:
        while self._running:
            cap = cv2.VideoCapture(self._url)
            if not cap.isOpened():
                logger.warning("FrameSource: failed to open %s; retry in 2s", self._url)
                time.sleep(2)
                continue
            logger.info("FrameSource: connected %s", self._url)
            try:
                while self._running:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        time.sleep(0.02)
                        continue
                    with self._lock:
                        self._frame = frame
            finally:
                cap.release()


class RobotCommandClient:
    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._latest_cmd = CmdVel()
        self._lock = threading.Lock()

    def update_cmd(self, cmd: CmdVel) -> None:
        with self._lock:
            self._latest_cmd = cmd

    def latest(self) -> CmdVel:
        with self._lock:
            return self._latest_cmd

    async def run(self) -> None:
        if not ENABLE_SEND_COMMANDS:
            logger.info("RobotCommandClient: ENABLE_SEND_COMMANDS=False (dry-run mode)")
            while True:
                await asyncio.sleep(3600)

        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    logger.info("RobotCommandClient: connected %s", self._url)
                    while True:
                        cmd = self.latest()
                        msg = {
                            "type": "joystick",
                            "linear": cmd.linear,
                            "angular": cmd.angular,
                            "force": 1.0,
                        }
                        await ws.send(json.dumps(msg))
                        await ws.send(json.dumps({"type": "heartbeat"}))
                        await asyncio.sleep(1.0 / CONTROL_HZ)
            except Exception as exc:
                logger.warning("RobotCommandClient: %s; retry in 2s", exc)
                await asyncio.sleep(2)


class HttpFileServer:
    def __init__(self, static_dir: Path, port: int) -> None:
        self._static_dir = static_dir
        self._port = port

    def run(self) -> None:
        static_dir = self._static_dir

        class _Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(static_dir), **kwargs)

            def log_message(self, fmt, *args):
                logger.debug("HTTP %s - " + fmt, self.address_string(), *args)

        socketserver.TCPServer.allow_reuse_address = True
        with socketserver.TCPServer(("", self._port), _Handler) as httpd:
            logger.info("HttpFileServer: http://localhost:%d", self._port)
            httpd.serve_forever()


class PathFollowBridge:
    def __init__(self) -> None:
        self._http_port = DEFAULT_HTTP_PORT
        self._ws_port = DEFAULT_WS_PORT
        self._source = FrameSource(DEFAULT_CAMERA_URL)
        self._robot = RobotCommandClient(DEFAULT_ROBOT_WS)
        self._clients: set = set()
        self._latest_obs = PathObservation()
        self._latest_cmd = CmdVel()
        self._last_update_ts = 0.0

    def run(self) -> None:
        self._source.start()
        static_dir = Path(__file__).parent / "web_static"
        threading.Thread(target=HttpFileServer(static_dir, self._http_port).run, daemon=True).start()
        logger.info("PathFollowBridge: http=:%d ws=:%d camera=%s robot=%s send_cmd=%s", self._http_port, self._ws_port, DEFAULT_CAMERA_URL, DEFAULT_ROBOT_WS, ENABLE_SEND_COMMANDS)
        try:
            asyncio.run(self._run_async())
        finally:
            self._source.stop()

    async def _run_async(self) -> None:
        await asyncio.gather(
            self._control_loop(),
            self._broadcast_loop(),
            self._robot.run(),
            self._serve_ws(),
        )

    async def _control_loop(self) -> None:
        period = 1.0 / CONTROL_HZ
        while True:
            frame = self._source.latest()
            if frame is not None:
                obs, _vis = extract_observation(frame)
                cmd = safety_guard(compute_command(obs), obs)
                self._latest_obs = obs
                self._latest_cmd = cmd
                self._robot.update_cmd(cmd)
                self._last_update_ts = time.time()
            await asyncio.sleep(period)

    async def _broadcast_loop(self) -> None:
        while True:
            if self._clients:
                payload = {
                    "type": WS_MSG_TYPE_STATUS,
                    "version": WS_MSG_VERSION,
                    "ts": time.time(),
                    "path_observation": asdict(self._latest_obs),
                    "cmd_vel": asdict(self._latest_cmd),
                    "control_hz": CONTROL_HZ,
                    "enable_send_commands": ENABLE_SEND_COMMANDS,
                    "last_update_ts": self._last_update_ts,
                }
                msg = json.dumps(payload)
                dead = set()
                for ws in self._clients.copy():
                    try:
                        await ws.send(msg)
                    except Exception:
                        dead.add(ws)
                if dead:
                    self._clients.difference_update(dead)
            await asyncio.sleep(0.1)

    async def _ws_handler(self, websocket) -> None:
        self._clients.add(websocket)
        try:
            async for _ in websocket:
                pass
        finally:
            self._clients.discard(websocket)

    async def _serve_ws(self) -> None:
        async with websockets.serve(self._ws_handler, "0.0.0.0", self._ws_port):
            logger.info("PathFollowBridge WS: ws://localhost:%d", self._ws_port)
            await asyncio.Future()


if __name__ == "__main__":
    PathFollowBridge().run()
