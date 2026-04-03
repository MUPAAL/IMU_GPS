#!/usr/bin/env python3
"""
sim_robot_ws_server.py — Simulated WebSocket server that mimics robot_bridge.py output.

Continuously broadcasts four message types to all connected clients:
  - imu        (20 Hz) : roll/pitch/yaw, quaternion, heading
  - rtk        (1  Hz) : lat/lon/alt, fix quality, satellites
  - odom       (20 Hz) : linear/angular velocity, state, battery
  - nav_status (5  Hz) : navigation state, waypoint progress, heading

This lets collaborators develop and test their data-consumer code
without needing the physical robot or any bridge processes running.

Usage:
    pip install websockets
    python sim_robot_ws_server.py
    python sim_robot_ws_server.py                    # sim on port 9889
    python sim_robot_ws_server.py --port 9889        # same as above (explicit)

Port note:
    Simulator default : 9889  (safe, no conflict)
    Real robot_bridge : 8889  (switch your listener to --port 8889 for real robot)
"""

import argparse
import asyncio
import json
import logging
import math
import time
from pathlib import Path

import websockets

# ── Logger ────────────────────────────────────────────────────────────────────
_py_name = Path(__file__).stem
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(f"{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL  (simulated robot state)
# ═════════════════════════════════════════════════════════════════════════════

class SimState:
    """Holds all simulated sensor / nav state, updated each tick."""

    # GPS base position (Columbia, MO — same as rtk_bridge default)
    BASE_LAT = 38.9412928598587
    BASE_LON = -92.31884600793728
    BASE_ALT = 220.0

    def __init__(self) -> None:
        self.t0 = time.monotonic()

    def _t(self) -> float:
        return time.monotonic() - self.t0

    # ── IMU ──────────────────────────────────────────────────────────────────

    def imu_msg(self) -> dict:
        t = self._t()
        roll  = 5.0  * math.sin(t * 0.3)          # deg, slow sway
        pitch = 3.0  * math.sin(t * 0.5 + 1.0)    # deg
        yaw   = (t * 10.0) % 360.0                 # deg, slow rotation

        # Convert euler → quaternion (ZYX intrinsic, degrees→radians)
        r, p, y_ = math.radians(roll), math.radians(pitch), math.radians(yaw)
        cy, sy = math.cos(y_ / 2), math.sin(y_ / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        qr = cr * cp * cy + sr * sp * sy   # w
        qi = sr * cp * cy - cr * sp * sy   # x
        qj = cr * sp * cy + sr * cp * sy   # y
        qk = cr * cp * sy - sr * sp * cy   # z  (BNO085 convention)

        heading_deg = yaw % 360.0
        dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        heading_dir = dirs[int((heading_deg + 22.5) / 45.0) % 8]

        return {
            "type": "imu",
            "rot": {"qi": round(qi, 6), "qj": round(qj, 6),
                    "qk": round(qk, 6), "qr": round(qr, 6)},
            "euler": {
                "roll":  round(roll,  3),
                "pitch": round(pitch, 3),
                "yaw":   round(yaw,   3),
                "north_offset_deg": 0.0,
            },
            "heading": {
                "raw": round(heading_deg, 2),
                "deg": round(heading_deg, 2),
                "dir": heading_dir,
            },
            "hz": 20.0,
        }

    # ── RTK ──────────────────────────────────────────────────────────────────

    def rtk_msg(self) -> dict:
        t = self._t()
        # Simulate slow GPS drift (±0.00005 deg ≈ ±5 m)
        lat = self.BASE_LAT + 0.00005 * math.sin(t * 0.05)
        lon = self.BASE_LON + 0.00005 * math.cos(t * 0.05)
        alt = self.BASE_ALT + 1.0 * math.sin(t * 0.1)
        return {
            "type":        "rtk",
            "lat":         round(lat, 9),
            "lon":         round(lon, 9),
            "alt":         round(alt, 2),
            "fix_quality": 4,          # 4 = RTK Fixed
            "num_sats":    14,
            "hdop":        0.8,
            "speed_knots": round(abs(math.sin(t * 0.2)) * 2.0, 2),
            "track_deg":   round((t * 5.0) % 360.0, 1),
            "source":      "rtk",
            "available":   True,
        }

    # ── Odometry ─────────────────────────────────────────────────────────────

    def odom_msg(self) -> dict:
        t = self._t()
        v = round(0.5 * math.sin(t * 0.4), 4)     # m/s
        w = round(0.2 * math.sin(t * 0.6), 4)     # rad/s
        soc = max(0, min(100, int(90 - t * 0.01))) # slowly draining battery
        return {
            "type":  "odom",
            "v":     v,
            "w":     w,
            "state": 1,     # 1 = ACTIVE
            "soc":   soc,
            "ts":    time.time(),
        }

    # ── Nav status ───────────────────────────────────────────────────────────

    def nav_status_msg(self) -> dict:
        t = self._t()
        reached = min(3, int(t / 15))              # advance waypoint every 15 s
        heading_deg = round((t * 10.0) % 360.0, 1)
        dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        heading_dir = dirs[int((heading_deg + 22.5) / 45.0) % 8]
        return {
            "type":           "nav_status",
            "state":          "navigating",
            "progress":       [reached, 5],
            "distance_m":     round(max(0.0, 10.0 - (t % 15)), 2),
            "heading_deg":    heading_deg,
            "heading_dir":    heading_dir,
            "target_bearing": heading_deg,
            "nav_mode":       "PURE_PURSUIT",
            "filter_mode":    "KALMAN",
            "tolerance_m":    0.5,
        }


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — PIPELINE  (broadcast loops)
# ═════════════════════════════════════════════════════════════════════════════

class SimBroadcaster:
    """
    CORE — Manages connected clients and runs two broadcast loops.

    Mirrors the real robot_bridge.py message cadence exactly:

      _nav_pack_loop()  10 Hz — sends imu, rtk, nav_status in one burst
                                (matches nav_bridge NavLoop default 10 Hz)
      _odom_loop()      20 Hz — independent serial odometry loop

    Clients therefore see on one connection:
        ... odom odom [imu rtk nav_status] odom odom [imu rtk nav_status] ...
    """

    def __init__(self) -> None:
        self._clients: set = set()
        self._lock = asyncio.Lock()
        self._state = SimState()

    async def _broadcast(self, msg: dict) -> None:
        if not self._clients:
            return
        data = json.dumps(msg)
        async with self._lock:
            clients = set(self._clients)
        dead = set()
        for ws in clients:
            try:
                await ws.send(data)
            except Exception:
                dead.add(ws)
        if dead:
            async with self._lock:
                self._clients -= dead

    async def _nav_pack_loop(self) -> None:
        # Mirrors NavBridgeClient in robot_bridge.py:
        # one nav_bridge message → three consecutive WS messages (imu, rtk, nav_status)
        while True:
            await self._broadcast(self._state.imu_msg())
            await self._broadcast(self._state.rtk_msg())
            await self._broadcast(self._state.nav_status_msg())
            await asyncio.sleep(1 / 10)   # 10 Hz — nav_bridge default

    async def _odom_loop(self) -> None:
        while True:
            await self._broadcast(self._state.odom_msg())
            await asyncio.sleep(1 / 20)   # 20 Hz — serial odometry loop

    async def handler(self, websocket) -> None:
        """Handle one WebSocket connection."""
        async with self._lock:
            self._clients.add(websocket)
        addr = websocket.remote_address
        logger.info("Client connected: %s  (total: %d)", addr, len(self._clients))
        try:
            async for _ in websocket:
                pass   # ignore inbound messages in simulator
        except Exception:
            pass
        finally:
            async with self._lock:
                self._clients.discard(websocket)
            logger.info("Client disconnected: %s  (total: %d)", addr, len(self._clients))

    async def run(self, host: str, port: int) -> None:
        async with websockets.serve(self.handler, host, port):
            logger.info("Simulator WebSocket server started on ws://%s:%d", host, port)
            logger.info("Message cadence: [imu+rtk+nav_status]@10Hz  odom@20Hz")
            await asyncio.gather(
                self._nav_pack_loop(),
                self._odom_loop(),
            )


# ═════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(description="Simulated robot WebSocket server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    # SIM port: 9889 (avoids conflict with real robot_bridge.py which uses 8889)
    # To connect against the real robot, run your listener with --port 8889 instead.
    parser.add_argument("--port", type=int, default=9889, help="WS port (default: 9889 for sim; real robot uses 8889)")
    args = parser.parse_args()

    broadcaster = SimBroadcaster()
    try:
        asyncio.run(broadcaster.run(args.host, args.port))
    except KeyboardInterrupt:
        logger.info("Simulator stopped")


if __name__ == "__main__":
    main()
