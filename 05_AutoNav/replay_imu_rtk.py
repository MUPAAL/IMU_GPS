"""
replay_imu_rtk.py — Offline replay of IMU and RTK data for autonav testing.

Starts two WebSocket servers that replay recorded JSONL data:
  - IMU  → ws://localhost:8766  (replays 01_IMU/data_log/imu_raw_v1.jsonl)
  - RTK  → ws://localhost:8776  (replays 02_RTK/data_log/rtk_raw_v1.jsonl)

Usage:
    python 05_AutoNav/replay_imu_rtk.py

Then start autonav_bridge normally:
    python 05_AutoNav/autonav_bridge.py

Notes:
  - Data is replayed at a fixed rate (REPLAY_HZ), not the original timestamps.
  - Both streams loop continuously.
  - Multiple clients can connect simultaneously.
"""

from __future__ import annotations

import asyncio
import json
import logging
from pathlib import Path

import websockets

# ── Configuration ─────────────────────────────────────────────────────────────

IMU_JSONL   = Path(__file__).parent.parent / "01_IMU" / "data_log" / "imu_raw_v1.jsonl"
RTK_JSONL   = Path(__file__).parent.parent / "02_RTK" / "data_log" / "rtk_raw_v1.jsonl"

IMU_HOST    = "0.0.0.0"
IMU_PORT    = 8766
RTK_HOST    = "0.0.0.0"
RTK_PORT    = 8776

IMU_HZ      = 20.0   # replay rate for IMU frames
RTK_HZ      = 5.0    # replay rate for RTK frames

# ── Logger ────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _load_jsonl(path: Path) -> list[str]:
    """Load all valid JSON lines from a JSONL file."""
    if not path.exists():
        logger.warning("JSONL not found: %s", path)
        return []
    lines = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    json.loads(line)  # validate
                    lines.append(line)
                except json.JSONDecodeError:
                    pass
    logger.info("Loaded %d frames from %s", len(lines), path)
    return lines


# ── Replay server ─────────────────────────────────────────────────────────────

class ReplayServer:
    """
    WebSocket server that broadcasts a cycling sequence of JSONL lines
    to all connected clients at a fixed rate.
    """

    def __init__(self, name: str, host: str, port: int, frames: list[str], hz: float) -> None:
        self._name = name
        self._host = host
        self._port = port
        self._frames = frames
        self._period = 1.0 / max(hz, 0.1)
        self._clients: set = set()

    async def _handle(self, websocket) -> None:
        addr = websocket.remote_address
        logger.info("%s: client connected: %s", self._name, addr)
        self._clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self._clients.discard(websocket)
            logger.info("%s: client disconnected: %s", self._name, addr)

    async def _broadcast_loop(self) -> None:
        if not self._frames:
            logger.warning("%s: no frames to replay — loop idle", self._name)
            await asyncio.Future()
            return
        idx = 0
        while True:
            frame = self._frames[idx % len(self._frames)]
            idx += 1
            if self._clients:
                dead: set = set()
                for ws in self._clients.copy():
                    try:
                        await ws.send(frame)
                    except websockets.exceptions.ConnectionClosed:
                        dead.add(ws)
                    except Exception as exc:
                        logger.debug("%s: send error: %s", self._name, exc)
                        dead.add(ws)
                self._clients.difference_update(dead)
            await asyncio.sleep(self._period)

    async def serve(self) -> None:
        logger.info(
            "%s: starting on ws://%s:%d  (%.1f Hz, %d frames)",
            self._name, self._host, self._port, 1.0 / self._period, len(self._frames),
        )
        asyncio.create_task(self._broadcast_loop())
        async with websockets.serve(self._handle, self._host, self._port):
            await asyncio.Future()


# ── Main ──────────────────────────────────────────────────────────────────────

async def main() -> None:
    imu_frames = _load_jsonl(IMU_JSONL)
    rtk_frames = _load_jsonl(RTK_JSONL)

    if not imu_frames:
        logger.warning("IMU frames empty — IMU server will be idle (no data to replay)")
    if not rtk_frames:
        logger.warning("RTK frames empty — RTK server will be idle (no data to replay)")

    imu_server = ReplayServer("IMU-Replay", IMU_HOST, IMU_PORT, imu_frames, IMU_HZ)
    rtk_server = ReplayServer("RTK-Replay", RTK_HOST, RTK_PORT, rtk_frames, RTK_HZ)

    logger.info("Replay servers ready. Start autonav_bridge to connect.")
    await asyncio.gather(
        imu_server.serve(),
        rtk_server.serve(),
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Stopped.")
