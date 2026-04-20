"""
heading_override.py — IMU heading override filter.

Sits between imu_bridge.py (WS :8766) and downstream consumers.
When rot.acc > threshold (default 2.0 rad, ~115° error), replaces heading.{raw,deg,dir}
and euler.yaw with a browser-supplied value, then clamps rot.acc to the
threshold so downstream consumers treat the frame as valid.
rot.acc is in radians (lower = better); π (~3.14) means fully uncalibrated.

Data flow:
    imu_bridge WS :8766
        → _upstream_loop()
        → _apply_override()
        → asyncio.Queue
        → WebSocketServer WS :8768  →  nav_bridge / autonav / recorder / browser
                            ↑
                  Browser HTTP :8767 sends {"set_heading_override": <deg>}
                                           {"clear_heading_override": true}

Usage:
    python heading_override.py
    python heading_override.py --upstream ws://localhost:8766 --ws-port 8767 --threshold 2.0
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import webbrowser
from pathlib import Path

import websockets

# ── Reuse server infrastructure from imu_bridge ───────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))
from imu_bridge import HttpFileServer, WebSocketServer

sys.path.insert(0, str(Path(__file__).parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

# ── Mutable state (single async thread — no lock needed) ─────────────────────
_override_heading: float | None = None
_accuracy_threshold: float = 2.0

# Reference-heading state: user sets a known bearing; delta from game_rot tracks changes.
_ref_bearing: float | None = None      # user-declared bearing at reference moment
_ref_heading_raw: float | None = None  # heading.raw from imu_bridge at that moment
_latest_raw_heading: float = 0.0       # rolling snapshot for set_ref_heading capture

_COMPASS = ["N","NNE","NE","ENE","E","ESE","SE","SSE",
            "S","SSW","SW","WSW","W","WNW","NW","NNW"]


# ═════════════════════════════════════════════════════════════════════════════
# CORE — filter function
# ═════════════════════════════════════════════════════════════════════════════

def _apply_override(raw_json: str) -> str:
    """
    CORE — Apply heading override to one IMU frame from imu_bridge.

    When rot.acc > _accuracy_threshold and an override heading is set,
    replaces heading and euler.yaw fields and clamps acc to the threshold.
    rot.acc is in radians (lower = better); threshold default is 2.0 rad (~115°).
    Passes the frame through unchanged otherwise.
    """
    try:
        # ── INPUT ──────────────────────────────────────────────────────────────
        # Enriched IMU frames arrive here from imu_bridge WS :8766.
        frame = json.loads(raw_json)
        # ───────────────────────────────────────────────────────────────────────
    except json.JSONDecodeError:
        return raw_json

    global _latest_raw_heading
    raw_heading = float(frame.get("heading", {}).get("raw", 0.0))
    _latest_raw_heading = raw_heading

    # ── Reference-heading mode (priority over static override) ─────────────────
    # User sets a known bearing once; heading.raw delta from imu_bridge tracks rotation.
    if _ref_bearing is not None and _ref_heading_raw is not None:
        delta = ((raw_heading - _ref_heading_raw + 180.0) % 360.0) - 180.0
        hdeg = (_ref_bearing + delta) % 360.0
        hdir = _COMPASS[int(round(hdeg / 22.5)) % 16]
        frame["heading"] = {"raw": raw_heading, "deg": round(hdeg, 3), "dir": hdir}
        frame["ref_heading_active"] = True
        frame["override_active"] = False
        return json.dumps(frame)

    # ── Static override mode (activates when magnetometer accuracy is poor) ────
    acc = float(frame.get("rot", {}).get("acc", 3.0))

    if acc > _accuracy_threshold and _override_heading is not None:
        hdeg = _override_heading % 360.0
        hdir = _COMPASS[int(round(hdeg / 22.5)) % 16]

        frame["heading"] = {"raw": round(hdeg, 3), "deg": round(hdeg, 3), "dir": hdir}
        if "euler" in frame:
            frame["euler"]["yaw"] = round(hdeg, 2)
        if "rot" in frame:
            frame["rot"]["acc"] = _accuracy_threshold
        frame["override_active"] = True
    else:
        frame["override_active"] = False

    frame["ref_heading_active"] = False

    # ── OUTPUT ─────────────────────────────────────────────────────────────────
    # Modified (or pass-through) frame queued here for WS :8768 broadcast.
    return json.dumps(frame)
    # ───────────────────────────────────────────────────────────────────────────


# ═════════════════════════════════════════════════════════════════════════════
# I/O — upstream client + browser message handler
# ═════════════════════════════════════════════════════════════════════════════

async def _upstream_loop(upstream_url: str, queue: asyncio.Queue) -> None:
    """Connect to imu_bridge, filter each frame, push result to broadcast queue."""
    while True:
        try:
            async with websockets.connect(upstream_url) as ws:
                async for msg in ws:
                    await queue.put(_apply_override(msg))
        except Exception:
            await asyncio.sleep(3.0)


def _handle_browser_msg(raw: str) -> None:
    """Process heading control messages from the browser."""
    global _override_heading, _ref_bearing, _ref_heading_raw
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        return
    if "set_heading_override" in msg:
        try:
            _override_heading = float(msg["set_heading_override"])
        except (ValueError, TypeError):
            pass
    elif "clear_heading_override" in msg:
        _override_heading = None
    elif "set_ref_heading" in msg:
        try:
            _ref_bearing = float(msg["set_ref_heading"]) % 360.0
            _ref_heading_raw = _latest_raw_heading
        except (ValueError, TypeError):
            pass
    elif "clear_ref_heading" in msg:
        _ref_bearing = None
        _ref_heading_raw = None


# ═════════════════════════════════════════════════════════════════════════════
# APPLICATION
# ═════════════════════════════════════════════════════════════════════════════

async def _main(upstream_url: str, http_port: int, ws_port: int, static_dir: Path) -> None:
    queue: asyncio.Queue = asyncio.Queue(maxsize=200)
    asyncio.create_task(_upstream_loop(upstream_url, queue))

    if static_dir.exists():
        threading.Thread(
            target=HttpFileServer(static_dir, http_port).run,
            daemon=True,
        ).start()
        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{http_port}")).start()
    else:
        print(f"[warn] static dir not found: {static_dir} — HTTP server not started.")

    await WebSocketServer(ws_port, queue, on_client_message=_handle_browser_msg).serve()


if __name__ == "__main__":
    import argparse

    imu_ws = _cfg.IMU_WS_PORT if _cfg else 8765
    p = argparse.ArgumentParser(
        description="IMU heading override filter — intercepts imu_bridge frames, applies heading when acc < threshold"
    )
    p.add_argument("--upstream",  default=f"ws://localhost:{imu_ws + 1}",
                   help="WebSocket URL of imu_bridge (default: ws://localhost:8766)")
    p.add_argument("--ws-port",   type=int, default=imu_ws + 2,
                   help="HTTP port for control UI; WebSocket binds to ws-port+1 (default: 8767)")
    p.add_argument("--threshold", type=float, default=2.0,
                   help="acc value below which override activates (default: 2.0, range 0-3)")
    args = p.parse_args()

    _accuracy_threshold = args.threshold

    try:
        asyncio.run(_main(
            upstream_url=args.upstream,
            http_port=args.ws_port,
            ws_port=args.ws_port + 1,
            static_dir=Path(__file__).parent / "web_static",
        ))
    except KeyboardInterrupt:
        pass
