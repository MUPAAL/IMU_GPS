"""
listen_autonav.py — Debug observer for autonav_bridge status stream.

Connects to ws://localhost:8806 and prints formatted navigation status.
Also saves raw JSONL to data_log/autonav_raw_<timestamp>.jsonl.

Usage:
    python listen_autonav.py

Control commands (type in terminal, press Enter):
    start   — begin navigation
    stop    — stop
    pause   — pause
    resume  — resume
"""

from __future__ import annotations

import asyncio
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8806"
DATA_LOG_DIR = Path(__file__).parent / "data_log"


def _fmt_status(d: dict) -> str:
    state = d.get("state", "?")
    wp = f"{d.get('current_wp_idx', '?')}/{d.get('total_wp', '?')}"
    dist = d.get("dist_to_wp_m")
    dist_s = f"{dist:.2f}m" if dist is not None else "---"
    bearing = d.get("target_bearing_deg")
    heading = d.get("heading_deg")
    error = d.get("bearing_error_deg")
    linear = d.get("linear", 0.0)
    angular = d.get("angular", 0.0)
    gps_age = d.get("gps_age_s", 0.0)
    imu_age = d.get("imu_age_s", 0.0)

    bearing_s = f"{bearing:.1f}°" if bearing is not None else "---"
    heading_s = f"{heading:.1f}°" if heading is not None else "---"
    error_s = f"{error:+.1f}°" if error is not None else "---"

    return (
        f"[{state:8s}] wp={wp:6s}  dist={dist_s:8s}  "
        f"bearing={bearing_s:7s}  heading={heading_s:7s}  err={error_s:7s}  "
        f"cmd=({linear:.2f}, {angular:+.2f})  "
        f"GPS age={gps_age:.1f}s  IMU age={imu_age:.1f}s"
    )


async def _stdin_reader(ws) -> None:
    """Read stdin lines and send control commands to autonav_bridge."""
    loop = asyncio.get_event_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        line = line.strip().lower()
        if line in ("start", "stop", "pause", "resume"):
            await ws.send(json.dumps({"type": line}))
            print(f"  → sent: {line}")
        elif line:
            print(f"  unknown command: {line!r}  (start / stop / pause / resume)")


async def main() -> None:
    DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = DATA_LOG_DIR / f"autonav_raw_{ts_str}.jsonl"

    print(f"Connecting to {WS_URL} ...")
    print(f"Logging to    {log_path}")
    print("Commands: start / stop / pause / resume")
    print("-" * 90)

    async with websockets.connect(WS_URL) as ws:
        asyncio.create_task(_stdin_reader(ws))
        with open(log_path, "w", encoding="utf-8") as f:
            async for raw in ws:
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                f.write(raw + "\n")
                f.flush()
                print(_fmt_status(data))


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
