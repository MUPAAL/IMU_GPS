#!/usr/bin/env python3
"""
demo_filter_by_type.py — Example: subscribe to only specific message types.

Shows how to selectively handle only the message types your code needs,
and how to access individual fields from each message.

Usage:
    python demo_filter_by_type.py                       # imu + odom only
    python demo_filter_by_type.py --types rtk nav_status
    python demo_filter_by_type.py --host 192.168.1.100 --port 8889
"""

import argparse
import asyncio
import json

import websockets


async def listen(host: str, port: int, types: set[str]) -> None:
    url = f"ws://{host}:{port}"
    print(f"Connecting to {url} — watching types: {sorted(types)}\n")

    while True:
        try:
            async with websockets.connect(url) as ws:
                print("Connected.\n")
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue

                    kind = msg.get("type", "")
                    if kind not in types:
                        continue

                    # ── Access individual fields ──────────────────────────────
                    if kind == "imu":
                        euler = msg.get("euler", {})
                        yaw   = euler.get("yaw", 0.0)
                        heading_dir = msg.get("heading", {}).get("dir", "?")
                        print(f"[IMU]  yaw={yaw:.1f}°  dir={heading_dir}")

                    elif kind == "rtk":
                        lat = msg.get("lat")
                        lon = msg.get("lon")
                        fix = msg.get("fix_quality", 0)
                        print(f"[RTK]  lat={lat:.8f}  lon={lon:.8f}  fix={fix}")

                    elif kind == "odom":
                        v   = msg.get("v", 0.0)
                        w   = msg.get("w", 0.0)
                        soc = msg.get("soc", 0)
                        print(f"[ODOM] v={v:+.3f}m/s  w={w:+.3f}rad/s  soc={soc}%")

                    elif kind == "nav_status":
                        state    = msg.get("state", "?")
                        progress = msg.get("progress", [0, 0])
                        dist     = msg.get("distance_m", 0.0)
                        print(f"[NAV]  state={state}  "
                              f"wp={progress[0]}/{progress[1]}  dist={dist:.1f}m")

        except (OSError, websockets.exceptions.WebSocketException) as exc:
            print(f"Disconnected: {exc} — retrying in 2s ...")
            await asyncio.sleep(2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Filter robot WebSocket by message type")
    parser.add_argument("--host",  default="localhost")
    # Default 9889 = simulator port; switch to 8889 for real robot_bridge.py
    parser.add_argument("--port",  type=int, default=9889, help="WS port (default: 9889 for sim; real robot uses 8889)")
    parser.add_argument(
        "--types", nargs="+",
        default=["imu", "rtk", "odom", "nav_status"],
        choices=["imu", "rtk", "odom", "nav_status"],
        help="Message types to display (default: imu rtk odom nav_status)",
    )
    args = parser.parse_args()
    try:
        asyncio.run(listen(args.host, args.port, set(args.types)))
    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()
