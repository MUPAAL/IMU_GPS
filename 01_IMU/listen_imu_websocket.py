#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to IMU data from imu_bridge.py

Usage:
    python listen_imu_websocket.py --url ws://localhost:8766
"""

import asyncio
import json
import argparse
from pathlib import Path
import websockets


async def listen_imu(ws_url: str):
    """Connect to imu_bridge.py WebSocket and listen for IMU data."""
    print(f"Connecting to {ws_url}...")

    raw_log_path = Path(".") / "data_log" / "imu_raw.jsonl"
    raw_log_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving IMU data...\n")
            print("Roll (°) | Pitch (°) | Yaw (°)  | Heading (°) | Direction | Hz   | Quaternion")
            print("-" * 95)
            
            frame_count = 0
            async for message in websocket:
                try:
                    raw_text = message.decode("utf-8", errors="replace") if isinstance(message, bytes) else str(message)
                    with raw_log_path.open("a", encoding="utf-8") as raw_log_file:
                        raw_log_file.write(raw_text)
                        raw_log_file.write("\n")

                    data = json.loads(raw_text)
                    
                    euler = data.get("euler", {})
                    heading_data = data.get("heading", {})
                    rot = data.get("rot", {})
                    hz = data.get("hz", 0)
                    
                    roll = euler.get("roll", 0)
                    pitch = euler.get("pitch", 0)
                    yaw = euler.get("yaw", 0)
                    heading = heading_data.get("deg", yaw)
                    direction = heading_data.get("dir", "N/A")
                    
                    qi = rot.get("qi", 0)
                    qj = rot.get("qj", 0)
                    qk = rot.get("qk", 0)
                    qr = rot.get("qr", 1)
                    
                    print(
                        f"{roll:7.2f} | {pitch:8.2f} | {yaw:8.2f} | {heading:11.2f} | {direction:9s} | "
                        f"{hz:4.1f} | [{qi:6.3f}, {qj:6.3f}, {qk:6.3f}, {qr:6.3f}]"
                    )
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure imu_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Listen to IMU data from imu_bridge.py WebSocket")
    parser.add_argument(
        "--url",
        default="ws://localhost:8766",
        help="WebSocket URL (default: ws://localhost:8766)"
    )
    args = parser.parse_args()
    
    try:
        asyncio.run(listen_imu(args.url))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
