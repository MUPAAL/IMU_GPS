#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to IMU data from imu_bridge.py

Usage:
    python listen_imu_websocket.py --url ws://localhost:8766
"""

import asyncio
import json
import argparse
import websockets


async def listen_imu(ws_url: str):
    """Connect to imu_bridge.py WebSocket and listen for IMU data."""
    print(f"Connecting to {ws_url}...")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving IMU data...\n")
            print("Roll (°) | Pitch (°) | Yaw (°)  | Hz   | Quaternion")
            print("-" * 70)
            
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    euler = data.get("euler", {})
                    rot = data.get("rot", {})
                    hz = data.get("hz", 0)
                    
                    roll = euler.get("roll", 0)
                    pitch = euler.get("pitch", 0)
                    yaw = euler.get("yaw", 0)
                    
                    qi = rot.get("qi", 0)
                    qj = rot.get("qj", 0)
                    qk = rot.get("qk", 0)
                    qr = rot.get("qr", 1)
                    
                    print(
                        f"{roll:7.2f} | {pitch:8.2f} | {yaw:8.2f} | "
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
