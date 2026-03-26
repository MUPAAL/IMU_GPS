#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to Robot data from robot_bridge.py

Usage:
    python listen_robot_websocket.py --url ws://localhost:8796
"""

import asyncio
import json
import argparse
import websockets


# State descriptions
ROBOT_STATES = {
    0: "BOOT",
    1: "MANUAL_READY",
    2: "MANUAL_ACTIVE",
    3: "CC_ACTIVE",
    4: "AUTO_READY",
    5: "AUTO_ACTIVE",
    6: "ESTOPPED",
}


async def listen_robot(ws_url: str):
    """Connect to robot_bridge.py WebSocket and listen for robot data."""
    print(f"Connecting to {ws_url}...")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving Robot data...\n")
            print("=" * 130)
            print("State           | Meas Speed | Meas AngRate | Cmd Speed | Cmd AngRate | Distance | Heading | Battery | Hz   | Uptime")
            print("-" * 130)
            
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    state_id = data.get("state", 0)
                    state_name = ROBOT_STATES.get(state_id, f"UNKNOWN({state_id})")
                    
                    meas_speed = data.get("meas_speed", 0)
                    meas_ang_rate = data.get("meas_ang_rate", 0)
                    cmd_speed = data.get("cmd_speed", 0)
                    cmd_ang_rate = data.get("cmd_ang_rate", 0)
                    total_distance_m = data.get("total_distance_m", 0)
                    heading_est_deg = data.get("heading_est_deg", 0)
                    soc = data.get("soc", 0)
                    hz = data.get("hz", 0)
                    uptime_s = data.get("uptime_s", 0)
                    
                    # Format battery indicator
                    battery_bars = int(soc / 10) if soc is not None else 0
                    battery_str = f"[{'█' * battery_bars}{'░' * (10 - battery_bars)}] {soc}%"
                    
                    # Convert seconds to minutes:seconds
                    uptime_min = int(uptime_s // 60)
                    uptime_sec = int(uptime_s % 60)
                    uptime_str = f"{uptime_min}m{uptime_sec}s"
                    
                    print(
                        f"{state_name:<15} | "
                        f"{meas_speed:>10.3f} | "
                        f"{meas_ang_rate:>12.3f} | "
                        f"{cmd_speed:>9.3f} | "
                        f"{cmd_ang_rate:>11.3f} | "
                        f"{total_distance_m:>8.2f} | "
                        f"{heading_est_deg:>7.1f}° | "
                        f"{battery_str:>20} | "
                        f"{hz:>4.1f} | "
                        f"{uptime_str}"
                    )
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure robot_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Listen to Robot data from robot_bridge.py WebSocket")
    parser.add_argument(
        "--url",
        default="ws://localhost:8796",
        help="WebSocket URL (default: ws://localhost:8796)"
    )
    args = parser.parse_args()
    
    try:
        asyncio.run(listen_robot(args.url))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
