#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to aggregated Nav data from nav_bridge.py

Usage:
    python listen_nav_websocket.py --url ws://localhost:8786
"""

import asyncio
import json
import argparse
import websockets


# Fix quality descriptions
FIX_QUALITY = {
    0: "No Fix",
    1: "GPS",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
}


async def listen_nav(ws_url: str):
    """Connect to nav_bridge.py WebSocket and listen for aggregated nav data."""
    print(f"Connecting to {ws_url}...")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving Navigation data...\n")
            print("=" * 140)
            print("IMU (Euler)           | RTK Position         | RTK Fix       | Navigation State")
            print("-" * 140)
            print("Roll  | Pitch | Yaw   | Lat (°)      | Lon (°)       | Fix Type   | Dist (m) | Heading  | Dir | Src")
            print("-" * 140)
            
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    # Extract IMU data
                    imu = data.get("imu", {})
                    imu_euler = imu.get("euler", {})
                    roll = imu_euler.get("roll")
                    pitch = imu_euler.get("pitch")
                    yaw = imu_euler.get("yaw")
                    
                    # Extract RTK data
                    rtk = data.get("rtk", {})
                    lat = rtk.get("lat")
                    lon = rtk.get("lon")
                    fix_quality = rtk.get("fix_quality", 0)
                    num_sats = rtk.get("num_sats", 0)
                    
                    # Extract Nav data
                    nav = data.get("nav", {})
                    distance_m = nav.get("target_distance_m")
                    heading_deg = nav.get("heading_deg")
                    heading_dir = nav.get("heading_dir", "N/A")
                    heading_source = nav.get("heading_source", "N/A")
                    
                    # Format output
                    lat_str = f"{lat:.8f}" if lat is not None else "N/A"
                    lon_str = f"{lon:.8f}" if lon is not None else "N/A"
                    roll_str = f"{roll:6.2f}°" if roll is not None else "   N/A"
                    pitch_str = f"{pitch:6.2f}°" if pitch is not None else "   N/A"
                    yaw_str = f"{yaw:6.2f}°" if yaw is not None else "   N/A"
                    fix_str = FIX_QUALITY.get(fix_quality, f"Unknown({fix_quality})")
                    dist_str = f"{distance_m:.1f}" if distance_m is not None else "N/A"
                    heading_str = f"{heading_deg:.1f}" if heading_deg is not None else "N/A"
                    
                    print(
                        f"{roll_str} | {pitch_str} | {yaw_str} | "
                        f"{lat_str} | {lon_str} | "
                        f"{fix_str:<10} | {dist_str:>7} | {heading_str:>6}° | {heading_dir:>4} | {heading_source:>6}"
                    )
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure nav_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Listen to aggregated Nav data from nav_bridge.py WebSocket")
    parser.add_argument(
        "--url",
        default="ws://localhost:8786",
        help="WebSocket URL (default: ws://localhost:8786)"
    )
    args = parser.parse_args()
    
    try:
        asyncio.run(listen_nav(args.url))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
