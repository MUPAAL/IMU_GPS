#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to AutoNav data from autonav_bridge.py

Usage:
    python listen_autonav_websocket.py --url ws://localhost:8806
"""

import asyncio
import json
import argparse
import websockets


async def listen_autonav(ws_url: str):
    """Connect to autonav_bridge.py WebSocket and listen for autonomous navigation data."""
    print(f"Connecting to {ws_url}...")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving AutoNav data...\n")
            print("=" * 150)
            print("Status           | Current Waypoint | Progress | Lat (°)      | Lon (°)       | Alt (m) | Target Heading | Dist to Target (m) | Speed (m/s)")
            print("-" * 150)
            
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    # Extract navigation state
                    status = data.get("status", "UNKNOWN")
                    curr_wp = data.get("current_waypoint", 0)
                    total_wp = data.get("total_waypoints", 0)
                    progress = f"{curr_wp}/{total_wp}"
                    
                    # Extract position
                    lat = data.get("lat")
                    lon = data.get("lon")
                    alt = data.get("alt")
                    
                    # Extract target info
                    target_heading = data.get("target_heading")
                    distance_to_target = data.get("distance_to_target")
                    
                    # Extract speed
                    speed = data.get("speed")
                    
                    # Format output
                    lat_str = f"{lat:.8f}" if lat is not None else "N/A"
                    lon_str = f"{lon:.8f}" if lon is not None else "N/A"
                    alt_str = f"{alt:.1f}" if alt is not None else "N/A"
                    heading_str = f"{target_heading:.1f}°" if target_heading is not None else "N/A"
                    dist_str = f"{distance_to_target:.1f}" if distance_to_target is not None else "N/A"
                    speed_str = f"{speed:.2f}" if speed is not None else "N/A"
                    
                    # Status indicator
                    status_emoji = {
                        "IDLE": "⏸",
                        "NAVIGATING": "🚀",
                        "PAUSED": "⏸",
                        "COMPLETED": "✓",
                        "ERROR": "✗",
                    }.get(status, "?")
                    
                    print(
                        f"{status:<15} | "
                        f"{progress:<16} | "
                        f"{curr_wp/total_wp*100:>6.1f}% | "
                        f"{lat_str} | {lon_str} | "
                        f"{alt_str:>7} | "
                        f"{heading_str:>14} | "
                        f"{dist_str:>18} | "
                        f"{speed_str:>10} {status_emoji}"
                    )
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure autonav_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Listen to AutoNav data from autonav_bridge.py WebSocket")
    parser.add_argument(
        "--url",
        default="ws://localhost:8806",
        help="WebSocket URL (default: ws://localhost:8806)"
    )
    args = parser.parse_args()
    
    try:
        asyncio.run(listen_autonav(args.url))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
