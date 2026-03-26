#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to RTK data from rtk_bridge.py

Usage:
    python listen_rtk_websocket.py --url ws://localhost:8776
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


async def listen_rtk(ws_url: str):
    """Connect to rtk_bridge.py WebSocket and listen for RTK data."""
    print(f"Connecting to {ws_url}...")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving RTK data...\n")
            print("Lat (°)      | Lon (°)       | Alt (m)  | Fix Type      | Sats | HDOP  | Speed | Track")
            print("-" * 100)
            
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    lat = data.get("lat")
                    lon = data.get("lon")
                    alt = data.get("alt")
                    fix_quality = data.get("fix_quality", 0)
                    num_sats = data.get("num_sats", 0)
                    hdop = data.get("hdop")
                    speed_knots = data.get("speed_knots")
                    track_deg = data.get("track_deg")
                    source = data.get("source", "unknown")
                    
                    fix_str = FIX_QUALITY.get(fix_quality, f"Unknown({fix_quality})")
                    
                    # Format output
                    lat_str = f"{lat:.8f}" if lat is not None else "N/A"
                    lon_str = f"{lon:.8f}" if lon is not None else "N/A"
                    alt_str = f"{alt:.1f}" if alt is not None else "N/A"
                    hdop_str = f"{hdop:.2f}" if hdop is not None else "N/A"
                    speed_str = f"{speed_knots:.1f}" if speed_knots is not None else "N/A"
                    track_str = f"{track_deg:.1f}" if track_deg is not None else "N/A"
                    
                    source_marker = "🛰" if source == "rtk" else "📍"
                    
                    print(
                        f"{lat_str} | {lon_str} | {alt_str:>7} | "
                        f"{fix_str:<13} | {num_sats:>4} | {hdop_str:>5} | "
                        f"{speed_str:>5} | {track_str:>5} {source_marker}"
                    )
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure rtk_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Listen to RTK data from rtk_bridge.py WebSocket")
    parser.add_argument(
        "--url",
        default="ws://localhost:8776",
        help="WebSocket URL (default: ws://localhost:8776)"
    )
    args = parser.parse_args()
    
    try:
        asyncio.run(listen_rtk(args.url))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
