#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to Recorder status from recorder_bridge.py

Usage:
    python listen_recorder_websocket.py --url ws://localhost:8826
"""

import asyncio
import json
import argparse
import websockets
from datetime import datetime


async def listen_recorder(ws_url: str):
    """Connect to recorder_bridge.py WebSocket and listen for recording status."""
    print(f"Connecting to {ws_url}...")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving Recorder status...\n")
            print("=" * 140)
            print("Recording | Current File             | Rows  | Elapsed | IMU | RTK | Robot | File Size")
            print("-" * 140)
            
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    recording = data.get("recording", False)
                    current_filename = data.get("current_filename", "N/A")
                    row_count = data.get("row_count", 0)
                    elapsed_s = data.get("elapsed_s", 0)
                    
                    sources = data.get("sources", {})
                    imu_connected = "✓" if sources.get("imu", {}).get("connected", False) else "✗"
                    rtk_connected = "✓" if sources.get("rtk", {}).get("connected", False) else "✗"
                    robot_connected = "✓" if sources.get("robot", {}).get("connected", False) else "✗"
                    
                    # Format elapsed time
                    mins, secs = divmod(int(elapsed_s), 60)
                    hours, mins = divmod(mins, 60)
                    elapsed_str = f"{hours:02d}:{mins:02d}:{secs:02d}"
                    
                    # Recording indicator
                    rec_status = "🔴 REC" if recording else "⏸ IDLE"
                    
                    # File size (from file_list)
                    file_list = data.get("file_list", [])
                    file_size_str = "N/A"
                    if current_filename and file_list:
                        for finfo in file_list:
                            if finfo.get("name") == current_filename:
                                size_bytes = finfo.get("size_bytes", 0)
                                if size_bytes < 1024:
                                    file_size_str = f"{size_bytes}B"
                                elif size_bytes < 1024*1024:
                                    file_size_str = f"{size_bytes/1024:.1f}KB"
                                else:
                                    file_size_str = f"{size_bytes/(1024*1024):.1f}MB"
                                break
                    
                    # Trim filename for display
                    filename_display = current_filename[-30:] if len(current_filename) > 30 else current_filename
                    
                    print(
                        f"{rec_status:<9} | {filename_display:<23} | "
                        f"{row_count:>5} | {elapsed_str} | "
                        f"{imu_connected} | {rtk_connected} | {robot_connected} | "
                        f"{file_size_str:>10}"
                    )
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure recorder_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Listen to Recorder status from recorder_bridge.py WebSocket")
    parser.add_argument(
        "--url",
        default="ws://localhost:8826",
        help="WebSocket URL (default: ws://localhost:8826)"
    )
    args = parser.parse_args()
    
    try:
        asyncio.run(listen_recorder(args.url))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
