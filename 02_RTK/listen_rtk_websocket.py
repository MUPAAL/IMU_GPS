#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to RTK data from rtk_bridge.py

Usage:
    python listen_rtk_websocket.py --url ws://localhost:8776
"""

import asyncio
import json
import argparse
from pathlib import Path
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

    raw_log_path = Path(".") / "data_log" / "rtk_raw.jsonl"
    raw_log_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving RTK data...\n")
            print("Active | Heading          | Baseline | RTK Source | Connected | Lat (°)      | Lon (°)       | Alt (m)  | Fix Type      | Sats | HDOP")
            print("-" * 132)
            
            frame_count = 0
            async for message in websocket:
                try:
                    raw_text = message.decode("utf-8", errors="replace") if isinstance(message, bytes) else str(message)
                    with raw_log_path.open("a", encoding="utf-8") as raw_log_file:
                        raw_log_file.write(raw_text)
                        raw_log_file.write("\n")

                    data = json.loads(raw_text)
                    
                    lat = data.get("lat")
                    lon = data.get("lon")
                    alt = data.get("alt")
                    fix_quality = data.get("fix_quality", 0)
                    num_sats = data.get("num_sats", 0)
                    hdop = data.get("hdop")
                    speed_knots = data.get("speed_knots")
                    heading_deg = data.get("heading_deg")
                    heading_valid = data.get("heading_valid", False)
                    heading_dir = data.get("heading_dir")
                    baseline_m = data.get("heading_baseline_m")
                    active_source = data.get("rtk_active_source", data.get("rtk_source", ""))
                    active_label = data.get("rtk_source_label", active_source)
                    source_frames = data.get("rtk_source_frames", [])
                    
                    fix_str = FIX_QUALITY.get(fix_quality, f"Unknown({fix_quality})")
                    
                    if heading_valid and heading_deg is not None:
                        heading_str = f"{heading_deg:.1f}° {heading_dir or ''}".strip()
                    else:
                        heading_str = "N/A"
                    baseline_str = f"{baseline_m:.2f}m" if baseline_m is not None else "N/A"

                    active_summary = f"{active_source or '-'} ({active_label or '-'})"
                    if not source_frames:
                        source_frames = [{
                            "source_id": active_source or "rtk",
                            "label": active_label or "RTK",
                            "connected": True,
                            "lat": lat,
                            "lon": lon,
                            "alt": alt,
                            "fix_quality": fix_quality,
                            "num_sats": num_sats,
                            "hdop": hdop,
                        }]

                    print(f"ACTIVE: {active_summary} | HEADING: {heading_str} | BASELINE: {baseline_str}")
                    for src in source_frames:
                        src_id = src.get("source_id", "?")
                        label = src.get("label", src_id)
                        connected = "yes" if src.get("connected") else "no"
                        src_lat = src.get("lat")
                        src_lon = src.get("lon")
                        src_alt = src.get("alt")
                        src_fix = FIX_QUALITY.get(src.get("fix_quality", 0), f"Unknown({src.get('fix_quality', 0)})")
                        src_sats = src.get("num_sats", 0)
                        src_hdop = src.get("hdop")

                        lat_str = f"{src_lat:.8f}" if src_lat is not None else "N/A"
                        lon_str = f"{src_lon:.8f}" if src_lon is not None else "N/A"
                        alt_str = f"{src_alt:.1f}" if src_alt is not None else "N/A"
                        hdop_str = f"{src_hdop:.2f}" if src_hdop is not None else "N/A"

                        print(
                            f"{label:<6} | {connected:^9} | {lat_str} | {lon_str} | {alt_str:>7} | "
                            f"{src_fix:<13} | {src_sats:>4} | {hdop_str:>5}"
                        )
                    print("-" * 132)
                    
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
