#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to Camera status from camera_bridge.py
+ optional MJPEG stream viewer using OpenCV.

WebSocket provides:
  - Status updates (JSON) — camera selection, streaming state, plugin info
  - Control interface — send start_stream, stop_stream, switch_camera commands

Video stream is delivered separately via HTTP MJPEG (not over WebSocket).

Usage:
    # Just listen to camera status
    python listen_camera_websocket.py --url ws://localhost:8816

    # Listen to status + display MJPEG stream in OpenCV window
    python listen_camera_websocket.py --url ws://localhost:8816 --mjpeg http://localhost:8080/
"""

import asyncio
import json
import argparse
import websockets
import threading
import time
from urllib.request import urlopen


# ── WebSocket Listener ────────────────────────────────────────────────────────

async def listen_camera(ws_url: str, mjpeg_url: str | None = None):
    """Connect to camera_bridge.py WebSocket and listen for camera status."""
    print(f"Connecting to {ws_url}...")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving Camera status...\n")
            print("=" * 120)
            print("Camera | Streaming | FPS  | Resolution | Plugin         | MJPEG URLs")
            print("-" * 120)
            
            # Start MJPEG viewer if requested
            viewer_thread = None
            if mjpeg_url:
                viewer_thread = threading.Thread(
                    target=_mjpeg_viewer,
                    args=(mjpeg_url,),
                    daemon=True,
                    name="mjpeg-viewer"
                )
                viewer_thread.start()
                print(f"(OpenCV window will open for MJPEG stream: {mjpeg_url})\n")
            
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    cam_selection = data.get("cam_selection", 1)
                    streaming = "✓ YES" if data.get("streaming", False) else "  NO"
                    fps = data.get("fps", 0)
                    width = data.get("width", 0)
                    height = data.get("height", 0)
                    plugin = data.get("active_plugin", "N/A")
                    
                    mjpeg_url_cam1 = data.get("mjpeg_url_cam1", "N/A")
                    mjpeg_url_cam2 = data.get("mjpeg_url_cam2", "N/A")
                    
                    res_str = f"{width}x{height}" if width and height else "N/A"
                    
                    mjpeg_str = f"1: {mjpeg_url_cam1} | 2: {mjpeg_url_cam2}"
                    
                    # Camera indicator
                    cam_marker = "📷" if cam_selection == 1 else "📹"
                    
                    print(
                        f"  {cam_selection} {cam_marker}  | {streaming} | "
                        f"{fps:>4.1f} | {res_str:<10} | {plugin:<14} | {mjpeg_str}"
                    )
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure camera_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


# ── MJPEG Stream Viewer ───────────────────────────────────────────────────────

def _mjpeg_viewer(mjpeg_url: str):
    """Display MJPEG stream in OpenCV window (requires cv2)."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("\n⚠ OpenCV not installed. Install with: pip install opencv-python")
        return
    
    print(f"Starting MJPEG viewer for {mjpeg_url}...")
    
    window_name = "Camera MJPEG Stream"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    
    stream = None
    bytes_data = b""
    frame_count = 0
    
    try:
        stream = urlopen(mjpeg_url, timeout=5)
        
        while True:
            try:
                # Read until we find a JPEG frame boundary
                bytes_data += stream.read(1024)
                
                # Find JPEG start (FFD8) and end (FFD9)
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    
                    # Decode and display
                    nparr = np.frombuffer(jpg, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        frame_count += 1
                        # Add FPS counter
                        cv2.putText(
                            frame,
                            f"Frame: {frame_count}",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 0),
                            2
                        )
                        cv2.imshow(window_name, frame)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
            
            except Exception as e:
                print(f"MJPEG viewer error: {e}")
                break
    
    except Exception as e:
        print(f"Could not connect to MJPEG stream: {e}")
        print(f"Make sure camera_bridge.py is running and stream URL is correct.")
    
    finally:
        if stream:
            stream.close()
        cv2.destroyAllWindows()


# ── Entry Point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Listen to Camera status from camera_bridge.py WebSocket + optional MJPEG viewer"
    )
    parser.add_argument(
        "--url",
        default="ws://localhost:8816",
        help="WebSocket URL (default: ws://localhost:8816)"
    )
    parser.add_argument(
        "--mjpeg",
        default=None,
        help="Optional MJPEG stream URL to display in OpenCV (e.g., http://localhost:8080/)"
    )
    args = parser.parse_args()
    
    try:
        asyncio.run(listen_camera(args.url, args.mjpeg))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
