#!/usr/bin/env python3
"""
listen_robot_websocket.py — Simple WebSocket listener for robot_bridge.

Connects to the robot WebSocket server and prints all incoming messages.
Useful for debugging and monitoring robot state.

Usage:
    python listen_robot_websocket.py [--host HOST] [--port PORT]
"""

import asyncio
import json
import argparse
import websockets


async def listen(host: str, port: int) -> None:
    url = f"ws://{host}:{port}/"
    print(f"Connecting to {url} …")
    while True:
        try:
            async with websockets.connect(url) as ws:
                print(f"Connected to {url}")
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                        print(json.dumps(msg, indent=2))
                    except json.JSONDecodeError:
                        print(f"[raw] {raw}")
        except (OSError, websockets.exceptions.WebSocketException) as exc:
            print(f"Disconnected: {exc}. Retrying in 2s…")
            await asyncio.sleep(2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot WebSocket listener")
    parser.add_argument("--host", default="localhost", help="Bridge hostname (default: localhost)")
    parser.add_argument("--port", type=int, default=8889, help="WS port (default: 8889)")
    args = parser.parse_args()
    asyncio.run(listen(args.host, args.port))


if __name__ == "__main__":
    main()
