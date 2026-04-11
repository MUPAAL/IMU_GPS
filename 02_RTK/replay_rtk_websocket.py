#!/usr/bin/env python3
"""
Replay recorded RTK JSONL data as a WebSocket stream.

This lets collaborators run downstream modules without physical RTK hardware.

Example:
    python replay_rtk_websocket.py
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import List

import websockets

INPUT_PATH = Path(__file__).parent / "data_log" / "rtk_raw_v2.jsonl"
HOST = "0.0.0.0"
PORT = 8776
HZ = 5.0
LOOP_FOREVER = True


async def _run_server(records: List[str], host: str, port: int, hz: float, loop_forever: bool) -> None:
    if hz <= 0:
        raise ValueError("HZ must be > 0")
    interval = 1.0 / hz

    clients: set = set()

    async def handler(ws) -> None:
        clients.add(ws)
        addr = ws.remote_address
        print(f"[replay] client connected: {addr}")
        try:
            async for _ in ws:
                # Replay server is source-only. Ignore inbound client messages.
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            clients.discard(ws)
            print(f"[replay] client disconnected: {addr}")

    async def broadcaster() -> None:
        print(f"[replay] loaded {len(records)} frames")
        while True:
            for raw in records:
                if not clients:
                    await asyncio.sleep(0.05)
                    continue

                dead: set = set()
                for ws in clients.copy():
                    try:
                        await ws.send(raw)
                    except websockets.exceptions.ConnectionClosed:
                        dead.add(ws)
                    except Exception:
                        dead.add(ws)

                if dead:
                    clients.difference_update(dead)
                await asyncio.sleep(interval)

            if not loop_forever:
                print("[replay] playback complete")
                break

    async with websockets.serve(handler, host, port):
        print(f"[replay] ws://{host}:{port} started")
        await broadcaster()


def _load_jsonl(path: Path) -> List[str]:
    if not path.exists():
        raise FileNotFoundError(f"Input file not found: {path}")

    rows: List[str] = []
    bad = 0

    with path.open("r", encoding="utf-8") as f:
        for _, line in enumerate(f, 1):
            text = line.strip()
            if not text:
                continue
            try:
                obj = json.loads(text)
            except json.JSONDecodeError:
                bad += 1
                continue
            rows.append(json.dumps(obj, ensure_ascii=False, separators=(",", ":")))

    if not rows:
        raise RuntimeError(f"No valid JSON frame found in {path}")
    if bad:
        print(f"[replay] skipped {bad} invalid lines")

    return rows


def main() -> None:
    records = _load_jsonl(INPUT_PATH)
    try:
        asyncio.run(_run_server(records, HOST, PORT, HZ, LOOP_FOREVER))
    except KeyboardInterrupt:
        print("\n[replay] stopped")


if __name__ == "__main__":
    main()
