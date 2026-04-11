#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import List

import websockets

INPUT_PATH = Path(__file__).parent / "data_log" / "path_follow_raw_v1.jsonl"
HOST = "0.0.0.0"
PORT = 8896
HZ = 10.0
LOOP_FOREVER = True


def load_rows(path: Path) -> List[str]:
    if not path.exists():
        raise FileNotFoundError(path)
    rows: List[str] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            t = line.strip()
            if not t:
                continue
            json.loads(t)
            rows.append(t)
    if not rows:
        raise RuntimeError("no valid rows")
    return rows


async def main() -> None:
    rows = load_rows(INPUT_PATH)
    clients: set = set()

    async def handler(ws) -> None:
        clients.add(ws)
        try:
            async for _ in ws:
                pass
        finally:
            clients.discard(ws)

    async def broadcast() -> None:
        dt = 1.0 / max(HZ, 0.1)
        while True:
            for row in rows:
                for ws in list(clients):
                    try:
                        await ws.send(row)
                    except Exception:
                        clients.discard(ws)
                await asyncio.sleep(dt)
            if not LOOP_FOREVER:
                break

    async with websockets.serve(handler, HOST, PORT):
        print(f"replay on ws://{HOST}:{PORT}")
        await broadcast()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped")
