#!/usr/bin/env python3
import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8896"


async def main() -> None:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = Path(".") / "data_log" / f"path_follow_raw_{ts}.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Connecting {WS_URL}")
    print(f"Raw log file: {log_path}")

    async with websockets.connect(WS_URL) as ws:
        async for msg in ws:
            data = json.loads(msg)
            with log_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(data, ensure_ascii=False))
                f.write("\n")
            o = data.get("path_observation", {})
            c = data.get("cmd_vel", {})
            print(f"obs={o.get('center_x_norm')} conf={o.get('confidence')} | cmd=({c.get('linear')}, {c.get('angular')}) {c.get('reason')}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped")
