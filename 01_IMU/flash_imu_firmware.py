#!/usr/bin/env python3
"""Flash the BNO085 ESP32-C3 firmware using Arduino CLI.

This helper compiles and uploads the sketch in 01_IMU/bno085_esp32c3.
It does not modify the runtime IMU bridge itself; it is a separate flashing tool.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKETCH_DIR = Path(__file__).resolve().parent / "bno085_esp32c3"

try:
    import config as _cfg
except ImportError:
    _cfg = None

DEFAULT_PORT = _cfg.IMU_SERIAL_PORT if _cfg and hasattr(_cfg, "IMU_SERIAL_PORT") else "/dev/cu.usbmodem101"
DEFAULT_FQBN = "esp32:esp32:esp32c3"


def run(cmd: list[str]) -> None:
    print(f"> {' '.join(cmd)}")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise SystemExit(result.returncode)


def find_cli() -> str:
    for name in ["arduino-cli", "arduino"]:
        path = shutil.which(name)
        if path:
            return path
    raise FileNotFoundError(
        "arduino-cli not found. Install Arduino CLI and ensure it is on PATH."
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Compile and upload the BNO085 ESP32-C3 firmware")
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port for ESP32 upload (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--fqbn",
        default=DEFAULT_FQBN,
        help=f"Arduino board FQBN for the ESP32-C3 (default: {DEFAULT_FQBN})",
    )
    parser.add_argument(
        "--sketch-dir",
        default=str(SKETCH_DIR),
        help="Path to the Arduino sketch directory",
    )
    parser.add_argument(
        "--compile-only",
        action="store_true",
        help="Compile the sketch without uploading.",
    )
    parser.add_argument(
        "--upload-only",
        action="store_true",
        help="Upload an already-compiled sketch. Requires prior compile.",
    )
    args = parser.parse_args()

    if args.compile_only and args.upload_only:
        raise SystemExit("Cannot use --compile-only and --upload-only together.")

    cli = find_cli()
    sketch_dir = Path(args.sketch_dir).resolve()
    if not sketch_dir.exists():
        raise SystemExit(f"Sketch directory not found: {sketch_dir}")

    if not args.upload_only:
        print("Compiling sketch...")
        run([cli, "compile", "--fqbn", args.fqbn, str(sketch_dir)])

    if not args.compile_only:
        print("Uploading firmware to ESP32...")
        run([cli, "upload", "-p", args.port, "--fqbn", args.fqbn, str(sketch_dir)])

    print("Firmware flash complete.")


if __name__ == "__main__":
    main()
