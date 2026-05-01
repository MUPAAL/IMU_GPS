#!/usr/bin/env python3
"""
convert_offsets_to_latlon.py

Read a CSV containing latitude/longitude offsets (columns `st lat` and `st lon`)
and write a new CSV with absolute `lat` and `lon` computed by adding a
provided base latitude/longitude.

Output is written to 05_AutoNav/path.csv for consumption by autonav_bridge.py.

Edit BASE_LAT, BASE_LON, INPUT_CSV, and OUTPUT_CSV constants at the top
to change behavior. Then run: python3 convert_offsets_to_latlon.py
"""

from pathlib import Path
import csv

# All inputs are configured here as constants. No CLI or runtime input required.
# Edit these values directly to change behavior.
BASE_LAT = 38.94221041
BASE_LON = -92.31965214

# Path to input CSV (relative to this script). Edit as needed.
INPUT_CSV = Path(__file__).with_name("offset.csv")
# Path to output CSV: 05_AutoNav/path.csv (one level up from scripts/)
OUTPUT_CSV = Path(__file__).parent.parent / "path.csv"


def parse_float(s):
    try:
        return float(s)
    except Exception:
        return 0.0


def convert(input_path: Path, output_path: Path, base_lat: float, base_lon: float):
    with input_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        if "st lat" not in fieldnames or "st lon" not in fieldnames:
            print("ERROR: input CSV must contain 'st lat' and 'st lon' columns.")
            return 1

        # Keep all original fields and ensure strict lat/lon columns exist.
        output_fieldnames = list(fieldnames)
        if "lat" not in output_fieldnames:
            output_fieldnames.append("lat")
        if "lon" not in output_fieldnames:
            output_fieldnames.append("lon")

        rows = []
        for r in reader:
            off_lat = parse_float(r.get("st lat", 0))
            off_lon = parse_float(r.get("st lon", 0))
            lat = base_lat + off_lat
            lon = base_lon + off_lon
            row_out = dict(r)
            row_out["lat"] = f"{lat:.12f}"
            row_out["lon"] = f"{lon:.12f}"
            rows.append(row_out)

    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=output_fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

    print(f"Wrote {len(rows)} rows to {output_path}")
    return 0


# Run conversion immediately using constants above. No CLI.
if __name__ == "__main__":
    if not INPUT_CSV.exists():
        print(f"ERROR: input not found: {INPUT_CSV}")
        raise SystemExit(2)
    rc = convert(INPUT_CSV, OUTPUT_CSV, BASE_LAT, BASE_LON)
    raise SystemExit(rc)
