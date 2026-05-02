"""
Run all report figures in sequence.
Usage:  python plot_all.py
"""
import subprocess
import sys
from pathlib import Path

scripts = [
    "plot_gps_path.py",
    "plot_velocity.py",
    "plot_heading.py",
    "plot_fix_quality.py",
    "plot_imu_attitude.py",
    "plot_speed_histogram.py",
]

here = Path(__file__).parent
for s in scripts:
    print(f"\n{'='*50}\nRunning {s}")
    result = subprocess.run([sys.executable, str(here / s)], capture_output=False)
    if result.returncode != 0:
        print(f"  [FAILED] {s}")

print("\nAll done. PNGs saved to:", here)
