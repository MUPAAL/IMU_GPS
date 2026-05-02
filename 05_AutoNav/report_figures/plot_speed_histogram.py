"""
Figure 6: Speed distribution during the run
- Histogram of linear velocity |v| and angular rate |w|
- Annotates time fractions: stopped / slow / fast
Output: speed_histogram.png
"""
import json
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260501_200339.jsonl"
OUT = Path(__file__).parent / "speed_histogram.png"

records = [json.loads(l) for l in DATA.read_text().splitlines()]
odom = [r for r in records if r["type"] == "odom"]

v = np.abs([r["v"] for r in odom])
w = np.abs([r["w"] for r in odom])

STOP_THR = 0.02  # m/s

stopped_frac = (v < STOP_THR).mean() * 100
moving_frac  = 100 - stopped_frac

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

# Linear speed histogram
ax1.hist(v, bins=40, color="steelblue", edgecolor="white", linewidth=0.3)
ax1.axvline(STOP_THR, color="crimson", ls="--", lw=1.2, label=f"Stop threshold ({STOP_THR} m/s)")
ax1.set_xlabel("|v|  (m/s)", fontsize=11)
ax1.set_ylabel("Count", fontsize=11)
ax1.set_title(
    f"Linear Speed Distribution\n"
    f"Stopped: {stopped_frac:.0f}%  Moving: {moving_frac:.0f}%",
    fontsize=10,
)
ax1.legend(fontsize=9)
ax1.grid(True, alpha=0.3, axis="y")

# Angular rate histogram
ax2.hist(w, bins=40, color="darkorange", edgecolor="white", linewidth=0.3)
ax2.set_xlabel("|w|  (rad/s)", fontsize=11)
ax2.set_ylabel("Count", fontsize=11)
ax2.set_title(
    f"Angular Rate Distribution\n"
    f"Mean |w|: {w.mean():.3f} rad/s  Max: {w.max():.3f} rad/s",
    fontsize=10,
)
ax2.grid(True, alpha=0.3, axis="y")

fig.suptitle(f"Odometry Speed Statistics  ({len(odom)} samples)", fontsize=12)
fig.tight_layout()
fig.savefig(OUT, dpi=150)
print(f"Saved: {OUT}")
