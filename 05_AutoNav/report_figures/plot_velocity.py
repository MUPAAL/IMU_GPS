"""
Figure 2: Commanded vs measured velocity over time
- Linear velocity (v) and angular rate (w) time series
- Shaded regions where v > 0 (robot moving forward)
Output: velocity_timeseries.png
"""
import json
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260501_200339.jsonl"
OUT = Path(__file__).parent / "velocity_timeseries.png"

records = [json.loads(l) for l in DATA.read_text().splitlines()]
odom = [r for r in records if r["type"] == "odom"]

ts0 = odom[0]["log_recv_ts"]
t = np.array([r["log_recv_ts"] - ts0 for r in odom])
v = np.array([r["v"] for r in odom])
w = np.array([r["w"] for r in odom])

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)

# Linear velocity
ax1.plot(t, v, color="steelblue", lw=1.0, label="Linear velocity v (m/s)")
ax1.axhline(0, color="gray", lw=0.5, ls="--")
ax1.fill_between(t, v, 0, where=(v > 0), alpha=0.15, color="steelblue", label="Moving forward")
ax1.set_ylabel("v  (m/s)", fontsize=11)
ax1.set_ylim(-0.1, max(abs(v).max() * 1.2, 0.2))
ax1.legend(fontsize=9, loc="upper right")
ax1.grid(True, alpha=0.3)

# Angular rate
ax2.plot(t, w, color="darkorange", lw=1.0, label="Angular rate w (rad/s)")
ax2.axhline(0, color="gray", lw=0.5, ls="--")
ax2.fill_between(t, w, 0, where=(w > 0), alpha=0.15, color="darkorange")
ax2.fill_between(t, w, 0, where=(w < 0), alpha=0.15, color="purple")
ax2.set_ylabel("w  (rad/s)", fontsize=11)
ax2.set_xlabel("Time (s)", fontsize=11)
ax2.legend(fontsize=9, loc="upper right")
ax2.grid(True, alpha=0.3)

fig.suptitle("Robot Odometry — Linear & Angular Velocity", fontsize=12, y=1.01)
fig.tight_layout()
fig.savefig(OUT, dpi=150, bbox_inches="tight")
print(f"Saved: {OUT}")
