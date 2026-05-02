"""
Figure 1: GPS trajectory map
- Scatter plot of RTK GPS track colored by fix quality
- Annotates start/end points
- Shows HDOP as color-coded confidence indicator
Output: gps_path.png
"""
import json
import math
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260501_200339.jsonl"
OUT = Path(__file__).parent / "gps_path.png"

records = [json.loads(l) for l in DATA.read_text().splitlines()]
rtk = [r for r in records if r["type"] == "rtk"]

lats = np.array([r["lat"] for r in rtk])
lons = np.array([r["lon"] for r in rtk])
hdops = np.array([r["hdop"] for r in rtk])
ts = np.array([r["log_recv_ts"] for r in rtk])

# Convert lat/lon to local ENU meters (origin = first point)
lat0, lon0 = lats[0], lons[0]
k = 111_320.0
mean_lat = math.radians(lat0)
xs = (lons - lon0) * k * math.cos(mean_lat)
ys = (lats - lat0) * k

fig, ax = plt.subplots(figsize=(6.2, 8.4), constrained_layout=True)

sc = ax.scatter(
    xs,
    ys,
    c=hdops,
    cmap="RdYlGn_r",
    vmin=1.0,
    vmax=2.0,
    s=12,
    linewidths=0,
    zorder=3,
)
cbar = fig.colorbar(sc, ax=ax, pad=0.02, shrink=0.92)
cbar.set_label("HDOP", fontsize=10)

# Start / end markers
ax.plot(xs[0], ys[0], "^", color="tab:green", ms=10, zorder=5, label="Start")
ax.plot(xs[-1], ys[-1], "s", color="tab:red", ms=10, zorder=5, label="End")

# Path line underneath
ax.plot(xs, ys, color="steelblue", lw=0.8, alpha=0.5, zorder=2)

ax.margins(x=0.12, y=0.05)
ax.set_aspect("equal", adjustable="datalim")
ax.set_anchor("C")

total_dist = np.sum(np.hypot(np.diff(xs), np.diff(ys)))
duration = ts[-1] - ts[0]

ax.set_xlabel("East (m)", fontsize=11)
ax.set_ylabel("North (m)", fontsize=11)
ax.set_title(
    f"RTK GPS Trajectory  |  {len(rtk)} fixes  |  "
    f"{total_dist:.1f} m  |  {duration:.0f} s",
    fontsize=11,
)
ax.legend(fontsize=10, loc="lower left", frameon=True, framealpha=0.9)
ax.grid(True, alpha=0.3)

fig.savefig(OUT, dpi=150, bbox_inches="tight", pad_inches=0.12)
print(f"Saved: {OUT}")
