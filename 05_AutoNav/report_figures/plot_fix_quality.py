"""
Figure 4: RTK fix quality and HDOP over time
- Top: fix_quality time series (color bands for fix type)
- Bottom: HDOP over time with shaded good/acceptable/poor bands
Output: rtk_fix_quality.png
"""
import json
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260501_200339.jsonl"
OUT = Path(__file__).parent / "rtk_fix_quality.png"

records = [json.loads(l) for l in DATA.read_text().splitlines()]
rtk = [r for r in records if r["type"] == "rtk"]

ts0 = records[0]["log_recv_ts"]
t = np.array([r["log_recv_ts"] - ts0 for r in rtk])
fq = np.array([r["fix_quality"] for r in rtk])
hdop = np.array([r["hdop"] for r in rtk])
nsats = np.array([r["num_sats"] for r in rtk])

FIX_LABELS = {0: "No fix", 1: "GPS", 2: "DGPS", 4: "RTK Fixed", 5: "RTK Float"}
FIX_COLORS = {0: "#e74c3c", 1: "#f39c12", 2: "#f1c40f", 4: "#27ae60", 5: "#2ecc71"}

fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)

# Fix quality
for fq_val, color in FIX_COLORS.items():
    mask = fq == fq_val
    if mask.any():
        axes[0].scatter(t[mask], fq[mask], c=color, s=8, label=FIX_LABELS[fq_val], zorder=3)
axes[0].set_yticks(sorted(FIX_COLORS.keys()))
axes[0].set_yticklabels([FIX_LABELS[k] for k in sorted(FIX_COLORS.keys())], fontsize=8)
axes[0].set_ylabel("Fix Type", fontsize=10)
axes[0].legend(fontsize=8, loc="lower right", ncol=3)
axes[0].grid(True, alpha=0.3, axis="x")

# HDOP
axes[1].plot(t, hdop, color="steelblue", lw=1.0)
axes[1].axhspan(0, 1.5, alpha=0.08, color="green", label="Good (< 1.5)")
axes[1].axhspan(1.5, 2.5, alpha=0.08, color="orange", label="Acceptable (1.5–2.5)")
axes[1].axhspan(2.5, 10, alpha=0.08, color="red", label="Poor (> 2.5)")
axes[1].set_ylabel("HDOP", fontsize=10)
axes[1].set_ylim(0, max(hdop.max() * 1.3, 3.0))
axes[1].legend(fontsize=8, loc="upper right")
axes[1].grid(True, alpha=0.3)

# Satellites
axes[2].plot(t, nsats, color="forestgreen", lw=1.0, label="Satellites tracked")
axes[2].fill_between(t, nsats, alpha=0.15, color="forestgreen")
axes[2].set_ylabel("Num. Satellites", fontsize=10)
axes[2].set_xlabel("Time (s)", fontsize=10)
axes[2].legend(fontsize=9)
axes[2].grid(True, alpha=0.3)

pct_fixed = (fq == 4).mean() * 100
fig.suptitle(
    f"RTK Fix Quality over Run  |  RTK Fixed: {pct_fixed:.0f}% of the time  |  "
    f"Mean HDOP: {hdop.mean():.2f}",
    fontsize=11,
)
fig.tight_layout()
fig.savefig(OUT, dpi=150)
print(f"Saved: {OUT}  (RTK fixed {pct_fixed:.1f}%)")
