"""
Figure 5: IMU roll, pitch, yaw over time
- Four subplots: roll / pitch / yaw (Euler angles in degrees) and IMU sampling rate
- Useful for showing terrain tilt during field run
Output: imu_attitude.png
"""
import json
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260501_200339.jsonl"
OUT = Path(__file__).parent / "imu_attitude.png"

records = [json.loads(l) for l in DATA.read_text().splitlines()]
imu = [r for r in records if r["type"] == "imu"]

ts0 = records[0]["log_recv_ts"]
t = np.array([r["log_recv_ts"] - ts0 for r in imu])
roll  = np.array([r["euler"]["roll"]  for r in imu])
pitch = np.array([r["euler"]["pitch"] for r in imu])
yaw   = np.array([r["euler"]["yaw"]   for r in imu])
hz    = np.array([r["hz"] for r in imu])

# Unwrap yaw so the plot stays continuous across 0/360-degree boundaries.
yaw = np.rad2deg(np.unwrap(np.deg2rad(yaw)))

fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)

for ax, data, label, color in zip(
    axes,
    [roll, pitch, yaw, hz],
    ["Roll (°)", "Pitch (°)", "Yaw (°)", "IMU rate (Hz)"],
    ["tomato", "goldenrod", "steelblue", "mediumpurple"],
):
    ax.plot(t, data, lw=0.7, color=color)
    ax.set_ylabel(label, fontsize=10)
    ax.axhline(0, color="gray", lw=0.4, ls="--")
    ax.grid(True, alpha=0.3)

axes[-1].set_xlabel("Time (s)", fontsize=10)
fig.suptitle(
    f"IMU Euler Angles & Sampling Rate  |  "
    f"Mean Hz: {hz.mean():.1f}  |  {len(imu)} samples",
    fontsize=11,
)
fig.tight_layout()
fig.savefig(OUT, dpi=150)
print(f"Saved: {OUT}")
