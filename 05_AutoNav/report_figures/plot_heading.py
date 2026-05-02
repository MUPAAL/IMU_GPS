"""
Figure 3: IMU heading over time + GPS track_deg comparison
- IMU heading (deg, 0–360)
- RTK ground track bearing (resampled to IMU timestamps)
- Highlights heading error (IMU – GPS track)
Output: heading_comparison.png
"""
import json
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260501_200339.jsonl"
OUT = Path(__file__).parent / "heading_comparison.png"

records = [json.loads(l) for l in DATA.read_text().splitlines()]
imu_recs = [r for r in records if r["type"] == "imu"]
rtk_recs = [r for r in records if r["type"] == "rtk" and r.get("track_deg") is not None]

ts0 = records[0]["log_recv_ts"]

imu_t = np.array([r["log_recv_ts"] - ts0 for r in imu_recs])
imu_h = np.array([r["heading"]["deg"] for r in imu_recs])

rtk_t = np.array([r["log_recv_ts"] - ts0 for r in rtk_recs])
rtk_track = np.array([r["track_deg"] for r in rtk_recs])


def unwrap_degrees(angle_deg):
	"""Convert a wrapped degree sequence into a continuous series."""
	if len(angle_deg) == 0:
		return angle_deg
	unwrapped = np.empty_like(angle_deg, dtype=float)
	unwrapped[0] = angle_deg[0]
	for idx in range(1, len(angle_deg)):
		delta = (angle_deg[idx] - angle_deg[idx - 1] + 180) % 360 - 180
		unwrapped[idx] = unwrapped[idx - 1] + delta
	return unwrapped

# Unwrap the raw RTK track first, then interpolate the continuous series.
rtk_track_unwrapped_source = unwrap_degrees(rtk_track)

# Resample RTK track to IMU timestamps
rtk_interp = np.interp(imu_t, rtk_t, rtk_track_unwrapped_source)

# Unwrap circular angles for plotting so wrap-around at 0/360° does not
# show up as a false discontinuity.
imu_h_plot = np.rad2deg(np.unwrap(np.deg2rad(imu_h)))
rtk_track_plot = np.rad2deg(np.unwrap(np.deg2rad(rtk_interp)))


def smooth_series(values, window_size=9):
	"""Apply a centered moving average for cleaner visualization."""
	if window_size <= 1 or len(values) < window_size:
		return values
	half_window = window_size // 2
	pad_left = half_window
	pad_right = window_size - half_window - 1
	padded = np.pad(values, (pad_left, pad_right), mode="edge")
	kernel = np.ones(window_size, dtype=float) / window_size
	return np.convolve(padded, kernel, mode="valid")

# Heading error normalized to [-180, 180]
raw_err = (imu_h - rtk_interp + 180) % 360 - 180

fig, axes = plt.subplots(2, 1, figsize=(11, 6.8), sharex=True)

axes[0].plot(imu_t, imu_h_plot, lw=1.15, color="#1f4e79", label="IMU heading (deg)")
axes[0].plot(imu_t, rtk_track_plot, lw=1.15, color="#d97706", alpha=0.95, ls="-", label="RTK ground track (unwrapped)")
axes[0].set_ylabel("Unwrapped angle (°)", fontsize=10)
axes[0].grid(True, alpha=0.3)

axes[1].plot(imu_t, raw_err, lw=0.9, color="crimson", label="Heading error (IMU − track)")
axes[1].axhline(0, color="gray", lw=0.5, ls="--")
axes[1].fill_between(imu_t, raw_err, 0, alpha=0.15, color="crimson")
axes[1].set_ylabel("Error (°)", fontsize=10)
axes[1].set_xlabel("Time (s)", fontsize=10)
axes[1].grid(True, alpha=0.3)

rmse = np.sqrt(np.mean(raw_err**2))
fig.suptitle(f"IMU Heading vs RTK Ground Track  |  RMSE = {rmse:.1f}°", fontsize=12)
fig.legend(
	loc="center left",
	bbox_to_anchor=(0.86, 0.74),
	fontsize=9,
	frameon=True,
	framealpha=0.92,
	edgecolor="0.75",
)
fig.tight_layout(rect=(0, 0, 0.86, 0.95))
fig.savefig(OUT, dpi=150, bbox_inches="tight", pad_inches=0.12)
print(f"Saved: {OUT}  (heading RMSE = {rmse:.2f} deg)")
