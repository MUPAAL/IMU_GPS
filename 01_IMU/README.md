# 01_IMU — Output Field Reference

WebSocket output frame (`ws://localhost:8766`), broadcast at ~50 Hz.

## Top-level fields

| Field | Unit | Description |
|-------|------|-------------|
| `type` | — | Always `"imu_frame"` |
| `version` | — | Protocol version, currently `1` |
| `hz` | Hz | Actual frame rate measured on the bridge |
| `ts` | ms | Timestamp from the ESP32 (milliseconds since boot, not wall clock) |
| `cal` | 0–3 | BNO085 calibration status. `3` = fully calibrated, `0` = uncalibrated |
| `steps` | count | Step counter from BNO085 (pedestrian use, not relevant for robot) |

## `rot` — Rotation Vector (magnetic north referenced)

Quaternion from the BNO085 rotation vector sensor report. Uses magnetometer, so it references magnetic north.

| Field | Description |
|-------|-------------|
| `qi` `qj` `qk` | Vector part of quaternion (i, j, k components) |
| `qr` | Scalar part of quaternion |

## `euler` — Euler Angles

Derived from `rot` quaternion. ZYX convention.

| Field | Unit | Description |
|-------|------|-------------|
| `roll` | deg | Rotation around X axis (left/right tilt) |
| `pitch` | deg | Rotation around Y axis (forward/back tilt) |
| `yaw` | deg | Rotation around Z axis (0 = sensor's reference, not necessarily north) |
| `north_offset_deg` | deg | Calibration offset applied to heading (set via browser UI) |

## `heading` — Magnetic Heading

Derived from `rot` quaternion, corrected by `north_offset_deg`. This is the primary heading output for navigation.

| Field | Unit | Description |
|-------|------|-------------|
| `raw` | deg | Heading before north offset correction |
| `deg` | deg | Heading after correction (0 = North, 90 = East, 180 = South, 270 = West) |
| `dir` | — | 16-point compass label (N, NNE, NE, ENE, E, …) |

## `game_rot` — Game Rotation Vector (no magnetometer)

Quaternion from the BNO085 game rotation vector report. Does **not** use the magnetometer — yaw drifts over time but is immune to magnetic interference.

| Field | Description |
|-------|-------------|
| `qi` `qj` `qk` | Vector part |
| `qr` | Scalar part |

## `accel` — Accelerometer

Raw acceleration including gravity. Unit: m/s².

| Field | Description |
|-------|-------------|
| `x` `y` `z` | Acceleration along each sensor axis |

## `lin_accel` — Linear Acceleration

Acceleration with gravity removed (sensor-fused). Unit: m/s². Near zero when stationary.

## `gravity` — Gravity Vector

Estimated gravity component only (sensor-fused). Unit: m/s². Useful for tilt estimation independent of motion.

## `gyro` — Gyroscope

Angular velocity. Unit: rad/s. Near zero when stationary.

| Field | Description |
|-------|-------------|
| `x` `y` `z` | Rotation rate around each sensor axis |

## `mag` — Magnetometer

Raw magnetic field strength. Unit: µT. Used internally by BNO085 for heading fusion; affected by nearby ferrous metal or electronics.

| Field | Description |
|-------|-------------|
| `x` `y` `z` | Field strength along each sensor axis |

## Notes

- **Which heading to use for navigation:** `heading.deg` — it is north-corrected and derived from the magnetometer-fused quaternion.
- **`cal: 0` means the heading is unreliable.** Wave the sensor in a figure-8 to calibrate the magnetometer.
- **`ts` is not wall clock time.** It resets on ESP32 reboot. Use `server_ts` from the RTK frame or `time.time()` on the bridge side if you need absolute time.
