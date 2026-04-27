# 02_RTK — Output Field Reference

WebSocket output frame (`ws://localhost:8776`), broadcast at ~5 Hz.

## Fields

| Field | Unit | Description |
|-------|------|-------------|
| `type` | — | Always `"rtk_frame"` |
| `version` | — | Protocol version, currently `1` |
| `lat` | deg | Latitude, WGS-84 decimal degrees (+N / -S) |
| `lon` | deg | Longitude, WGS-84 decimal degrees (+E / -W) |
| `alt` | m | Altitude above mean sea level |
| `fix_quality` | — | GPS fix type (see table below) |
| `num_sats` | count | Number of satellites in use |
| `hdop` | — | Horizontal dilution of precision. Lower is better (< 1.0 excellent, > 2.0 poor) |
| `speed_knots` | knots | Ground speed from RMC sentence |
| `track_deg` | deg | True course over ground from RMC (0 = North, 90 = East) |
| `rtk_ts` | Unix s | Timestamp from the GPS receiver (from GGA sentence, wall clock) |
| `server_ts` | Unix s | Timestamp when the bridge serialized this frame |
| `source` | — | `"rtk"` = real fix, `"default"` = no fix yet (fallback coordinates shown) |

## `fix_quality` values

| Value | Meaning |
|-------|---------|
| 0 | No fix |
| 1 | Standard GPS |
| 2 | DGPS differential |
| 4 | RTK fixed — highest accuracy (~1 cm) |
| 5 | RTK float — good accuracy (~10 cm) |

## Notes

- **`source: "default"`** means no GPS fix has been received yet. `lat`/`lon` will show the fallback coordinates defined in `config.py`, not a real position.
- **`rtk_ts` vs `server_ts`:** `rtk_ts` comes from the GPS receiver's internal clock (accurate). `server_ts` is the host machine time when the frame was broadcast — the small gap between them is serial read + parse latency.
- `speed_knots` and `track_deg` come from the RMC sentence and are `null` when the RMC status is void (no valid fix).
