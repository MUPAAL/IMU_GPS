"""
autonav_algo.py — Navigation algorithm core.

Edit this file to change how the robot steers.
compute() is called by autonav_bridge at CONTROL_HZ (config.py).

Interface contract (do not rename):
    reset()      — called on start / stop / resume to clear all state
    confirm_wp() — called when user clicks CONTINUE in the dashboard
    compute()    — called every tick, returns (linear, angular, new_wp_idx, arrived, debug)

── Strategy (simple) ────────────────────────────────────────────────────────
 1. Sliding-average GPS filter (MA_WINDOW frames) — suppress RTK multipath noise.
 2. Exponential low-pass heading filter (HEADING_ALPHA) — smooth IMU swing.
 3. Arrival check on RAW GPS. Intermediate waypoints pause and wait for the
    user to click CONTINUE; the final waypoint triggers "arrived" automatically.
 4. Pure Pursuit: pick the first upcoming waypoint >= LOOKAHEAD_M away as the
    steering target.  Fall back to the current waypoint when none qualifies.
 5. PID on bearing error → angular velocity.
 6. Linear speed: full speed straight ahead; stop (linear=0) when error >
    TURN_IN_PLACE_DEG so the robot rotates before moving forward; scale speed
    linearly between TURN_IN_PLACE_DEG and 0° when TURN_SLOWDOWN is on.
    Decelerate near the final waypoint.

── Tuning quick-ref ─────────────────────────────────────────────────────────
  Oscillates left-right  → lower KP, raise DEAD_ZONE_DEG, lower HEADING_ALPHA
  Heading display lags   → raise HEADING_ALPHA
  Skips / circles wp     → raise REACH_TOL_M, lower LOOKAHEAD_M
  Overshoots on turns    → raise TURN_IN_PLACE_DEG, raise KD
  Wrong turn direction   → flip ANGULAR_SIGN  (+1 or -1)
"""

import math
import sys
from pathlib import Path
from collections import deque

# ── Load config ───────────────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

def _c(attr, default):
    return getattr(_cfg, attr, default) if _cfg else default

ALGO_DEBUG    = False  # set True to print debug values every tick

# ── Parameters (all overridable in config.py) ─────────────────────────────────
LOOKAHEAD_M       = _c("AUTONAV_LOOKAHEAD_M",       1.0)
REACH_TOL_M       = _c("AUTONAV_REACH_TOL_M",       0.5)
ARRIVE_FRAMES     = _c("AUTONAV_ARRIVE_FRAMES",      1)
DECEL_RADIUS_M    = _c("AUTONAV_DECEL_RADIUS_M",     1.5)

MAX_LINEAR        = _c("AUTONAV_MAX_LINEAR_VEL",     1.0)
MIN_LINEAR        = _c("AUTONAV_MIN_LINEAR_VEL",     0.1)
MAX_ANGULAR       = _c("AUTONAV_MAX_ANGULAR_VEL",    1.0)

KP                = _c("AUTONAV_PID_KP",             0.15)
KI                = _c("AUTONAV_PID_KI",             0.005)
KD                = _c("AUTONAV_PID_KD",             0.15)

MA_WINDOW         = _c("AUTONAV_MA_WINDOW",          5)
HEADING_ALPHA     = _c("AUTONAV_HEADING_ALPHA",      0.3)

DEAD_ZONE_DEG     = _c("AUTONAV_DEAD_ZONE_DEG",      3.0)
TURN_IN_PLACE_DEG = _c("AUTONAV_TURN_IN_PLACE_DEG",  10.0)
TURN_SLOWDOWN     = _c("AUTONAV_TURN_SLOWDOWN",      True)

ANGULAR_SIGN      = -1  # flip to +1 if robot turns the wrong direction

# ── State ─────────────────────────────────────────────────────────────────────
_lat_buf:        deque      = deque(maxlen=MA_WINDOW)
_lon_buf:        deque      = deque(maxlen=MA_WINDOW)
_heading_filt:   float|None = None

_integral:       float = 0.0
_prev_error:     float = 0.0
_arrive_counter: int   = 0
_waiting_at_wp:  bool  = False  # paused at intermediate wp, waiting for CONTINUE
_confirm_advance: bool = False  # set by confirm_wp() to release the pause


def reset() -> None:
    """Clear all state. Called on start, stop, resume."""
    global _lat_buf, _lon_buf, _heading_filt
    global _integral, _prev_error, _arrive_counter
    global _waiting_at_wp, _confirm_advance
    _lat_buf         = deque(maxlen=MA_WINDOW)
    _lon_buf         = deque(maxlen=MA_WINDOW)
    _heading_filt    = None
    _integral        = 0.0
    _prev_error      = 0.0
    _arrive_counter  = 0
    _waiting_at_wp   = False
    _confirm_advance = False


def confirm_wp() -> None:
    """User clicked CONTINUE — release the pause and advance to the next waypoint."""
    global _confirm_advance
    _confirm_advance = True


# ── Geometry ──────────────────────────────────────────────────────────────────

def _haversine(lat1, lon1, lat2, lon2) -> float:
    R = 6_371_000
    r = math.radians
    dlat = r(lat2 - lat1)
    dlon = r(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(r(lat1))*math.cos(r(lat2))*math.sin(dlon/2)**2
    return 2 * R * math.asin(math.sqrt(a))


def _bearing(lat1, lon1, lat2, lon2) -> float:
    r = math.radians
    dlon = r(lon2 - lon1)
    x = math.sin(dlon) * math.cos(r(lat2))
    y = math.cos(r(lat1))*math.sin(r(lat2)) - math.sin(r(lat1))*math.cos(r(lat2))*math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


# ── Main compute ──────────────────────────────────────────────────────────────

def compute(lat, lon, heading_deg, waypoints, wp_idx, dt_s) -> tuple:
    """
    Returns: (linear m/s, angular rad/s, new_wp_idx, arrived, debug_dict)
    """
    global _lat_buf, _lon_buf, _heading_filt
    global _integral, _prev_error, _arrive_counter
    global _waiting_at_wp, _confirm_advance

    # 1. GPS sliding average
    _lat_buf.append(lat)
    _lon_buf.append(lon)
    flat = sum(_lat_buf) / len(_lat_buf)
    flon = sum(_lon_buf) / len(_lon_buf)

    # 2. Heading low-pass (handles 359°→1° wraparound)
    if _heading_filt is None:
        _heading_filt = heading_deg
    else:
        diff = (heading_deg - _heading_filt + 540) % 360 - 180
        _heading_filt = (_heading_filt + HEADING_ALPHA * diff + 360) % 360

    # 3. Arrival detection (raw GPS for immediate response)
    wp = waypoints[wp_idx]
    d_raw = _haversine(lat, lon, wp["lat"], wp["lon"])

    if _waiting_at_wp:
        if _confirm_advance:
            # User clicked CONTINUE — advance to next waypoint
            wp_idx          += 1
            _arrive_counter  = 0
            _integral        = 0.0
            _waiting_at_wp   = False
            _confirm_advance = False
            _lat_buf.clear()
            _lon_buf.clear()
            wp = waypoints[wp_idx] if wp_idx < len(waypoints) else wp
        else:
            # Hold — robot stopped, waiting for user
            d_final = _haversine(lat, lon, waypoints[-1]["lat"], waypoints[-1]["lon"])
            return 0.0, 0.0, wp_idx, False, {
                "target_bearing_deg":   None,
                "heading_filtered_deg": round(_heading_filt, 1),
                "bearing_error_deg":    None,
                "dist_to_wp_m":         round(d_raw, 2),
                "dist_to_final_m":      round(d_final, 2),
                "waiting_at_wp":        True,
                "waiting_wp_idx":       wp_idx,
            }
    elif d_raw < REACH_TOL_M:
        _arrive_counter += 1
        if _arrive_counter >= ARRIVE_FRAMES:
            _arrive_counter = 0
            _integral       = 0.0
            if wp_idx < len(waypoints) - 1:
                _waiting_at_wp = True   # intermediate — pause for CONTINUE
            else:
                wp_idx += 1             # final — done immediately
    else:
        _arrive_counter = 0

    # 4. Mission complete
    if wp_idx >= len(waypoints):
        return 0.0, 0.0, wp_idx, True, {
            "target_bearing_deg":   None,
            "heading_filtered_deg": round(_heading_filt, 1),
            "bearing_error_deg":    None,
            "dist_to_wp_m":         None,
            "dist_to_final_m":      0.0,
        }

    # 5. Pure Pursuit: first upcoming waypoint >= LOOKAHEAD_M; fall back to current wp
    target = waypoints[wp_idx]
    for w in waypoints[wp_idx:]:
        if _haversine(flat, flon, w["lat"], w["lon"]) >= LOOKAHEAD_M:
            target = w
            break

    # 6. Geometry
    target_bearing = _bearing(flat, flon, target["lat"], target["lon"])
    dist_to_wp     = _haversine(flat, flon, waypoints[wp_idx]["lat"], waypoints[wp_idx]["lon"])
    dist_to_final  = _haversine(flat, flon, waypoints[-1]["lat"], waypoints[-1]["lon"])

    # 7. Bearing error + dead zone
    error = (target_bearing - _heading_filt + 540) % 360 - 180
    if abs(error) < DEAD_ZONE_DEG:
        error = 0.0

    # 8. PID → angular
    _integral  += error * dt_s
    _integral   = max(-50.0, min(50.0, _integral))
    d_term      = KD * (error - _prev_error) / max(dt_s, 1e-3)
    _prev_error = error

    angular = KP * error + KI * _integral + d_term
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
    angular *= ANGULAR_SIGN

    # 9. Linear speed
    # Decelerate near the final waypoint
    if dist_to_final < DECEL_RADIUS_M:
        linear = max(MIN_LINEAR, MAX_LINEAR * (dist_to_final / DECEL_RADIUS_M))
    else:
        linear = MAX_LINEAR

    # Stop forward motion for large heading errors (rotate in place first)
    if TURN_IN_PLACE_DEG > 0 and abs(error) > TURN_IN_PLACE_DEG:
        linear = 0.0
    elif TURN_SLOWDOWN and error != 0.0:
        turn_scale = max(0.0, 1.0 - abs(error) / 60.0)
        linear = max(MIN_LINEAR, linear * turn_scale)

    if ALGO_DEBUG:
        print(f"[ALGO] wp={wp_idx}/{len(waypoints)} error={error:.1f}° "
              f"target={target_bearing:.1f}° hdg={_heading_filt:.1f}° "
              f"dist={dist_to_wp:.1f}m linear={linear:.2f} angular={angular:.2f}")

    return linear, angular, wp_idx, False, {
        "target_bearing_deg":   round(target_bearing, 1),
        "heading_filtered_deg": round(_heading_filt, 1),
        "bearing_error_deg":    round(error, 1),
        "dist_to_wp_m":         round(dist_to_wp, 2),
        "dist_to_final_m":      round(dist_to_final, 2),
    }
