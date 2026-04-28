"""
autonav_algo.py — Navigation algorithm core.

This is the only file you need to edit to change how the robot steers.

compute() is called by autonav_bridge at CONTROL_HZ (config.py).
Inputs are raw sensor values — algo handles all filtering and geometry internally.

Interface contract (do not rename these):
    reset()   — called on start/stop/resume to clear all state
    compute() — called every tick, returns (linear, angular, new_wp_idx, arrived, debug)

Oscillation tuning guide (if robot swings left-right near waypoints):
    1. Lower CONTROL_HZ in config.py (5→2 Hz): longer gap between commands,
       gives robot more time to physically settle before next correction.
    2. Lower HEADING_ALPHA (0.2→0.1): heading filter responds more slowly,
       ignores transient swings caused by inertia.
    3. Lower KP (0.8→0.2): prevents full-speed angular output at small errors.
       With KP=0.8, any error >1.25° already saturates MAX_ANGULAR — essentially
       bang-bang control, which causes sustained oscillation.
    4. Raise DEAD_ZONE_DEG (3→5): if oscillation amplitude is ±5°, widen the
       dead zone to cover it so small errors don't trigger corrections.
    5. Raise KD (0.05→0.15): stronger derivative damping to absorb overshoot.
    Recommended first test: KP=0.2, HEADING_ALPHA=0.1, DEAD_ZONE_DEG=5.0
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

ALGO_DEBUG      = False  # set True to print filter + PID values every tick

# ── Geometry / waypoint ───────────────────────────────────────────────────────
LOOKAHEAD_M     = _c("AUTONAV_LOOKAHEAD_M",     1.0)
REACH_TOL_M     = _c("AUTONAV_REACH_TOL_M",     0.5)
ARRIVE_FRAMES   = _c("AUTONAV_ARRIVE_FRAMES",   1)
DECEL_RADIUS_M  = _c("AUTONAV_DECEL_RADIUS_M",  1.5)

# ── Speed ─────────────────────────────────────────────────────────────────────
MAX_LINEAR      = _c("AUTONAV_MAX_LINEAR_VEL",  1.0)
MIN_LINEAR      = _c("AUTONAV_MIN_LINEAR_VEL",  0.1)
MAX_ANGULAR     = _c("AUTONAV_MAX_ANGULAR_VEL", 1.0)

# ── PID ───────────────────────────────────────────────────────────────────────
KP              = _c("AUTONAV_PID_KP",          0.15)
KI              = _c("AUTONAV_PID_KI",          0.005)
KD              = _c("AUTONAV_PID_KD",          0.15)

# ── Filters ───────────────────────────────────────────────────────────────────
MA_WINDOW       = _c("AUTONAV_MA_WINDOW",       5)
HEADING_ALPHA   = _c("AUTONAV_HEADING_ALPHA",   0.3)

# ── Steering ──────────────────────────────────────────────────────────────────
DEAD_ZONE_DEG   = _c("AUTONAV_DEAD_ZONE_DEG",   3.0)
TURN_SLOWDOWN   = _c("AUTONAV_TURN_SLOWDOWN",   True)
TURN_IN_PLACE_DEG = _c("AUTONAV_TURN_IN_PLACE_DEG", 10.0)

ANGULAR_SIGN    = -1    # +1: positive heading error → turn left/CCW.
                        # Set -1 if robot turns the wrong way.

# ── Module-level state ────────────────────────────────────────────────────────

_lat_buf:        deque      = deque(maxlen=MA_WINDOW)
_lon_buf:        deque      = deque(maxlen=MA_WINDOW)
_heading_filt:   float|None = None

_integral:       float = 0.0
_prev_error:     float = 0.0
_arrive_counter: int   = 0


def reset() -> None:
    """Clear all state. Called on start, stop, resume."""
    global _lat_buf, _lon_buf, _heading_filt
    global _integral, _prev_error, _arrive_counter
    _lat_buf        = deque(maxlen=MA_WINDOW)
    _lon_buf        = deque(maxlen=MA_WINDOW)
    _heading_filt   = None
    _integral       = 0.0
    _prev_error     = 0.0
    _arrive_counter = 0


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    R = 6_371_000
    r = math.radians
    dlat = r(lat2 - lat1)
    dlon = r(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(r(lat1)) * math.cos(r(lat2)) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from (lat1,lon1) to (lat2,lon2), degrees [0, 360)."""
    r = math.radians
    dlon = r(lon2 - lon1)
    x = math.sin(dlon) * math.cos(r(lat2))
    y = math.cos(r(lat1)) * math.sin(r(lat2)) - math.sin(r(lat1)) * math.cos(r(lat2)) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


# ── Algorithm ─────────────────────────────────────────────────────────────────

def compute(
    lat:         float,
    lon:         float,
    heading_deg: float,
    waypoints:   list,
    wp_idx:      int,
    dt_s:        float,
) -> tuple:
    """
    lat, lon      : raw RTK coordinates (filtered internally via sliding average)
    heading_deg   : raw IMU heading, degrees, 0=north clockwise (filtered internally)
    waypoints     : list of {"lat": ..., "lon": ...}
    wp_idx        : current waypoint index (pass back the value returned last tick)
    dt_s          : seconds since last tick

    Returns: (linear m/s, angular rad/s, new_wp_idx, arrived, debug_dict)
      linear      : forward speed before speed_ratio scaling (bridge multiplies)
      angular     : already includes ANGULAR_SIGN
      new_wp_idx  : updated waypoint index for bridge state machine
      arrived     : True when all waypoints are complete
      debug_dict  : intermediate values forwarded to web dashboard
    """
    global _lat_buf, _lon_buf, _heading_filt
    global _integral, _prev_error, _arrive_counter

    # ── 1. Filter position (sliding average) ──────────────────────────────────
    # Averages the last MA_WINDOW GPS readings to suppress RTK multipath jumps.
    # A sudden 0.3 m position spike translates to ~8° bearing error at 2 m range;
    # the moving average dilutes such spikes across the window.
    _lat_buf.append(lat)
    _lon_buf.append(lon)
    flat = sum(_lat_buf) / len(_lat_buf)
    flon = sum(_lon_buf) / len(_lon_buf)

    # ── 2. Filter heading (exponential low-pass) ──────────────────────────────
    # Smooths IMU heading to absorb inertial swings after commands are sent.
    # The 359°→1° wraparound is handled by computing the shortest angular diff first.
    if _heading_filt is None:
        _heading_filt = heading_deg          # cold start: seed filter with first reading
    else:
        diff = (heading_deg - _heading_filt + 540) % 360 - 180   # shortest-path diff
        _heading_filt = (_heading_filt + HEADING_ALPHA * diff + 360) % 360

    if ALGO_DEBUG:
        print(f"[FILTER] raw_lat={lat:.7f} flat={flat:.7f} "
              f"raw_hdg={heading_deg:.1f} fhdg={_heading_filt:.1f}")

    # ── 3. Arrival detection ──────────────────────────────────────────────────
    # Uses RAW GPS (not filtered) so the check responds immediately even when
    # the MA filter is still catching up. Single-frame confirmation: as soon as
    # raw distance < REACH_TOL_M, advance wp_idx and flush the position filter
    # so the bearing to the next waypoint is computed fresh next tick.
    wp = waypoints[wp_idx]
    d_to_wp_raw = _haversine(lat, lon, wp["lat"], wp["lon"])

    if d_to_wp_raw < REACH_TOL_M:
        _arrive_counter += 1
        if _arrive_counter >= ARRIVE_FRAMES:
            wp_idx += 1
            _arrive_counter = 0
            _integral = 0.0
            _lat_buf.clear()   # flush so next bearing uses fresh GPS
            _lon_buf.clear()
    else:
        _arrive_counter = 0

    d_to_wp = _haversine(flat, flon, wp["lat"], wp["lon"])  # filtered, for display

    # ── 4. Check mission complete ─────────────────────────────────────────────
    if wp_idx >= len(waypoints):
        debug = {
            "target_bearing_deg":   None,
            "heading_filtered_deg": round(_heading_filt, 1),
            "bearing_error_deg":    None,
            "dist_to_wp_m":         None,
            "dist_to_final_m":      0.0,
        }
        return 0.0, 0.0, wp_idx, True, debug

    # ── 5. Pure Pursuit: pick lookahead waypoint ──────────────────────────────
    # Scans forward from current wp_idx and picks the first waypoint that is at
    # least LOOKAHEAD_M away. Falls back to the final waypoint if all remaining
    # waypoints are closer than LOOKAHEAD_M (robot is near the end of the path).
    target = waypoints[-1]
    for w in waypoints[wp_idx:]:
        if _haversine(flat, flon, w["lat"], w["lon"]) >= LOOKAHEAD_M:
            target = w
            break

    # ── 6. Geometry (all from filtered position) ──────────────────────────────
    target_bearing = _bearing(flat, flon, target["lat"], target["lon"])
    dist_to_wp     = _haversine(flat, flon, waypoints[wp_idx]["lat"], waypoints[wp_idx]["lon"])
    dist_to_final  = _haversine(flat, flon, waypoints[-1]["lat"], waypoints[-1]["lon"])

    # ── 7. Bearing error + dead zone ──────────────────────────────────────────
    # Positive error = target is to the right of current heading → turn right.
    # Dead zone suppresses micro-corrections that cause continuous motor chatter.
    # If robot oscillates ±5°, raise DEAD_ZONE_DEG to 5.0 to break the cycle.
    error = (target_bearing - _heading_filt + 540) % 360 - 180
    if abs(error) < DEAD_ZONE_DEG:
        error = 0.0

    # ── 8. PID ────────────────────────────────────────────────────────────────
    # Integral anti-windup: clamp prevents runaway accumulation during long turns.
    # Derivative uses previous filtered error, not raw sensor diff, to reduce noise.
    # TUNING NOTE: with KP=0.8, errors >1.25° saturate MAX_ANGULAR → bang-bang
    # behaviour → oscillation. Lower KP to 0.1~0.3 for proportional response.
    _integral  += error * dt_s
    _integral   = max(-50.0, min(50.0, _integral))
    d_term      = KD * (error - _prev_error) / max(dt_s, 1e-3)
    _prev_error = error

    angular = KP * error + KI * _integral + d_term
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
    angular *= ANGULAR_SIGN

    # ── 9. Linear speed with end-of-path deceleration + turn slowdown ────────
    if dist_to_final < DECEL_RADIUS_M:
        linear = MAX_LINEAR * (dist_to_final / DECEL_RADIUS_M)
        linear = max(MIN_LINEAR, linear)
    else:
        linear = MAX_LINEAR

    if TURN_IN_PLACE_DEG > 0 and abs(error) > TURN_IN_PLACE_DEG:
        linear = 0.0
    elif TURN_SLOWDOWN and error != 0.0:
        # Scale speed down linearly with heading error: full speed at 0°, MIN_LINEAR at 60°+
        turn_scale = max(0.0, 1.0 - abs(error) / 60.0)
        linear = max(MIN_LINEAR, linear * turn_scale)

    if ALGO_DEBUG:
        print(f"[ALGO] error={error:.1f}° target={target_bearing:.1f}° "
              f"hdg={_heading_filt:.1f}° dist={dist_to_wp:.1f}m "
              f"linear={linear:.3f} angular={angular:.3f} "
              f"wp={wp_idx}/{len(waypoints)}")

    debug = {
        "target_bearing_deg":   round(target_bearing, 1),
        "heading_filtered_deg": round(_heading_filt, 1),
        "bearing_error_deg":    round(error, 1),
        "dist_to_wp_m":         round(dist_to_wp, 2),
        "dist_to_final_m":      round(dist_to_final, 2),
    }
    return linear, angular, wp_idx, False, debug
