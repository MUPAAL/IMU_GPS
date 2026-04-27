"""
autonav_algo.py — Navigation control algorithm.

This is the only file you need to edit to change how the robot steers.

compute() is called by autonav_bridge at CONTROL_HZ.
Inputs are plain numbers — no I/O, no classes, no framework.

Interface contract (do not rename these):
    reset()   — called on start/stop/resume to clear integrator state
    compute() — called every tick, returns (linear m/s, angular rad/s)

Waypoint selection and state machine are handled by autonav_bridge.
"""

# ── Parameters ────────────────────────────────────────────────────────────────

ALGO_DEBUG      = False  # set True to print raw dicts and extracted values each tick

LOOKAHEAD_M     = 2.0   # pure pursuit: lookahead distance to select target waypoint (m)
REACH_TOL_M     = 1.5   # waypoint arrival radius (m)
ARRIVE_FRAMES   = 5     # consecutive frames inside REACH_TOL_M to confirm arrival
DECEL_RADIUS_M  = 3.0   # start decelerating within this distance of the final waypoint (m)
MAX_LINEAR      = 1.0   # max forward speed (m/s)
MIN_LINEAR      = 0.1   # min forward speed when decelerating (m/s)
MAX_ANGULAR     = 1.0   # max angular velocity (rad/s)

KP = 0.8
KI = 0.01
KD = 0.05

# ── PID state ─────────────────────────────────────────────────────────────────

_integral   = 0.0
_prev_error = 0.0


def reset():
    """Clear integrator. Called on start, stop, resume."""
    global _integral, _prev_error
    _integral   = 0.0
    _prev_error = 0.0


# ── Algorithm ─────────────────────────────────────────────────────────────────

def compute(heading_deg, target_bearing_deg, dist_to_wp_m, dist_to_final_m, dt_s):
    """
    heading_deg        : current robot heading, degrees, 0=north, clockwise
    target_bearing_deg : bearing from robot to lookahead waypoint, degrees
    dist_to_wp_m       : distance to the current waypoint (m)
    dist_to_final_m    : distance to the last waypoint (m)
    dt_s               : seconds since last tick

    Returns: (linear m/s, angular rad/s)
    """
    global _integral, _prev_error

    # Heading error in (-180, 180]: positive = target is to the right
    error = (target_bearing_deg - heading_deg + 540) % 360 - 180

    # PID
    _integral  += error * dt_s
    _integral   = max(-50.0, min(50.0, _integral))      # anti-windup clamp
    d_term      = KD * (error - _prev_error) / max(dt_s, 1e-3)
    _prev_error = error

    angular = KP * error + KI * _integral + d_term
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

    # Decelerate near the end of the path
    if dist_to_final_m < DECEL_RADIUS_M:
        linear = MAX_LINEAR * (dist_to_final_m / DECEL_RADIUS_M)
        linear = max(MIN_LINEAR, linear)
    else:
        linear = MAX_LINEAR

    return linear, angular
