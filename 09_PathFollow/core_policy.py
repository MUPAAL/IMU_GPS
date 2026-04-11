"""
09_PathFollow/core_policy.py

Core algorithm file (edit this file for future algorithm changes):
- observation extraction
- command computation
- safety guard
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

# Algorithm parameters
MAX_LINEAR = 0.30
MAX_ANGULAR = 0.60
K_ANG = 0.9
NO_PATH_LINEAR = 0.0
NO_PATH_ANGULAR = 0.0

YELLOW_HSV_LOW = np.array([18, 80, 80], dtype=np.uint8)
YELLOW_HSV_HIGH = np.array([40, 255, 255], dtype=np.uint8)


@dataclass
class PathObservation:
    found: bool = False
    center_x_px: float | None = None
    center_y_px: float | None = None
    center_x_norm: float | None = None
    heading_err_deg: float | None = None
    confidence: float = 0.0
    frame_w: int = 0
    frame_h: int = 0


@dataclass
class CmdVel:
    linear: float = 0.0
    angular: float = 0.0
    mode: str = "center_follow"
    reason: str = "no_path"


def extract_observation(frame: np.ndarray) -> tuple[PathObservation, np.ndarray]:
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_HSV_LOW, YELLOW_HSV_HIGH)

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    obs = PathObservation(frame_w=w, frame_h=h)
    vis = frame.copy()

    ys, xs = np.where(mask > 0)
    if xs.size < 50:
        return obs, vis

    cx = float(np.mean(xs))
    cy = float(np.mean(ys))
    center_x_norm = ((cx - (w / 2.0)) / (w / 2.0))
    center_x_norm = float(np.clip(center_x_norm, -1.0, 1.0))
    heading_err_deg = center_x_norm * 45.0
    confidence = float(np.clip(xs.size / float(w * h * 0.12), 0.0, 1.0))

    obs.found = True
    obs.center_x_px = round(cx, 2)
    obs.center_y_px = round(cy, 2)
    obs.center_x_norm = round(center_x_norm, 4)
    obs.heading_err_deg = round(heading_err_deg, 2)
    obs.confidence = round(confidence, 3)

    cv2.circle(vis, (int(cx), int(cy)), 6, (0, 255, 0), -1)
    cv2.line(vis, (w // 2, h), (w // 2, 0), (255, 255, 255), 1)
    cv2.putText(
        vis,
        f"err={obs.center_x_norm:+.3f} conf={obs.confidence:.2f}",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    return obs, vis


def compute_command(obs: PathObservation) -> CmdVel:
    if not obs.found or obs.center_x_norm is None:
        return CmdVel(linear=NO_PATH_LINEAR, angular=NO_PATH_ANGULAR, reason="no_path")

    ang = -K_ANG * obs.center_x_norm
    ang = float(np.clip(ang, -MAX_ANGULAR, MAX_ANGULAR))

    slow = 1.0 - min(abs(obs.center_x_norm), 1.0)
    lin = MAX_LINEAR * (0.35 + 0.65 * slow)
    lin = float(np.clip(lin, 0.0, MAX_LINEAR))

    return CmdVel(linear=round(lin, 3), angular=round(ang, 3), reason="follow")


def safety_guard(cmd: CmdVel, obs: PathObservation) -> CmdVel:
    # Reserved for future safety policies.
    if not obs.found:
        return CmdVel(linear=NO_PATH_LINEAR, angular=NO_PATH_ANGULAR, reason="safety_no_path")
    return cmd
