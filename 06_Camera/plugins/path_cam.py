"""
PathCamProcessor — yellow path detection applied on top of the running RGB stream.

This is a FrameProcessor: it receives a BGR frame from the already-running
camera source and returns a processed frame.  The camera is never restarted
when switching to/from this processor.

display_mode options:
    "target"    — yellow-only view with the best contour highlighted green (default)
    "composite" — three panels side by side: Live | Yellow | Target (debug)
"""

from __future__ import annotations

import logging

import numpy as np

import cv2


from . import FrameProcessor, register_processor

logger = logging.getLogger(__name__)

# ── Yellow HSV thresholds ─────────────────────────────────────────────────────
_YELLOW_LOWER = np.array([15, 50, 50])
_YELLOW_UPPER = np.array([37, 255, 255])

# ── ROI bounds as fractions of frame size ────────────────────────────────────
_ROI_X_MIN, _ROI_X_MAX = 0.25, 0.75
_ROI_Y_MIN = 0.55  # top edge; extends to bottom of frame

# ── Minimum contour area to consider (px²) ───────────────────────────────────
_MIN_CONTOUR_AREA = 40


@register_processor
class PathCamProcessor(FrameProcessor):
    """
    Yellow tape path detection processor.

    Applies HSV yellow masking and contour scoring to select the best
    path target in the bottom-center ROI.  Works on any BGR frame;
    does not manage the camera device.
    """

    PROCESSOR_NAME = "path_cam"
    PROCESSOR_LABEL = "Path Detection"
    PROCESSOR_DESCRIPTION = (
        "Yellow tape path detection with target selection via HSV masking"
    )

    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {
                "key": "display_mode",
                "type": "enum",
                "default": "target",
                "label": "Display Mode",
                "options": [
                    {"value": "target", "label": "target"},
                    {"value": "composite", "label": "composite"},
                ],
            },
        ]

    def __init__(self, **kwargs) -> None:
        self._display_mode = kwargs.get("display_mode", "target")
        self.last_target = None  # (x_norm, y_norm) or None; -1=left, 0=center, +1=right

    def reconfigure(self, **kwargs) -> None:
        self._display_mode = kwargs.get("display_mode", self._display_mode)

    # ── CORE ──────────────────────────────────────────────────────────────────

    @classmethod
    def required_streams(cls) -> list[str]:
        return ["rgb"]

    def process(self, frames: dict[str, np.ndarray | None]) -> np.ndarray | None:
        """Apply yellow detection and contour scoring; return composed view."""
        img = frames.get("rgb")
        if img is None:
            return None
        h, w = img.shape[:2]

        # HSV yellow mask
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, _YELLOW_LOWER, _YELLOW_UPPER)

        # Morphological closing to fill small gaps
        kernel = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        # Yellow-only image
        yellow_only = cv2.bitwise_and(img, img, mask=yellow_mask)

        # ROI: bottom-center region
        roi_x1 = int(w * _ROI_X_MIN)
        roi_x2 = int(w * _ROI_X_MAX)
        roi_y1 = int(h * _ROI_Y_MIN)
        roi_mask = yellow_mask[roi_y1:h, roi_x1:roi_x2]

        # Find contours in ROI
        contours, _ = cv2.findContours(
            roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Score: prefer center-x and close to bottom
        best_contour = None
        best_score = None
        roi_center_x = (roi_x2 - roi_x1) // 2
        roi_height = h - roi_y1

        for contour in contours:
            if cv2.contourArea(contour) < _MIN_CONTOUR_AREA:
                continue
            x, y, cw, ch = cv2.boundingRect(contour)
            center_distance = abs((x + cw // 2) - roi_center_x)
            bottom_distance = abs(roi_height - (y + ch))
            score = center_distance + 2 * bottom_distance
            if best_score is None or score < best_score:
                best_score = score
                best_contour = contour

        # Store normalized target for external reading
        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"]) + roi_x1
                cy = int(M["m01"] / M["m00"]) + roi_y1
                self.last_target = (
                    round((cx / w - 0.5) * 2, 3),  # -1=left, 0=center, +1=right
                    round(cy / h, 3),               # 0=top, 1=bottom
                )
        else:
            self.last_target = None

        # Highlight best contour green in target view
        target_view = yellow_only.copy()
        if best_contour is not None:
            shifted = best_contour.copy()
            shifted[:, 0, 0] += roi_x1
            shifted[:, 0, 1] += roi_y1
            chosen_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(chosen_mask, [shifted], -1, 255, thickness=cv2.FILLED)
            target_view[chosen_mask > 0] = (0, 255, 0)

        # OUTPUT ──────────────────────────────────────────────────────────────
        if self._display_mode == "composite":
            panels = [
                img,
                yellow_only,
                target_view
            ]
            return np.concatenate(panels, axis=1)
        return target_view