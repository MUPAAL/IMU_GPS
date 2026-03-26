"""
PathCamSource — OAK-D camera plugin with yellow path detection and target selection.

Uses depthai v3 Camera node API to capture frames, applies HSV-based yellow
masking, morphological cleanup, and contour scoring to identify the best
path target in the bottom-center ROI.

display_mode options:
    "target"    — yellow-only view with the best contour highlighted green (default)
    "composite" — three panels side by side: Live | Yellow | Target (debug)
"""

from __future__ import annotations

import logging

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import depthai as dai
except ImportError:
    dai = None

from . import FrameSource, register_plugin

logger = logging.getLogger(__name__)

# ── Yellow HSV thresholds ─────────────────────────────────────────────────────
_YELLOW_LOWER = np.array([15, 50, 50])
_YELLOW_UPPER = np.array([37, 255, 255])

# ── ROI bounds as fractions of frame size ────────────────────────────────────
_ROI_X_MIN, _ROI_X_MAX = 0.25, 0.75
_ROI_Y_MIN = 0.55  # top edge of ROI; extends to bottom of frame

# ── Minimum contour area to consider (px²) ───────────────────────────────────
_MIN_CONTOUR_AREA = 40

# ── Composite panel size per panel ───────────────────────────────────────────
_PANEL_W = 640
_PANEL_H = 360


@register_plugin
class PathCamSource(FrameSource):
    """
    OAK-D camera source with yellow tape path detection.

    Captures full-resolution frames via depthai v3 Camera node, applies
    HSV yellow masking and contour scoring to select the best path target.
    """

    PLUGIN_NAME = "path_cam"
    PLUGIN_LABEL = "Path Detection"
    PLUGIN_DESCRIPTION = (
        "Yellow tape path detection with target selection via HSV masking"
    )

    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {"key": "device_ip",      "type": "str",  "default": None,        "label": "Device IP"},
            {"key": "capture_width",  "type": "int",  "default": 1920,        "label": "Capture Width"},
            {"key": "capture_height", "type": "int",  "default": 1080,        "label": "Capture Height"},
            {"key": "display_mode",   "type": "str",  "default": "target",    "label": "Display Mode (target/composite)"},
        ]

    def __init__(self, **kwargs) -> None:
        self._device_ip      = kwargs.get("device_ip")
        self._capture_width  = kwargs.get("capture_width", 1920)
        self._capture_height = kwargs.get("capture_height", 1080)
        self._display_mode   = kwargs.get("display_mode", "target")
        self._device         = None
        self._pipeline       = None
        self._queue          = None

    # ── INPUT ─────────────────────────────────────────────────────────────────

    def open(self) -> None:
        """Create depthai pipeline and start the camera."""
        if dai is None:
            raise RuntimeError("depthai library not installed")
        if cv2 is None:
            raise RuntimeError("opencv-python library not installed")

        try:
            if self._device_ip:
                self._device = dai.Device(dai.DeviceInfo(self._device_ip))
            else:
                self._device = dai.Device()

            self._pipeline = dai.Pipeline(self._device)
            cam = self._pipeline.create(dai.node.Camera).build()
            self._queue = (
                cam.requestOutput((self._capture_width, self._capture_height))
                .createOutputQueue()
            )
            self._pipeline.start()
            logger.info(
                "PathCamSource: opened (ip=%s, %dx%d, mode=%s)",
                self._device_ip, self._capture_width, self._capture_height,
                self._display_mode,
            )
        except Exception as exc:
            logger.error("PathCamSource: failed to open: %s", exc)
            self.close()
            raise

    def close(self) -> None:
        """Stop pipeline and release depthai resources."""
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as exc:
                logger.warning("PathCamSource: error stopping pipeline: %s", exc)
            self._pipeline = None
        if self._device is not None:
            try:
                self._device.close()
            except Exception as exc:
                logger.warning("PathCamSource: error closing device: %s", exc)
            self._device = None
        self._queue = None

    def get_frame(self) -> np.ndarray | None:
        """Non-blocking frame grab. Returns processed BGR array or None."""
        if self._queue is None:
            return None
        try:
            in_frame = self._queue.tryGet()
            if in_frame is None:
                return None
            return self._process_frame(in_frame.getCvFrame())
        except Exception as exc:
            logger.warning("PathCamSource: get_frame error: %s", exc)
            return None

    # ── CORE ──────────────────────────────────────────────────────────────────

    def _process_frame(self, img: np.ndarray) -> np.ndarray:
        """Apply yellow detection and contour scoring; return composed view."""
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
            return self._build_composite(img, yellow_only, target_view)
        return target_view

    def _build_composite(
        self,
        live: np.ndarray,
        yellow: np.ndarray,
        target: np.ndarray,
    ) -> np.ndarray:
        """Return a single frame: three panels concatenated horizontally."""
        panels = [
            cv2.resize(live,   (_PANEL_W, _PANEL_H)),
            cv2.resize(yellow, (_PANEL_W, _PANEL_H)),
            cv2.resize(target, (_PANEL_W, _PANEL_H)),
        ]
        return np.concatenate(panels, axis=1)  # shape: (_PANEL_H, _PANEL_W*3, 3)
