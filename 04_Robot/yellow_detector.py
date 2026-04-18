"""
yellow_detector.py — Yellow tape path detection MJPEG proxy.

Reads the raw MJPEG stream from camera_bridge (CAM1 or CAM2),
applies HSV yellow detection, and re-serves three output streams:

    /live      — raw passthrough (no processing)
    /mask      — black + yellow only
    /detect    — composite: Live | Mask | Target (green contour)

Usage:
    python yellow_detector.py
    python yellow_detector.py --cam1-url http://10.95.76.11:8080 --port 8082

All streams available at:
    http://localhost:8082/live
    http://localhost:8082/mask
    http://localhost:8082/detect
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

# ── config.py integration ─────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
try:
    import config as _cfg
except ImportError:
    _cfg = None

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("yellow_detector")

# ── Yellow HSV thresholds ─────────────────────────────────────────────────────
YELLOW_LOWER = np.array([15, 50, 50])
YELLOW_UPPER = np.array([37, 255, 255])

# ── ROI bounds as fractions of frame size ────────────────────────────────────
ROI_X_MIN, ROI_X_MAX = 0.25, 0.75
ROI_Y_MIN = 0.55

# ── Minimum contour area (px²) ────────────────────────────────────────────────
MIN_CONTOUR_AREA = 40

# ── JPEG quality ──────────────────────────────────────────────────────────────
JPEG_QUALITY = 80


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 1 — FRAME PROCESSOR
# ══════════════════════════════════════════════════════════════════════════════

def process_frame(img: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Apply yellow detection to a BGR frame.

    Returns:
        live    — original frame unchanged
        mask    — black + yellow only
        detect  — composite: Live | Mask | Target
    """
    h, w = img.shape[:2]

    # HSV yellow mask
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

    # Morphological closing to fill small gaps
    kernel = np.ones((3, 3), np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

    # Yellow-only (black background)
    mask_view = cv2.bitwise_and(img, img, mask=yellow_mask)

    # ROI: bottom-center region
    roi_x1 = int(w * ROI_X_MIN)
    roi_x2 = int(w * ROI_X_MAX)
    roi_y1 = int(h * ROI_Y_MIN)
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
        if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
            continue
        x, y, cw, ch = cv2.boundingRect(contour)
        center_distance = abs((x + cw // 2) - roi_center_x)
        bottom_distance = abs(roi_height - (y + ch))
        score = center_distance + 2 * bottom_distance
        if best_score is None or score < best_score:
            best_score = score
            best_contour = contour

    # Build target view: yellow on black + green for best contour
    target_view = mask_view.copy()
    if best_contour is not None:
        shifted = best_contour.copy()
        shifted[:, 0, 0] += roi_x1
        shifted[:, 0, 1] += roi_y1
        chosen_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(chosen_mask, [shifted], -1, 255, thickness=cv2.FILLED)
        target_view[chosen_mask > 0] = (0, 255, 0)

        # Draw crosshair on live view showing target center
        M = cv2.moments(best_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"]) + roi_x1
            cy = int(M["m01"] / M["m00"]) + roi_y1
            cv2.circle(img, (cx, cy), 8, (0, 255, 0), 2)
            cv2.line(img, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
            cv2.line(img, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

    # Draw ROI box on live view
    cv2.rectangle(img, (roi_x1, roi_y1), (roi_x2, h), (0, 180, 255), 1)

    # Composite: resize all to same height then concatenate
    panel_w = w // 3
    live_panel   = cv2.resize(img,         (panel_w, h))
    mask_panel   = cv2.resize(mask_view,   (panel_w, h))
    target_panel = cv2.resize(target_view, (panel_w, h))

    # Add labels
    for panel, label in [
        (live_panel,   "LIVE"),
        (mask_panel,   "MASK"),
        (target_panel, "TARGET"),
    ]:
        cv2.putText(
            panel, label, (6, 16),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA,
        )

    composite = np.concatenate([live_panel, mask_panel, target_panel], axis=1)

    return img, mask_view, composite


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 2 — FRAME BUFFER
# ══════════════════════════════════════════════════════════════════════════════

class FrameBuffer:
    """Thread-safe triple buffer: live / mask / detect JPEGs."""

    def __init__(self):
        self._lock = threading.Lock()
        self._live:    bytes = b""
        self._mask:    bytes = b""
        self._detect:  bytes = b""

    def update(self, live: np.ndarray, mask: np.ndarray, detect: np.ndarray) -> None:
        def enc(frame: np.ndarray) -> bytes:
            ok, buf = cv2.imencode(
                ".jpg", frame,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
            )
            return buf.tobytes() if ok else b""

        live_j   = enc(live)
        mask_j   = enc(mask)
        detect_j = enc(detect)

        with self._lock:
            self._live   = live_j
            self._mask   = mask_j
            self._detect = detect_j

    def get(self, stream: str) -> bytes:
        with self._lock:
            if stream == "live":
                return self._live
            elif stream == "mask":
                return self._mask
            else:
                return self._detect


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 3 — CAPTURE LOOP
# ══════════════════════════════════════════════════════════════════════════════

def capture_loop(cam_url: str, buffer: FrameBuffer, startup_delay: float = 5.0) -> None:
    """Continuously read from MJPEG source, process, update buffer."""
    if startup_delay > 0:
        logger.info("capture_loop: waiting %.1fs for camera to start...", startup_delay)
        time.sleep(startup_delay)
    while True:
        logger.info("capture_loop: connecting to %s", cam_url)
        cap = cv2.VideoCapture(cam_url)
        if not cap.isOpened():
            logger.warning("capture_loop: cannot open %s — retry in 3s", cam_url)
            time.sleep(3)
            continue

        logger.info("capture_loop: connected to %s", cam_url)
        consecutive_failures = 0

        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                consecutive_failures += 1
                if consecutive_failures > 10:
                    logger.warning("capture_loop: too many failures — reconnecting")
                    break
                time.sleep(0.05)
                continue

            consecutive_failures = 0
            try:
                live, mask, detect = process_frame(frame)
                buffer.update(live, mask, detect)
            except Exception as exc:
                logger.warning("capture_loop: process error: %s", exc)

        cap.release()
        logger.info("capture_loop: disconnected — retry in 2s")
        time.sleep(2)

# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 4 — MJPEG HTTP SERVER
# ══════════════════════════════════════════════════════════════════════════════

def make_handler(buffer: FrameBuffer):
    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?")[0].rstrip("/")

            # Route: /live  /mask  /detect  (default → /detect)
            if path == "/live":
                stream = "live"
            elif path == "/mask":
                stream = "mask"
            else:
                stream = "detect"

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            try:
                while True:
                    jpeg = buffer.get(stream)
                    if not jpeg:
                        time.sleep(0.03)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(0.04)  # ~25 fps
            except Exception:
                pass

        def log_message(self, fmt, *args):
            pass  # suppress per-request logs

    return _Handler


# ══════════════════════════════════════════════════════════════════════════════
# BLOCK 5 — MAIN
# ══════════════════════════════════════════════════════════════════════════════

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Yellow tape path detection MJPEG proxy")
    parser.add_argument(
        "--cam1-url",
        default=f"http://{_cfg.CAM1_IP if _cfg else '10.95.76.11'}:{_cfg.CAM1_STREAM_PORT if _cfg else 8080}/",
        help="Camera 1 MJPEG URL",
    )
    parser.add_argument(
        "--cam2-url",
        default=f"http://{_cfg.CAM2_IP if _cfg else '10.95.76.10'}:{_cfg.CAM2_STREAM_PORT if _cfg else 8081}/",
        help="Camera 2 MJPEG URL",
    )
    parser.add_argument(
        "--port", type=int, default=8082,
        help="Port to serve processed streams (default: 8082)",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    buffer = FrameBuffer()

    # Start with cam1 by default
    capture_thread = threading.Thread(
        target=capture_loop,
        args=(args.cam1_url, buffer, 5.0),
        daemon=True,
        name="capture",
    )
    capture_thread.start()

    logger.info("yellow_detector: serving on http://0.0.0.0:%d", args.port)
    logger.info("  /live   → raw feed with target crosshair")
    logger.info("  /mask   → black + yellow only")
    logger.info("  /detect → composite (Live | Mask | Target)")

    handler = make_handler(buffer)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("yellow_detector: stopped")


if __name__ == "__main__":
    main()