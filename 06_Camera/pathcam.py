"""
Robot-side local camera viewer with yellow masking and target selection.
Displays:
1. Normal live video
2. Yellow-only view
3. Yellow-only view where the selected target blob is shown in green

Usage:
    cd m2_system/00_robot_side
    python cam.py

Controls:
    q - quit
"""

import logging
import os
import signal
from pathlib import Path

import cv2
import depthai as dai
import numpy as np

# Default to Camera 2, since that is now the front camera
_DEVICE_IP = os.environ.get("DEVICE_IP", "10.95.76.11")

# ── Logging configuration ──────────────────────────────────
_py_name = Path(__file__).stem
Path("log").mkdir(exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(f"log/{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

_stop = False


def _signal_handler(signum, frame):
    global _stop
    logger.info(f"Signal {signum} received, stopping...")
    _stop = True


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def main():
    global _stop
    logger.info(f"Starting camera viewer with yellow mask using device IP: {_DEVICE_IP}")

    try:
        logger.info(f"Connecting to device IP: {_DEVICE_IP}")
        _device = dai.Device(dai.DeviceInfo(_DEVICE_IP))
        pipeline_cm = dai.Pipeline(_device)

        with pipeline_cm as pipeline:
            cam = pipeline.create(dai.node.Camera).build()
            q = cam.requestOutput((1920, 1080)).createOutputQueue()

            pipeline.start()

            # Create windows
            cv2.namedWindow("Live Camera View", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Yellow Only View", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Chosen Path Target View", cv2.WINDOW_NORMAL)

            # Place windows side by side
            cv2.moveWindow("Live Camera View", 0, 0)
            cv2.moveWindow("Yellow Only View", 650, 0)
            cv2.moveWindow("Chosen Path Target View", 1300, 0)

            while pipeline.isRunning() and not _stop:
                frame = q.get()

                if frame is not None:
                    img = frame.getCvFrame()
                    h, w = img.shape[:2]

                    # Convert to HSV
                    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                    # Yellow range
                    lower_yellow = np.array([15, 50, 50])
                    upper_yellow = np.array([37, 255, 255])

                    # Mask yellow
                    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                    # Morphological cleanup
                    kernel = np.ones((3, 3), np.uint8)
                    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

                    # Yellow-only output
                    yellow_only = cv2.bitwise_and(img, img, mask=yellow_mask)

                    # -------------------------------------------------
                    # Select the yellow blob to follow
                    # -------------------------------------------------

                    # Search only in bottom-middle region
                    roi_x1 = int(w * 0.25)
                    roi_x2 = int(w * 0.75)
                    roi_y1 = int(h * 0.55)
                    roi_y2 = h

                    roi_mask = yellow_mask[roi_y1:roi_y2, roi_x1:roi_x2]

                    contours, _ = cv2.findContours(
                        roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )

                    best_contour = None
                    best_score = None

                    roi_center_x = (roi_x2 - roi_x1) // 2
                    roi_bottom_y = roi_y2 - roi_y1

                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if area < 40:
                            continue

                        x, y, cw, ch = cv2.boundingRect(contour)
                        contour_center_x = x + (cw // 2)
                        contour_bottom_y = y + ch

                        # Prefer contours near the middle and near the bottom
                        center_distance = abs(contour_center_x - roi_center_x)
                        bottom_distance = abs(roi_bottom_y - contour_bottom_y)

                        score = center_distance + (2 * bottom_distance)

                        if best_score is None or score < best_score:
                            best_score = score
                            best_contour = contour

                    # Create third view starting from yellow-only image
                    chosen_view = yellow_only.copy()

                    if best_contour is not None:
                        # Shift contour from ROI coordinates back to full image coordinates
                        shifted_contour = best_contour.copy()
                        shifted_contour[:, 0, 0] += roi_x1
                        shifted_contour[:, 0, 1] += roi_y1

                        # Create mask for chosen contour
                        chosen_mask = np.zeros((h, w), dtype=np.uint8)
                        cv2.drawContours(chosen_mask, [shifted_contour], -1, 255, thickness=cv2.FILLED)

                        # Turn chosen yellow object green
                        chosen_view[chosen_mask > 0] = (0, 255, 0)

                    # Resize for display only
                    display_img = cv2.resize(img, (640, 360))
                    display_yellow = cv2.resize(yellow_only, (640, 360))
                    display_chosen = cv2.resize(chosen_view, (640, 360))

                    # Show windows
                    cv2.imshow("Live Camera View", display_img)
                    cv2.imshow("Yellow Only View", display_yellow)
                    cv2.imshow("Chosen Path Target View", display_chosen)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except Exception as e:
        logger.error(f"Camera viewer error: {e}")
        raise

    finally:
        cv2.destroyAllWindows()
        logger.info("Camera viewer stopped")


if __name__ == "__main__":
    main()