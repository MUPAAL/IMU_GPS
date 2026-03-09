"""
SimpleColorSource — OAK-D single color camera plugin using depthai v3 API.
"""

from __future__ import annotations

import logging

import numpy as np

try:
    import depthai as dai
except ImportError:
    dai = None

from . import FrameSource, register_plugin

logger = logging.getLogger(__name__)


@register_plugin
class SimpleColorSource(FrameSource):
    """
    OAK-D single color camera source using depthai v3 API.

    Captures color frames from an OAK-D PoE or USB device.
    """

    PLUGIN_NAME = "simple_color"
    PLUGIN_LABEL = "RGB Preview"
    PLUGIN_DESCRIPTION = "Raw color camera preview via depthai v3 API"

    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {"key": "device_ip", "type": "str", "default": None, "label": "Device IP"},
            {"key": "fps", "type": "int", "default": 30, "label": "FPS"},
            {"key": "width", "type": "int", "default": 1280, "label": "Width"},
            {"key": "height", "type": "int", "default": 720, "label": "Height"},
        ]

    def __init__(self, **kwargs) -> None:
        self._device_ip = kwargs.get("device_ip")
        self._fps = kwargs.get("fps", 30)
        self._width = kwargs.get("width", 1280)
        self._height = kwargs.get("height", 720)
        self._device = None
        self._pipeline = None
        self._queue = None

    def open(self) -> None:
        """Create depthai pipeline and start the camera."""
        if dai is None:
            raise RuntimeError("depthai library not installed")

        try:
            if self._device_ip:
                dev_info = dai.DeviceInfo(self._device_ip)
                self._device = dai.Device(dev_info)
            else:
                self._device = dai.Device()

            self._pipeline = dai.Pipeline(self._device)
            cam = self._pipeline.create(dai.node.ColorCamera)
            cam.setPreviewSize(self._width, self._height)
            cam.setFps(self._fps)
            cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam.setInterleaved(False)

            self._queue = cam.preview.createOutputQueue(maxSize=1, blocking=False)
            self._pipeline.start()
            logger.info(
                "SimpleColorSource: opened camera (ip=%s, %dx%d @ %d fps)",
                self._device_ip, self._width, self._height, self._fps,
            )
        except Exception as exc:
            logger.error("SimpleColorSource: failed to open camera: %s", exc)
            self.close()
            raise

    def close(self) -> None:
        """Release depthai resources."""
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as exc:
                logger.warning("SimpleColorSource: error stopping pipeline: %s", exc)
            self._pipeline = None
        if self._device is not None:
            try:
                self._device.close()
            except Exception as exc:
                logger.warning("SimpleColorSource: error closing device: %s", exc)
            self._device = None
        self._queue = None

    def get_frame(self) -> np.ndarray | None:
        """Non-blocking frame grab. Returns BGR numpy array or None."""
        if self._queue is None:
            return None
        try:
            in_frame = self._queue.tryGet()
            if in_frame is not None:
                return in_frame.getCvFrame()
            return None
        except Exception as exc:
            logger.warning("SimpleColorSource: get_frame error: %s", exc)
            return None
