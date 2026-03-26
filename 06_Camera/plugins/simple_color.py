"""
SimpleColorProcessor — pass-through RGB preview processor.

Receives the pre-captured RGB frame from CameraDevice and returns it
unchanged.  No depthai code here; camera lifecycle is managed by
CameraDevice in camera_bridge.py.
"""

from __future__ import annotations

import numpy as np

from . import FrameProcessor, register_processor


@register_processor
class SimpleColorProcessor(FrameProcessor):
    """
    Raw RGB camera output — no image processing applied.
    """

    PROCESSOR_NAME = "simple_color"
    PROCESSOR_LABEL = "RGB Preview"
    PROCESSOR_DESCRIPTION = "Raw RGB camera output"

    @classmethod
    def required_streams(cls) -> list[str]:
        return ["rgb"]

    @classmethod
    def config_schema(cls) -> list[dict]:
        return []

    def process(self, frames: dict[str, np.ndarray | None]) -> np.ndarray | None:
        return frames.get("rgb")
