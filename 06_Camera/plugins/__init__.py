"""
Plugin framework for camera frame processors.

FrameProcessor — stateless or stateful image transform applied on top of a
                 running CameraDevice stream.  No camera management; just
                 declares required_streams() and implements process().
                 Registered with @register_processor.

All plugins are FrameProcessors.  Camera device lifecycle (open/close) is
managed exclusively by CameraDevice in camera_bridge.py.
"""

from __future__ import annotations

import abc
import importlib
import logging
from pathlib import Path

import numpy as np

__all__ = [
    "FrameProcessor", "register_processor", "get_processor",
    "list_processors", "list_plugins",
]

logger = logging.getLogger(__name__)


# ── FrameProcessor ABC ──────────────────────────────────────────────────────

class FrameProcessor(abc.ABC):
    """
    Image transform applied on top of a running CameraDevice.

    Declares which streams it needs via required_streams(), receives a
    dict of {stream_name: BGR frame | None}, and returns an output BGR frame.
    Switching processors never restarts the camera pipeline.
    """

    PROCESSOR_NAME: str = ""
    PROCESSOR_LABEL: str = ""
    PROCESSOR_DESCRIPTION: str = ""

    @classmethod
    def required_streams(cls) -> list[str]:
        """Return list of stream names this processor needs (e.g. ['rgb'])."""
        return ["rgb"]

    @classmethod
    def config_schema(cls) -> list[dict]:
        """Declare configurable params: [{"key", "type", "default", "label"}, ...]"""
        return []

    def reconfigure(self, **kwargs) -> None:
        """Apply updated config without recreating the processor (optional override)."""

    @abc.abstractmethod
    def process(self, frames: dict[str, np.ndarray | None]) -> np.ndarray | None:
        """
        Transform frames dict and return a single BGR output frame.

        Args:
            frames: dict mapping stream name → BGR numpy array (or None if unavailable)

        Returns:
            Output BGR numpy array, or None to skip this frame.
        """


# ── Processor Registry ───────────────────────────────────────────────────────

_processor_registry: dict[str, type[FrameProcessor]] = {}


def register_processor(cls: type[FrameProcessor]) -> type[FrameProcessor]:
    """Decorator: register a FrameProcessor subclass."""
    name = cls.PROCESSOR_NAME
    if not name:
        raise ValueError(f"{cls.__name__} has no PROCESSOR_NAME")
    _processor_registry[name] = cls
    logger.info("Processor registered: '%s' (%s)", name, cls.PROCESSOR_LABEL)
    return cls


def get_processor(name: str) -> type[FrameProcessor]:
    """Look up a registered processor by name."""
    if name not in _processor_registry:
        raise KeyError(
            f"Unknown processor: '{name}'. Available: {list(_processor_registry.keys())}"
        )
    return _processor_registry[name]


def list_processors() -> list[dict]:
    """Return metadata for all registered processors."""
    return [
        {
            "name": c.PROCESSOR_NAME,
            "label": c.PROCESSOR_LABEL,
            "description": c.PROCESSOR_DESCRIPTION,
            "config_schema": c.config_schema(),
            "required_streams": c.required_streams(),
        }
        for c in _processor_registry.values()
    ]


def list_plugins() -> list[dict]:
    """Alias for list_processors() — all plugins are processors."""
    return list_processors()


# ── Auto-discovery ──────────────────────────────────────────────────────────

def _auto_discover() -> None:
    """Import all .py files in this directory to trigger @register_processor."""
    pkg_dir = Path(__file__).parent
    for py_file in sorted(pkg_dir.glob("*.py")):
        if py_file.name.startswith("_"):
            continue
        module_name = f"{__name__}.{py_file.stem}"
        try:
            importlib.import_module(module_name)
        except Exception as exc:
            logger.warning("Failed to load plugin '%s': %s", py_file.stem, exc)


_auto_discover()
