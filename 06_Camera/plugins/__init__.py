"""
Plugin framework for camera frame sources.

Provides the FrameSource ABC, plugin registry, and auto-discovery.
New plugins: create a .py file in this directory with a @register_plugin class.
"""

from __future__ import annotations

import abc
import importlib
import logging
from pathlib import Path

import numpy as np

__all__ = ["FrameSource", "register_plugin", "get_plugin", "list_plugins"]

logger = logging.getLogger(__name__)


# ── FrameSource ABC ─────────────────────────────────────────────────────────

class FrameSource(abc.ABC):
    """Abstract base class for camera frame providers (plugin interface)."""

    PLUGIN_NAME: str = ""
    PLUGIN_LABEL: str = ""
    PLUGIN_DESCRIPTION: str = ""

    @classmethod
    def config_schema(cls) -> list[dict]:
        """Declare configurable params: [{"key", "type", "default", "label"}, ...]"""
        return []

    @abc.abstractmethod
    def open(self) -> None:
        """Initialize camera resources."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release all camera resources."""

    @abc.abstractmethod
    def get_frame(self) -> np.ndarray | None:
        """Return latest BGR frame or None."""


# ── Plugin Registry ─────────────────────────────────────────────────────────

_plugin_registry: dict[str, type[FrameSource]] = {}


def register_plugin(cls: type[FrameSource]) -> type[FrameSource]:
    """Decorator: register a FrameSource subclass."""
    name = cls.PLUGIN_NAME
    if not name:
        raise ValueError(f"{cls.__name__} has no PLUGIN_NAME")
    _plugin_registry[name] = cls
    logger.info("Plugin registered: '%s' (%s)", name, cls.PLUGIN_LABEL)
    return cls


def get_plugin(name: str) -> type[FrameSource]:
    """Look up a registered plugin by name."""
    if name not in _plugin_registry:
        raise KeyError(
            f"Unknown plugin: '{name}'. Available: {list(_plugin_registry.keys())}"
        )
    return _plugin_registry[name]


def list_plugins() -> list[dict]:
    """Return metadata for all registered plugins."""
    return [
        {
            "name": c.PLUGIN_NAME,
            "label": c.PLUGIN_LABEL,
            "description": c.PLUGIN_DESCRIPTION,
            "config_schema": c.config_schema(),
        }
        for c in _plugin_registry.values()
    ]


# ── Auto-discovery ──────────────────────────────────────────────────────────

def _auto_discover() -> None:
    """Import all .py files in this directory to trigger @register_plugin."""
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
