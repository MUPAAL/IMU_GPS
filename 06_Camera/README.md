# 06_Camera — Plugin Development Guide

## How to Add a Plugin

1. Create a `.py` file in `plugins/` (e.g. `plugins/my_source.py`)
2. Subclass `FrameSource`, set the three class attributes, implement the three abstract methods
3. Decorate with `@register_plugin`
4. Done — no other files need to be modified

The plugin is auto-discovered at startup and appears in the browser dropdown.

## Minimal Plugin Template

```python
"""
my_source.py — One-line description of your plugin.
"""

from __future__ import annotations

import logging

import numpy as np

from . import FrameSource, register_plugin

logger = logging.getLogger(__name__)


@register_plugin
class MySource(FrameSource):
    # ── Required class attributes ──
    PLUGIN_NAME = "my_source"            # Unique ID (used in CLI --plugin)
    PLUGIN_LABEL = "My Source"           # Display name in browser dropdown
    PLUGIN_DESCRIPTION = "Short description shown in the UI"

    # ── Optional: declare configurable parameters ──
    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {"key": "device_ip", "type": "str",  "default": None, "label": "Device IP"},
            {"key": "fps",       "type": "int",  "default": 30,   "label": "FPS"},
            {"key": "width",     "type": "int",  "default": 1280, "label": "Width"},
            {"key": "height",    "type": "int",  "default": 720,  "label": "Height"},
        ]

    def __init__(self, **kwargs) -> None:
        """Receive config values matching config_schema keys."""
        self._fps = kwargs.get("fps", 30)
        # ... store other config values

    def open(self) -> None:
        """Initialize camera / video / hardware resources."""
        logger.info("MySource: opening ...")
        # ... setup code here

    def close(self) -> None:
        """Release all resources. Must be safe to call multiple times."""
        logger.info("MySource: closing ...")
        # ... cleanup code here

    def get_frame(self) -> np.ndarray | None:
        """Return one BGR frame (H x W x 3 uint8 ndarray), or None if unavailable."""
        # ... grab frame here
        return None
```

## FrameSource Contract

| Item | Requirement |
|------|-------------|
| `PLUGIN_NAME` | Unique string, used as `--plugin` CLI value and WS protocol key |
| `PLUGIN_LABEL` | Human-readable name shown in browser UI dropdown |
| `PLUGIN_DESCRIPTION` | One-line description shown in browser UI |
| `config_schema()` | Return `list[dict]` — each dict has `key`, `type`, `default`, `label` |
| `__init__(**kwargs)` | Accept config values as keyword arguments |
| `open()` | Initialize resources; raise on failure (MJPEGServer will catch it) |
| `close()` | Release resources; must be idempotent (may be called after partial `open()`) |
| `get_frame()` | Non-blocking; return BGR `np.ndarray` or `None`; never raise |

## config_schema Types

| `type` value | Python type | Browser UI |
|-------------|-------------|------------|
| `"str"` | `str` | Text input |
| `"int"` | `int` | Number input |
| `"float"` | `float` | Number input |
| `"bool"` | `bool` | Checkbox |

## File Structure

```
06_Camera/
├── camera_bridge.py          # Main app (MJPEGServer, CameraPipeline, WS, HTTP)
├── plugins/
│   ├── __init__.py           # FrameSource ABC + registry + auto-discovery
│   └── simple_color.py       # Built-in: OAK-D RGB preview (depthai v3)
├── requirements.txt
└── web_static/
```

## Usage

```bash
# Default plugin (simple_color)
python camera_bridge.py --cam1-ip 10.95.76.11

# Specify a custom plugin at startup
python camera_bridge.py --plugin my_source

# Verify plugin is discovered
python -c "from plugins import list_plugins; print(list_plugins())"
```

Plugins can also be switched at runtime from the browser UI dropdown — no restart needed.
