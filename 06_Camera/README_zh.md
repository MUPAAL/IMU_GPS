# 06_Camera — 插件开发指南

## 如何添加插件

1. 在 `plugins/` 下新建一个 `.py` 文件（例如 `plugins/my_source.py`）
2. 继承 `FrameSource`，设置三个类属性，实现三个抽象方法
3. 用 `@register_plugin` 装饰器标记
4. 完成 —— 无需修改任何已有文件

启动时自动发现，浏览器下拉框中自动出现。

## 最小插件模板

```python
"""
my_source.py — 一句话描述你的插件。
"""

from __future__ import annotations

import logging

import numpy as np

from . import FrameSource, register_plugin

logger = logging.getLogger(__name__)


@register_plugin
class MySource(FrameSource):
    # ── 必填类属性 ──
    PLUGIN_NAME = "my_source"            # 唯一 ID（用于 CLI --plugin 参数）
    PLUGIN_LABEL = "My Source"           # 浏览器下拉框显示名
    PLUGIN_DESCRIPTION = "Short description shown in the UI"

    # ── 可选：声明可配置参数 ──
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
        # ... 存储其他配置值

    def open(self) -> None:
        """Initialize camera / video / hardware resources."""
        logger.info("MySource: opening ...")
        # ... 初始化代码

    def close(self) -> None:
        """Release all resources. Must be safe to call multiple times."""
        logger.info("MySource: closing ...")
        # ... 清理代码

    def get_frame(self) -> np.ndarray | None:
        """Return one BGR frame (H x W x 3 uint8 ndarray), or None if unavailable."""
        # ... 获取帧
        return None
```

## FrameSource 接口契约

| 项目 | 要求 |
|------|------|
| `PLUGIN_NAME` | 唯一字符串，作为 `--plugin` CLI 参数值和 WS 协议标识 |
| `PLUGIN_LABEL` | 浏览器 UI 下拉框中显示的可读名称 |
| `PLUGIN_DESCRIPTION` | 浏览器 UI 中显示的一句话描述 |
| `config_schema()` | 返回 `list[dict]`，每个 dict 包含 `key`、`type`、`default`、`label` |
| `__init__(**kwargs)` | 接收与 `config_schema` 对应的配置值作为关键字参数 |
| `open()` | 初始化资源；失败时抛异常（MJPEGServer 会捕获） |
| `close()` | 释放资源；必须幂等（可能在 `open()` 部分执行后被调用） |
| `get_frame()` | 非阻塞；返回 BGR `np.ndarray` 或 `None`；禁止抛异常 |

## config_schema 类型对照

| `type` 值 | Python 类型 | 浏览器 UI |
|-----------|-------------|-----------|
| `"str"` | `str` | 文本输入框 |
| `"int"` | `int` | 数字输入框 |
| `"float"` | `float` | 数字输入框 |
| `"bool"` | `bool` | 复选框 |

## 文件结构

```
06_Camera/
├── camera_bridge.py          # 主程序（MJPEGServer、CameraPipeline、WS、HTTP）
├── plugins/
│   ├── __init__.py           # FrameSource ABC + 注册表 + 自动发现
│   └── simple_color.py       # 内置插件：OAK-D RGB 预览（depthai v3）
├── requirements.txt
└── web_static/
```

## 使用方法

```bash
# 默认插件（simple_color）
python camera_bridge.py --cam1-ip 10.95.76.11

# 启动时指定自定义插件
python camera_bridge.py --plugin my_source

# 验证插件是否被发现
python -c "from plugins import list_plugins; print(list_plugins())"
```

插件也可以在运行时通过浏览器 UI 下拉框切换，无需重启。
