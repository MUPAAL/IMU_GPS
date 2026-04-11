# IMU + RTK 导航系统

## 快速开始

```bash
pip install pyserial websockets depthai opencv-python numpy
./start_bridges.sh
```

一键在 tmux 中启动 01_IMU · 02_RTK · 03_Nav · 04_Robot · 06_Camera。
后台挂起：`Ctrl-B D` · 结束会话：`tmux kill-session -t bridges`

---

面向农业机器人的实时传感器融合平台，整合 BNO085 惯性测量单元与 RTK-GPS 接收器。系统由七个独立模块组成 —— IMU 可视化、RTK 地图、集成导航面板、机器人控制、自主导航引擎、摄像头流、数据录制 —— 通过 WebSocket 桥接通信，全部在浏览器中查看。

## 系统架构

```
┌─────────────┐     serial/SPI      ┌──────────────┐   WS :8766
│  BNO085 IMU │ ──────────────────→ │  imu_bridge  │ ──────────┐
│  (ESP32-C3) │                     │  (01_IMU)    │           │
└─────────────┘                     └──────────────┘           │
                                                               ▼
┌─────────────┐     serial/UART     ┌──────────────┐   WS :8776    ┌──────────────┐  WS :8786
│  RTK GPS    │ ──────────────────→ │  rtk_bridge  │ ──────────┬──→│  nav_bridge  │ ─────────→  浏览器
│  接收器     │                     │  (02_RTK)    │           │   │  (03_Nav)    │           http :8785
└─────────────┘                     └──────────────┘           │   └──────────────┘
                                                               │          ▲
                                                               └──────────┘

┌─────────────┐     serial/USB-CDC  ┌───────────────┐  WS :8796
│  Farm-ng    │ ←──────────────────→│  robot_bridge │ ─────────→  浏览器
│  Amiga CAN  │   (O:/S: + WASD/V) │  (04_Robot)   │           http :8795
│ (Feather M4)│                     └───────────────┘
└─────────────┘                            ▲
                                           │
                    ┌──────────────────┐    │  WS :8806
                    │  autonav_bridge  │────┤─────────→  浏览器
                    │  (05_AutoNav)    │    │           http :8805
                    └──────────────────┘    │
                      ▲ IMU  ▲ RTK         │ 速度指令
                      │      │             │
                    ┌──────────────────┐    │  WS :8826
                    │   sim/replay hub │────┘─────────→  浏览器
                    │  (07_SimReplay)  │
                    └──────────────────┘

┌─────────────┐                      ┌───────────────┐  WS :8816
│  OAK-D PoE  │ ──── TCP/IP ───────→ │ camera_bridge │ ─────────→  浏览器
│  摄像头     │     depthai v3       │  (06_Camera)  │           http :8815
└─────────────┘                      └───────────────┘       MJPEG :8080/8081
```

每个桥接器同时在 HTTP 端口提供静态网页 UI：

| 模块 | HTTP | WebSocket | 说明 |
|------|------|-----------|------|
| `01_IMU` | 8765 | 8766 | 3D IMU 姿态 + 传感器数据卡片 |
| `02_RTK` | 8775 | 8776 | Leaflet 地图 + 路径点管理 |
| `03_Nav` | 8785 | 8786 | 集成面板（3D + 地图 + 全部数据面板） |
| `04_Robot` | 8795 | 8796 | Amiga 机器人控制器（遥测 + WASD/速度控制） |
| `05_AutoNav` | 8805 | 8806 | 自主导航引擎（GPS+IMU PID/PurePursuit 控制） |
| `06_Camera` | 8815 | 8816 | OAK-D 摄像头 MJPEG 流（视频在 8080/8081） |
| `07_SimReplay` | - | - | 离线联调用回放/模拟工具集合 |

## 目录结构

```
IMU_GPS/
├── config.py                    # ★ 所有模块（01–06）的超参数统一配置文件
├── start_bridges.sh             # ★ 一键启动脚本（01–04 + 06，tmux 瓦片布局）
│
├── 01_IMU/
│   ├── bno085_esp32c3/          # ESP32-C3 Arduino 固件（SPI，50 Hz JSON 输出）
│   │   └── bno085_esp32c3.ino
│   ├── imu_bridge.py            # 串口 → WebSocket 桥接
│   ├── listen_imu_websocket.py  # 调试用 WS 客户端
│   ├── requirements.txt
│   └── web_static/
│
├── 02_RTK/
│   ├── rtk_bridge.py            # NMEA 串口 → WebSocket 桥接
│   ├── requirements.txt
│   └── web_static/
│
├── 03_Nav/
│   ├── nav_bridge.py            # IMU + RTK 聚合器 → 统一 WS 数据流
│   ├── listen_nav_websocket.py  # 调试用 WS 客户端
│   ├── requirements.txt
│   └── web_static/
│
├── 04_Robot/
│   ├── robot_bridge.py          # Amiga 串口桥接（双向）
│   ├── listen_robot_websocket.py
│   ├── send_robot_only_demo.py
│   ├── requirements.txt
│   └── web_static/
│
├── 05_AutoNav/
│   ├── autonav_bridge.py        # 自主导航引擎（PID/PurePursuit + GPS 滤波器）
│   ├── requirements.txt
│   └── web_static/
│
├── 06_Camera/
│   ├── camera_bridge.py         # OAK-D MJPEG 桥接 + 两层插件编排器
│   ├── plugins/
│   │   ├── __init__.py          # FrameProcessor ABC + 注册表 + 自动发现
│   │   ├── simple_color.py      # RGB 直通预览
│   │   ├── path_cam.py          # 黄色胶带路径检测（HSV 掩码）
│   │   ├── depth_cam.py         # 深度彩图 + RGB 混合（需要 --stereo）
│   │   ├── obstacle_cam.py      # 近障预警（需要 --stereo --disparity）
│   │   └── disparity_demo.py    # 最简视差 demo（需要 --stereo --disparity）
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── camera_visualizer.js
│       ├── style.css
│       └── snapshots/           # 自动创建；存放各次会话的 HTML 像素检视快照
│
├── 07_SimReplay/
│   ├── start_nav_with_replay.sh # 一键：01回放 + 02回放 + 03导航 + 监听（tmux）
│   ├── sim_robot_ws_server.py   # 本地机器人 WS 模拟器
│   ├── demo_filter_by_type.py   # 模拟器/机器人 WS 按类型过滤监听示例
│   └── README.md
│
├── CIRCUITPY/                   # Adafruit Feather M4 CAN 的 CircuitPython 固件
│   └── code.py
│
└── CLAUDE.md                    # AI 编码规范
```

## 全局配置 — `config.py`

所有模块（01–06）的默认超参数集中在一个文件中管理。修改后重启对应桥接器即可生效，无需改动各模块脚本：

```python
# 01_IMU
IMU_SERIAL_PORT  = "/dev/cu.usbmodem101"
IMU_BAUD         = 921600
IMU_WS_PORT      = 8765
IMU_NORTH_OFFSET = 0.0              # 航向校准偏移（度）

# 06_Camera（示例）
CAM_FPS              = 25
CAM_WIDTH            = 640
CAM_HEIGHT           = 400
CAM_ENABLE_STEREO    = True         # 开启深度流
CAM_ENABLE_DISPARITY = False        # 开启原始视差流（额外负载，默认关闭）
```

CLI 参数优先级高于 `config.py`，可在运行时临时覆盖任意参数。

## 快速开始

### 环境要求

- Python 3.10+
- Arduino IDE（用于烧录固件）
- `tmux`（用于 `start_bridges.sh`）

### 1. 安装依赖

```bash
pip install pyserial websockets
```

### 2. 一键启动所有桥接器（推荐）

```bash
./start_bridges.sh
```

在单个 tmux session 中以瓦片布局启动模块 01–04 和 06：

```
┌──────────────┬──────────────┐
│  01_IMU      │  02_RTK      │
├──────────────┼──────────────┤
│  03_Nav      │  04_Robot    │
├──────────────┴──────────────┤
│        06_Camera            │
└─────────────────────────────┘
```

- 后台挂起：`Ctrl-B D`
- 结束会话：`tmux kill-session -t bridges`
- 重新连接：再次运行 `./start_bridges.sh`（session 已存在时直接 attach）

### 3. 单独运行各模块

```bash
cd 01_IMU && python imu_bridge.py          # http://localhost:8765
cd 02_RTK && python rtk_bridge.py          # http://localhost:8775
cd 03_Nav && python nav_bridge.py          # http://localhost:8785
cd 04_Robot && python robot_bridge.py      # http://localhost:8795
cd 05_AutoNav && python autonav_bridge.py  # http://localhost:8805
cd 06_Camera && python camera_bridge.py   # http://localhost:8815
cd 07_SimReplay && ./start_nav_with_replay.sh # 用 01/02 回放喂给 03_Nav
```

### 4. 摄像头 — 深度与视差开关

```bash
# 仅 RGB（最流畅，负载最低）
python camera_bridge.py

# RGB + 深度（推荐，3D 感知）
python camera_bridge.py --stereo

# RGB + 深度 + 原始视差（启用 obstacle_cam / disparity_demo 插件）
python camera_bridge.py --stereo --disparity
```

或在 `config.py` 中永久设置：

```python
CAM_ENABLE_STEREO    = True
CAM_ENABLE_DISPARITY = False   # 仅调试视差插件时开启
```

## 模块详情

### 01_IMU — IMU 桥接器

- **数据流**：`串口 → SerialReader → IMUPipeline → asyncio.Queue → WebSocketServer → 浏览器`
- **Pipeline 阶段**：`_parse → _enrich_euler（航向计算）→ _enrich_hz → _serialize`
- **功能**：实时 3D 姿态（Three.js）、罗盘 HUD、北偏校准、锁定偏航、顶视北向视图、11 个传感器卡片；**航向在后端计算**后广播
- **调试工具**：`listen_imu_websocket.py`

### 02_RTK — RTK 桥接器

- **数据流**：`串口 → SerialReader → NMEAPipeline → BroadcastLoop → WebSocketServer → 浏览器`
- **Pipeline 阶段**：`_verify_checksum → _dispatch → _parse_gga / _parse_rmc`
- **功能**：Leaflet 地图（卫星/OSM/离线切片）、路径点增删改查、CSV 导入导出、路径模拟、轨迹记录、中英文切换

### 03_Nav — 导航面板

- **数据流**：`imu_bridge(WS) + rtk_bridge(WS) → NavLoop(10 Hz) → NavController → WebSocketServer → 浏览器`
- **布局**：上方 3D 视图（40%）+ 下方地图（60%）| 右侧数据面板（320 px）
- **功能**：单页整合所有 IMU + RTK 功能、路径点到达判定、IMU 后端航向直接转发
- **调试工具**：`listen_nav_websocket.py`

### 04_Robot — Amiga 机器人控制器

- **数据流**：`串口 ↔ SerialReader → RobotPipeline → asyncio.Queue → WebSocketServer → 浏览器`
- **Pipeline 阶段**：`_parse → _enrich_state → _enrich_hz → _enrich_odometry → _serialize`
- **串口协议**：`O:{speed},{ang_rate},{state},{soc}` 遥测（~20 Hz）；接受 WASD 和 `V{speed},{ang_rate}\n` 速度命令
- **功能**：WASD/按钮控制、速度滑块、紧急停止、电量进度条、里程计、Three.js 俯视图
- **调试工具**：`listen_robot_websocket.py`、`send_robot_only_demo.py`

### 05_AutoNav — 自主导航引擎

- **数据流**：`imu_bridge + rtk_bridge + robot_bridge → AutoNavPipeline → 速度指令 → robot_bridge`
- **组件**：GeoUtils、MovingAverageFilter、KalmanFilter（4D）、PIDController、P2PController、PurePursuitController、WaypointManager、CoveragePlanner（Boustrophedon 蛇形覆盖）
- **导航模式**：P2P（方位角控制）/ Pure Pursuit（前视点路径跟踪）
- **滤波模式**：滑动平均 / 卡尔曼（含 IMU 加速度 + 里程计速度观测）
- **状态机**：`IDLE → NAVIGATING → FINISHED`

### 06_Camera — OAK-D 摄像头桥接器

#### 两层架构

```
第一层 — CameraDevice（启动时初始化一次，切换插件时不重启）
  ├─ RGB 节点       → rgb 队列        （始终开启）
  ├─ StereoDepth 节点 → depth 队列   （--stereo 时开启）
  └─ StereoDepth 节点 → disparity 队列（--stereo --disparity 时开启）

第二层 — FrameProcessor（原子切换，零停流）
  plugin.required_streams() → ['rgb'] | ['rgb','depth'] | ...
  plugin.process(frames)    → 输出帧 → MJPEG 编码 → 浏览器
```

#### 插件系统

在 `plugins/` 目录下新建 `.py` 文件，写一个带 `@register_processor` 装饰器的 `FrameProcessor` 子类即可。启动时自动发现，浏览器下拉框中自动出现，无需修改任何现有代码：

```python
from . import FrameProcessor, register_processor

@register_processor
class MyPlugin(FrameProcessor):
    PROCESSOR_NAME = "my_plugin"
    PROCESSOR_LABEL = "我的插件"
    PROCESSOR_DESCRIPTION = "..."

    @classmethod
    def required_streams(cls):
        return ["rgb"]           # 声明需要哪些流

    def process(self, frames):
        img = frames["rgb"]     # ndarray（BGR，uint8）
        # ... 你的处理逻辑 ...
        return img
```

内置插件：

| 插件 | 需要的流 | 说明 |
|------|---------|------|
| `simple_color` | rgb | RGB 直通预览 |
| `path_cam` | rgb | 黄色胶带路径检测（HSV 掩码 + 轮廓评分） |
| `depth_cam` | rgb, depth | 深度彩图 + RGB 混合显示 |
| `obstacle_cam` | rgb, disparity | 近障预警（DANGER / CAUTION / CLEAR 三档） |
| `disparity_demo` | disparity | 最简原始视差彩图 —— 供二次开发参考 |

#### 帧快照（像素检视工具）

在浏览器控制面板点击 **Capture Frame**，当前帧以自包含 HTML 文件形式保存到 `web_static/snapshots/`，并自动在新标签页打开：

- Canvas 渲染捕获的输出帧
- 鼠标悬停在任意像素上，显示：
  - 所有插件：Canvas RGB 值
  - `rgb` 流：传感器原始 R/G/B
  - `depth` 流：距离（**毫米**）
  - `disparity` 流：视差值（**像素**）

#### 流控制开关

| CLI 参数 | `config.py` 键 | 默认值 | 作用 |
|----------|---------------|--------|------|
| `--stereo` / `--no-stereo` | `CAM_ENABLE_STEREO` | `True` | 开启左右目 + StereoDepth → depth 流 |
| `--disparity` / `--no-disparity` | `CAM_ENABLE_DISPARITY` | `False` | 额外开启原始视差队列（增加 CPU/带宽负载） |

**推荐组合**：

| 场景 | 配置 |
|------|------|
| 仅 RGB，追求最流畅 | `STEREO=False` |
| 需要深度感知（推荐）| `STEREO=True, DISPARITY=False` |
| 调试视差插件 | `STEREO=True, DISPARITY=True` |

### 07_SimReplay — 回放/模拟中心

- **目标**：通过录制数据回放，让下游模块在无硬件时也能联调。
- **一键链路**：`./07_SimReplay/start_nav_with_replay.sh` 会启动 IMU 回放 + RTK 回放 + Nav bridge + Nav 监听。
- **典型场景**：`03_Nav` 依赖 `01_IMU` 和 `02_RTK` 输入，但手头没有设备时。

## 硬件连接

### BNO085 IMU（ESP32-C3）

| 信号 | GPIO |
|------|------|
| MOSI | 1 |
| MISO | 6 |
| SCK | 7 |
| CS | 0 |
| INT | 5 |
| RST | 2 |
| BOOT | 9（长按 3 秒保存校准） |

- **库**：Adafruit BNO08x（Arduino Library Manager）
- **接口**：SPI，1 MHz
- **输出**：921600 波特率 UART JSON，约 50 Hz

### RTK GPS 接收器

- **接口**：UART（NMEA 0183）
- **默认波特率**：9600
- **解析语句**：GGA（位置/定位/卫星）、RMC（速度/航向）

### OAK-D PoE 摄像头

- **连接方式**：TCP/IP（depthai v3，`dai.DeviceInfo(ip)`）
- **默认 IP**：cam1 = `10.95.76.11`，cam2 = `10.95.76.10`
- **使用插槽**：CAM_A（RGB）、CAM_B（左目）、CAM_C（右目）
- **立体输出分辨率**：左右目各 640×400 px；RGB 可在 `config.py` 中配置（默认 640×400）

## 代码规范

所有代码遵循 `CLAUDE.md` 中定义的规范：

- OOP + Pipeline 设计模式
- `@dataclass` 作为数据模型
- I/O 边界处标注 `INPUT / CORE / OUTPUT` 横幅
- 代码注释、日志、CLI 输出全部使用英文
- 使用 `logging` 模块，同时输出到 `.log` 文件

## 许可证

内部项目 —— 未发布。
