# IMU + RTK 导航系统

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
                    │ recorder_bridge  │────┘─────────→  浏览器
                    │  (07_Recorder)   │              http :8825
                    └──────────────────┘

┌─────────────┐                     ┌───────────────┐  WS :8816
│  OAK-D      │ ──────────────────→ │ camera_bridge │ ─────────→  浏览器
│  摄像头     │     depthai / USB   │  (06_Camera)  │           http :8815
└─────────────┘                     └───────────────┘        MJPEG :8080/8081
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
| `07_Recorder` | 8825 | 8826 | 多源 CSV 数据录制器 |

## 目录结构

```
IMU_GPS/
├── 01_IMU/
│   ├── bno085_esp32c3/          # ESP32-C3 Arduino 固件（SPI，50 Hz JSON 输出）
│   │   └── bno085_esp32c3.ino
│   ├── imu_bridge.py            # 串口 → WebSocket 桥接（OOP + Pipeline）
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── imu_visualizer.js    # Three.js r160 + OrbitControls
│       └── style.css            # 暗色主题
│
├── 02_RTK/
│   ├── rtk_bridge.py            # NMEA 串口 → WebSocket 桥接（OOP + Pipeline）
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── rtk_visualizer.js    # Leaflet 地图 + 路径点 + 模拟
│       ├── style.css            # 浅色主题
│       └── assets/tiles/        # 离线地图切片（可选）
│
├── 03_Nav/
│   ├── nav_bridge.py            # 聚合器：将 IMU + RTK 合并为单一 WS 数据流
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── nav_visualizer.js    # Three.js + Leaflet 整合
│       ├── style.css            # 浅色主题
│       └── assets/tiles → ../../02_RTK/web_static/assets/tiles
│
├── 04_Robot/
│   ├── robot_bridge.py          # Amiga 机器人串口桥接（双向，OOP + Pipeline）
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── robot_visualizer.js  # Three.js 俯视图 + 控制面板
│       └── style.css            # 暗色主题 + 控制组件
│
├── 05_AutoNav/
│   ├── autonav_bridge.py        # 自主导航引擎（PID/PurePursuit + GPS 滤波器）
│   ├── requirements.txt         # websockets, numpy
│   └── web_static/
│       ├── index.html
│       ├── autonav_visualizer.js # Leaflet 地图 + 航点 + 覆盖路径规划
│       └── style.css            # 浅色主题
│
├── 06_Camera/
│   ├── camera_bridge.py         # OAK-D MJPEG 流 + 插件编排器
│   ├── plugins/
│   │   ├── __init__.py          # FrameSource ABC + 注册表 + 自动发现
│   │   └── simple_color.py      # RGB 预览插件（depthai v3）
│   ├── requirements.txt         # websockets, depthai, opencv-python, numpy
│   └── web_static/
│       ├── index.html
│       ├── camera_visualizer.js  # MJPEG 显示 + 摄像头切换 + 插件 UI
│       └── style.css            # 暗色主题
│
├── 07_Recorder/
│   ├── recorder_bridge.py       # 多源 CSV 录制器（IMU+RTK+Robot）
│   ├── requirements.txt         # websockets
│   └── web_static/
│       ├── index.html
│       ├── recorder_visualizer.js # 录制控制 + 文件管理
│       └── style.css            # 浅色主题
│
├── CIRCUITPY/                   # Adafruit Feather M4 CAN 的 CircuitPython 固件
│   └── code.py                  # Farm-ng Amiga CAN 桥接（O:/S: 输出，WASD/V 输入）
│
└── CLAUDE.md                    # AI 编码规范
```

## 硬件连接

### BNO085 IMU（ESP32-C3）

| 信号 | GPIO |
|------|------|
| MOSI | 1    |
| MISO | 6    |
| SCK  | 7    |
| CS   | 0    |
| INT  | 5    |
| RST  | 2    |
| BOOT | 9（长按 3 秒保存校准） |

- **库**：Adafruit BNO08x（通过 Arduino Library Manager 安装）
- **接口**：SPI，1 MHz
- **输出**：921600 波特率 UART JSON，约 50 Hz

### RTK GPS 接收器

- **接口**：UART（NMEA 0183 协议）
- **默认波特率**：9600
- **解析语句**：GGA（位置/定位/卫星）、RMC（速度/航向）

## 快速开始

### 环境要求

- Python 3.10+
- Arduino IDE（用于烧录固件）

### 1. 安装依赖

```bash
pip install pyserial websockets
```

### 2. 烧录 ESP32-C3 固件

在 Arduino IDE 中打开 `01_IMU/bno085_esp32c3/bno085_esp32c3.ino`，选择 ESP32-C3 开发板，上传。

### 3. 运行独立模块

```bash
# 终端 1 — IMU
cd 01_IMU
python imu_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765
# 浏览器：http://localhost:8765

# 终端 2 — RTK
cd 02_RTK
python rtk_bridge.py --port /dev/ttyACM1 --baud 9600 --ws-port 8775
# 浏览器：http://localhost:8775
```

### 4. 运行集成导航面板

```bash
# 终端 3 — Nav（需要 IMU + RTK 桥接器已在运行）
cd 03_Nav
python nav_bridge.py --nav-port 8785 --imu-ws ws://localhost:8766 --rtk-ws ws://localhost:8776
# 浏览器：http://localhost:8785
```

### 5. 运行 Amiga 机器人控制器

```bash
# 终端 4 — Robot（需要连接 CIRCUITPY Feather M4 CAN）
cd 04_Robot
python robot_bridge.py --port /dev/ttyACM1 --baud 115200 --ws-port 8795
# 浏览器：http://localhost:8795
```

### 6. 运行自主导航

```bash
# 需要 IMU + RTK + Robot 桥接器已在运行
pip install numpy
cd 05_AutoNav
python autonav_bridge.py \
  --imu-ws ws://localhost:8766 --rtk-ws ws://localhost:8776 \
  --robot-ws ws://localhost:8796
# 浏览器：http://localhost:8805 → 上传航点 CSV → 开始导航
```

### 7. 运行摄像头流

```bash
# 需要连接 OAK-D 摄像头
pip install depthai opencv-python numpy
cd 06_Camera
python camera_bridge.py --cam1-ip 10.95.76.11
# 浏览器：http://localhost:8815（控制面板）
# MJPEG：http://localhost:8080（视频流）

# 启动时指定插件：
python camera_bridge.py --cam1-ip 10.95.76.11 --plugin simple_color
```

### 8. 运行数据录制器

```bash
# 需要 IMU + RTK + Robot 桥接器已在运行
cd 07_Recorder
python recorder_bridge.py
# 浏览器：http://localhost:8825 → 开始录制 → 下载 CSV
```

## 模块详情

### 01_IMU — IMU 桥接器

- **数据流**：`串口 → SerialReader → IMUPipeline → asyncio.Queue → WebSocketServer → 浏览器`
- **Pipeline 阶段**：`_parse → _enrich_euler → _enrich_hz → _serialize`
- **功能**：实时 3D 姿态显示（Three.js）、罗盘 HUD、北偏校准、锁定偏航模式、顶视北向视图、11 个传感器数据卡片

### 02_RTK — RTK 桥接器

- **数据流**：`串口 → SerialReader → NMEAPipeline → BroadcastLoop → WebSocketServer → 浏览器`
- **Pipeline 阶段**：`_verify_checksum → _dispatch → _parse_gga / _parse_rmc`
- **功能**：Leaflet 地图（卫星/OSM/离线切片）、路径点管理、CSV 导入导出、路径编辑、路径模拟、轨迹记录、中英文切换

### 03_Nav — 导航面板

- **数据流**：`imu_bridge(WS) + rtk_bridge(WS) → NavLoop(10 Hz) → NavController → NavWebSocketServer → 浏览器`
- **布局**：上方 3D 视图（40%）+ 下方地图（60%）| 右侧数据面板（320 px）
- **功能**：在单页中整合所有 IMU + RTK 功能、统一 WebSocket 连接、航向计算、路径点到达判定、北偏校准转发、顶视北向视图

### 04_Robot — Amiga 机器人控制器

- **数据流**：`串口 ↔ SerialReader → RobotPipeline → asyncio.Queue → WebSocketServer → 浏览器`
- **Pipeline 阶段**：`_parse → _enrich_state → _enrich_hz → _enrich_odometry → _serialize`
- **串口协议**：`O:{speed},{ang_rate},{state},{soc}` 遥测（~20 Hz），`S:READY`/`S:ACTIVE` 状态；接受 WASD 单字符和 `V{speed},{ang_rate}\n` 速度命令
- **功能**：WASD 键盘/按钮控制、速度滑块、紧急停止、状态切换、电池 SOC 进度条、速度/角速度可视化条、里程计（航向+距离）、Three.js 俯视机器人视图

### 05_AutoNav — 自主导航引擎

- **数据流**：`imu_bridge(WS) + rtk_bridge(WS) + robot_bridge(WS) → AutoNavPipeline → 速度指令 → robot_bridge(WS)`
- **组件**：GeoUtils、MovingAverageFilter、KalmanFilter（4D 位置+速度）、PIDController、P2PController、PurePursuitController、WaypointManager、CoveragePlanner（Boustrophedon 蛇形覆盖）
- **状态机**：`IDLE → NAVIGATING → FINISHED`
- **导航模式**：P2P（点对点方位角控制）/ Pure Pursuit（前视点路径跟踪）
- **滤波模式**：滑动平均（GPS 窗口均值）/ 卡尔曼（4D，含 IMU 加速度 + 里程计速度观测）
- **功能**：CSV 航点上传、自适应到达容差（根据 RTK 质量调整）、GPS 超时检测、覆盖路径生成（割草机模式）、Leaflet 地图 UI

### 06_Camera — OAK-D 摄像头 MJPEG 流（可插拔）

- **数据流**：`OAK-D 摄像头 → FrameSource 插件 → MJPEGServer（HTTP multipart）→ 浏览器 <img>`
- **组件**：`plugins/` 包（FrameSource ABC + 注册表 + 自动发现）、SimpleColorSource（depthai v3）、MJPEGServer、CameraPipeline
- **插件系统**：在 `plugins/` 目录下新建 `.py` 文件，写一个带 `@register_plugin` 装饰器的 `FrameSource` 子类即可——启动时自动发现，自动出现在浏览器下拉框中并带有 `config_schema()` 配置表单。零侵入，无需修改现有代码。
- **功能**：双摄像头支持（cam1/cam2 独立 MJPEG 端口）、摄像头切换、启停控制、FPS 追踪、WS 状态广播（1 Hz）、运行时插件切换 + 每插件配置 UI
- **注意**：视频走 HTTP MJPEG，WebSocket 仅用于控制/状态

### 07_Recorder — 多源数据录制器

- **数据流**：`imu_bridge(WS) + rtk_bridge(WS) + robot_bridge(WS) → RecordLoop(5 Hz) → DataRecorder → CSV`
- **组件**：DataRecorder（线程安全 CSV 写入器）、RecorderPipeline、三个 WS 客户端（只读）
- **CSV 列**：时间戳、四元数（i/j/k/r）、欧拉角（yaw/pitch/roll）、GPS（经纬度/高度/定位/卫星/HDOP/速度/航向）、机器人（速度/角速度/状态/电量/距离/航向）
- **功能**：开始/停止录制、文件列表（下载/删除）、数据源连接状态指示、双 HTTP 路径（静态 UI + CSV 文件下载）

## 代码规范

所有代码遵循 `CLAUDE.md` 中定义的规范：

- OOP + Pipeline 设计模式
- `@dataclass` 作为数据模型
- I/O 边界处标注 `INPUT / CORE / OUTPUT` 横幅
- 代码注释、日志、CLI 输出全部使用英文
- 使用 `logging` 模块，同时输出到 `.log` 文件

## 许可证

内部项目 —— 未发布。
