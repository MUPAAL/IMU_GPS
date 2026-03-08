# IMU + RTK 导航系统

面向农业机器人的实时传感器融合平台，整合 BNO085 惯性测量单元与 RTK-GPS 接收器。系统由三个独立模块组成 —— IMU 可视化、RTK 地图、以及集成导航面板 —— 通过 WebSocket 桥接通信，全部在浏览器中查看。

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
```

每个桥接器同时在 HTTP 端口提供静态网页 UI：

| 模块 | HTTP | WebSocket | 说明 |
|------|------|-----------|------|
| `01_IMU` | 8765 | 8766 | 3D IMU 姿态 + 传感器数据卡片 |
| `02_RTK` | 8775 | 8776 | Leaflet 地图 + 路径点管理 |
| `03_Nav` | 8785 | 8786 | 集成面板（3D + 地图 + 全部数据面板） |

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

## 代码规范

所有代码遵循 `CLAUDE.md` 中定义的规范：

- OOP + Pipeline 设计模式
- `@dataclass` 作为数据模型
- I/O 边界处标注 `INPUT / CORE / OUTPUT` 横幅
- 代码注释、日志、CLI 输出全部使用英文
- 使用 `logging` 模块，同时输出到 `.log` 文件

## 许可证

内部项目 —— 未发布。
