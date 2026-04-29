# 05_AutoNav — 自动导航

Pure Pursuit + PID 路径跟踪模块。读取 IMU 朝向和 RTK 位置，沿航点自主行驶，向 `04_Robot` 发送速度指令。

## 文件结构

```
05_AutoNav/
├── autonav_bridge.py   # I/O 框架：传感器读取、状态机、WS/HTTP 服务
├── autonav_algo.py     # 控制算法：从 config.py 读取参数，修改转向行为改这里
├── path.csv            # 默认航点文件（可通过 UI LOAD CSV 按钮在线替换）
├── listen_autonav.py   # 调试工具：打印实时导航状态
├── replay_imu_rtk.py   # 离线工具：无硬件时回放 IMU/RTK 数据
└── web_static/         # 浏览器 Dashboard（自动打开 http://localhost:8805）
```

## 快速启动

```bash
python autonav_bridge.py
```

启动后浏览器自动打开 `http://localhost:8805`。

## Dashboard 操作

| 按钮 | 效果 |
|------|------|
| **START** | 从第 0 个航点开始导航 |
| **STOP** | 停止导航，回到 idle |
| **PAUSE** | 原地暂停，保留当前航点索引 |
| **RESUME** | 从暂停位置继续 |
| **▲ W / ▼ S** | 手动直行（仅 idle 状态）；按住移动，松开停止 |
| **LOAD CSV** | 运行时加载新航点文件，导航重置为 idle |
| **MARK POS** | 记录当前 GPS 位置作为前方校准点 |
| **CALIBRATE** | 根据标记点计算航向偏移并发送给 `01_IMU` |

键盘快捷键：按住 `W` / `S` 等同于按钮手动驾驶。

## 数据流

```
imu_bridge  :8766 ──→ ImuWsClient  ─┐
                                     ├─ AutoNavLoop → algo.compute() → joystick cmd
rtk_bridge  :8776 ──→ RtkWsClient  ─┘                                      │
                                                                             ↓
path.csv（或上传的 CSV）──────────────────────────────────────→  robot_bridge :8889
                                                                             │
                                          AutoNavWsServer :8806 ◄────────────┘
                                          （状态广播 + 控制指令）
```

## 调参指南 — `config.py`

**所有参数统一在根目录 `config.py` 中管理。** 修改后重启 `autonav_bridge.py` 即可生效，无需改动其他文件。

### 路径 / 航点

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `AUTONAV_LOOKAHEAD_M` | `1.0` | Pure Pursuit 前视距离（m）。增大 → 路径更平滑但切角更多；减小 → 跟点更紧。 |
| `AUTONAV_REACH_TOL_M` | `0.5` | 到达判定半径（m）。原始 GPS 进入此范围即推进到下个航点。RTK cm 级精度可低至 0.3 m。 |
| `AUTONAV_ARRIVE_FRAMES` | `1` | 在 `REACH_TOL_M` 内连续帧数确认到达。1 = 单帧即确认（RTK 精度足够）。若出现误到达可改为 2–3。 |
| `AUTONAV_DECEL_RADIUS_M` | `1.5` | 距**终点**多远开始减速（m）。机器越重、速度越快，需要越大的值。 |

### 速度

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `AUTONAV_MAX_LINEAR_VEL` | `1.0` | 最大前进速度（m/s），速度比例滑块之前的硬上限。 |
| `AUTONAV_MIN_LINEAR_VEL` | `0.1` | 终点减速阶段最低速度（m/s），防止机器在接近终点时完全停止。 |
| `AUTONAV_MAX_ANGULAR_VEL` | `1.0` | 最大角速度（rad/s），PID 输出的限幅值。 |
| `AUTONAV_MANUAL_SPEED` | `0.4` | W/S 手动驾驶速度（m/s）。 |

### 转向行为

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `AUTONAV_TURN_IN_PLACE_DEG` | `10.0` | 航向误差超过此值（°）时停止前进、原地转向对准后再走。设 `0` 禁用（始终边走边转）。值越大转弯越积极。 |
| `AUTONAV_TURN_SLOWDOWN` | `True` | 转向时按误差比例降低前进速度（误差 0° 全速，60°+ 降到 `MIN_LINEAR`）。减少纠偏时的过冲。 |
| `AUTONAV_DEAD_ZONE_DEG` | `3.0` | 小于此值的误差不发出纠偏指令，防止持续微颤和电机抖动。若直行时轻微摆动可调大到 5–8°。 |

### PID 增益

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `AUTONAV_PID_KP` | `0.15` | 比例增益。饱和点 = `MAX_ANGULAR / KP`（0.15 时：误差 >6.7° 满幅输出）。增大响应更快；过大出现振荡。 |
| `AUTONAV_PID_KI` | `0.005` | 积分增益。修正持续的稳态偏差。保持小值——过大导致慢速积分振荡。 |
| `AUTONAV_PID_KD` | `0.15` | 微分增益。阻尼过冲。转弯后还在甩头可增大（→ 0.3）；指令抖动则减小。 |

### 滤波

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `AUTONAV_MA_WINDOW` | `5` | GPS 滑动平均窗口（帧）。5 帧 @ 5 Hz = 1 秒平滑，抑制 RTK 多径跳变。感觉路径响应迟钝时减小。 |
| `AUTONAV_HEADING_ALPHA` | `0.3` | heading 指数低通系数（0 < α ≤ 1）。α=1.0：不滤波。α=0.3：约 0.6 s 时间常数。α=0.1：约 2 s（很慢）。机器物理摆动引起振荡时减小；heading 显示滞后时增大。 |

### 田间调参速查

| 现象 | 解决方法 |
|------|----------|
| 蛇形 / 左右振荡 | 降低 `KP`，增大 `DEAD_ZONE_DEG`，降低 `HEADING_ALPHA` |
| heading 显示滞后 | 增大 `HEADING_ALPHA` |
| 到达航点失败、绕圈 | 增大 `REACH_TOL_M`，降低 `ARRIVE_FRAMES` |
| 切角过早 / 跳过航点 | 减小 `LOOKAHEAD_M`，减小 `REACH_TOL_M` |
| 转弯后过冲甩头 | 增大 `KD`，启用 `TURN_IN_PLACE_DEG` |
| 机器转向方向错误 | 在 `autonav_algo.py` 中设置 `ANGULAR_SIGN = -1` |

## 航点文件 (`path.csv`)

制表符或逗号分隔。目前只读取 `lat` 和 `lon`，`tolerance_m` 和 `max_speed` 预留供未来按航点覆盖。

```
id,lat,lon,tolerance_m,max_speed
0,38.94130,-92.31896,0.5,1
1,38.94140,-92.31880,0.5,1
```

**运行时加载**：点击 Dashboard 的 **LOAD CSV** 按钮，无需重启即可更换航点文件。导航重置为 idle，航点表格立即更新。

## 航向校准

IMU 的 `heading.deg` 依赖 `north_offset_deg`，出发前在现场校准：

1. 将机器人放在起点，点击 **MARK POS** 记录当前 GPS 位置。
2. 用 W 键或摇杆让机器人**直线前进**几米后停止。
3. 将机器人**退回起点**。
4. 点击 **CALIBRATE** — 系统自动计算 `bearing(起点→标记点)`，减去 `heading.raw`，通过 WebSocket 发送 `set_north_offset` 给 `01_IMU`，`heading.deg` 立即生效。

校准面板显示：
- **MARKED** — 已记录的前方标记点坐标
- **CURRENT** — 实时 GPS 位置
- **BEARING** — 当前位置→标记点的方位角（回到起点后即为应有的航向）
- **OFFSET** — 最近一次发送的 `north_offset_deg`（绿色 = 已生效）

## 调试

开启详细终端输出 — 在 `autonav_algo.py` 顶部设置：

```python
ALGO_DEBUG = True
```

无硬件离线测试：

```bash
# 终端 1：回放录制的 IMU + RTK 数据
python replay_imu_rtk.py

# 终端 2：启动导航
python autonav_bridge.py
```

监听输出流：

```bash
python listen_autonav.py
```

## 端口

| 用途 | 端口 |
|------|------|
| HTTP Dashboard | 8805 |
| AutoNav WS（状态广播 + 控制） | 8806 |
| 读取 IMU WS | 8766 |
| 读取 RTK WS | 8776 |
| 写入 Robot WS | 8889 |

## 安全机制

- **传感器超时**：GPS 或 IMU 数据超过 `AUTONAV_GPS_TIMEOUT_S`（默认 5 s）自动暂停，恢复后自动续航。
- **GPS fix 过滤**：`fix_quality == 0`（无信号/默认坐标）不计入 GPS age，防止虚假"有信号"读数。
- **Watchdog 心跳**：每秒向 `robot_bridge` 发送零速指令，防止机器人因通信中断失控。
- **手动驾驶联锁**：W/S 按钮仅在 `idle` 状态有效，不会覆盖正在运行的导航任务。
- **M4 指令 Watchdog**：`CIRCUITPY/code.py` 中若 500 ms 内未收到 `V` 指令，自动清零电机速度。确保 Pi–M4 串口或 robot_bridge WS 连接断开时机器人立即停止。

## WS 客户端设计要点 — 必须消费所有传入消息

`RobotWsClient` 连接 `robot_bridge` 的目的是**发送**摇杆指令，但 `robot_bridge` 同时也会以 **20 Hz** 向所有已连接的 WS 客户端**广播** odom 数据。

如果客户端从不读取这些消息，会触发下面这条断线链：

```
robot_bridge 以 20 Hz 发送 odom
  → autonav 从不读取
  → websockets 内部队列满
  → OS TCP 接收缓冲区满（~87 KB ÷ 2 KB/s ≈ 43 s）
  → TCP Window = 0（接收方告知发送方"停发"）
  → robot_bridge 的 await ws.send(odom) 挂起等待
  → robot_bridge asyncio 事件循环被占住
  → autonav 发送的 joystick 命令等不到 ACK，超时
  → 连接断开
```

**规则**：不需要接收数据的 WS 客户端，也必须持续消费（并丢弃）传入消息：

```python
# 正确 — 消费并丢弃
async for _ in ws:
    pass

# 错误 — 从不读取，缓冲区积压，触发 TCP 背压
await ws.wait_closed()
```

适用范围：本项目中所有"只写"WS 客户端（`ImuWsClient`、`RtkWsClient`、`RobotWsClient`）。TCP 背压会把接收方向的堵塞传导成发送方向的故障，双向连接上任意一侧堆积都可能导致整条连接失效。
