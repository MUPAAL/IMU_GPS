# 04_Robot — 输出字段说明

WebSocket 输出地址：`ws://localhost:8889`。所有消息类型通过同一连接广播。

---

## `type: "odom"` — 里程计（20 Hz）

从 Feather M4 串口行解析，格式：`O:{v},{w},{state},{soc}`

| 字段 | 单位 | 说明 |
|------|------|------|
| `type` | — | `"odom"` |
| `v` | m/s | 线速度（前进为正） |
| `w` | rad/s | 角速度（左转为正） |
| `state` | int | 固件驱动状态整数 |
| `soc` | % | 电池电量 |
| `ts` | Unix 秒 | 宿主机解析时间戳 |

---

## `type: "imu"` — IMU 帧（从 01_IMU 转发，约 50 Hz）

直接从 `imu_bridge`（`ws://localhost:8766`）转发。字段与 `01_IMU` 输出一致，完整说明见 `01_IMU/README_zh.md`。

主要字段：

| 字段 | 说明 |
|------|------|
| `rot` | 四元数（qi, qj, qk, qr），以磁北为参考 |
| `euler` | 横滚 / 俯仰 / 偏航（度） |
| `heading.deg` | 北向修正后的航向角（0=正北，90=正东） |
| `heading.dir` | 16 方位罗盘标签 |
| `accel` | 加速度计 x/y/z（m/s²） |
| `gyro` | 陀螺仪 x/y/z（rad/s） |
| `cal` | 校准状态 0–3 |
| `hz` | IMU 帧率 |

---

## `type: "rtk"` — RTK 帧（从 02_RTK 转发，约 5 Hz）

直接从 `rtk_bridge`（`ws://localhost:8776`）转发。字段与 `02_RTK` 输出一致，完整说明见 `02_RTK/README_zh.md`。

主要字段：

| 字段 | 说明 |
|------|------|
| `lat` / `lon` | 十进制度坐标（WGS-84） |
| `fix_quality` | 0=无定位，1=GPS，4=RTK 固定，5=RTK 浮点 |
| `num_sats` | 使用中的卫星数 |
| `hdop` | 水平精度因子 |
| `available` | `fix_quality > 0` 或 `source == "rtk"` 时为 `true` |

---

## `type: "state_status"` — 驱动状态变更（事件触发）

浏览器连接时发送一次，之后每当 Feather M4 上报状态变更时再次发送。

| 字段 | 说明 |
|------|------|
| `active` | `true` = 自动模式激活，`false` = 就绪（手动） |

---

## 输入（浏览器 → 机器人）

| 消息 | 说明 |
|------|------|
| `{"type": "joystick", "linear": 0.5, "angular": 0.1}` | 速度指令，限幅至 `config.py` 中的 `±MAX_LINEAR` / `±MAX_ANGULAR` |
| `{"type": "toggle_state"}` | 切换 Feather M4 的自动/就绪状态（通过串口发送 `\r`） |
| `{"type": "heartbeat"}` | 保活心跳。若超过 `WATCHDOG_TIMEOUT` 秒未收到，机器人紧急停止 |

## 备注

- `imu` 和 `rtk` 帧为透传代理——`robot_bridge` 不修改数据内容，仅覆盖 `type` 字段。
- 看门狗计时器在收到每条 `heartbeat` 或 `joystick` 消息时重置。默认超时为 2 秒。
- Feather M4 串口格式：里程计为 `O:{v},{w},{state},{soc}\n`，状态变更为 `S:ACTIVE` / `S:READY`。
