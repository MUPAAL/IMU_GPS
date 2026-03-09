"""
ESP32 + BNO085 IMU 串口测试脚本

用法:
    python esp32_imu_test.py --list
    python esp32_imu_test.py --port /dev/ttyACM0
    python esp32_imu_test.py --port /dev/ttyACM0 --baud 115200

串口协议（ESP32 固件输出）:
    格式: "roll,pitch,yaw,accuracy\\n"
    示例: "12.34,-5.67,135.20,3\\n"
    字段:
        roll     float  度，绕 X 轴
        pitch    float  度，绕 Y 轴
        yaw      float  度，绕 Z 轴（BNO085 ROTATION_VECTOR/ARVR 模式下即磁北方位角）
        accuracy int    0-3，BNO085 磁力计校准精度（0=未校准，3=高精度）
    兼容 3 字段格式（无 accuracy），此时 accuracy 默认 -1

方位角坐标系说明:
    BNO085 ROTATION_VECTOR (NED): yaw=0 → 磁北，顺时针增大，bearing = yaw % 360
    BNO085 ARVR (ENU):           yaw=0 → 东，  逆时针增大，bearing = (90 - yaw) % 360
    验证方法：让机器人朝北，观察 bearing 是否接近 0°
"""

import argparse
import logging
import platform
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

# ─── 日志配置 ───────────────────────────────────────────────────────────────
_log_file = Path(__file__).parent / "esp32_imu_test.log"
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(_log_file, encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)


# ─── 工具函数 ────────────────────────────────────────────────────────────────

def list_serial_ports() -> None:
    """列出所有可用串口，帮助用户找到 ESP32 对应端口。"""
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("未检测到任何串口设备。")
        return
    print(f"发现 {len(ports)} 个串口:")
    for p in ports:
        print(f"  {p.device:<25}  {p.description}")


def _default_port() -> str:
    """根据操作系统返回默认串口路径。"""
    if platform.system() == "Darwin":
        # macOS：查找第一个匹配 usbmodem 的串口
        candidates = [
            p.device for p in serial.tools.list_ports.comports()
            if "usbmodem" in p.device.lower() or "usbserial" in p.device.lower()
        ]
        return candidates[0] if candidates else "/dev/cu.usbmodem1101"
    return "/dev/ttyACM0"


def _yaw_to_bearing(yaw: float) -> float:
    """
    将 BNO085 yaw 值转换为罗盘方位角 [0, 360)。

    ARVR_STABILIZED_RV 使用 ENU 坐标系，yaw=0=East，逆时针为正。
    罗盘方位角（顺时针，0=North）= (90 - yaw) % 360

    注意：若朝北时 bearing 不接近 0°，说明坐标系假设有误，
    尝试改为 yaw % 360（NED 坐标系公式）。
    """
    return (90.0 - yaw) % 360.0
    # return yaw % 360


def _bearing_to_cardinal(bearing: float) -> str:
    """将方位角（度）转换为 8 方位基本方向缩写。"""
    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    idx = int((bearing + 22.5) / 45.0) % 8
    return directions[idx]


def _parse_line(raw: str) -> tuple[float, float, float, int] | None:
    """
    解析一行串口数据。

    返回 (roll, pitch, yaw, accuracy)，解析失败返回 None。
    兼容 3 字段（无 accuracy）和 4 字段格式。
    """
    parts = raw.strip().split(",")
    if len(parts) == 4:
        roll = float(parts[0])
        pitch = float(parts[1])
        yaw = float(parts[2])
        accuracy = int(parts[3])
        return roll, pitch, yaw, accuracy
    elif len(parts) == 3:
        roll = float(parts[0])
        pitch = float(parts[1])
        yaw = float(parts[2])
        return roll, pitch, yaw, -1
    return None


# ─── 主循环 ──────────────────────────────────────────────────────────────────

def run(port: str, baud: int, declination: float = 0.0) -> None:
    """打开串口并持续读取 IMU 数据，Ctrl+C 优雅退出。"""
    logger.info(f"尝试打开串口: {port}  波特率: {baud}")
    try:
        ser = serial.Serial(port, baud, timeout=2.0)
    except serial.SerialException as e:
        logger.error(f"串口打开失败 ({port}): {e}")
        sys.exit(1)

    logger.info(f"串口已连接: {ser.name}")
    print("-" * 70)
    print("实时 IMU 数据（Ctrl+C 退出）")
    print("-" * 70)

    yaw_values: list[float] = []
    line_count = 0
    start_time = time.time()
    SUMMARY_INTERVAL = 20  # 每 N 行打印一次统计摘要

    try:
        while True:
            try:
                raw = ser.readline().decode("ascii", errors="replace")
            except serial.SerialException as e:
                logger.error(f"串口读取中断: {e}")
                break

            raw = raw.strip()
            if not raw:
                continue

            # 过滤固件启动信息/诊断行（以 # 开头），记录日志但不解析
            if raw.startswith("#"):
                logger.info(f"[固件] {raw}")
                continue

            result = None
            try:
                result = _parse_line(raw)
            except (ValueError, IndexError) as e:
                logger.warning(f"解析失败 [{e}] 原始数据: {repr(raw)}")
                continue

            if result is None:
                logger.warning(f"字段数不符（期望3或4）: {repr(raw)}")
                continue

            roll, pitch, yaw, accuracy = result
            bearing = _yaw_to_bearing(yaw)
            cardinal = _bearing_to_cardinal(bearing)

            # 校准状态标记
            if accuracy == -1:
                acc_str = "N/A "
            elif accuracy >= 2:
                acc_str = f"{accuracy}/3 ✓"
            else:
                acc_str = f"{accuracy}/3 ✗"

            # 真北方位角（修正磁偏角），仅当 declination != 0 时显示
            bearing_true = (bearing + declination) % 360.0
            true_str = f"  True: {bearing_true:6.1f}°" if declination != 0.0 else ""

            # 实时输出行
            print(
                f"Roll: {roll:+7.2f}°  "
                f"Pitch: {pitch:+7.2f}°  "
                f"Yaw: {yaw:+8.2f}°  →  "
                f"Bearing: {bearing:6.1f}°  {cardinal:<2}  "
                f"Accuracy: {acc_str}"
                f"{true_str}"
            )

            # accuracy < 2 时显示醒目警告（仅在前 5 秒内）
            if accuracy != -1 and accuracy < 2 and (time.time() - start_time) < 5.0:
                print("  *** 警告：磁力计未校准(accuracy<2)，方位角不可信！请做八字形运动 ***")

            # 记录 yaw 用于统计
            yaw_values.append(yaw)
            line_count += 1

            # 每 SUMMARY_INTERVAL 行输出一次统计摘要
            if line_count % SUMMARY_INTERVAL == 0:
                recent = yaw_values[-SUMMARY_INTERVAL:]
                logger.info(
                    f"[统计 last {SUMMARY_INTERVAL}] "
                    f"Yaw min={min(recent):.2f}°  max={max(recent):.2f}°  "
                    f"range={max(recent)-min(recent):.2f}°  "
                    f"total_lines={line_count}"
                )

    except KeyboardInterrupt:
        print()
        logger.info("用户中断（Ctrl+C），退出中...")
    finally:
        ser.close()
        logger.info(f"串口已关闭，共读取 {line_count} 行数据。")
        if yaw_values:
            logger.info(
                f"[全局统计] Yaw min={min(yaw_values):.2f}°  "
                f"max={max(yaw_values):.2f}°  "
                f"range={max(yaw_values)-min(yaw_values):.2f}°"
            )


# ─── CLI 入口 ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="ESP32 + BNO085 IMU 串口测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python esp32_imu_test.py --list
  python esp32_imu_test.py --port /dev/ttyACM0
  python esp32_imu_test.py --port /dev/cu.usbmodem1101 --baud 115200
        """,
    )
    parser.add_argument(
        "--list", action="store_true",
        help="列出所有可用串口后退出",
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="串口路径（默认: Linux=/dev/ttyACM0, macOS 自动检测）",
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="波特率（默认: 115200）",
    )
    parser.add_argument(
        "--declination", type=float, default=-0.5, # 密苏里 -0.5°
        help=(
            "磁偏角（度），正=磁北偏东。用于将磁北转为真北，便于和 iPhone 对比。"
            "如果不需要真北校正，保持默认 0.0 即可。-0.5 是密苏里的典型值，实际值可能因位置和时间略有变化。"
        ),
    )

    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return

    port = args.port if args.port else _default_port()
    run(port, args.baud, args.declination)


if __name__ == "__main__":
    main()
