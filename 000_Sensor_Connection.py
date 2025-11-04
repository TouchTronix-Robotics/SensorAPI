"""
(optional) conda init
conda create -n TactX

conda activate TactX


# 安装依赖
pip install pyserial numpy

# 直接扫描，默认 921600 波特、20x8、2字节像素，单口探测 3 秒
python 000_Sensor_Connection.py

# 或者自定义参数（比如探测更久）
python 000_Sensor_Connection.py --probe 5.0

# 如果你的设备不是 921600
python 000_Sensor_Connection.py --baud 460800

serial port name on different devices
windows: "COM3"
Ubuntu: "/dev/ttyUSB0"
"""
import argparse
import time
from typing import Optional

import numpy as np
import serial
from serial.tools import list_ports

HEADER = bytes([0xFF, 0xFF])

def detect_one_port(
    port: str,
    baud: int,
    num_rows: int,
    num_cols: int,
    bytes_per_pixel: int,
    probe_seconds: float,
    read_chunk: int = 512,
    serial_timeout: float = 0.01,
) -> bool:
    """
    打开指定串口并在 probe_seconds 内尝试检测一帧：
    帧定义：HEADER(2) + payload(num_rows*num_cols*bytes_per_pixel)
    返回 True 表示检测到合法一帧。
    """
    payload_size = num_rows * num_cols * bytes_per_pixel
    total_size = len(HEADER) + payload_size

    try:
        with serial.Serial(port, baudrate=baud, timeout=serial_timeout) as ser:
            ser.reset_input_buffer()
            t_end = time.perf_counter() + probe_seconds
            buf = bytearray()

            while time.perf_counter() < t_end:
                chunk = ser.read(read_chunk)
                if chunk:
                    buf += chunk

                    # 控制缓存大小，防止无限增长
                    if len(buf) > max(4096, 4 * total_size):
                        del buf[: -2048]

                    # 试着找头并解析一帧
                    while True:
                        start = buf.find(HEADER)
                        if start < 0:
                            # 只保留可能的头部前缀
                            if len(buf) > len(HEADER) - 1:
                                del buf[: - (len(HEADER) - 1)]
                            break

                        if len(buf) - start < total_size:
                            # 长度不够，等待更多数据
                            # 丢弃头之前的无用字节
                            if start > 0:
                                del buf[:start]
                            break

                        # 提取 payload
                        frame_start = start + len(HEADER)
                        frame_end = start + total_size
                        payload = bytes(buf[frame_start: frame_start + payload_size])

                        # 尝试按 >u2 解析（与你现有 ForceSensor 保持一致）
                        try:
                            arr = np.frombuffer(payload, dtype=">u2").reshape(num_rows, num_cols)
                            # 解析成功即认为此端口是有效的传感器口
                            return True
                        except Exception:
                            # 不是合法帧，跳过一个字节后继续匹配下一个潜在头
                            del buf[: start + 1]
                            continue

                        # 正常不会到这里
                else:
                    # 没数据，小睡一下
                    time.sleep(0.001)

    except (serial.SerialException, OSError):
        return False

    return False


def main():
    parser = argparse.ArgumentParser(description="Scan serial ports and detect ForceSensor.")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--rows", type=int, default=20, help="Number of rows (default: 20)")
    parser.add_argument("--cols", type=int, default=8, help="Number of cols (default: 8)")
    parser.add_argument("--bpp", type=int, default=2, help="Bytes per pixel (default: 2)")
    parser.add_argument("--probe", type=float, default=3.0, help="Probe seconds per port (default: 3s)")
    parser.add_argument("--timeout", type=float, default=0.01, help="Serial read timeout (default: 0.01s)")
    args = parser.parse_args()

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    print("Found ports:")
    for p in ports:
        print(f"  - {p.device} ({p.description})")

    print("\nProbing ports...\n")
    good = []
    for p in ports:
        dev = p.device
        ok = detect_one_port(
            port=dev,
            baud=args.baud,
            num_rows=args.rows,
            num_cols=args.cols,
            bytes_per_pixel=args.bpp,
            probe_seconds=args.probe,
            serial_timeout=args.timeout,
        )
        print(f"[{dev}] {'OK (sensor detected)' if ok else 'NO DATA / NOT MATCH'}")
        if ok:
            good.append(dev)

    print("\nSummary:")
    if good:
        for g in good:
            print(f"  * {g}  <-- sensor present")
    else:
        print("  No sensor detected on any port.")

if __name__ == "__main__":
    main()
