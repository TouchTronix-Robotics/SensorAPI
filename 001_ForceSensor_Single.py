# example_forcesensor_only.py
import time
import numpy as np
from TactXAPI.ForceSensor import ForceSensor  # 这里按你的文件名/路径导入

def main():
    fs = ForceSensor(port="/dev/ttyUSB0", baud_rate=921600)

    target_hz = 100.0
    period = 1.0 / target_hz
    next_deadline = time.perf_counter()

    try:
        print(f"Start reading at ~{target_hz} Hz using ForceSensor only...")
        t_end = time.perf_counter() + 15.0  # 跑 5 秒
        while time.perf_counter() < t_end:
            # 1) 等到下一个周期点（简易的定频节流）
            now = time.perf_counter()
            if now < next_deadline:
                time.sleep(next_deadline - now)
            next_deadline += period

            # 2) 读取一帧（find_frame 内部会阻塞直到拿到完整帧）
            frame = fs.find_frame()  # np.uint16, 形状 (20, 8)
            print (frame)

            # 3) 做点处理（例如拿最大值）
            idx = int(np.argmax(frame))
            row, col = divmod(idx, fs.num_cols)
            max_val = int(frame[row, col])
            print(f"max={max_val} @ ({row},{col})")

    except KeyboardInterrupt:
        pass
    finally:
        # 关闭串口
        fs.ser.close()
        print("Stopped.")

if __name__ == "__main__":
    main()
