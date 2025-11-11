# example_forcesensor_only.py
import time
import numpy as np
from TactXAPI.ForceSensor import ForceSensor  # Import according to your file name/path

def main():
    fs = ForceSensor(port="/dev/ttyUSB0", baud_rate=921600)

    target_hz = 100.0
    period = 1.0 / target_hz
    next_deadline = time.perf_counter()

    try:
        print(f"Start reading at ~{target_hz} Hz using ForceSensor only...")
        t_end = time.perf_counter() + 15.0  # Run for 15 seconds
        while time.perf_counter() < t_end:
            # 1) Wait until the next cycle point (simple fixed-rate throttling)
            now = time.perf_counter()
            if now < next_deadline:
                time.sleep(next_deadline - now)
            next_deadline += period

            # 2) Read one frame (find_frame will block until a complete frame is received)
            frame = fs.find_frame()  # np.uint16, shape (20, 8)
            print(frame)

            # 3) Do some processing (for example, get the maximum value)
            idx = int(np.argmax(frame))
            row, col = divmod(idx, fs.num_cols)
            max_val = int(frame[row, col])
            print(f"max={max_val} @ ({row},{col})")

    except KeyboardInterrupt:
        pass
    finally:
        # Close the serial port
        fs.ser.close()
        print("Stopped.")

if __name__ == "__main__":
    main()
