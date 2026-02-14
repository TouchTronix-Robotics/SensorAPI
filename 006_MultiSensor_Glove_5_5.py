# example_forcesensor_multi5.py
import time
import numpy as np

# Import YOUR existing multiheader class (adjust path/name as needed)
# from TactXAPI.ForceSensorMultiHeader import ForceSensorMultiHeader
from TactXAPI.ForceSensorMultiHeader import ForceSensorMultiHeader  # <- change if your filename differs


HEADERS = {
    bytes([0xAA, 0xAA]): "AA",
    bytes([0xBB, 0xBB]): "BB",
    bytes([0xCC, 0xCC]): "CC",
    bytes([0xDD, 0xDD]): "DD",
    bytes([0xEE, 0xEE]): "EE",
}


def max_info(frame: np.ndarray, num_cols: int):
    idx = int(np.argmax(frame))
    row, col = divmod(idx, num_cols)
    return int(frame[row, col]), row, col


def read_specific_sensor(fs: ForceSensorMultiHeader, sensor_name: str) -> np.ndarray:
    """
    Block until we receive a NEW frame for the specified sensor name ("AA"..."EE").
    """
    while True:
        name, mat = fs.read_one_frame()
        if name == sensor_name:
            return mat


def read_all_5_sensors_once(fs: ForceSensorMultiHeader, names=("AA", "BB", "CC", "DD", "EE")):
    """
    Block until we receive ONE fresh frame for ALL given sensors.
    Returns dict: {name: mat}
    """
    got = {}
    need = set(names)
    while need:
        name, mat = fs.read_one_frame()
        if name in need:
            got[name] = mat
            need.remove(name)
    return got


def main():
    # Use  MultiHeader class directly
    fs = ForceSensorMultiHeader(
        port="/dev/ttyUSB0",
        baud_rate=921600,
        num_rows=5,
        num_cols=5,
        timeout=0.01,
        headers=HEADERS,     # <-- key change: tell it 5 sensor headers
        endian=">u2",        # big-endian uint16 (same as your old code)
    )

    target_hz = 100.0
    period = 1.0 / target_hz
    next_deadline = time.perf_counter()

    # Choose a mode:
    MODE = "all"      # "any" | "one" | "all"
    ONE_SENSOR = "AA" # used when MODE == "one"

    try:
        print(f"Start reading at ~{target_hz} Hz, MODE={MODE} ...")
        t_end = time.perf_counter() + 15.0

        while time.perf_counter() < t_end:
            # --- simple rate limiting (same as your original) ---
            now = time.perf_counter()
            if now < next_deadline:
                time.sleep(next_deadline - now)
            next_deadline += period

            # --- read frames based on mode ---
            if MODE == "any":
                # Read whichever sensor arrives next
                name, frame = fs.read_one_frame()
                max_val, row, col = max_info(frame, fs.num_cols)
                print(f"[{name}] max={max_val} @ ({row},{col})")

            elif MODE == "one":
                # Block until the specified sensor arrives
                frame = read_specific_sensor(fs, ONE_SENSOR)
                max_val, row, col = max_info(frame, fs.num_cols)
                print(f"[{ONE_SENSOR}] max={max_val} @ ({row},{col})")

            elif MODE == "all":
                # Block until all 5 sensors have produced one fresh frame
                frames = read_all_5_sensors_once(fs, names=("AA", "BB", "CC", "DD", "EE"))
                for name in ("AA", "BB", "CC", "DD", "EE"):
                    frame = frames[name]
                    max_val, row, col = max_info(frame, fs.num_cols)
                    print(f"[{name}] max={max_val} @ ({row},{col})")
                print("---- all 5 updated ----")

            else:
                raise ValueError("MODE must be: any | one | all")

    except KeyboardInterrupt:
        pass
    finally:
        fs.ser.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
