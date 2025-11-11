# multisensor_100hz
import time
from TactXAPI.ForceSensor import ForceSensor
from TactXAPI.MultiSensor import MultiSensor  

def main():
    # 1) Initialize sensors
    left  = ForceSensor("/dev/ttyUSB0", baud_rate=921600)
    right = ForceSensor("/dev/ttyUSB1", baud_rate=921600)

    # 2) Pass sensors to MultiSensor; set 100 Hz read rate and 100 Hz emit rate
    ms = MultiSensor(
        sensors={"left": left, "right": right},
        read_hz=100.0,  # Throttle each reading thread to ~100 Hz (actual device rate may be lower)
        emit_hz=100.0,  # Trigger the unified on_emit callback at ~100 Hz
    )

    # Unified external callback: triggered each cycle with the latest frames from all sensors
    def on_emit(batch, ts):
        # batch: dict{name: frame}
        lf = batch["left"]
        rf = batch["right"]

        # Example: print the maximum value of each sensor
        lv, lr, lc = ms.get_max("left")
        rv, rr, rc = ms.get_max("right")
        print(f"[emit ts={ts:.6f}] left_max={lv}@({lr},{lc}) | right_max={rv}@({rr},{rc})")

    ms.on_emit = on_emit

    try:
        print("Start MultiSensor at 100 Hz (read & emit)...")
        ms.start()
        time.sleep(5.0)  # Run for 5 seconds
    except KeyboardInterrupt:
        pass
    finally:
        ms.close()
        print("Stopped.")

if __name__ == "__main__":
    main()
