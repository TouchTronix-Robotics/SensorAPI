# multisensor_100hz
import time
from TactXAPI.ForceSensor import ForceSensor
from TactXAPI.MultiSensor import MultiSensor  

def main():
    # 1) 
    left  = ForceSensor("/dev/ttyUSB0", baud_rate=921600)
    right = ForceSensor("/dev/ttyUSB1", baud_rate=921600)

    # 2) 丢给 MultiSensor 管理；设定 100 Hz 读取 + 100 Hz 对外发布
    ms = MultiSensor(
        sensors={"left": left, "right": right},
        read_hz=100.0,  # 读线程节流到 ~100Hz（设备更慢则以设备为准）
        emit_hz=100.0,  # 统一以 ~100Hz 触发 on_emit 回调
    )

    # 对外统一回调：每次触发拿到所有传感器的“最新帧”
    def on_emit(batch, ts):
        # batch: dict{name: frame}
        lf = batch["left"]
        rf = batch["right"]

        # 举例：打印每块的最大值
        lv, lr, lc = ms.get_max("left")
        rv, rr, rc = ms.get_max("right")
        print(f"[emit ts={ts:.6f}] left_max={lv}@({lr},{lc}) | right_max={rv}@({rr},{rc})")

    ms.on_emit = on_emit

    try:
        print("Start MultiSensor at 100 Hz (read & emit)...")
        ms.start()
        time.sleep(5.0)  # 跑 5 秒
    except KeyboardInterrupt:
        pass
    finally:
        ms.close()
        print("Stopped.")

if __name__ == "__main__":
    main()
