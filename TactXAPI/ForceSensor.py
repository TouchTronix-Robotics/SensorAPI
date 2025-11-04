import serial
import numpy as np

class ForceSensor:
    def __init__(self,
                 port: str,
                 baud_rate: int = 921600,
                 num_rows: int = 20,
                 num_cols: int = 8,
                 timeout: float = 0.01):
        self.port=port
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.BYTES_PER_PIXEL = 2
        self.FRAME_PAYLOAD_SIZE = num_rows * num_cols * self.BYTES_PER_PIXEL
        self.HEADER = bytes([0xFF, 0xFF])
        self.TOTAL_FRAME_SIZE = 2 + self.FRAME_PAYLOAD_SIZE

        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.data = np.zeros((num_rows, num_cols), dtype=np.uint16)

    def find_frame(self) -> np.ndarray:
        """
        循环读取串口，直到找到一帧完整数据，然后 break 并返回解析后的矩阵。
        """
        buffer = b''
        while True:
            chunk = self.ser.read(512)
            if not chunk:
                # 没读到数据就继续尝试
                continue

            buffer += chunk
            start = buffer.find(self.HEADER)
            # 如果找到了 header，且后面有足够的字节组成一帧
            if start != -1 and len(buffer) >= start + self.TOTAL_FRAME_SIZE:
                # 提取 payload
                frame_bytes = buffer[start+2 : start+2 + self.FRAME_PAYLOAD_SIZE]
                # 丢弃已处理的数据
                buffer = buffer[start + self.TOTAL_FRAME_SIZE:]
                # 解析为 16 位大端整数并 reshape
                self.data = np.frombuffer(frame_bytes, dtype='>u2').reshape(
                    (self.num_rows, self.num_cols)
                )
                break

            # 防止 buffer 无限增长
            if len(buffer) > 1024:
                buffer = buffer[-256:]

        return self.data

    def get_max_value(self) -> tuple[int, int, int]:
        """
        返回 (max_value, row, col)
        """
        self.find_frame()
        idx = int(np.argmax(self.data))
        row, col = divmod(idx, self.num_cols)
        return int(self.data[row, col]), row, col

    def get_data(self) -> np.ndarray:
        """
        直接返回最近一次缓存的 sensor 数据，不做新读取。
        """
        return self.data
