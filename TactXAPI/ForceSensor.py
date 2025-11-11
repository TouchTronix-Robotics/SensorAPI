import serial
import numpy as np

class ForceSensor:
    def __init__(self,
                 port: str,
                 baud_rate: int = 921600,
                 num_rows: int = 20,
                 num_cols: int = 8,
                 timeout: float = 0.01):
        self.port = port
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
        Continuously read from the serial port until a complete frame of data is found,
        then parse and return the decoded matrix.
        """
        buffer = b''
        while True:
            chunk = self.ser.read(512)
            if not chunk:
                # No data read — keep trying
                continue

            buffer += chunk
            start = buffer.find(self.HEADER)
            # If the header is found and there are enough bytes for a full frame
            if start != -1 and len(buffer) >= start + self.TOTAL_FRAME_SIZE:
                # Extract the payload
                frame_bytes = buffer[start+2 : start+2 + self.FRAME_PAYLOAD_SIZE]
                # Discard processed data
                buffer = buffer[start + self.TOTAL_FRAME_SIZE:]
                # Parse as 16-bit big-endian integers and reshape
                self.data = np.frombuffer(frame_bytes, dtype='>u2').reshape(
                    (self.num_rows, self.num_cols)
                )
                break

            # Prevent the buffer from growing indefinitely
            if len(buffer) > 1024:
                buffer = buffer[-256:]

        return self.data

    def get_max_value(self) -> tuple[int, int, int]:
        """
        Return (max_value, row, col)
        """
        self.find_frame()
        idx = int(np.argmax(self.data))
        row, col = divmod(idx, self.num_cols)
        return int(self.data[row, col]), row, col

    def get_data(self) -> np.ndarray:
        """
        Return the most recently cached sensor data without performing a new read.
        """
        return self.data
