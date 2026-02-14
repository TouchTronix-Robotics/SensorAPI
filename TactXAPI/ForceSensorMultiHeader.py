import serial
import numpy as np
from typing import Dict, Tuple, Optional

class ForceSensorMultiHeader:
    def __init__(
        self,
        port: str,
        baud_rate: int = 921600,
        num_rows: int = 20,
        num_cols: int = 8,
        timeout: float = 0.01,
        headers: Optional[Dict[bytes, str]] = None,
        endian: str = ">u2",
        read_chunk: int = 512,
        max_buffer: int = 4096,
        keep_tail: int = 512,
    ):
        """
        headers: dict mapping header bytes -> name
                 e.g. {b'\xAA\xAA': 'AA', b'\xEE\xEE': 'EE'}
        endian:  numpy dtype string for payload parsing ('>u2' big-endian uint16)
        """
        self.port = port
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.BYTES_PER_PIXEL = 2
        self.FRAME_PAYLOAD_SIZE = num_rows * num_cols * self.BYTES_PER_PIXEL
        self.HEADER_LEN = 2
        self.TOTAL_FRAME_SIZE = self.HEADER_LEN + self.FRAME_PAYLOAD_SIZE

        if headers is None:
            headers = {bytes([0xAA, 0xAA]): "AA", bytes([0xBB, 0xBB]): "BB", bytes([0xCC, 0xCC]): "CC", bytes([0xDD, 0xDD]): "DD", bytes([0xEE, 0xEE]): "EE"}
        self.headers = headers  # bytes -> name

        self.endian = endian
        self.read_chunk = read_chunk
        self.max_buffer = max_buffer
        self.keep_tail = keep_tail

        self.ser = serial.Serial(port, baud_rate, timeout=timeout)

        # Store latest data per header-name
        self.data_by_name: Dict[str, np.ndarray] = {
            name: np.zeros((num_rows, num_cols), dtype=np.uint16)
            for name in self.headers.values()
        }
        # Store latest data per header-bytes (optional convenience)
        self.data_by_header: Dict[bytes, np.ndarray] = {
            h: self.data_by_name[name] for h, name in self.headers.items()
        }

        # rolling buffer
        self._buffer = b""

    def _find_any_header(self, buf: bytes) -> Tuple[int, Optional[bytes]]:
        """
        Return (position, header_bytes) for the earliest occurring header in buf.
        If none found: (-1, None)
        """
        best_pos = None
        best_header = None
        for h in self.headers.keys():
            p = buf.find(h)
            if p != -1 and (best_pos is None or p < best_pos):
                best_pos = p
                best_header = h
        if best_pos is None:
            return -1, None
        return best_pos, best_header

    def read_one_frame(self) -> Tuple[str, np.ndarray]:
        """
        Block until we parse ONE complete frame for ANY header in the list.
        Returns (name, matrix).
        """
        while True:
            chunk = self.ser.read(self.read_chunk)
            if chunk:
                self._buffer += chunk

            # Try to locate the earliest header among all headers
            start, h = self._find_any_header(self._buffer)

            # If found header and enough bytes after it for a full frame
            if h is not None and len(self._buffer) >= start + self.TOTAL_FRAME_SIZE:
                name = self.headers[h]

                payload = self._buffer[start + self.HEADER_LEN : start + self.TOTAL_FRAME_SIZE]
                # drop processed bytes (everything up to end of this frame)
                self._buffer = self._buffer[start + self.TOTAL_FRAME_SIZE :]

                mat = np.frombuffer(payload, dtype=self.endian).reshape((self.num_rows, self.num_cols))

                # store
                self.data_by_name[name] = mat
                self.data_by_header[h] = mat

                return name, mat

            # Prevent infinite growth
            if len(self._buffer) > self.max_buffer:
                self._buffer = self._buffer[-self.keep_tail :]

    def read_frames(self, n: int = 1) -> Dict[str, np.ndarray]:
        """
        Read n frames (could be mixed headers). Returns latest matrices by name.
        """
        for _ in range(n):
            self.read_one_frame()
        return dict(self.data_by_name)

    def get_data(self, header_or_name) -> np.ndarray:
        """
        header_or_name can be:
          - bytes header: b'\\xAA\\xAA'
          - name: 'AA'
        Returns latest cached matrix for that key.
        """
        if isinstance(header_or_name, (bytes, bytearray)):
            h = bytes(header_or_name)
            if h not in self.data_by_header:
                raise KeyError(f"Unknown header {h!r}. Known: {list(self.data_by_header.keys())}")
            return self.data_by_header[h]
        else:
            name = str(header_or_name)
            if name not in self.data_by_name:
                raise KeyError(f"Unknown name {name!r}. Known: {list(self.data_by_name.keys())}")
            return self.data_by_name[name]

    def get_max_value(self, header_or_name) -> Tuple[int, int, int]:
        """
        Read until we get at least ONE frame for any header,
        then compute max on the requested header/name cached matrix.
        If you want to ensure the max is from a *fresh* frame of that header,
        call read_one_frame() repeatedly until returned name matches.
        """
        # ensure we have something recent-ish
        self.read_one_frame()

        mat = self.get_data(header_or_name)
        idx = int(np.argmax(mat))
        row, col = divmod(idx, self.num_cols)
        return int(mat[row, col]), row, col