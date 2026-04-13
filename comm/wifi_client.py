"""
wifi_client.py - Xử lý kết nối Socket TCP thô với ESP32.

Module này chỉ chứa class quản lý kết nối mạng ở mức thấp nhất:
mở socket, gửi bytes, nhận bytes, đóng socket.
KHÔNG chứa logic threading hay giải mã giao thức.
"""

import socket


class WifiClient:
    """
    Quản lý kết nối TCP tới ESP32.
    
    Chỉ đóng vai trò là lớp truyền tải (transport layer):
    - Mở/đóng kết nối TCP
    - Gửi dữ liệu thô (raw bytes)
    - Nhận dữ liệu thô (raw bytes)
    """

    # Thời gian chờ đọc dữ liệu mặc định (giây)
    DEFAULT_TIMEOUT = 0.05
    # Kích thước buffer đọc (bytes)
    RECV_BUFFER_SIZE = 1024

    def __init__(self, ip: str = "192.168.4.1", port: int = 8080):
        """
        Khởi tạo client với địa chỉ ESP32.

        Args:
            ip: Địa chỉ IP của ESP32 (mặc định là AP mode: 192.168.4.1)
            port: Cổng TCP trên ESP32 (mặc định: 8080)
        """
        self.ip = ip
        self.port = int(port)
        self._sock: socket.socket | None = None

    @property
    def is_connected(self) -> bool:
        """Kiểm tra trạng thái kết nối hiện tại."""
        return self._sock is not None

    def connect(self) -> None:
        """
        Mở kết nối TCP tới ESP32.

        Raises:
            ConnectionError: Khi không thể kết nối tới ESP32.
            OSError: Khi có lỗi hệ thống mạng.
        """
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
        self._sock.settimeout(self.DEFAULT_TIMEOUT)
        self._sock.connect((self.ip, self.port))

    def send(self, data: bytes) -> None:
        """
        Gửi dữ liệu thô qua kết nối TCP.

        Args:
            data: Dữ liệu bytes cần gửi tới ESP32.

        Raises:
            ConnectionError: Khi socket chưa được mở.
        """
        if not self._sock:
            raise ConnectionError("Socket chưa được kết nối.")
        self._sock.sendall(data)

    def receive(self) -> bytes:
        """
        Nhận dữ liệu thô từ ESP32 qua TCP.

        Returns:
            Bytes dữ liệu nhận được từ ESP32.

        Raises:
            ConnectionAbortedError: Khi ESP32 chủ động đóng kết nối.
            socket.timeout: Khi ESP32 chưa kịp trả lời trong thời gian chờ.
        """
        if not self._sock:
            raise ConnectionError("Socket chưa được kết nối.")
        
        data = self._sock.recv(self.RECV_BUFFER_SIZE)
        if not data:
            raise ConnectionAbortedError("ESP32 chủ động đóng kết nối TCP.")
        return data

    def close(self) -> None:
        """Đóng kết nối socket an toàn (không raise exception)."""
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
