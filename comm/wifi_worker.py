"""
wifi_worker.py - Luồng ngầm (QThread) nhận dữ liệu telemetry từ drone.

Module này điều phối giữa WifiClient (kết nối thô) và MSPParser (giải mã):
1. Dùng WifiClient để gửi/nhận bytes qua TCP
2. Dùng MSPParser để giải mã giao thức MSP
3. Phát Signal chứa dữ liệu đã giải mã lên UI

KHÔNG chứa logic socket hay logic bóc tách gói tin.
"""

import time
import random
import socket
from PySide6.QtCore import QThread, Signal

from comm.wifi_client import WifiClient
from comm.msp_parser import MSPParser, MSP_ANALOG, MSP_ATTITUDE


class WifiWorker(QThread):
    """
    Luồng chạy ngầm, liên tục hỏi FC lấy dữ liệu và phát lên UI.

    Signals:
        connection_status(bool, str): Trạng thái mạng (thành_công, lời_nhắn)
        telemetry_data(dict): Dữ liệu đã giải mã gửi lên UI hiển thị
    """

    # ── Signal giao tiếp với main.py ──
    connection_status = Signal(bool, str)  # (Thành công/Thất bại, Lời nhắn)
    telemetry_data = Signal(dict)          # Dữ liệu telemetry đã giải mã

    # ── Cấu hình tần số lấy mẫu ──
    POLLING_INTERVAL = 0.05  # 20Hz (50ms/lần)

    def __init__(self, ip: str = "192.168.4.1", port: int = 8080, is_mock: bool = False):
        """
        Khởi tạo worker thread.

        Args:
            ip: Địa chỉ IP của ESP32
            port: Cổng TCP trên ESP32
            is_mock: True = chạy giả lập, False = kết nối thật
        """
        super().__init__()
        self.ip = ip
        self.port = int(port)
        self.is_mock = is_mock
        self.is_running = True

        # Khởi tạo module kết nối và giải mã
        self._client: WifiClient | None = None
        self._parser = MSPParser()

    def run(self):
        """Điểm bắt đầu của thread — tự chạy khi gọi self.start()."""
        if self.is_mock:
            self._run_mock_mode()
        else:
            self._run_real_mode()

    # ══════════════════════════════════════════════
    # CHẾ ĐỘ KẾT NỐI THẬT (ESP32 qua TCP)
    # ══════════════════════════════════════════════

    def _run_real_mode(self):
        """Vòng lặp chính: kết nối ESP32 → liên tục hỏi FC → giải mã → phát lên UI."""
        self._client = WifiClient(self.ip, self.port)

        try:
            # Bước 1: Mở kết nối TCP tới ESP32
            self._client.connect()
            self.connection_status.emit(True, f"Đã kết nối Wifi với {self.ip}")

            # Danh sách các lệnh MSP sẽ hỏi FC luân phiên
            cmds_to_request = [MSP_ANALOG, MSP_ATTITUDE]
            cmd_index = 0

            # Bước 2: Vòng lặp hỏi-đáp liên tục
            while self.is_running:
                try:
                    # PC HỎI: Đóng gói lệnh MSP và gửi qua TCP
                    current_cmd = cmds_to_request[cmd_index]
                    msg_to_send = self._parser.pack_msg(current_cmd)
                    self._client.send(msg_to_send)

                    # ESP/FC ĐÁP: Nhận dữ liệu thô từ TCP
                    raw_data = self._client.receive()

                    # GIẢI MÃ: Ném data thô vào parser để bóc tách
                    parsed_dict = self._parser.parse_buffer(raw_data)

                    # Nếu có dữ liệu hợp lệ (pin, góc nghiêng...), phát lên UI
                    if parsed_dict:
                        self.telemetry_data.emit(parsed_dict)

                except socket.timeout:
                    pass  # Bỏ qua nếu FC chưa kịp trả lời

                # Luân phiên hỏi lệnh tiếp theo (pin → góc → pin → ...)
                cmd_index = (cmd_index + 1) % len(cmds_to_request)

                # Nghỉ giữa các lần hỏi (20Hz)
                time.sleep(self.POLLING_INTERVAL)

        except Exception as e:
            self.connection_status.emit(False, f"Lỗi mạng: {str(e)}")
        finally:
            self._close_connection()

    # ══════════════════════════════════════════════
    # CHẾ ĐỘ GIẢ LẬP (Mock Test)
    # ══════════════════════════════════════════════

    def _run_mock_mode(self):
        """Giả lập dữ liệu telemetry để test UI không cần phần cứng."""
        self.connection_status.emit(True, "Đang chạy chế độ Mock Test (Giả lập)")

        # Thông số gốc giả lập cho pin 6S (19.8V rỗng - 25.2V đầy)
        mock_volt = 25.2
        mock_pitch = 0.0

        while self.is_running:
            # Giả lập pin tụt dần (Lipo 6S: 19.8V cạn - 25.2V đầy)
            mock_volt -= random.uniform(0.001, 0.01)
            if mock_volt < 19.5:
                mock_volt = 25.2

            # Giả lập dao động góc nghiêng
            mock_pitch += random.uniform(-1.0, 1.0)
            mock_pitch = max(-30, min(30, mock_pitch))

            # Giả lập vòng tua động cơ 1960kv (PWM range: 1000-2000)
            motor_base = random.randint(1100, 1500)

            # Đóng gói và phát dữ liệu giả lên UI
            fake_data = {
                "voltage": mock_volt,
                "pitch": mock_pitch,
                "roll": random.uniform(-5.0, 5.0),
                "yaw": random.uniform(0, 360),
                "motor1": motor_base + random.randint(-20, 20),
                "motor2": motor_base + random.randint(-20, 20),
                "motor3": motor_base + random.randint(-20, 20),
                "motor4": motor_base + random.randint(-20, 20),
            }
            self.telemetry_data.emit(fake_data)

            # Nghỉ giữa các lần gửi (20Hz)
            time.sleep(self.POLLING_INTERVAL)

        self.connection_status.emit(False, "Đã dừng Mock Test")

    # ══════════════════════════════════════════════
    # ĐIỀU KHIỂN TỪ BÊN NGOÀI
    # ══════════════════════════════════════════════

    def _close_connection(self):
        """Đóng kết nối TCP thông qua WifiClient."""
        if self._client:
            self._client.close()
            self._client = None

    def stop(self):
        """
        Dừng worker thread an toàn.
        Được gọi từ main.py khi người dùng bấm nút Ngắt Kết Nối.
        """
        self.is_running = False
        self._close_connection()