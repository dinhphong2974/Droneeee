"""
wifi_worker.py - Luồng ngầm (QThread) nhận dữ liệu telemetry từ drone.

Module này điều phối giữa WifiClient (kết nối thô) và MSPParser (giải mã):
1. Dùng WifiClient để gửi/nhận bytes qua TCP
2. Dùng MSPParser để giải mã giao thức MSP
3. Phát Signal chứa dữ liệu đã giải mã lên UI
4. Nhận lệnh từ UI thread qua command queue (thread-safe)

Lệnh MSP polling:
- MSP_ANALOG (110): Điện áp pin
- MSP_ATTITUDE (108): Góc nghiêng
- MSP_ALTITUDE (109): Độ cao barometer
- MSP_STATUS (101): Trạng thái ARM
- MSP_RAW_GPS (106): Tọa độ GPS BZ 251

KHÔNG chứa logic socket hay logic bóc tách gói tin.
"""

import time
import struct
import random
import socket
import math
from queue import Queue, Empty
from PySide6.QtCore import QThread, Signal

from comm.wifi_client import WifiClient
from comm.msp_parser import (
    MSPParser, MSP_ANALOG, MSP_ATTITUDE, MSP_ALTITUDE, MSP_STATUS,
    MSP_RAW_GPS, MSP_SET_RAW_RC, MSP_HEADER_REQUEST
)


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
        self.ip = ip or "192.168.4.1"
        self.port = int(port) if port is not None else 8080
        self.is_mock = is_mock
        self.is_running = True

        # Khởi tạo module kết nối và giải mã
        self._client: WifiClient | None = None
        self._parser = MSPParser()

        # Queue nhận lệnh từ UI thread (thread-safe)
        # ★ TASK-10: Giới hạn 10 lệnh — tránh tích lũy vô hạn khi mất kết nối
        self._command_queue: Queue = Queue(maxsize=10)

        # ── Mock mode state ──
        self._mock_channels = [1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000]
        self._mock_armed = False
        self._mock_altitude = 0.0
        # Mock GPS (Hà Nội mặc định)
        self._mock_gps_lat = 21.028500
        self._mock_gps_lon = 105.854200
        self._mock_gps_fix = 2   # 3D fix
        self._mock_gps_sats = 12
        self._mock_gps_speed = 0.0
        self._mock_home_lat = 21.028500
        self._mock_home_lon = 105.854200

    def run(self):
        """Điểm bắt đầu của thread — tự chạy khi gọi self.start()."""
        if self.is_mock:
            self._run_mock_mode()
        else:
            self._run_real_mode()

    # ══════════════════════════════════════════════
    # GIAO DIỆN CÔNG KHAI (gọi từ Main Thread)
    # ══════════════════════════════════════════════

    def send_command(self, data: bytes):
        """
        Thread-safe: Đưa lệnh MSP vào hàng đợi để gửi tới FC.

        Được gọi từ FlightController (Main Thread).
        Worker thread sẽ lấy và gửi qua TCP ở vòng lặp tiếp theo.

        ★ TASK-10: Nếu queue đầy, bỏ lệnh cũ nhất (drone chỉ cần lệnh mới nhất).

        Args:
            data: Frame MSP đã đóng gói (output của MSPParser.pack_msg)
                  hoặc lệnh cấu hình failsafe (VD: "FS:rth")
        """
        if self._command_queue.full():
            try:
                self._command_queue.get_nowait()  # Bỏ lệnh cũ nhất
            except Empty:
                pass
        self._command_queue.put_nowait(data)

    # ══════════════════════════════════════════════
    # CHẾ ĐỘ KẾT NỐI THẬT (ESP32 qua TCP)
    # ══════════════════════════════════════════════

    def _run_real_mode(self):
        """Vòng lặp chính: kết nối ESP32 → gửi lệnh + hỏi telemetry → phát lên UI."""
        self._client = WifiClient(self.ip, self.port)

        try:
            # Bước 1: Mở kết nối TCP tới ESP32
            self._client.connect()
            self.connection_status.emit(True, f"Đã kết nối Wifi với {self.ip}")

            # Danh sách các lệnh MSP sẽ hỏi FC luân phiên
            # Thêm MSP_RAW_GPS để lấy tọa độ từ GPS BZ 251
            cmds_to_request = [MSP_ANALOG, MSP_ATTITUDE, MSP_ALTITUDE, MSP_STATUS, MSP_RAW_GPS]
            cmd_index = 0

            # Bước 2: Vòng lặp hỏi-đáp liên tục
            while self.is_running:
                try:
                    # ── Gửi lệnh điều khiển từ hàng đợi (nếu có) ──
                    self._drain_command_queue()

                    # ── Poll telemetry từ FC ──
                    current_cmd = cmds_to_request[cmd_index]
                    msg_to_send = self._parser.pack_msg(current_cmd)
                    self._client.send(msg_to_send)

                    # ESP/FC ĐÁP: Nhận dữ liệu thô từ TCP
                    raw_data = self._client.receive()

                    # GIẢI MÃ: Ném data thô vào parser để bóc tách
                    parsed_dict = self._parser.parse_buffer(raw_data)

                    # Nếu có dữ liệu hợp lệ, phát lên UI
                    if parsed_dict:
                        self.telemetry_data.emit(parsed_dict)

                except socket.timeout:
                    pass  # Bỏ qua nếu FC chưa kịp trả lời

                # Luân phiên hỏi lệnh tiếp theo
                cmd_index = (cmd_index + 1) % len(cmds_to_request)

                # Nghỉ giữa các lần hỏi (20Hz)
                time.sleep(self.POLLING_INTERVAL)

        except Exception as e:
            self.connection_status.emit(False, f"Lỗi mạng: {str(e)}")
        finally:
            self._close_connection()

    def _drain_command_queue(self):
        """Lấy và gửi lệnh đang chờ trong hàng đợi qua TCP.

        ★ TASK-10: Giới hạn 2 lệnh/vòng để tránh burst nhiều frame
        liên tiếp gây nghẽn ESP32 UART buffer.
        """
        MAX_CMDS_PER_TICK = 2
        sent = 0
        while sent < MAX_CMDS_PER_TICK:
            try:
                cmd_data = self._command_queue.get_nowait()
                if self._client and self._client.is_connected:
                    self._client.send(cmd_data)
                sent += 1
            except Empty:
                break

    # ══════════════════════════════════════════════
    # CHẾ ĐỘ GIẢ LẬP (Mock Test)
    # ══════════════════════════════════════════════

    def _run_mock_mode(self):
        """Giả lập dữ liệu telemetry + GPS + phản hồi lệnh bay để test UI."""
        self.connection_status.emit(True, "Đang chạy chế độ Mock Test (Giả lập)")

        # Thông số gốc giả lập cho pin 6S (19.8V rỗng - 25.2V đầy)
        mock_volt = 25.2
        mock_pitch = 0.0

        while self.is_running:
            # ── Xử lý lệnh từ FlightController (nếu có) ──
            self._process_mock_commands()

            # ── Giả lập vật lý bay (altitude thay đổi theo throttle) ──
            self._update_mock_physics()

            # ── Giả lập GPS (vị trí thay đổi nhẹ khi armed) ──
            self._update_mock_gps()

            # Giả lập pin tụt dần (Lipo 6S: 19.8V cạn - 25.2V đầy)
            mock_volt -= random.uniform(0.001, 0.01)
            if mock_volt < 19.5:
                mock_volt = 25.2

            # Giả lập dao động góc nghiêng
            mock_pitch += random.uniform(-1.0, 1.0)
            mock_pitch = max(-30, min(30, mock_pitch))

            # Giả lập vòng tua động cơ 1960kv theo throttle
            throttle = self._mock_channels[2]
            motor_base = max(1000, int(throttle * 0.9 + 100)) if self._mock_armed else 1000

            # Đóng gói và phát dữ liệu giả lên UI (telemetry + GPS)
            fake_data = {
                "voltage": mock_volt,
                "pitch": mock_pitch,
                "roll": random.uniform(-5.0, 5.0),
                "yaw": random.uniform(0, 360),
                "motor1": motor_base + random.randint(-20, 20),
                "motor2": motor_base + random.randint(-20, 20),
                "motor3": motor_base + random.randint(-20, 20),
                "motor4": motor_base + random.randint(-20, 20),
                "altitude": self._mock_altitude,
                "vario": 0.0,
                "is_armed": self._mock_armed,
                "flight_mode_flags": (1 if self._mock_armed else 0),
                # ── GPS Data (từ GPS BZ 251 giả lập) ──
                "gps_fix_type": self._mock_gps_fix,
                "gps_num_sat": self._mock_gps_sats,
                "latitude": self._mock_gps_lat,
                "longitude": self._mock_gps_lon,
                "gps_altitude": self._mock_altitude,
                "ground_speed": self._mock_gps_speed,
                "ground_course": random.uniform(0, 360),
            }
            self.telemetry_data.emit(fake_data)

            # Nghỉ giữa các lần gửi (20Hz)
            time.sleep(self.POLLING_INTERVAL)

        self.connection_status.emit(False, "Đã dừng Mock Test")

    def _process_mock_commands(self):
        """Đọc lệnh từ hàng đợi và cập nhật trạng thái giả lập."""
        while True:
            try:
                cmd_data = self._command_queue.get_nowait()

                # Bỏ qua lệnh cấu hình failsafe (prefix FS:)
                if isinstance(cmd_data, bytes) and cmd_data.startswith(b'FS:'):
                    continue

                # Tìm frame MSP_SET_RAW_RC: $M< + size(16) + cmd(200) + payload(16B) + checksum
                if (isinstance(cmd_data, bytes) and len(cmd_data) >= 22
                        and cmd_data[0:3] == MSP_HEADER_REQUEST
                        and cmd_data[4] == MSP_SET_RAW_RC):
                    payload = cmd_data[5:21]
                    channels = list(struct.unpack('<8H', payload))
                    self._mock_channels = channels
                    # AUX1 (index 4) > 1700 → ARM (dải kích hoạt: 2000)
                    self._mock_armed = channels[4] > 1700

                    # AUX3 (index 6) > 1700 → Safe Land giả lập
                    if channels[6] > 1700 and self._mock_armed:
                        # Giả lập hạ cánh: giảm altitude dần
                        self._mock_altitude = max(0.0, self._mock_altitude - 0.3)
                        if self._mock_altitude <= 0.1:
                            self._mock_armed = False
                            self._mock_altitude = 0.0

                    # AUX4 (index 7) > 1700 → RTH giả lập
                    if channels[7] > 1700 and self._mock_armed:
                        # Giả lập RTH: di chuyển về home dần
                        self._mock_gps_lat += (self._mock_home_lat - self._mock_gps_lat) * 0.05
                        self._mock_gps_lon += (self._mock_home_lon - self._mock_gps_lon) * 0.05
            except Empty:
                break

    def _update_mock_physics(self):
        """Giả lập vật lý: altitude thay đổi theo throttle khi armed."""
        if not self._mock_armed:
            self._mock_altitude = max(0.0, self._mock_altitude - 0.1)
            return

        throttle = self._mock_channels[2]
        aux2_althold = self._mock_channels[5] > 1800  # ALTHOLD+POSHOLD: 2000

        if aux2_althold:
            # ALTHOLD+POSHOLD mode: giữ altitude ổn định
            self._mock_altitude += random.uniform(-0.05, 0.05)
            self._mock_altitude = max(0.0, self._mock_altitude)
        else:
            # Manual mode: altitude phụ thuộc throttle
            if throttle > 1400:
                climb_rate = (throttle - 1450) / 200.0
                self._mock_altitude += climb_rate * 0.05
                self._mock_altitude = max(0.0, self._mock_altitude)
            elif self._mock_altitude > 0:
                self._mock_altitude = max(0.0, self._mock_altitude - 0.05)

    def _update_mock_gps(self):
        """Giả lập GPS: vị trí thay đổi nhẹ khi armed (drift tự nhiên)."""
        if self._mock_armed:
            # Giả lập drift GPS nhỏ khi đang bay
            self._mock_gps_lat += random.uniform(-0.000002, 0.000002)
            self._mock_gps_lon += random.uniform(-0.000002, 0.000002)
            self._mock_gps_speed = random.uniform(0.0, 2.0) if self._mock_altitude > 0.5 else 0.0
            self._mock_gps_sats = random.randint(10, 14)
        else:
            self._mock_gps_speed = 0.0
            self._mock_gps_sats = random.randint(8, 14)

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

        ★ TASK-09: KHÔNG đóng socket từ Main Thread — để tránh race condition
        với Worker Thread đang dùng socket. Worker thread sẽ tự dọn dẹp
        trong finally block (line 160-161) khi is_running=False.
        """
        self.is_running = False
        self.wait(3000)  # Chờ worker thread tự kết thúc tối đa 3s