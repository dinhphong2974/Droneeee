"""
wifi_worker.py - Luồng ngầm (QThread) nhận dữ liệu telemetry từ drone.

Module này điều phối giữa WifiClient (kết nối thô) và MSPParser (giải mã):
1. Dùng WifiClient để gửi/nhận bytes qua TCP
2. Dùng MSPParser để giải mã giao thức MSP
3. Phát Signal chứa dữ liệu đã giải mã lên UI
4. Nhận lệnh từ UI thread qua command queue (thread-safe)

Lệnh MSP polling:
- MSP_SONAR_ALTITUDE (58): Độ cao bề mặt từ LiDAR MTF-02
- MSP_ANALOG (110): Điện áp pin
- MSP_ATTITUDE (108): Góc nghiêng
- MSP_ALTITUDE (109): Độ cao barometer
- MSP_STATUS (101): Trạng thái ARM
- MSP_RAW_GPS (106): Tọa độ GPS BZ 251
- MSP_STATUS_EX (150): Sensor health (OptFlow, LiDAR, Compass)

KHÔNG chứa logic socket hay logic bóc tách gói tin.
"""

import time
import struct
import random
import socket
import threading
from queue import Queue, Empty
from PySide6.QtCore import QThread, Signal

from comm.wifi_client import WifiClient
from comm.msp_parser import (
    MSPParser, MSP_ANALOG, MSP_ATTITUDE, MSP_ALTITUDE, MSP_STATUS,
    MSP_RAW_GPS, MSP_SONAR_ALTITUDE, MSP_SET_RAW_RC, MSP_HEADER_REQUEST,
    MSP_STATUS_EX
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
    ping_updated = Signal(int)             # RTT (ms) giữa GCS ↔ ESP32
    command_acked = Signal(str)            # Loại ACK nhận từ ESP32 (RC/FS/EM)

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
        self._command_queue: Queue = Queue()

        # Emergency queue — ưu tiên cao nhất, được drain trước command_queue
        self._emergency_queue: Queue = Queue()
        self._emergency_abort = threading.Event()  # Cờ interrupt cho emergency

        # ── PING/PONG latency measurement ──
        self._ping_interval = 1.0        # Gửi PING mỗi 1 giây
        self._last_ping_time = 0.0       # Thời điểm gửi PING lần cuối

        # ── Text response buffer (PONG/ACK) ──
        self._text_buffer = b''          # Buffer cho text response chưa hoàn chỉnh

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

        Args:
            data: Frame MSP đã đóng gói (output của MSPParser.pack_msg)
                  hoặc lệnh cấu hình failsafe (VD: "FS:rth")
        """
        self._command_queue.put(data)

    def send_emergency_command(self, data: bytes):
        """
        Thread-safe: Gửi lệnh KHẨN CẤP — ưu tiên cao nhất.

        Xóa toàn bộ command queue thường và đặt lệnh emergency lên đầu.
        Được gọi từ FlightController.force_disarm() / force_safe_land().

        Args:
            data: Frame MSP khẩn cấp (DISARM / Safe Land)
        """
        # 1. Set cờ abort để interrupt bất kỳ blocking nào
        self._emergency_abort.set()

        # 2. Xóa tất cả lệnh thường trong queue (không còn cần thiết)
        while not self._command_queue.empty():
            try:
                self._command_queue.get_nowait()
            except Empty:
                break

        # 3. Đưa lệnh emergency vào priority queue
        self._emergency_queue.put(data)

    # ══════════════════════════════════════════════
    # CHẾ ĐỘ KẾT NỐI THẬT (ESP32 qua TCP)
    # ══════════════════════════════════════════════

    def _run_real_mode(self):
        """Vòng lặp chính: kết nối ESP32 → gửi lệnh + hỏi telemetry → phát lên UI."""
        self._client = WifiClient(self.ip, self.port)

        # BUG-08 FIX: Reset text buffer khi bắt đầu kết nối mới.
        # Nếu kết nối trước bị đứt giữa chừng PONG packet, buffer giữ dữ liệu rác
        # → khi kết nối lại sẽ merge với data mới → parse sai PONG timestamp.
        self._text_buffer = b''
        # Reset ping timer → PING được gửi ngay khi kết nối thay vì đợi 1s
        self._last_ping_time = 0.0

        try:
            # Bước 1: Mở kết nối TCP tới ESP32
            self._client.connect()
            self.connection_status.emit(True, f"Đã kết nối Wifi với {self.ip}")

            # Danh sách các lệnh MSP sẽ hỏi FC luân phiên
            # Thêm MSP_SONAR_ALTITUDE để lấy độ cao LiDAR MTF-02
            cmds_to_request = [
                MSP_ANALOG, MSP_ATTITUDE, MSP_ALTITUDE,
                MSP_STATUS, MSP_RAW_GPS, MSP_SONAR_ALTITUDE,
                MSP_STATUS_EX  # Sensor health: OptFlow, LiDAR, Compass
            ]
            cmd_index = 0

            # Bước 2: Vòng lặp hỏi-đáp liên tục
            while self.is_running:
                try:
                    # ── Gửi PING định kỳ (mỗi 1 giây) ──
                    now = time.time()
                    if now - self._last_ping_time >= self._ping_interval:
                        ping_ts = int(now * 1000)  # ms timestamp
                        ping_msg = f"PING:{ping_ts}".encode()
                        self._client.send(ping_msg)
                        self._last_ping_time = now

                    # ── Gửi lệnh điều khiển từ hàng đợi (nếu có) ──
                    self._drain_command_queue()

                    # ── Poll telemetry từ FC ──
                    current_cmd = cmds_to_request[cmd_index]
                    msg_to_send = self._parser.pack_msg(current_cmd)
                    self._client.send(msg_to_send)

                    # ESP/FC ĐÁP: Nhận dữ liệu thô từ TCP
                    raw_data = self._client.receive()

                    # TÁCH STREAM: Text (PONG/ACK) vs Binary (MSP)
                    msp_data = self._extract_text_responses(raw_data)

                    # GIẢI MÃ MSP: Ném data thô vào parser để bóc tách
                    if msp_data:
                        parsed_dict = self._parser.parse_buffer(msp_data)
                    else:
                        parsed_dict = None

                    # Nếu có dữ liệu hợp lệ, phát lên UI
                    if parsed_dict:
                        self.telemetry_data.emit(parsed_dict)

                except socket.timeout:
                    pass  # Bỏ qua nếu FC chưa kịp trả lời
                except ConnectionAbortedError:
                    # ESP32 chủ động đóng kết nối TCP
                    self.connection_status.emit(False, "ESP32 đã ngắt kết nối")
                    break

                # Luân phiên hỏi lệnh tiếp theo
                cmd_index = (cmd_index + 1) % len(cmds_to_request)

                # Nghỉ giữa các lần hỏi (20Hz)
                time.sleep(self.POLLING_INTERVAL)

        except Exception as e:
            self.connection_status.emit(False, f"Lỗi mạng: {str(e)}")
        finally:
            self._close_connection()

    def _extract_text_responses(self, raw_data: bytes) -> bytes:
        """
        Tách text responses (PONG/ACK) và binary MSP data từ TCP stream.

        ESP32 gửi 2 loại data trên cùng 1 TCP stream:
        - Text: PONG:<ts>\\n, ACK:<type>\\n — kết thúc bằng newline
        - Binary: $M> MSP response — không có newline marker

        Method này tách text, xử lý ngay (emit signal), và trả về
        phần binary còn lại cho MSP parser.

        Args:
            raw_data: Bytes thô nhận từ TCP

        Returns:
            bytes: Phần binary (MSP) còn lại sau khi loại bỏ text
        """
        if not raw_data:
            return b''

        # Gộp với buffer chưa hoàn chỉnh từ lần trước
        data = self._text_buffer + raw_data
        self._text_buffer = b''

        msp_parts = []
        i = 0

        while i < len(data):
            # Kiểm tra text prefix
            if data[i:i+5] == b'PONG:' or data[i:i+4] == b'ACK:':
                # Tìm newline kết thúc text line
                nl_pos = data.find(b'\n', i)
                if nl_pos == -1:
                    # Chưa nhận đủ → lưu buffer chờ lần sau
                    self._text_buffer = data[i:]
                    break

                line = data[i:nl_pos].decode(errors='ignore').strip()

                if line.startswith('PONG:'):
                    # Tính RTT từ timestamp gốc
                    try:
                        sent_ts = int(line[5:])
                        rtt = int(time.time() * 1000) - sent_ts
                        if 0 <= rtt < 5000:  # Bỏ giá trị bất thường
                            self.ping_updated.emit(rtt)
                    except (ValueError, OverflowError):
                        pass

                elif line.startswith('ACK:'):
                    ack_type = line[4:]
                    self.command_acked.emit(ack_type)

                i = nl_pos + 1  # Bỏ qua newline
            else:
                # Binary data (MSP) — giữ lại
                msp_parts.append(data[i:i+1])
                i += 1

        return b''.join(msp_parts)

    def _drain_command_queue(self):
        """Gửi lệnh: emergency queue trước, command queue sau."""
        # Priority 1: Emergency commands (ưu tiên cao nhất)
        while True:
            try:
                cmd_data = self._emergency_queue.get_nowait()
                if self._client and self._client.is_connected:
                    self._client.send(cmd_data)
            except Empty:
                break

        # Reset emergency flag sau khi đã gửi hết
        self._emergency_abort.clear()

        # Priority 2: Normal commands
        while True:
            try:
                cmd_data = self._command_queue.get_nowait()
                if self._client and self._client.is_connected:
                    self._client.send(cmd_data)
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

            # ── Giả lập LiDAR MTF-02 (surface altitude) ──
            if self._mock_altitude <= 2.5 and self._mock_altitude > 0:
                mock_surface_alt = self._mock_altitude + random.uniform(-0.02, 0.02)
                mock_surface_alt = max(0.0, mock_surface_alt)
                mock_surface_quality = 255
            else:
                mock_surface_alt = -1.0  # Ngoài tầm LiDAR
                mock_surface_quality = 0

            # Đóng gói và phát dữ liệu giả lên UI (telemetry + GPS + LiDAR)
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
                # ── LiDAR Data (MTF-02 giả lập) ──
                "surface_altitude": mock_surface_alt,
                "surface_quality": mock_surface_quality,
            }
            self.telemetry_data.emit(fake_data)

            # Nghỉ giữa các lần gửi (20Hz)
            time.sleep(self.POLLING_INTERVAL)

        self.connection_status.emit(False, "Đã dừng Mock Test")

    def _process_mock_commands(self):
        """Đọc lệnh từ hàng đợi và cập nhật trạng thái giả lập."""
        # Gộp cả emergency queue và command queue
        all_queues = [self._emergency_queue, self._command_queue]
        for queue in all_queues:
            while True:
                try:
                    cmd_data = queue.get_nowait()

                    # Bỏ qua lệnh cấu hình failsafe (prefix FS:)
                    if isinstance(cmd_data, bytes) and cmd_data.startswith(b'FS:'):
                        continue

                    # BUG-01 FIX: Strip prefix EM: (emergency marker) trước khi parse
                    # force_disarm()/force_safe_land() gắn prefix này để ESP32 ưu tiên,
                    # nhưng mock mode phải bóc prefix trước khi kiểm tra MSP header.
                    if isinstance(cmd_data, bytes) and cmd_data.startswith(b'EM:'):
                        cmd_data = cmd_data[3:]  # Bỏ 3 bytes 'EM:'

                    # Tìm frame MSP_SET_RAW_RC: $M< + size(16) + cmd(200) + payload(16B) + checksum
                    # BUG-01 FIX: Kiểm tra len trước struct.unpack để tránh crash
                    if (isinstance(cmd_data, bytes)
                            and len(cmd_data) >= 22
                            and cmd_data[0:3] == MSP_HEADER_REQUEST
                            and cmd_data[4] == MSP_SET_RAW_RC):
                        payload = cmd_data[5:21]
                        if len(payload) < 16:  # Guard: payload phải đủ 16 bytes
                            continue
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

        # Reset emergency flag
        self._emergency_abort.clear()

    def _update_mock_physics(self):
        """Giả lập vật lý: altitude thay đổi theo throttle khi armed."""
        if not self._mock_armed:
            self._mock_altitude = max(0.0, self._mock_altitude - 0.1)
            return

        throttle = self._mock_channels[2]
        aux2_althold = self._mock_channels[5] > 1800  # ALTHOLD+POSHOLD: 2000

        if aux2_althold:
            # BUG-07 FIX: ALTHOLD+POSHOLD mode — mô phỏng INAV altitude hold behavior:
            #   throttle > 1500 → leo (climb), = 1500 → giữ (hover), < 1500 → hạ
            # Giống INAV ALTHOLD thực tế: 1700μs ≈ 2-3m/s climb rate
            climb_rate = (throttle - 1500) / 1000.0  # Normalize: range ~-0.5 to +0.5 m/s per tick
            if abs(climb_rate) < 0.02:  # Dead-band quanh center (±20μs)
                # Hover: chỉ jitter nhỏ
                self._mock_altitude += random.uniform(-0.02, 0.02)
            else:
                self._mock_altitude += climb_rate * 0.5  # Scale xuống cho mock (50ms tick)
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

        Chỉ set cờ is_running = False, để thread tự dọn dẹp trong finally block.
        Không gọi _close_connection() ở đây vì sẽ race condition với worker thread.
        """
        self.is_running = False