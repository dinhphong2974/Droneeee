"""
msp_parser.py - Đóng gói và giải mã giao thức MSP (MultiWii Serial Protocol).

Module này xử lý toàn bộ logic bóc tách gói tin MSP:
- Đóng gói lệnh request ($M<) gửi tới Flight Controller
- Giải mã response ($M>) nhận về từ Flight Controller
- Quản lý buffer chống đứt gói do lag mạng Wifi
- Tính và xác thực checksum chống nhiễu

Lệnh MSP hỗ trợ:
- MSP_WP_GETINFO (20): Thông tin mission (số waypoint)
- MSP_SONAR_ALTITUDE (58): Độ cao bề mặt từ LiDAR/Sonar (MTF-02)
- MSP_STATUS (101): Trạng thái ARM + flight mode flags
- MSP_RAW_GPS (106): Tọa độ GPS, số vệ tinh, tốc độ (từ GPS BZ 251)
- MSP_ATTITUDE (108): Góc nghiêng Roll, Pitch, Yaw
- MSP_ALTITUDE (109): Độ cao barometer + vario
- MSP_ANALOG (110): Điện áp pin, dòng điện
- MSP_SET_RAW_RC (200): Gửi 8 kênh RC (ARM, throttle, AUX1-4)
- MSP_SET_WP (209): Gửi waypoint xuống FC cho NAV WP mission

Thông số phần cứng liên quan:
- Pin: Lipo 6S (19.8V rỗng → 25.2V đầy)
- Động cơ: 1960kv (PWM range: 1000-2000μs)
- GPS: BZ 251 (GPS + La bàn)
- LiDAR: Micoair MTF-02 AIO (Optical Flow + Rangefinder, range ≤2.5m)
"""

import struct

# ══════════════════════════════════════════════
# MÃ LỆNH MSP (MultiWii Serial Protocol)
# ══════════════════════════════════════════════

# ── Lệnh đọc dữ liệu (FC → PC) ──
MSP_WP_GETINFO      = 20    # Lấy thông tin mission (tổng số waypoint, max WP)
MSP_SONAR_ALTITUDE  = 58    # Lấy độ cao bề mặt từ rangefinder/LiDAR (MTF-02)
MSP_STATUS          = 101   # Lấy trạng thái ARM + flight mode flags
MSP_RAW_GPS         = 106   # Lấy tọa độ GPS, vệ tinh, tốc độ (từ GPS BZ 251)
MSP_ATTITUDE        = 108   # Lấy góc nghiêng (Roll, Pitch, Yaw)
MSP_ALTITUDE        = 109   # Lấy độ cao ước lượng (barometer)
MSP_ANALOG          = 110   # Lấy thông số pin (Voltage, Current)
MSP_STATUS_EX       = 150   # Trạng thái mở rộng: sensor health + arming disable flags

# ── Lệnh ghi dữ liệu (PC → FC) ──
MSP_SET_RAW_RC = 200   # Gửi 8 kênh RC xuống FC (ARM, throttle, AUX...)
MSP_SET_WP     = 209   # Gửi 1 waypoint xuống FC cho NAV WP mission

# ── Header giao thức ──
MSP_HEADER_REQUEST  = b'$M<'   # PC gửi đi → FC
MSP_HEADER_RESPONSE = b'$M>'   # FC trả về → PC

# ── Cấu hình pin Lipo 6S ──
LIPO_6S_VOLTAGE_DIVIDER = 10.0  # FC gửi điện áp nhân 10 (VD: 245 → 24.5V)
CURRENT_DIVIDER = 100.0          # FC gửi dòng điện nhân 100

# ── GPS ──
GPS_COORD_SCALE = 10_000_000.0  # Hệ số chia tọa độ GPS (lat/lon x 10^7)

# ── Kích thước frame tối thiểu ──
MIN_FRAME_SIZE = 6  # Header(3) + Size(1) + Cmd(1) + Checksum(1)

# ── Giới hạn buffer chống tràn bộ nhớ ──
MAX_BUFFER_SIZE = 4096  # Cắt buffer nếu vượt quá (byte rác liên tục)
MAX_PAYLOAD_SIZE = 255  # Giới hạn cứng MSP v1: size field là 1 byte (0–255)


class MSPParser:
    """
    Bộ giải mã giao thức MSP cho giao tiếp với Flight Controller (INAV).

    Xử lý hai chiều:
    - pack_msg(): PC → ESP32 → FC (đóng gói lệnh request)
    - parse_buffer(): FC → ESP32 → PC (giải mã response)
    """

    def __init__(self):
        """Khởi tạo parser với buffer rỗng để gom gói tin bị đứt khúc."""
        self.buffer = bytearray()

    # ══════════════════════════════════════════════
    # ĐÓNG GÓI LỆNH GỬI ĐI (PC → FC)
    # ══════════════════════════════════════════════

    def pack_msg(self, cmd: int, payload: bytes = b'') -> bytes:
        """
        Đóng gói lệnh MSP gửi tới Flight Controller.

        Cấu trúc frame: $M< + Size + Cmd + Payload + Checksum
        Checksum = XOR(size, cmd, payload_bytes...)

        Args:
            cmd: Mã lệnh MSP (VD: MSP_ANALOG = 110)
            payload: Dữ liệu đính kèm (thường rỗng cho lệnh request)

        Returns:
            bytes: Frame MSP hoàn chỉnh sẵn sàng gửi qua TCP
        """
        size = len(payload)
        msg = bytearray(MSP_HEADER_REQUEST)
        msg.append(size)
        msg.append(cmd)
        msg.extend(payload)

        # Tính Checksum bằng phép XOR
        checksum = self._calculate_checksum(size, cmd, payload)
        msg.append(checksum)

        return bytes(msg)

    def pack_set_raw_rc(self, channels: list[int]) -> bytes:
        """
        Đóng gói lệnh MSP_SET_RAW_RC gửi 8 kênh RC xuống Flight Controller.

        Thứ tự kênh INAV: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]
        Mỗi kênh là uint16 LE, range 1000-2000μs.

        Args:
            channels: List 8 giá trị kênh RC (1000-2000μs)

        Returns:
            bytes: Frame MSP hoàn chỉnh sẵn sàng gửi qua TCP
        """
        if len(channels) != 8:
            raise ValueError(f"Cần đúng 8 kênh RC, nhận được {len(channels)}")
        payload = struct.pack('<8H', *channels)
        return self.pack_msg(MSP_SET_RAW_RC, payload)

    def pack_set_wp(self, wp_no: int, lat: float, lon: float, alt_cm: int,
                    p1: int = 0, p2: int = 0, p3: int = 0) -> bytes:
        """
        Đóng gói lệnh MSP_SET_WP gửi 1 waypoint xuống FC.

        INAV sử dụng lệnh này để nạp mission waypoint vào bộ nhớ FC.
        Sau khi nạp tất cả WP, bật mode NAV WP (qua AUX) để FC tự bay.

        Args:
            wp_no: Số thứ tự waypoint (1-N cho mission, 0=RTH, 255=POSHOLD)
            lat: Vĩ độ (độ thập phân, VD: 21.0285)
            lon: Kinh độ (độ thập phân, VD: 105.8542)
            alt_cm: Độ cao tương đối so với Home (cm)
            p1: Tham số 1 — Tốc độ (cm/s) hoặc thời gian dừng (giây)
            p2: Tham số 2 — Dự phòng
            p3: Tham số 3 — Cờ đặc biệt (0x48 = flag cho WP cuối cùng)

        Returns:
            bytes: Frame MSP hoàn chỉnh
        """
        # Chuyển đổi tọa độ sang đơn vị INAV (nhân 10^7)
        lat_int = int(lat * GPS_COORD_SCALE)
        lon_int = int(lon * GPS_COORD_SCALE)

        # Đóng gói payload 21 bytes: wp_no(1) + action(1) + lat(4) + lon(4) + alt(4) + p1(2) + p2(2) + p3(2) + flag(1)
        # Cấu trúc INAV MSP_SET_WP:
        # Byte 0: wp_no (uint8)
        # Byte 1: action (uint8) — 1=WAYPOINT, 4=RTH
        # Byte 2-5: lat (int32, x10^7)
        # Byte 6-9: lon (int32, x10^7)
        # Byte 10-13: alt (int32, cm relative to home)
        # Byte 14-15: p1 (int16) — speed cm/s
        # Byte 16-17: p2 (int16)
        # Byte 18: p3 (uint8) — flags (0x48 = last WP flag)
        action = 1  # WAYPOINT action code
        payload = struct.pack('<BB i i i h h B',
                              wp_no, action,
                              lat_int, lon_int, alt_cm,
                              p1, p2, p3)
        return self.pack_msg(MSP_SET_WP, payload)

    # ══════════════════════════════════════════════
    # GIẢI MÃ DỮ LIỆU NHẬN VỀ (FC → PC)
    # ══════════════════════════════════════════════

    def parse_buffer(self, data: bytes) -> dict:
        """
        Nhận byte thô từ mạng, lọc rác, tìm frame MSP hợp lệ và giải mã.

        Quy trình xử lý:
        1. Gom data mới vào buffer (chống đứt gói do lag Wifi)
        2. Tìm header $M> trong buffer
        3. Cắt bỏ byte rác trước header
        4. Kiểm tra đủ độ dài frame
        5. Xác thực checksum chống nhiễu
        6. Giải mã payload theo từng loại lệnh

        Args:
            data: Dữ liệu bytes thô nhận từ TCP socket

        Returns:
            dict: Dữ liệu đã giải mã, VD: {"voltage": 24.5, "roll": 1.2}
                  Trả về dict rỗng nếu không có frame hợp lệ.
        """
        self.buffer.extend(data)
        parsed_data = {}

        # Chống tràn bộ nhớ: cắt buffer nếu nhận quá nhiều byte rác
        if len(self.buffer) > MAX_BUFFER_SIZE:
            self.buffer = self.buffer[-MAX_BUFFER_SIZE:]

        # Quét buffer tìm tất cả frame MSP response
        while MSP_HEADER_RESPONSE in self.buffer:
            start_idx = self.buffer.find(MSP_HEADER_RESPONSE)

            # Cắt bỏ byte rác nằm trước header
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]

            # Kiểm tra đủ độ dài tối thiểu (6 bytes)
            if len(self.buffer) < MIN_FRAME_SIZE:
                break  # Chưa đủ, chờ vòng lặp mạng tiếp theo bơm thêm data

            size = self.buffer[3]
            cmd = self.buffer[4]

            # BUG GUARD: Reject frame nếu size > MAX_PAYLOAD_SIZE (255)
            # MSP v1 dùng 1 byte cho size field → max 255 bytes payload.
            # Rogue frame / byte rác có thể giả làm header khiến parser chờ mãi.
            # Xử lý: bỏ qua byte header này và tiếp tục tìm kiếm header mới.
            if size > MAX_PAYLOAD_SIZE:
                self.buffer = self.buffer[1:]  # Skip byte '$', tìm header tiếp theo
                continue

            frame_length = MIN_FRAME_SIZE + size

            # Kiểm tra đã nhận đủ toàn bộ Payload + Checksum chưa
            if len(self.buffer) < frame_length:
                break  # Chưa đủ, giữ buffer chờ thêm data

            # Tách frame hoàn chỉnh ra khỏi buffer
            frame = self.buffer[:frame_length]
            self.buffer = self.buffer[frame_length:]

            payload = frame[5:5 + size]
            received_checksum = frame[-1]

            # Xác thực checksum chống nhiễu sóng
            calc_checksum = self._calculate_checksum(size, cmd, payload)
            if calc_checksum != received_checksum:
                continue  # Gói tin bị lỗi/nhiễu, bỏ qua

            # Giải mã payload theo loại lệnh MSP
            decoded = self._decode_payload(cmd, size, payload)
            parsed_data.update(decoded)

        return parsed_data

    # ══════════════════════════════════════════════
    # CÁC HÀM NỘI BỘ
    # ══════════════════════════════════════════════

    @staticmethod
    def _calculate_checksum(size: int, cmd: int, payload: bytes) -> int:
        """
        Tính checksum MSP bằng phép XOR.

        Checksum = size XOR cmd XOR payload[0] XOR payload[1] XOR ...

        Args:
            size: Độ dài payload
            cmd: Mã lệnh MSP
            payload: Dữ liệu payload

        Returns:
            int: Giá trị checksum (0-255)
        """
        checksum = size ^ cmd
        for b in payload:
            checksum ^= b
        return checksum

    @staticmethod
    def _decode_payload(cmd: int, size: int, payload: bytes) -> dict:
        """
        Giải mã payload theo từng loại lệnh MSP.

        Args:
            cmd: Mã lệnh MSP
            size: Kích thước payload
            payload: Bytes payload đã qua checksum

        Returns:
            dict: Dữ liệu đã giải mã theo key-value

        Lệnh hỗ trợ:
        - MSP_WP_GETINFO (20): Thông tin mission
        - MSP_SONAR_ALTITUDE (58): Độ cao bề mặt LiDAR (MTF-02)
        - MSP_STATUS (101): Trạng thái ARM + flight mode flags
        - MSP_RAW_GPS (106): Tọa độ GPS, số vệ tinh, tốc độ
        - MSP_ATTITUDE (108): Góc Roll, Pitch, Yaw
        - MSP_ALTITUDE (109): Độ cao barometer + vario
        - MSP_ANALOG (110): Điện áp pin 6S, dòng điện
        """
        result = {}

        try:
            if cmd == MSP_ANALOG and size >= 7:
                # Cấu trúc: vbat(1B uint) + mah_drawn(2B uint) + rssi(2B uint) + amperage(2B int)
                vbat, mah, rssi, amp = struct.unpack('<B H H h', payload[:7])
                # INAV gửi điện áp x10 (VD: 245 → 24.5V cho pin Lipo 6S)
                result['voltage'] = vbat / LIPO_6S_VOLTAGE_DIVIDER
                result['current'] = amp / CURRENT_DIVIDER

            elif cmd == MSP_ATTITUDE and size >= 6:
                # Cấu trúc: roll(2B int) + pitch(2B int) + yaw(2B int)
                roll, pitch, yaw = struct.unpack('<h h h', payload[:6])
                # Roll và Pitch gửi x10 (VD: 15 → 1.5°)
                result['roll'] = roll / 10.0
                result['pitch'] = pitch / 10.0
                # Yaw gửi theo độ chuẩn (0-360°)
                result['yaw'] = yaw

            elif cmd == MSP_ALTITUDE and size >= 6:
                # Cấu trúc: est_alt(4B int, cm) + vario(2B int, cm/s)
                est_alt, vario = struct.unpack('<i h', payload[:6])
                result['altitude'] = est_alt / 100.0  # cm → mét
                result['vario'] = vario / 100.0        # cm/s → m/s

            elif cmd == MSP_STATUS and size >= 11:
                # Cấu trúc: cycle_time(2B) + i2c_errors(2B) + sensors(2B)
                #          + flight_mode_flags(4B) + config_profile(1B)
                cycle_time, i2c_err, sensors, flags, profile = struct.unpack(
                    '<H H H I B', payload[:11]
                )
                result['is_armed'] = bool(flags & (1 << 0))  # Bit 0 = ARMED
                result['flight_mode_flags'] = flags

            elif cmd == MSP_RAW_GPS and size >= 16:
                # ═══════════════════════════════════════
                # MSP_RAW_GPS (106) — Dữ liệu từ GPS BZ 251
                # ═══════════════════════════════════════
                # Cấu trúc: fix_type(1B) + num_sat(1B) + lat(4B int32, x10^7)
                #          + lon(4B int32, x10^7) + alt(2B int16, mét)
                #          + ground_speed(2B uint16, cm/s)
                #          + ground_course(2B uint16, độ x10)
                fix_type, num_sat, lat_raw, lon_raw, alt, speed, course = struct.unpack(
                    '<B B i i h H H', payload[:16]
                )
                result['gps_fix_type'] = fix_type       # 0=No fix, 1=2D, 2=3D
                result['gps_num_sat'] = num_sat         # Số vệ tinh
                result['latitude'] = lat_raw / GPS_COORD_SCALE   # Vĩ độ thập phân
                result['longitude'] = lon_raw / GPS_COORD_SCALE  # Kinh độ thập phân
                result['gps_altitude'] = alt             # Mét
                result['ground_speed'] = speed / 100.0   # cm/s → m/s
                result['ground_course'] = course / 10.0  # Độ (0-360)

                # HDOP nếu có byte thứ 16-17
                if size >= 18:
                    hdop = struct.unpack('<H', payload[16:18])[0]
                    result['gps_hdop'] = hdop / 100.0

            elif cmd == MSP_WP_GETINFO and size >= 3:
                # ═══════════════════════════════════════
                # MSP_WP_GETINFO (20) — Thông tin mission
                # ═══════════════════════════════════════
                # Cấu trúc: reserved(1B) + max_waypoints(1B) + is_valid(1B)
                reserved, max_wp, is_valid = struct.unpack('<B B B', payload[:3])
                result['wp_max'] = max_wp
                result['wp_is_valid'] = bool(is_valid)

            elif cmd == MSP_STATUS_EX and size >= 15:
                # ═══════════════════════════════════════
                # MSP_STATUS_EX (150) — Trạng thái mở rộng INAV
                # ═══════════════════════════════════════
                # Cấu trúc (phần quan trọng):
                #   cycle_time(2B) + i2c_errors(2B) + sensors_present(2B)
                #   + flight_mode_flags(4B) + config_profile(1B)
                #   + system_load(2B)
                # → sensors_present (offset 4-5) là bitfield sensor:
                #   Bit 0 = ACC, Bit 1 = BARO, Bit 2 = MAG (compass)
                #   Bit 3 = GPS, Bit 4 = RANGEFINDER, Bit 5 = OPFLOW
                #   Bit 6 = PITOT
                sensors_present = struct.unpack('<H', payload[4:6])[0]
                result['sensor_acc']         = bool(sensors_present & (1 << 0))
                result['sensor_baro']        = bool(sensors_present & (1 << 1))
                result['sensor_mag']         = bool(sensors_present & (1 << 2))
                result['sensor_gps']         = bool(sensors_present & (1 << 3))
                result['sensor_rangefinder'] = bool(sensors_present & (1 << 4))
                result['sensor_opflow']      = bool(sensors_present & (1 << 5))
                result['sensor_pitot']       = bool(sensors_present & (1 << 6))

                # flight_mode_flags tại offset 6-9
                flags = struct.unpack('<I', payload[6:10])[0]
                result['is_armed'] = bool(flags & (1 << 0))
                result['flight_mode_flags'] = flags

                # system_load tại offset 11-12
                if size >= 13:
                    sys_load = struct.unpack('<H', payload[11:13])[0]
                    result['system_load'] = sys_load  # % CPU load

            elif cmd == MSP_SONAR_ALTITUDE and size >= 4:
                # ═══════════════════════════════════════
                # MSP_SONAR_ALTITUDE (58) — Độ cao bề mặt từ LiDAR MTF-02
                # ═══════════════════════════════════════
                # Cấu trúc: surface_altitude(4B int32, cm)
                # Giá trị âm = ngoài tầm đo (out of range)
                # MTF-02 AIO: rangefinder range ≤ 2.5m (250cm)
                surface_alt_cm = struct.unpack('<i', payload[:4])[0]
                if surface_alt_cm >= 0:
                    result['surface_altitude'] = surface_alt_cm / 100.0  # cm → mét
                    result['surface_quality'] = 255  # MSP_SONAR không gửi quality riêng
                else:
                    result['surface_altitude'] = -1.0  # Đánh dấu ngoài tầm
                    result['surface_quality'] = 0

        except struct.error:
            # Firmware FC phiên bản khác có thể có cấu trúc byte khác
            pass

        return result
