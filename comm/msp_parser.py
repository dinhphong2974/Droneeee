"""
msp_parser.py - Đóng gói và giải mã giao thức MSP (MultiWii Serial Protocol).

Module này xử lý toàn bộ logic bóc tách gói tin MSP:
- Đóng gói lệnh request ($M<) gửi tới Flight Controller
- Giải mã response ($M>) nhận về từ Flight Controller
- Quản lý buffer chống đứt gói do lag mạng Wifi
- Tính và xác thực checksum chống nhiễu

Thông số phần cứng liên quan:
- Pin: Lipo 6S (19.8V rỗng → 25.2V đầy)
- Động cơ: 1960kv (PWM range: 1000-2000μs)
"""

import struct

# ── Mã lệnh MSP (MultiWii Serial Protocol) ──
MSP_ATTITUDE = 108  # Lệnh lấy góc nghiêng (Roll, Pitch, Yaw)
MSP_ANALOG = 110    # Lệnh lấy thông số pin (Voltage, Current)

# ── Header giao thức ──
MSP_HEADER_REQUEST = b'$M<'   # PC gửi đi → FC
MSP_HEADER_RESPONSE = b'$M>'  # FC trả về → PC

# ── Cấu hình pin Lipo 6S ──
LIPO_6S_VOLTAGE_DIVIDER = 10.0  # FC gửi điện áp nhân 10 (VD: 245 → 24.5V)
CURRENT_DIVIDER = 100.0          # FC gửi dòng điện nhân 100

# ── Kích thước frame tối thiểu ──
MIN_FRAME_SIZE = 6  # Header(3) + Size(1) + Cmd(1) + Checksum(1)


class MSPParser:
    """
    Bộ giải mã giao thức MSP cho giao tiếp với Flight Controller (INAV/Betaflight).

    Xử lý hai chiều:
    - pack_msg(): PC → ESP32 → FC (đóng gói lệnh request)
    - parse_buffer(): FC → ESP32 → PC (giải mã response)
    """

    def __init__(self):
        """Khởi tạo parser với buffer rỗng để gom gói tin bị đứt khúc."""
        self.buffer = bytearray()

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
        - MSP_ANALOG (110): Điện áp pin 6S, dòng điện
        - MSP_ATTITUDE (108): Góc Roll, Pitch, Yaw
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

        except struct.error:
            # Firmware FC phiên bản khác có thể có cấu trúc byte khác
            pass

        return result
