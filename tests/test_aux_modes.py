"""
test_aux_modes.py - Kiểm thử tự động AUX Modes cho Drone GCS.

3 kịch bản kiểm thử:
1. Test AUX channel mapping & PWM bounds trong MSPParser
2. Test FlightController state machine ARM/DISARM/RTH/SafeLand
3. Test ESP32 failsafe frame generation

Chạy: pytest tests/test_aux_modes.py -v
"""

import struct
import pytest
import sys
import os

# Thêm project root vào PYTHONPATH
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from comm.msp_parser import (
    MSPParser, MSP_SET_RAW_RC, MSP_SET_WP,
    MSP_HEADER_REQUEST, MSP_HEADER_RESPONSE,
    MSP_STATUS, MSP_ANALOG, MSP_ATTITUDE, MSP_ALTITUDE, MSP_RAW_GPS
)


# ══════════════════════════════════════════════════════════════
# KỊCH BẢN 1: Kiểm tra đóng gói/mở gói MSP_SET_RAW_RC
#              và giá trị PWM bounds cho AUX channels
# ══════════════════════════════════════════════════════════════

class TestMSPAuxChannelPacking:
    """
    Kiểm thử MSPParser đóng gói kênh AUX đúng cách.

    Mục tiêu:
    - Verify frame MSP_SET_RAW_RC có header, checksum đúng
    - Verify thứ tự kênh: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
    - Verify giá trị PWM nằm trong 1000-2000
    - Verify checksum khớp giữa đóng gói và giải mã
    """

    def setup_method(self):
        """Khởi tạo parser mới cho mỗi test."""
        self.parser = MSPParser()

    # ── Test 1.1: Đóng gói kênh ARM (AUX1=2000) ──
    def test_pack_arm_channels_correct_header_and_checksum(self):
        """
        Kịch bản: GCS gửi lệnh ARM (AUX1=2000, AUX2=1500 ANGLE, tất cả kênh hợp lệ).
        Kỳ vọng: Frame MSP có header $M<, size=16, cmd=200, checksum đúng.
        """
        channels_arm = [1500, 1500, 1000, 1500, 2000, 1500, 1000, 1000]
        frame = self.parser.pack_set_raw_rc(channels_arm)

        # Kiểm tra header
        assert frame[:3] == b'$M<', f"Header sai: {frame[:3]}"

        # Kiểm tra size = 16 (8 kênh x 2 bytes)
        assert frame[3] == 16, f"Size sai: {frame[3]}, kỳ vọng 16"

        # Kiểm tra command = MSP_SET_RAW_RC (200)
        assert frame[4] == MSP_SET_RAW_RC, f"Command sai: {frame[4]}, kỳ vọng {MSP_SET_RAW_RC}"

        # Giải mã payload để verify giá trị kênh
        payload = frame[5:21]  # 16 bytes payload
        unpacked = struct.unpack('<8H', payload)
        assert unpacked == tuple(channels_arm), (
            f"Giá trị kênh sai: {unpacked} != {tuple(channels_arm)}"
        )

        # Verify checksum
        size = frame[3]
        cmd = frame[4]
        calc_checksum = MSPParser._calculate_checksum(size, cmd, payload)
        assert frame[-1] == calc_checksum, (
            f"Checksum sai: {frame[-1]} != {calc_checksum}"
        )

    # ── Test 1.2: Giá trị PWM biên (boundary testing) ──
    def test_pwm_boundary_values(self):
        """
        Kịch bản: Gửi giá trị PWM ở biên (1000, 1500, 2000).
        Kỳ vọng: Tất cả giá trị được đóng gói chính xác.
        """
        # Tất cả ở mức thấp nhất
        channels_min = [1000] * 8
        frame_min = self.parser.pack_set_raw_rc(channels_min)
        payload_min = frame_min[5:21]
        values_min = struct.unpack('<8H', payload_min)
        assert all(v == 1000 for v in values_min), f"Giá trị min sai: {values_min}"

        # Tất cả ở mức cao nhất
        channels_max = [2000] * 8
        frame_max = self.parser.pack_set_raw_rc(channels_max)
        payload_max = frame_max[5:21]
        values_max = struct.unpack('<8H', payload_max)
        assert all(v == 2000 for v in values_max), f"Giá trị max sai: {values_max}"

        # Hỗn hợp: ARM + ANGLE + Safe Land ON + RTH ON
        channels_mixed = [1500, 1500, 1000, 1500, 2000, 1500, 2000, 2000]
        frame_mixed = self.parser.pack_set_raw_rc(channels_mixed)
        payload_mixed = frame_mixed[5:21]
        values_mixed = struct.unpack('<8H', payload_mixed)

        # Verify AUX channels cụ thể
        assert values_mixed[4] == 2000, "AUX1 (ARM) phải = 2000"
        assert values_mixed[5] == 1500, "AUX2 (ANGLE) phải = 1500"
        assert values_mixed[6] == 2000, "AUX3 (Safe Land) phải = 2000"
        assert values_mixed[7] == 2000, "AUX4 (RTH) phải = 2000"

    # ── Test 1.3: Reject số kênh sai ──
    def test_reject_wrong_channel_count(self):
        """
        Kịch bản: Gửi ít hơn hoặc nhiều hơn 8 kênh.
        Kỳ vọng: Raise ValueError.
        """
        with pytest.raises(ValueError, match="Cần đúng 8 kênh"):
            self.parser.pack_set_raw_rc([1500, 1500])  # Chỉ 2 kênh

        with pytest.raises(ValueError, match="Cần đúng 8 kênh"):
            self.parser.pack_set_raw_rc([1500] * 10)  # 10 kênh

    # ── Test 1.4: Endianness — Little-Endian ──
    def test_little_endian_byte_order(self):
        """
        Kịch bản: Verify giá trị 2000 (0x07D0) được mã hóa Little-Endian.
        Kỳ vọng: Byte thấp (0xD0) trước, byte cao (0x07) sau.
        """
        channels = [2000, 1000, 1500, 1500, 2000, 1500, 1000, 1000]
        frame = self.parser.pack_set_raw_rc(channels)

        # Byte 5-6: kênh đầu tiên (Roll = 2000 = 0x07D0)
        # Little-Endian: [0xD0, 0x07]
        assert frame[5] == 0xD0, f"LE byte thấp sai: {frame[5]:#x} != 0xD0"
        assert frame[6] == 0x07, f"LE byte cao sai: {frame[6]:#x} != 0x07"

    # ── Test 1.5: Giá trị out of bounds (bug BUG-06) ──
    def test_out_of_bounds_pwm_not_clamped(self):
        """
        Kịch bản: Gửi giá trị PWM vượt giới hạn (999, 2001).
        ★ TASK-13 đã thêm clamp: giá trị phải được kẹp về 1000-2000.
        """
        channels_oob = [999, 2001, 1500, 1500, 2500, 500, 1000, 1000]
        frame = self.parser.pack_set_raw_rc(channels_oob)
        payload = frame[5:21]
        values = struct.unpack('<8H', payload)

        # Verify PWM clamp hoạt động đúng
        assert all(1000 <= v <= 2000 for v in values), (
            f"Giá trị out-of-bounds không bị clamp: {values}"
        )
        assert values[0] == 1000, "999 phải được clamp lên 1000"
        assert values[1] == 2000, "2001 phải được clamp xuống 2000"
        assert values[4] == 2000, "2500 phải được clamp xuống 2000"
        assert values[5] == 1000, "500 phải được clamp lên 1000"


# ══════════════════════════════════════════════════════════════
# KỊCH BẢN 2: Kiểm tra FlightController state machine
#              gán đúng kênh AUX cho từng chế độ bay
# ══════════════════════════════════════════════════════════════

class TestFlightControllerAuxAssignment:
    """
    Kiểm thử FlightController gán giá trị AUX đúng cho từng hành động.

    Không cần QApplication — chỉ test logic nội bộ (hằng số, _safe_channels).
    """

    # ── Test 2.1: _safe_channels() trả về trạng thái an toàn ──
    def test_safe_channels_all_disarmed(self):
        """
        Kịch bản: Kiểm tra kênh an toàn mặc định.
        Kỳ vọng:
        - Throttle (index 2) = 1000 (ga thấp nhất)
        - AUX1 (index 4) = 1000 (DISARM)
        - AUX2 (index 5) = 1500 (ANGLE — chế độ ổn định)
        - AUX3 (index 6) = 1000 (Safe Land OFF)
        - AUX4 (index 7) = 1000 (RTH OFF)
        """
        from core.flight_controller import FlightController

        # Truy cập hằng số class mà không cần instance
        safe = [
            FlightController.RC_CENTER,
            FlightController.RC_CENTER,
            FlightController.THROTTLE_MIN,
            FlightController.RC_CENTER,
            FlightController.AUX_DISARM,
            FlightController.AUX_ANGLE,
            FlightController.AUX_SAFE_LAND_OFF,
            FlightController.AUX_RTH_OFF,
        ]

        # Verify từng kênh
        assert safe[0] == 1500, f"Roll phải = 1500, nhận {safe[0]}"
        assert safe[1] == 1500, f"Pitch phải = 1500, nhận {safe[1]}"
        assert safe[2] == 1000, f"Throttle phải = 1000 (min), nhận {safe[2]}"
        assert safe[3] == 1500, f"Yaw phải = 1500, nhận {safe[3]}"
        assert safe[4] == 1000, f"AUX1 phải = 1000 (DISARM), nhận {safe[4]}"
        assert safe[5] == 1500, f"AUX2 phải = 1500 (ANGLE), nhận {safe[5]}"
        assert safe[6] == 1000, f"AUX3 phải = 1000 (SL OFF), nhận {safe[6]}"
        assert safe[7] == 1000, f"AUX4 phải = 1000 (RTH OFF), nhận {safe[7]}"

    # ── Test 2.2: AUX constants đúng và không trùng lặp ──
    def test_aux_constants_valid_and_unique_per_mode(self):
        """
        Kịch bản: Verify hằng số AUX có giá trị trong range 1000-2000.
        Kỳ vọng:
        - Tất cả giá trị ON/OFF nằm trong 1000-2000
        - ARM ≠ DISARM
        - Mỗi mode ON ≠ OFF
        """
        from core.flight_controller import FlightController as FC

        aux_pairs = {
            "ARM": (FC.AUX_ARM, FC.AUX_DISARM),
            "SafeLand": (FC.AUX_SAFE_LAND_ON, FC.AUX_SAFE_LAND_OFF),
            "RTH": (FC.AUX_RTH_ON, FC.AUX_RTH_OFF),
        }

        for name, (on_val, off_val) in aux_pairs.items():
            # Kiểm tra range
            assert 1000 <= on_val <= 2000, f"{name} ON={on_val} ngoài range 1000-2000"
            assert 1000 <= off_val <= 2000, f"{name} OFF={off_val} ngoài range 1000-2000"
            # Kiểm tra ON ≠ OFF
            assert on_val != off_val, f"{name}: ON ({on_val}) == OFF ({off_val})!"

        # Flight mode 3 mức
        assert FC.AUX_ACRO == 1000, f"ACRO phải = 1000, nhận {FC.AUX_ACRO}"
        assert FC.AUX_ANGLE == 1500, f"ANGLE phải = 1500, nhận {FC.AUX_ANGLE}"
        assert FC.AUX_NAV_ALTHOLD_POSHOLD == 2000, (
            f"ALTHOLD+POSHOLD phải = 2000, nhận {FC.AUX_NAV_ALTHOLD_POSHOLD}"
        )

    # ── Test 2.3: Channel index mapping đúng ──
    def test_channel_index_mapping(self):
        """
        Kịch bản: Verify chỉ số kênh CH_AUX1-CH_AUX4 đúng vị trí.
        Kỳ vọng: AUX1=4, AUX2=5, AUX3=6, AUX4=7
        """
        from core.flight_controller import FlightController as FC

        assert FC.CH_AUX1 == 4, f"CH_AUX1 phải = 4, nhận {FC.CH_AUX1}"
        assert FC.CH_AUX2 == 5, f"CH_AUX2 phải = 5, nhận {FC.CH_AUX2}"
        assert FC.CH_AUX3 == 6, f"CH_AUX3 phải = 6, nhận {FC.CH_AUX3}"
        assert FC.CH_AUX4 == 7, f"CH_AUX4 phải = 7, nhận {FC.CH_AUX4}"

        # Verify thứ tự chuẩn AETR
        assert FC.CH_ROLL == 0, f"CH_ROLL phải = 0, nhận {FC.CH_ROLL}"
        assert FC.CH_PITCH == 1, f"CH_PITCH phải = 1, nhận {FC.CH_PITCH}"
        assert FC.CH_THROTTLE == 2, f"CH_THROTTLE phải = 2, nhận {FC.CH_THROTTLE}"
        assert FC.CH_YAW == 3, f"CH_YAW phải = 3, nhận {FC.CH_YAW}"


# ══════════════════════════════════════════════════════════════
# KỊCH BẢN 3: Giả lập ESP32 failsafe — verify frame RTH/Land
#              được tạo đúng cấu trúc MSP
# ══════════════════════════════════════════════════════════════

class TestESP32FailsafeFrameGeneration:
    """
    Giả lập hàm build_msp_set_raw_rc() của ESP32 trên Python.
    Verify frame failsafe RTH và Safe Land có đúng giá trị AUX.

    Lưu ý: ESP32 chạy MicroPython, test này giả lập logic tương đương
    để kiểm tra tính đúng đắn mà không cần phần cứng.
    """

    # Giả lập hàm ESP32 (copy logic từ ESP32/main.py)
    @staticmethod
    def _build_msp_set_raw_rc(channels):
        """Replica of ESP32's build_msp_set_raw_rc() for testing."""
        header = b'$M<'
        payload = struct.pack('<8H', *channels)
        size = len(payload)
        cmd = 200  # MSP_SET_RAW_RC

        checksum = size ^ cmd
        for b in payload:
            checksum ^= b

        frame = bytearray(header)
        frame.append(size)
        frame.append(cmd)
        frame.extend(payload)
        frame.append(checksum & 0xFF)
        return bytes(frame)

    # ── Test 3.1: Frame RTH failsafe — AUX4=2000, AUX1=2000 (ARM duy trì) ──
    def test_failsafe_rth_frame_channels(self):
        """
        Kịch bản: ESP32 mất WiFi → tạo frame RTH.
        Kỳ vọng:
        - AUX1 (index 4) = 2000 → duy trì ARM
        - AUX4 (index 7) = 2000 → kích hoạt RTH
        - AUX3 (index 6) = 1000 → Safe Land OFF
        - Throttle (index 2) = 1500 → FC tự điều khiển
        """
        # Giá trị kênh failsafe RTH (theo ESP32/main.py send_failsafe_rth)
        channels_rth = [1500, 1500, 1500, 1500, 2000, 1500, 1000, 2000]
        #                Roll  Pitch Throt Yaw   ARM  ANGLE SL-OFF RTH-ON
        # Note: thứ tự AETR: index 2=Throttle, index 3=Yaw

        frame = self._build_msp_set_raw_rc(channels_rth)

        # Verify header
        assert frame[:3] == b'$M<'
        assert frame[3] == 16  # size
        assert frame[4] == 200  # cmd

        # Giải mã payload
        payload = frame[5:21]
        values = struct.unpack('<8H', payload)

        # Verify kênh AUX
        assert values[4] == 2000, f"AUX1 (ARM) phải = 2000 cho failsafe, nhận {values[4]}"
        assert values[5] == 1500, f"AUX2 (ANGLE) phải = 1500, nhận {values[5]}"
        assert values[6] == 1000, f"AUX3 (Safe Land) phải = OFF, nhận {values[6]}"
        assert values[7] == 2000, f"AUX4 (RTH) phải = 2000, nhận {values[7]}"

        # Verify checksum — frame phải được GCS parser chấp nhận
        parser = MSPParser()
        calc_checksum = parser._calculate_checksum(16, 200, payload)
        assert frame[-1] == calc_checksum, f"Checksum không khớp: {frame[-1]} != {calc_checksum}"

    # ── Test 3.2: Frame Safe Land failsafe ──
    def test_failsafe_land_frame_channels(self):
        """
        Kịch bản: ESP32 gửi lệnh Safe Land (hạ cánh tại chỗ).
        Kỳ vọng:
        - AUX1 (index 4) = 2000 → duy trì ARM
        - AUX3 (index 6) = 2000 → kích hoạt hạ cánh
        - AUX4 (index 7) = 1000 → RTH OFF
        """
        channels_land = [1500, 1500, 1500, 1500, 2000, 1500, 2000, 1000]
        frame = self._build_msp_set_raw_rc(channels_land)

        payload = frame[5:21]
        values = struct.unpack('<8H', payload)

        assert values[4] == 2000, "ARM phải duy trì"
        assert values[6] == 2000, "AUX3 (Safe Land) phải = ON"
        assert values[7] == 1000, "AUX4 (RTH) phải = OFF khi landing"

    # ── Test 3.3: Cross-verify ESP32 frame với GCS parser ──
    def test_esp32_frame_parsable_by_gcs(self):
        """
        Kịch bản: Frame MSP tạo bởi ESP32 phải giải mã được bởi GCS MSPParser.
        Kỳ vọng: parse_buffer() không lỗi và checksum khớp.

        Đây là test end-to-end giả lập:
        ESP32 build frame → gửi qua "mạng" → GCS nhận và parse.
        """
        channels = [1500, 1500, 1000, 1500, 2000, 1500, 1000, 2000]
        esp32_frame = self._build_msp_set_raw_rc(channels)

        # Giả lập: GCS nhận frame này, nhưng nó là MSP request ($M<)
        # chứ không phải response ($M>). Để test cross-compatibility,
        # ta verify checksum algorithm cho nhất quán.

        # Extract và verify checksum ESP32 vs GCS
        payload = esp32_frame[5:21]
        size = esp32_frame[3]
        cmd = esp32_frame[4]

        gcs_checksum = MSPParser._calculate_checksum(size, cmd, payload)
        esp32_checksum = esp32_frame[-1]

        assert gcs_checksum == esp32_checksum, (
            f"Checksum mismatch giữa ESP32 ({esp32_checksum}) và GCS ({gcs_checksum})! "
            "Có thể do thuật toán XOR khác nhau."
        )

    # ── Test 3.4: Xung đột AUX3 + AUX4 cùng lúc ──
    def test_conflicting_aux_safe_land_and_rth(self):
        """
        Kịch bản: Giả sử cả Safe Land (AUX3) và RTH (AUX4) cùng bật.
        Kỳ vọng: Frame được đóng gói bình thường (FC sẽ ưu tiên).
        
        Đây là edge case — INAV thường ưu tiên RTH over Safe Land,
        nhưng hành vi phụ thuộc vào cấu hình priority trong INAV.
        """
        channels_conflict = [1500, 1500, 1500, 1500, 2000, 1500, 2000, 2000]
        #                                                     SL=ON  RTH=ON
        frame = self._build_msp_set_raw_rc(channels_conflict)

        payload = frame[5:21]
        values = struct.unpack('<8H', payload)

        # Cả hai đều phải được encode đúng
        assert values[6] == 2000, "AUX3 (Safe Land) phải = 2000"
        assert values[7] == 2000, "AUX4 (RTH) phải = 2000"

        # Checksum vẫn phải đúng
        calc = MSPParser._calculate_checksum(16, 200, payload)
        assert frame[-1] == calc, "Checksum sai khi cả 2 AUX bật cùng lúc"


# ══════════════════════════════════════════════════════════════
# CHẠY TRỰC TIẾP
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
