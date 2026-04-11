import sys
import os
import struct
import pytest

# Đảm bảo import được từ thư mục gốc
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.flight_controller import FlightController
from comm.msp_parser import (
    MSPParser, MSP_SONAR_ALTITUDE, MSP_ANALOG, MSP_ALTITUDE,
    MAX_BUFFER_SIZE
)

class MockDroneState:
    """Mock DroneState cho unit test — tất cả các field cần thiết."""
    def __init__(self):
        self.is_armed = False
        self.altitude = 0.0
        self.gps_fix_type = 0
        self.gps_num_sat = 0
        self.latitude = 0.0
        self.longitude = 0.0
        # ── MTF-02 LiDAR fields ──
        self.surface_altitude = -1.0
        self.surface_quality = 0
        self.has_valid_surface = False


# ══════════════════════════════════════════════
# TESTS CŨ (giữ nguyên)
# ══════════════════════════════════════════════

def test_aux_modes_mapping():
    """1. Kiểm thử định nghĩa kênh AUX_MODES không vượt quá giới hạn PWM an toàn"""
    fc = FlightController(MockDroneState())
    
    # Định nghĩa cấu hình PWM tiêu chuẩn
    assert fc.AUX_DISARM == 1000
    assert fc.AUX_ARM == 2000
    assert fc.AUX_ACRO == 1000
    assert fc.AUX_ANGLE == 1500
    assert fc.AUX_NAV_ALTHOLD_POSHOLD == 2000
    
    safe_channels = fc._safe_channels()
    # AUX1 (ARM) = 1000, AUX2 (Flight Mode) = 1500, AUX3 (Land) = 1000, AUX4 (RTH) = 1000
    assert safe_channels[fc.CH_AUX1] == 1000
    assert safe_channels[fc.CH_AUX2] == 1500
    assert safe_channels[fc.CH_AUX3] == 1000
    assert safe_channels[fc.CH_AUX4] == 1000

def test_arming_safety_sequence():
    """2. Kiểm thử logic ARMING_SAFETY_SEQUENCE (phải bắt đầu bằng luồng gửi DISARM)"""
    fc = FlightController(MockDroneState())

    # Mock worker có send_command để nhận MSP frames
    class MockWorker:
        def __init__(self):
            self.commands = []
        def send_command(self, data):
            self.commands.append(data)

    fc._worker = MockWorker()
    
    # Bắt đầu trình tự arm
    fc.arm()
    
    # INAV yêu cầu kênh gạt về DISARM trước tiên trong vòng WAIT_RC_LINK
    assert fc.state == "WAIT_RC_LINK_ARM_ONLY"
    assert fc._channels[fc.CH_AUX1] == 1000

def test_msp_raw_rc_parser_byte_mapping():
    """3. Kiểm thử MSP_RAW_RC_PARSER_BYTE_MAPPING có đúng chuẩn 16 bytes payload"""
    parser = MSPParser()
    channels = [1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000]
    result = parser.pack_set_raw_rc(channels)
    
    # Kiểm tra Header
    assert result[0:3] == b'$M<'
    # Kiểm tra độ dài Payload (Size) = 16
    assert result[3] == 16
    # Kiểm tra Mã Lệnh (CMD) = MSP_SET_RAW_RC = 200
    assert result[4] == 200
    # Gói tin cuối dùng tính checksum (độ dài = Header(3) + Size(1) + Cmd(1) + Payload(16) + Checksum(1) = 22)
    assert len(result) == 22


# ══════════════════════════════════════════════
# TESTS MỚI — MSP_SONAR_ALTITUDE (MTF-02 LiDAR)
# ══════════════════════════════════════════════

def _build_msp_response(cmd: int, payload: bytes) -> bytes:
    """Helper: Xây dựng frame MSP response ($M>) hoàn chỉnh cho test."""
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b
    frame = bytearray(b'$M>')
    frame.append(size)
    frame.append(cmd)
    frame.extend(payload)
    frame.append(checksum & 0xFF)
    return bytes(frame)


def test_msp_sonar_altitude_valid():
    """4. Kiểm thử giải mã MSP_SONAR_ALTITUDE — giá trị hợp lệ (150cm = 1.5m)"""
    parser = MSPParser()
    payload = struct.pack('<i', 150)  # 150cm = 1.5m
    frame = _build_msp_response(MSP_SONAR_ALTITUDE, payload)
    
    result = parser.parse_buffer(frame)
    
    assert 'surface_altitude' in result
    assert abs(result['surface_altitude'] - 1.5) < 0.01
    assert result['surface_quality'] == 255


def test_msp_sonar_altitude_out_of_range():
    """5. Kiểm thử giải mã MSP_SONAR_ALTITUDE — ngoài tầm đo (giá trị âm)"""
    parser = MSPParser()
    payload = struct.pack('<i', -1)  # Ngoài tầm
    frame = _build_msp_response(MSP_SONAR_ALTITUDE, payload)
    
    result = parser.parse_buffer(frame)
    
    assert 'surface_altitude' in result
    assert result['surface_altitude'] == -1.0
    assert result['surface_quality'] == 0


def test_msp_sonar_altitude_zero():
    """6. Kiểm thử giải mã MSP_SONAR_ALTITUDE — chạm đất (0cm)"""
    parser = MSPParser()
    payload = struct.pack('<i', 0)  # 0cm — trên mặt đất
    frame = _build_msp_response(MSP_SONAR_ALTITUDE, payload)
    
    result = parser.parse_buffer(frame)
    
    assert result['surface_altitude'] == 0.0
    assert result['surface_quality'] == 255


def test_msp_sonar_altitude_checksum_error():
    """7. Kiểm thử MSP_SONAR_ALTITUDE — checksum sai bị reject"""
    parser = MSPParser()
    payload = struct.pack('<i', 200)
    frame = bytearray(_build_msp_response(MSP_SONAR_ALTITUDE, payload))
    frame[-1] ^= 0xFF  # Đảo checksum
    
    result = parser.parse_buffer(bytes(frame))
    
    assert 'surface_altitude' not in result  # Gói tin bị reject


# ══════════════════════════════════════════════
# TESTS — SENSOR FUSION (_effective_altitude)
# ══════════════════════════════════════════════

def test_effective_altitude_uses_lidar_when_in_range():
    """8. Sensor Fusion: dùng LiDAR khi ≤2.5m và dữ liệu hợp lệ"""
    state = MockDroneState()
    state.altitude = 2.0         # Barometer: 2.0m
    state.surface_altitude = 1.8  # LiDAR: 1.8m (chính xác hơn)
    state.has_valid_surface = True

    fc = FlightController(state)
    
    # Phải chọn LiDAR vì trong tầm 2.5m
    assert abs(fc._effective_altitude - 1.8) < 0.01


def test_effective_altitude_uses_baro_when_out_of_range():
    """9. Sensor Fusion: dùng Barometer khi LiDAR ngoài tầm"""
    state = MockDroneState()
    state.altitude = 5.0           # Barometer: 5.0m
    state.surface_altitude = -1.0  # LiDAR: ngoài tầm
    state.has_valid_surface = False

    fc = FlightController(state)
    
    # Phải chọn Barometer vì LiDAR không có dữ liệu
    assert abs(fc._effective_altitude - 5.0) < 0.01


def test_effective_altitude_uses_baro_when_lidar_exceeds_max():
    """10. Sensor Fusion: dùng Barometer khi LiDAR vượt tầm tối đa"""
    state = MockDroneState()
    state.altitude = 3.0          # Barometer: 3.0m
    state.surface_altitude = 3.0  # LiDAR đọc 3.0m (vượt LIDAR_MAX_RANGE_M=2.5m)
    state.has_valid_surface = True

    fc = FlightController(state)
    
    # LiDAR đọc 3.0m > 2.5m max range → phải dùng Barometer
    assert abs(fc._effective_altitude - 3.0) < 0.01


# ══════════════════════════════════════════════
# TESTS — BUFFER OVERFLOW PROTECTION
# ══════════════════════════════════════════════

def test_buffer_overflow_protection():
    """11. Kiểm thử buffer không vượt quá MAX_BUFFER_SIZE"""
    parser = MSPParser()
    
    # Gửi 8KB dữ liệu rác (vượt MAX_BUFFER_SIZE = 4096)
    garbage = bytes([0xAA] * 8192)
    parser.parse_buffer(garbage)
    
    assert len(parser.buffer) <= MAX_BUFFER_SIZE


def test_buffer_recovers_after_overflow():
    """12. Buffer vẫn parse được sau khi bị cắt do overflow"""
    parser = MSPParser()
    
    # Bơm rác để trigger overflow
    garbage = bytes([0xBB] * 6000)
    parser.parse_buffer(garbage)
    
    # Sau đó gửi frame hợp lệ
    payload = struct.pack('<i', 100)  # 100cm = 1.0m
    frame = _build_msp_response(MSP_SONAR_ALTITUDE, payload)
    result = parser.parse_buffer(frame)
    
    assert 'surface_altitude' in result
    assert abs(result['surface_altitude'] - 1.0) < 0.01


# ══════════════════════════════════════════════
# TESTS — LiDAR CONSTANTS
# ══════════════════════════════════════════════

def test_lidar_constants():
    """13. Kiểm thử hằng số LiDAR hợp lệ"""
    fc = FlightController(MockDroneState())
    
    assert fc.LIDAR_MAX_RANGE_M == 2.5
    assert fc.LIDAR_SOFT_LAND_THRESHOLD == 1.0
    assert fc.LIDAR_GROUND_PROXIMITY == 0.15
    assert fc.LIDAR_GROUND_PROXIMITY < fc.LIDAR_SOFT_LAND_THRESHOLD
    assert fc.LIDAR_SOFT_LAND_THRESHOLD < fc.LIDAR_MAX_RANGE_M


# ══════════════════════════════════════════════
# TESTS — EMERGENCY COMMANDS (force_disarm / force_safe_land)
# ══════════════════════════════════════════════

class MockWorker:
    """Mock WifiWorker cho emergency command tests."""
    def __init__(self):
        self.commands = []
        self.emergency_commands = []

    def send_command(self, data):
        self.commands.append(data)

    def send_emergency_command(self, data):
        self.emergency_commands.append(data)


def test_force_disarm_stops_state_machine():
    """14. force_disarm() phải stop timer VÀ set IDLE ngay lập tức"""
    fc = FlightController(MockDroneState())
    fc._worker = MockWorker()

    # Giả lập đang takeoff
    fc._state = "THROTTLE_RAMP"
    fc._timer.start(fc.TICK_INTERVAL_MS)

    fc.force_disarm()

    assert fc._state == "IDLE"
    assert not fc._timer.isActive()


def test_force_disarm_nav_off_sequence():
    """15. force_disarm() phải gửi sequence: NAV OFF → DISARM qua emergency queue"""
    fc = FlightController(MockDroneState())
    worker = MockWorker()
    fc._worker = worker

    fc._state = "THROTTLE_RAMP"

    fc.force_disarm()

    # Phải gửi ít nhất 3 emergency frames (1 NAV off + 2 DISARM)
    assert len(worker.emergency_commands) >= 3

    # emergency_commands[i] = b'EM:' (3 bytes) + MSP frame ($M< + size + cmd + payload + cs)
    # Layout: [0:3]=EM: [3:6]=$M< [6]=size [7]=cmd [8:24]=payload(16B) [24]=checksum
    # ⇒ payload offset: 3 (prefix) + 5 (header+size+cmd) = 8
    # Frame đầu tiên: AUX2=ANGLE (1500), AUX1=ARM (2000) — tắt NAV, chưa disarm
    first_frame = worker.emergency_commands[0]
    payload = first_frame[8:24]     # 3 bytes EM: + 5 bytes MSP header = offset 8
    channels = list(struct.unpack('<8H', payload))
    assert channels[4] == fc.AUX_ARM       # Vẫn ARM
    assert channels[5] == fc.AUX_ANGLE     # Tắt NAV modes

    # Frame thứ 2+: AUX1=DISARM (1000)
    second_frame = worker.emergency_commands[1]
    payload2 = second_frame[8:24]   # Cùng offset
    channels2 = list(struct.unpack('<8H', payload2))
    assert channels2[4] == fc.AUX_DISARM   # DISARM


def test_force_safe_land_state_transition():
    """16. force_safe_land() phải transition sang SAFE_LANDING"""
    fc = FlightController(MockDroneState())
    worker = MockWorker()
    fc._worker = worker

    # Giả lập đang takeoff
    fc._state = "THROTTLE_RAMP"

    fc.force_safe_land()

    assert fc._state == "SAFE_LANDING"

    # Phải gửi emergency frames với AUX3=SAFE_LAND_ON
    assert len(worker.emergency_commands) >= 2

    # emergency_commands[i] = b'EM:' (3 bytes prefix) + MSP frame
    # payload offset = 3 (EM:) + 5 (MSP header) = 8
    frame = worker.emergency_commands[0]
    payload = frame[8:24]            # offset 8 sau prefix EM:
    channels = list(struct.unpack('<8H', payload))
    assert channels[6] == fc.AUX_SAFE_LAND_ON   # Safe Land ON
    assert channels[5] == fc.AUX_ANGLE           # Tắt NAV modes

    fc._timer.stop()  # Cleanup


def test_safe_land_stops_takeoff_timer():
    """17. safe_land() thường phải stop timer trước khi activate"""
    fc = FlightController(MockDroneState())
    worker = MockWorker()
    fc._worker = worker

    # Giả lập đang takeoff với timer chạy
    fc._state = "THROTTLE_RAMP"
    fc._timer.start(fc.TICK_INTERVAL_MS)

    fc.safe_land()

    # Timer phải restart ở state mới (SAFE_LANDING)
    assert fc._state == "SAFE_LANDING"

    fc._timer.stop()  # Cleanup


def test_disarm_sets_idle_before_stop():
    """18. disarm() phải set state IDLE trước khi stop timer (race condition guard)"""
    fc = FlightController(MockDroneState())
    fc._worker = MockWorker()

    fc._state = "ARMING"

    fc.disarm()

    assert fc._state == "IDLE"
    assert not fc._timer.isActive()


def test_emergency_queue_cleared_on_force_disarm():
    """19. force_disarm() qua send_emergency_command() xóa normal queue"""
    from comm.wifi_worker import WifiWorker
    worker = WifiWorker(is_mock=True)

    # Đặt vài lệnh thường vào queue
    worker.send_command(b'normal_cmd_1')
    worker.send_command(b'normal_cmd_2')
    worker.send_command(b'normal_cmd_3')

    # Gửi emergency command
    worker.send_emergency_command(b'EMERGENCY')

    # Normal queue phải bị xóa sạch
    assert worker._command_queue.empty()

    # Emergency queue phải có 1 lệnh
    assert not worker._emergency_queue.empty()
    cmd = worker._emergency_queue.get_nowait()
    assert cmd == b'EMERGENCY'

