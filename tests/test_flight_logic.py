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
        # ── v3: Sensor health fields ──
        self.sensor_opflow = False
        self.sensor_rangefinder = False
        # ── v3: GPS history ──
        self._gps_history = []

    def get_stable_gps(self):
        """Mock get_stable_gps — trả về trung bình hoặc None."""
        if len(self._gps_history) < 10:
            return None
        avg_lat = sum(p[0] for p in self._gps_history) / len(self._gps_history)
        avg_lon = sum(p[1] for p in self._gps_history) / len(self._gps_history)
        return (avg_lat, avg_lon)

    def record_gps_history(self):
        """Mock record_gps_history."""
        if self.latitude != 0.0:
            self._gps_history.append((self.latitude, self.longitude))


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
    """8. Sensor Fusion: dùng LiDAR khi ≤1m (LIDAR_TRUST_RANGE_M) và dữ liệu hợp lệ"""
    state = MockDroneState()
    state.altitude = 2.0         # Barometer: 2.0m
    state.surface_altitude = 0.8  # LiDAR: 0.8m (trong tầm tin cậy ≤1m)
    state.has_valid_surface = True

    fc = FlightController(state)
    
    # Phải chọn LiDAR vì trong tầm 1.0m
    assert abs(fc._effective_altitude - 0.8) < 0.01


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
    """10. Sensor Fusion: dùng Barometer khi LiDAR vượt tầm tin cậy (>1m)"""
    state = MockDroneState()
    state.altitude = 3.0          # Barometer: 3.0m
    state.surface_altitude = 1.5  # LiDAR đọc 1.5m (vượt LIDAR_TRUST_RANGE_M=1.0m)
    state.has_valid_surface = True

    fc = FlightController(state)
    
    # LiDAR đọc 1.5m > 1.0m trust range → phải dùng Barometer
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
    assert fc.LIDAR_TRUST_RANGE_M == 1.0
    assert fc.LIDAR_SOFT_LAND_THRESHOLD == 1.0
    assert fc.LIDAR_GROUND_PROXIMITY == 0.15
    assert fc.LIDAR_GROUND_PROXIMITY < fc.LIDAR_SOFT_LAND_THRESHOLD
    assert fc.LIDAR_TRUST_RANGE_M <= fc.LIDAR_MAX_RANGE_M


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
    """14. force_disarm() phải chuyển sang FORCE_DISARMING và khởi động timer repeated-send.

    Behavior mới (FIX): Không còn fire-and-forget. force_disarm() chuyển sang
    state FORCE_DISARMING → timer 10Hz tiếp tục gửi DISARM qua emergency queue
    cho đến khi FC xác nhận is_armed=False hoặc timeout 5s.
    """
    fc = FlightController(MockDroneState())
    fc._worker = MockWorker()

    # Giả lập đang takeoff
    fc._state = "NAV_CLIMB"
    fc._timer.start(fc.TICK_INTERVAL_MS)

    fc.force_disarm()

    # Behavior mới: state phải là FORCE_DISARMING (không phải IDLE ngay lập tức)
    # Timer cũng sẽ chạy repeated-send, nhưng không kiểm tra isActive() vì
    # QTimer cần Qt event loop đang chạy trong môi trường test
    assert fc._state == "FORCE_DISARMING"

    fc._timer.stop()  # Cleanup


def test_force_disarm_nav_off_sequence():
    """15. force_disarm() phải gửi sequence: NAV OFF → DISARM qua emergency queue"""
    fc = FlightController(MockDroneState())
    worker = MockWorker()
    fc._worker = worker

    fc._state = "NAV_CLIMB"

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
    fc._state = "NAV_CLIMB"

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
    """17. safe_land() phải chuyển qua NAV_OFF_BEFORE_SAFE_LAND trước SAFE_LANDING"""
    fc = FlightController(MockDroneState())
    worker = MockWorker()
    fc._worker = worker

    # Giả lập đang takeoff với timer chạy
    fc._state = "NAV_CLIMB"
    fc._timer.start(fc.TICK_INTERVAL_MS)

    fc.safe_land()

    # State phải là NAV_OFF_BEFORE_SAFE_LAND (chờ 300ms tắt NAV)
    assert fc._state == "NAV_OFF_BEFORE_SAFE_LAND"

    # Simulate 300ms trôi qua → tick chuyển sang SAFE_LANDING
    import time as _time
    fc._safe_land_start_time = _time.time() - 0.5  # Quá 300ms
    fc._tick()
    assert fc._state == "SAFE_LANDING"

    fc._timer.stop()  # Cleanup


def test_disarm_sets_disarming_state():
    """18. disarm() phải chuyển qua NAV_OFF_BEFORE_DISARM trước DISARMING.

    Behavior mới (FIX v2): disarm() chuyển sang NAV_OFF_BEFORE_DISARM →
    chờ 300ms tắt NAV → DISARMING → chờ FC xác nhận DISARM → IDLE.
    """
    fc = FlightController(MockDroneState())
    fc._worker = MockWorker()

    fc._state = "ARMING"

    fc.disarm()

    # Bước 1: Phải ở NAV_OFF_BEFORE_DISARM (không phải DISARMING ngay)
    assert fc._state == "NAV_OFF_BEFORE_DISARM"

    # Bước 2: Simulate 300ms trôi qua → chuyển sang DISARMING
    import time as _time
    fc._disarm_start_time = _time.time() - 0.5  # Quá 300ms
    fc._tick()
    assert fc._state == "DISARMING"

    # Bước 3: Simulate FC xác nhận DISARM → IDLE
    state = MockDroneState()
    state.is_armed = False
    fc._drone_state = state
    fc._tick()

    assert fc._state == "IDLE"
    assert not fc._timer.isActive()

    fc._timer.stop()  # Cleanup


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


# ══════════════════════════════════════════════
# TESTS v3 — TAKEOFF OPTIMIZATION
# ══════════════════════════════════════════════

def test_armed_wait_keeps_angle_and_idle():
    """20. v3 B1: _handle_armed_wait() phải giữ AUX2=ANGLE(1500) + Throttle=IDLE(1000).
    
    KHÔNG được bật AUX2=2000 (NAV_WP) trước khi upload WP.
    """
    state = MockDroneState()
    state.is_armed = True
    fc = FlightController(state)
    worker = MockWorker()
    fc._worker = worker

    # Giả lập vào ARMED_WAIT
    fc._state = "ARMED_WAIT"
    fc._armed_wait_start = 0  # Đã hết thời gian chờ
    fc._tick()  # Trigger 1 tick

    # Phải đã chuyển sang WP_UPLOAD
    assert fc._state == "WP_UPLOAD"
    # Kiểm tra channels trước khi transition
    assert fc._channels[fc.CH_AUX2] == fc.AUX_ANGLE         # 1500 — KHÔNG phải 2000!
    assert fc._channels[fc.CH_THROTTLE] == fc.THROTTLE_MIN   # 1000 — KHÔNG phải 1500!

    fc._timer.stop()


def test_wp_activate_rising_edge():
    """21. v3 B4: AUX2 chỉ được = 2000 SAU khi WP_ACTIVATE delay hết.
    
    Trước delay: AUX2=1500 (ANGLE). Sau delay: AUX2=2000 (NAV_WP).
    Đây là 'rising edge' để INAV nhận diện trigger NAV_WP.
    """
    import time as _time
    state = MockDroneState()
    state.is_armed = True
    fc = FlightController(state)
    worker = MockWorker()
    fc._worker = worker

    # Giả lập đang ở WP_ACTIVATE, vừa mới upload (chưa hết delay)
    fc._state = "WP_ACTIVATE"
    fc._wp_upload_time = _time.time()  # Vừa bắt đầu
    fc._tick()

    # Chưa hết delay → phải giữ ANGLE + Idle
    assert fc._state == "WP_ACTIVATE"  # Chưa chuyển
    assert fc._channels[fc.CH_AUX2] == fc.AUX_ANGLE
    assert fc._channels[fc.CH_THROTTLE] == fc.THROTTLE_MIN

    # Giả lập hết delay
    fc._wp_upload_time = _time.time() - 2.0  # 2s trước
    fc._tick()

    # Hết delay → phải bung 2000 + 1500
    assert fc._state == "NAV_CLIMB"
    assert fc._channels[fc.CH_AUX2] == fc.AUX_NAV_ALTHOLD_POSHOLD  # 2000!
    assert fc._channels[fc.CH_THROTTLE] == fc.RC_CENTER             # 1500!

    fc._timer.stop()


def test_altitude_reached_waits_1s():
    """22. v3 B5: ALTITUDE_REACHED phải chờ 1 giây trước khi sang HOLDING.
    
    Drone 3.5-inch cần thời gian PID settle sau khi đạt độ cao.
    """
    import time as _time
    state = MockDroneState()
    state.is_armed = True
    state.altitude = 3.0
    fc = FlightController(state)
    worker = MockWorker()
    fc._worker = worker

    # Vừa mới đạt độ cao
    fc._state = "ALTITUDE_REACHED"
    fc._reached_alt_time = _time.time()  # Vừa bắt đầu
    fc._tick()

    # Chưa đủ 1s → PHẢI vẫn ở ALTITUDE_REACHED
    assert fc._state == "ALTITUDE_REACHED"

    # Giả lập đã qua 1s
    fc._reached_alt_time = _time.time() - 1.5
    fc._tick()

    # Đã đủ 1s → phải sang HOLDING (KHÔNG phải IDLE!)
    assert fc._state == "HOLDING"

    fc._timer.stop()


def test_gps_drift_aborts_takeoff():
    """23. v3 B7: GPS trôi > 3m giữa stable_gps và tức thời → abort."""
    state = MockDroneState()
    state.is_armed = True
    state.gps_fix_type = 2
    state.gps_num_sat = 10
    # GPS tức thời ở Hà Nội
    state.latitude = 21.028500
    state.longitude = 105.854200
    # GPS history ở vị trí khác (cách ~50m)
    for _ in range(15):
        state._gps_history.append((21.028900, 105.854200))

    fc = FlightController(state)
    worker = MockWorker()
    fc._worker = worker
    errors = []
    fc.error_occurred.connect(lambda msg: errors.append(msg))

    import time as _time
    fc._state = "WP_UPLOAD"
    fc._wp_upload_time = _time.time()  # Chưa timeout → Haversine check sẽ chạy
    fc._tick()

    # Phải abort do GPS drift > 3m
    assert fc._state == "IDLE"
    assert len(errors) > 0
    assert "GPS trôi" in errors[0] or "multipath" in errors[0]

    fc._timer.stop()


def test_wp_upload_timeout():
    """24. v3 B8: WP_UPLOAD kẹt quá WP_UPLOAD_TIMEOUT_S → abort."""
    import time as _time
    state = MockDroneState()
    state.is_armed = True
    state.gps_fix_type = 2
    state.gps_num_sat = 10
    # KHÔNG cho GPS history đủ 10 mẫu → get_stable_gps() trả None

    fc = FlightController(state)
    worker = MockWorker()
    fc._worker = worker
    errors = []
    fc.error_occurred.connect(lambda msg: errors.append(msg))

    fc._state = "WP_UPLOAD"
    fc._wp_upload_time = _time.time() - 10.0  # 10s trước — vượt timeout 3s
    fc._tick()

    assert fc._state == "IDLE"
    assert len(errors) > 0
    assert "GPS" in errors[0]

    fc._timer.stop()


def test_gps_history_reset_on_takeoff():
    """25. v3 T1: takeoff_and_hold() phải clear _gps_history cũ."""
    state = MockDroneState()
    state.latitude = 21.028500
    state.longitude = 105.854200
    # Nhét GPS cũ
    for _ in range(15):
        state._gps_history.append((21.028500, 105.854200))
    assert len(state._gps_history) == 15

    fc = FlightController(state)
    worker = MockWorker()
    fc._worker = worker

    fc.takeoff_and_hold(3.0)

    # _gps_history phải đã bị clear
    assert len(state._gps_history) == 0

    fc._timer.stop()


# ══════════════════════════════════════════════
# TESTS v4 — MANUAL TAKEOFF
# ══════════════════════════════════════════════

def test_manual_takeoff_sets_flag():
    """26. manual_takeoff_and_hold() phải set _is_manual_takeoff=True."""
    state = MockDroneState()
    fc = FlightController(state)
    fc._worker = MockWorker()

    fc.manual_takeoff_and_hold(5.0)

    assert fc._is_manual_takeoff is True
    assert fc._state == "PRE_ARM_CHECK"
    assert len(state._gps_history) == 0  # GPS history đã clear

    fc._timer.stop()


def test_manual_armed_wait_branches_to_manual():
    """27. _handle_armed_wait() phân nhánh sang MANUAL_ANGLE_IDLE khi _is_manual_takeoff=True."""
    state = MockDroneState()
    state.is_armed = True
    fc = FlightController(state)
    fc._worker = MockWorker()

    fc._is_manual_takeoff = True
    fc._state = "ARMED_WAIT"
    fc._armed_wait_start = 0  # Hết thời gian chờ
    fc._tick()

    assert fc._state == "MANUAL_ANGLE_IDLE"

    fc._timer.stop()


def test_manual_ramp_throttle_increases():
    """28. MANUAL_THROTTLE_RAMP phải tăng throttle mỗi tick."""
    state = MockDroneState()
    state.is_armed = True
    fc = FlightController(state)
    fc._worker = MockWorker()

    fc._state = "MANUAL_THROTTLE_RAMP"
    fc._manual_ramp_throttle = 1000
    import time as _time
    fc._manual_stage_start = _time.time()
    fc._tick()

    assert fc._manual_ramp_throttle == 1050  # 1000 + MANUAL_RAMP_STEP(50)
    assert fc._channels[fc.CH_THROTTLE] == 1050

    fc._timer.stop()


def test_manual_abort_resets_flag():
    """29. _abort() phải reset _is_manual_takeoff=False."""
    state = MockDroneState()
    fc = FlightController(state)
    fc._worker = MockWorker()
    fc._is_manual_takeoff = True
    errors = []
    fc.error_occurred.connect(lambda msg: errors.append(msg))

    fc._abort("Test abort")

    assert fc._is_manual_takeoff is False
    assert fc._state == "IDLE"

    fc._timer.stop()


def test_manual_constants_valid():
    """30. Hằng số Manual Takeoff hợp lệ cho drone 3.5-inch 6S."""
    fc = FlightController(MockDroneState())

    assert fc.MANUAL_HOVER_THROTTLE == 1400
    assert fc.MANUAL_CLIMB_THROTTLE == 1600
    assert fc.MANUAL_RAMP_STEP == 50
    assert fc.MANUAL_MIN_SWITCH_ALT == 2.0
    assert fc.MANUAL_ANGLE_IDLE_S == 1.0
    assert fc.MANUAL_NAV_SETTLE_S == 1.5
    # Climb throttle phải > hover throttle
    assert fc.MANUAL_CLIMB_THROTTLE > fc.MANUAL_HOVER_THROTTLE


def test_nav_off_delay_constant():
    """31. Hằng số NAV_OFF_DELAY_S phải là 300ms."""
    fc = FlightController(MockDroneState())
    assert fc.NAV_OFF_DELAY_S == 0.3
