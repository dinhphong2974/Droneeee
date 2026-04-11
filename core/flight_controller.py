"""
flight_controller.py - Điều phối lệnh bay tự động.

Module này quản lý các chuỗi lệnh bay phức tạp:
- ARM / DISARM (với cơ chế chờ RC link ổn định — INAV safety)
- Takeoff & Hold (cất cánh bằng NAV ALTHOLD + POSHOLD của INAV firmware)
- Safe Land (hạ cánh khẩn cấp tại chỗ qua AUX3)
- RTH (Return To Home qua AUX4)
- Mission waypoint upload (MSP_SET_WP)

Cơ chế cất cánh (INAV ALTHOLD):
- Bật NAV ALTHOLD + NAV POSHOLD (AUX2=2000) ngay sau khi ARM
- Throttle > 1500 = lệnh leo (climb rate), INAV PID tự điều khiển motor
- Throttle = 1500 = giữ độ cao (hover), INAV tự lock
- GCS chỉ giám sát độ cao từ telemetry, KHÔNG điều khiển motor trực tiếp

Sensor Fusion (do INAV firmware xử lý):
- LiDAR MTF-02: INAV tự sử dụng qua nav_surface_control = AUTO
- Barometer: INAV tự kết hợp với LiDAR cho sensor fusion
- GCS không cần code sensor fusion thủ công

Sử dụng QTimer để chạy state machine không chặn UI.
Gửi lệnh MSP qua WifiWorker.send_command().
Đọc trạng thái từ DroneState.

Ánh xạ kênh AUX (cập nhật 2026-04-05):
- AUX1 (CH5): ARM/DISARM → 2000=ARM, 1000=DISARM
- AUX2 (CH6): Flight Mode → 1000=Acro, 1500=ANGLE, 2000=ALTHOLD+POSHOLD
- AUX3 (CH7): Safe Land → 2000=Kích hoạt, 1000=Bình thường
- AUX4 (CH8): RTH → 2000=Kích hoạt, 1000=Bình thường

Cấu hình INAV CLI cần thiết:
- nav_use_midthr_for_althold = 1  (50% throttle = hover)
- nav_mc_hover_thr = 1500          (hint hover throttle)
- nav_mc_auto_climb_rate = 300     (climb rate tối đa 3m/s)
- nav_surface_control = AUTO       (INAV tự dùng LiDAR MTF-02)

KHÔNG truy cập trực tiếp socket hay widget UI.
"""

import time
import struct
from PySide6.QtCore import QObject, QTimer, Signal

from comm.msp_parser import MSPParser, MSP_SET_RAW_RC, MSP_SET_WP


class FlightController(QObject):
    """
    Điều phối bay tự động: ARM, Takeoff, Hold, Safe Land, RTH, Mission.

    State machine chạy trên Main Thread qua QTimer (10Hz).
    Gửi MSP_SET_RAW_RC liên tục để duy trì quyền điều khiển RC trên FC.

    Signals:
        status_update(str): Cập nhật trạng thái cho UI
        takeoff_complete(): Báo hiệu đã đạt độ cao mục tiêu
        error_occurred(str): Thông báo lỗi
        state_changed(str): Thay đổi trạng thái state machine
        mode_activated(str): Thông báo mode bay đang chạy (cho emergency overlay)
    """

    # ── Signals ──
    status_update = Signal(str)
    takeoff_complete = Signal()
    error_occurred = Signal(str)
    state_changed = Signal(str)
    mode_activated = Signal(str)    # Tên mode đang chạy (ARM, Takeoff, Mission...)

    # ── Hằng số bay ──
    ALTITUDE_TOLERANCE = 0.3        # ±0.3m (chấp nhận được với barometer + GPS)
    THROTTLE_MIN = 1000             # Ga tối thiểu (μs)
    THROTTLE_CLIMB_CMD = 1700       # Tín hiệu "leo" cho INAV ALTHOLD (μs)
                                    # Trong ALTHOLD: >1500 = climb, 1500 = hover, <1500 = descend
                                    # 1700 ≈ climb rate 2-3m/s (tuỳ nav_mc_auto_climb_rate)
    THROTTLE_DESCEND_CMD = 1350     # Tín hiệu "hạ" nhẹ cho INAV ALTHOLD (μs)
    RC_CENTER = 1500                # Giá trị trung tâm kênh RC

    # ══════════════════════════════════════════════
    # HẰNG SỐ KÊNH AUX — Cập nhật theo cấu hình INAV Modes tab
    # ══════════════════════════════════════════════
    # Lưu ý: Mức tín hiệu sử dụng PWM value (Low: 1000, Mid: 1500, High: 2000)

    # AUX1 (CH5): ARM/DISARM
    AUX_DISARM = 1000              # AUX1 = 1000 → DISARM (tắt motor)
    AUX_ARM = 2000                 # AUX1 = 2000 → ARM (bật motor)

    # AUX2 (CH6): Chế độ bay (Flight Modes)
    AUX_ACRO = 1000                # AUX2 = 1000 → Manual/Acro mode
    AUX_ANGLE = 1500               # AUX2 = 1500 → ANGLE mode (tự cân bằng)
    AUX_NAV_ALTHOLD_POSHOLD = 2000 # AUX2 = 2000 → ALTHOLD + POSHOLD (dải kích hoạt: 1900-2100 trên INAV)

    # AUX3 (CH7): Safe Land / Failsafe Landing
    AUX_SAFE_LAND_OFF = 1000       # AUX3 = 1000 → Bình thường
    AUX_SAFE_LAND_ON = 2000        # AUX3 = 2000 → Kích hoạt hạ cánh tại chỗ

    # AUX4 (CH8): NAV RTH (Return To Home)
    AUX_RTH_OFF = 1000             # AUX4 = 1000 → Bình thường
    AUX_RTH_ON = 2000              # AUX4 = 2000 → Kích hoạt RTH (bay về Home)

    # ── Timeout ──
    TICK_INTERVAL_MS = 100          # Tần số state machine (10Hz)
    RC_LINK_WAIT_S = 1.5            # Chờ INAV reset ARM_SWITCH safety (giây)
                                    # INAV cần thấy AUX1=DISARM liên tục ≥1s trước khi cho phép ARM
    ARM_TIMEOUT_S = 5.0             # Timeout chờ ARM (giây) — tăng lên để chắc chắn
    CLIMB_TIMEOUT_S = 15.0          # Timeout đạt độ cao (giây)
    ARMED_WAIT_S = 0.5              # Chờ ổn định sau ARM (giây)

    # ── LiDAR MTF-02 — Sensor Fusion ──
    LIDAR_MAX_RANGE_M = 2.5         # Tầm đo tối đa MTF-02 AIO (2.5m)
    LIDAR_SOFT_LAND_THRESHOLD = 1.0 # Dưới 1m → dùng LiDAR để hạ cánh mềm
    LIDAR_GROUND_PROXIMITY = 0.15   # Nhỏ hơn 15cm → coi là đã chạm đất

    # ── Mission Upload ──
    WP_UPLOAD_DELAY_S = 0.005       # Delay 5ms giữa các WP frames (tránh TCP queue flood)
    WP_UPLOAD_TIMEOUT_S = 3.0       # Timeout chờ upload WP xong
    NAV_WP_ACTIVATE_WAIT_S = 1.0    # Chờ 1s sau upload trước khi bật NAV WP


    # ── Chỉ số kênh RC (thứ tự gửi trong MSP_SET_RAW_RC) ──
    # Thứ tự: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]
    CH_ROLL = 0        # Roll (Aileron)
    CH_PITCH = 1       # Pitch (Elevator)
    CH_THROTTLE = 2    # Throttle
    CH_YAW = 3         # Yaw (Rudder)
    CH_AUX1 = 4        # AUX1 → ARM (CH5 trên FC)
    CH_AUX2 = 5        # AUX2 → Flight Mode (CH6 trên FC)
    CH_AUX3 = 6        # AUX3 → Safe Land (CH7 trên FC)
    CH_AUX4 = 7        # AUX4 → RTH (CH8 trên FC)

    def __init__(self, drone_state, parent=None):
        """
        Khởi tạo flight controller.

        Args:
            drone_state: DroneState instance dùng chung
            parent: QObject parent (thường là GCSApp)
        """
        super().__init__(parent)

        self._drone_state = drone_state
        self._worker = None  # WifiWorker reference — gán bởi GCSApp
        self._parser = MSPParser()

        # ── State machine ──
        self._state = "IDLE"
        self._target_altitude = 3.0
        self._channels = self._safe_channels()

        # ── Timestamps cho timeout ──
        self._arm_start_time = 0.0
        self._armed_wait_start = 0.0
        self._climb_start_time = 0.0
        self._wp_upload_time = 0.0

        # ── Takeoff mode ──
        self._use_nav_wp_takeoff = True  # Mặc định dùng NAV WP; chuyển False nếu GPS yếu

        # ── QTimer chạy state machine (10Hz) ──
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

    # ══════════════════════════════════════════════
    # API CÔNG KHAI
    # ══════════════════════════════════════════════

    def set_worker(self, worker):
        """Cập nhật reference tới WifiWorker hiện tại. Gọi từ GCSApp."""
        self._worker = worker

    def arm(self):
        """
        ARM drone bằng cách bật AUX1 (đợi RC link ổn định rồi mới bật).

        Quy trình an toàn INAV:
        1. Gửi RC với AUX1=DISARM liên tục trong 1.5 giây
           (INAV yêu cầu thấy cần gạt ở mức DISARM trước khi cho phép ARM)
        2. Sau khi ổn định, gạt AUX1=ARM (2000)
        3. Chờ FC xác nhận ARM qua MSP_STATUS
        """
        if not self._worker:
            self.error_occurred.emit("Chưa kết nối — không thể ARM")
            return
        if self._drone_state.is_armed:
            self.status_update.emit("Drone đã ARM sẵn")
            return

        # Bắt đầu với DISARM để INAV reset ARM_SWITCH safety
        self._channels = self._safe_channels()
        self._send_rc()
        self._arm_start_time = time.time()
        self._transition("WAIT_RC_LINK_ARM_ONLY")
        self._timer.start(self.TICK_INTERVAL_MS)
        self.status_update.emit("Khởi tạo kết nối RC...")
        self.mode_activated.emit("ARM")

    def disarm(self):
        """DISARM drone — tắt AUX, hạ ga, dừng state machine."""
        self._state = "IDLE"  # Set state TRƯỚC để tick cuối (nếu có) sẽ tự dừng
        self._timer.stop()
        self._channels = self._safe_channels()
        self._send_rc()
        self._transition("IDLE")
        self.status_update.emit("Đã gửi lệnh DISARM")
        self.mode_activated.emit("")  # Tắt overlay

    def takeoff_and_hold(self, target_alt: float = 3.0):
        """
        Chuỗi tự động: ARM → Bật NAV ALTHOLD → Ra lệnh leo → Giữ độ cao.

        Sử dụng INAV ALTHOLD (AUX2=2000) ngay sau khi ARM.
        Throttle > 1500 = lệnh leo (climb rate), INAV PID tự điều khiển motor.
        Khi đạt độ cao → đưa throttle về 1500 = INAV tự giữ.
        GPS BZ 251 cung cấp tọa độ giúp FC giữ vị trí chính xác (POSHOLD).

        Args:
            target_alt: Độ cao mục tiêu (mét). Người dùng nhập từ dialog.
        """
        if not self._worker:
            self.error_occurred.emit("Chưa kết nối — không thể cất cánh")
            return

        self._target_altitude = target_alt
        self._channels = self._safe_channels()
        self._transition("PRE_ARM_CHECK")
        self._timer.start(self.TICK_INTERVAL_MS)
        self.mode_activated.emit(f"Takeoff {target_alt:.0f}m")

    def safe_land(self):
        """
        Kích hoạt Safe Land — FC tự hạ cánh tại chỗ.

        Gửi AUX3=2000 (CH7) để FC kích hoạt Emergency Landing.
        FC chiếm quyền điều khiển, giảm ga từ từ và hạ cánh.
        Sau khi chạm đất, FC tự DISARM.

        Dùng cho:
        - Nút "Safe Land" trên Manual Control tab
        - Failsafe khi mất kết nối WiFi
        """
        # STOP bất kỳ state machine nào đang chạy (takeoff, climb...)
        self._timer.stop()

        if not self._worker:
            self.error_occurred.emit("Chưa kết nối — không thể Safe Land")
            return

        self.status_update.emit("Đang kích hoạt Safe Land...")
        self._channels[self.CH_AUX3] = self.AUX_SAFE_LAND_ON
        self._channels[self.CH_AUX4] = self.AUX_RTH_OFF  # Đảm bảo RTH tắt
        self._channels[self.CH_AUX2] = self.AUX_ANGLE     # Về ANGLE, tránh xung đột POSHOLD
        self._channels[self.CH_THROTTLE] = self.RC_CENTER  # FC tự điều khiển
        self._send_rc()

        # Tiếp tục gửi RC liên tục qua timer ở state SAFE_LANDING
        self._transition("SAFE_LANDING")
        self._timer.start(self.TICK_INTERVAL_MS)
        self.mode_activated.emit("Safe Land")

    # ══════════════════════════════════════════════
    # FORCE EMERGENCY COMMANDS — BYPASS STATE MACHINE
    # ══════════════════════════════════════════════

    def force_disarm(self):
        """
        FORCE DISARM — Lệnh khẩn cấp ưu tiên cao nhất.

        Khác biệt với disarm() thường:
        1. STOP timer TRƯỚC → ngăn tick ghi đè channels
        2. Gửi sequence INAV-safe: tắt NAV modes → DISARM
           (INAV bỏ qua DISARM khi đang ở NAV_TAKEOFF mode)
        3. Gửi NHIỀU frame liên tiếp để đảm bảo FC nhận được
        4. Sử dụng emergency queue (ưu tiên cao nhất)
        """
        # 1. STOP state machine NGAY LẬP TỨC
        self._state = "IDLE"
        self._timer.stop()

        if not self._worker:
            self._transition("IDLE")
            self.mode_activated.emit("")
            return

        # 2. Frame 1: Tắt NAV modes (bypass INAV NAV_TAKEOFF protection)
        #    INAV bỏ qua DISARM khi đang ở NAV mode → phải tắt NAV trước
        nav_off_channels = [
            self.RC_CENTER,         # Roll
            self.RC_CENTER,         # Pitch
            self.THROTTLE_MIN,      # Throttle — cắt ga
            self.RC_CENTER,         # Yaw
            self.AUX_ARM,           # AUX1 = vẫn ARM (chưa disarm)
            self.AUX_ANGLE,         # AUX2 = ANGLE (tắt ALTHOLD/POSHOLD)
            self.AUX_SAFE_LAND_OFF, # AUX3 = Safe Land OFF
            self.AUX_RTH_OFF,       # AUX4 = RTH OFF
        ]
        self._channels = nav_off_channels
        self._send_rc_emergency()   # Gửi qua emergency queue

        # 3. Frame 2-3: DISARM (AUX1=1000)
        self._channels = self._safe_channels()
        self._send_rc_emergency()
        self._send_rc_emergency()   # Gửi thêm 1 lần để đảm bảo

        # 4. Chuyển state machine về IDLE
        self._transition("IDLE")
        self.status_update.emit("⛔ FORCE DISARM — Motor tắt")
        self.mode_activated.emit("")

    def force_safe_land(self):
        """
        FORCE SAFE LAND — Hạ cánh khẩn cấp ưu tiên cao.

        Khác biệt với safe_land() thường:
        1. STOP takeoff timer trước
        2. Tắt NAV modes (bypass INAV NAV_TAKEOFF lock)
        3. Kích hoạt Safe Land ngay lập tức
        4. Khởi động lại timer ở state SAFE_LANDING
        5. Sử dụng emergency queue (ưu tiên cao nhất)
        """
        # 1. STOP bất kỳ state machine nào đang chạy
        self._state = "IDLE"
        self._timer.stop()

        if not self._worker:
            self._transition("IDLE")
            self.mode_activated.emit("")
            return

        # 2. Tắt NAV modes + kích hoạt Safe Land
        self._channels = [
            self.RC_CENTER,         # Roll
            self.RC_CENTER,         # Pitch
            self.RC_CENTER,         # Throttle — FC tự điều khiển
            self.RC_CENTER,         # Yaw
            self.AUX_ARM,           # AUX1 = Duy trì ARM
            self.AUX_ANGLE,         # AUX2 = ANGLE (tắt NAV modes)
            self.AUX_SAFE_LAND_ON,  # AUX3 = KÍCH HOẠT SAFE LAND
            self.AUX_RTH_OFF,       # AUX4 = RTH OFF
        ]
        self._send_rc_emergency()   # Frame 1: Tắt NAV + bật Safe Land
        self._send_rc_emergency()   # Frame 2: Đảm bảo

        # 3. Chuyển sang state SAFE_LANDING và restart timer
        self._transition("SAFE_LANDING")
        self._timer.start(self.TICK_INTERVAL_MS)
        self.status_update.emit("🛬 FORCE Safe Land — Đang hạ cánh...")
        self.mode_activated.emit("Safe Land")

    def rth(self):
        """
        Kích hoạt RTH (Return To Home) — drone bay về vị trí Home.

        Gửi AUX4=2000 (CH8) để FC bật NAV RTH.
        INAV sử dụng GPS BZ 251 + la bàn để tự bay về tọa độ Home
        (vị trí được INAV chốt tại thời điểm ARM đầu tiên).

        Dùng cho:
        - Nút RTH trên UI
        - Lệnh RTH từ mission logic khi drone bay quá xa
        """
        if not self._worker:
            self.error_occurred.emit("Chưa kết nối — không thể RTH")
            return

        self.status_update.emit("Đang kích hoạt RTH...")
        self._channels[self.CH_AUX4] = self.AUX_RTH_ON
        self._channels[self.CH_AUX3] = self.AUX_SAFE_LAND_OFF  # Đảm bảo Safe Land tắt
        self._channels[self.CH_THROTTLE] = self.RC_CENTER  # FC tự điều khiển
        self._send_rc()

        # Tiếp tục gửi RC liên tục qua timer
        self._transition("RTH_ACTIVE")
        if not self._timer.isActive():
            self._timer.start(self.TICK_INTERVAL_MS)
        self.mode_activated.emit("RTH")

    def abort(self):
        """Dừng khẩn cấp — cắt ga, DISARM, dừng timer."""
        self._abort("Người dùng yêu cầu dừng khẩn cấp")

    def upload_mission(self, waypoints: list[dict]):
        """
        Upload danh sách waypoint xuống FC qua MSP_SET_WP.

        INAV lưu waypoint vào bộ nhớ RAM. Sau khi upload xong,
        bật mode NAV WP để FC tự bay theo lộ trình.

        Args:
            waypoints: List dict, mỗi dict chứa:
                       {"lat": float, "lon": float, "alt": float (mét)}
        """
        if not self._worker:
            self.error_occurred.emit("Chưa kết nối — không thể upload mission")
            return

        self.status_update.emit(f"Đang upload {len(waypoints)} waypoints...")

        for i, wp in enumerate(waypoints):
            wp_no = i + 1  # WP bắt đầu từ 1 (0 = RTH special)
            lat = wp["lat"]
            lon = wp["lon"]
            alt_cm = int(wp["alt"] * 100)  # Mét → cm

            # Waypoint cuối cùng cần flag p3=0xA5 (165) để INAV biết mission kết thúc
            # Flag cho WP cuối: flag bit 0xa5 = last WP marker trong INAV
            p3 = 0xA5 if (i == len(waypoints) - 1) else 0

            frame = self._parser.pack_set_wp(wp_no, lat, lon, alt_cm, p1=0, p2=0, p3=p3)
            self._worker.send_command(frame)

            # BUG-06 FIX: Delay nhỏ giữa các WP frames tránh flood TCP queue ESP32
            # Với mission lớn (>20 WP), gửi liên tiếp có thể khiến ESP32 drop gói
            if i < len(waypoints) - 1:  # Không delay sau frame cuối
                time.sleep(self.WP_UPLOAD_DELAY_S)

            self.status_update.emit(
                f"Upload WP {wp_no}/{len(waypoints)}: "
                f"({lat:.6f}, {lon:.6f}) alt={wp['alt']:.0f}m"
            )

        self.status_update.emit(f"✓ Đã upload {len(waypoints)} waypoints lên FC")

    def send_failsafe_config(self, behavior: str):
        """
        Gửi cấu hình hành vi failsafe xuống ESP32.

        Args:
            behavior: "rth" hoặc "ignore"
                      - "rth": Mất WiFi → ESP32 gửi RTH
                      - "ignore": Mất WiFi → ESP32 không can thiệp
        """
        if not self._worker:
            return
        config_cmd = f"FS:{behavior}".encode()
        self._worker.send_command(config_cmd)
        self.status_update.emit(f"Đã gửi cấu hình failsafe: {behavior}")

    @property
    def state(self) -> str:
        """Trạng thái hiện tại của state machine."""
        return self._state

    @property
    def is_active(self) -> bool:
        """True nếu đang thực hiện chuỗi bay (không phải IDLE)."""
        return self._state != "IDLE"

    @property
    def _effective_altitude(self) -> float:
        """
        Sensor Fusion: Chọn nguồn độ cao tốt nhất hiện có.

        Ưu tiên:
        1. LiDAR MTF-02 khi: dữ liệu hợp lệ VÀ trong tầm đo (≤ LIDAR_MAX_RANGE_M)
        2. Barometer (MSP_ALTITUDE) khi LiDAR ngoài tầm hoặc không có dữ liệu

        INAV firmware đã tự sensor-fusion internally; property này chỉ dùng
        cho GCS-side state machine (kiểm tra đạt độ cao mục tiêu, v.v.)

        Returns:
            float: Độ cao ước lượng tốt nhất (mét)
        """
        has_valid_lidar = (
            self._drone_state.has_valid_surface
            and 0.0 <= self._drone_state.surface_altitude <= self.LIDAR_MAX_RANGE_M
        )
        if has_valid_lidar:
            return self._drone_state.surface_altitude
        return self._drone_state.altitude


    # ══════════════════════════════════════════════
    # STATE MACHINE — VÒNG LẶP CHÍNH (10Hz)
    # ══════════════════════════════════════════════

    def _tick(self):
        """Được gọi mỗi 100ms bởi QTimer."""
        # Kiểm tra kết nối mỗi tick
        if self._state != "IDLE" and not self._worker:
            self._abort("Mất kết nối trong lúc bay")
            return

        if self._state == "IDLE":
            self._timer.stop()

        elif self._state == "PRE_ARM_CHECK":
            self._handle_pre_arm_check()

        elif self._state == "WAIT_RC_LINK":
            self._handle_wait_rc_link()

        elif self._state == "WAIT_RC_LINK_ARM_ONLY":
            self._handle_wait_rc_link_arm_only()

        elif self._state == "ARMING":
            self._handle_arming()

        elif self._state == "ARMING_ONLY":
            self._handle_arming_only()

        elif self._state == "ARMED_WAIT":
            self._handle_armed_wait()

        elif self._state == "WP_UPLOAD":
            self._handle_wp_upload()

        elif self._state == "WP_ACTIVATE":
            self._handle_wp_activate()

        elif self._state == "NAV_CLIMB":
            self._handle_nav_climb()

        elif self._state == "ALTITUDE_REACHED":
            self._handle_altitude_reached()

        elif self._state == "HOLDING":
            self._handle_holding()

        elif self._state == "SAFE_LANDING":
            self._handle_safe_landing()

        elif self._state == "RTH_ACTIVE":
            self._handle_rth_active()

    # ══════════════════════════════════════════════
    # XỬ LÝ TỪNG TRẠNG THÁI
    # ══════════════════════════════════════════════

    def _handle_pre_arm_check(self):
        """Kiểm tra điều kiện an toàn trước khi ARM."""
        # Quyết định cơ chế cất cánh: NAV WP (GPS) hay ALTHOLD fallback (indoor)
        has_3d_fix = (
            self._drone_state.gps_fix_type >= 2
            and self._drone_state.gps_num_sat >= 6
        )
        
        # Xác thực Cảm biến Cắt Lớp (Strict Sensor Validation)
        has_sensors_ok = self._drone_state.sensor_opflow and self._drone_state.sensor_rangefinder
        
        self._use_nav_wp_takeoff = has_3d_fix and has_sensors_ok

        if self._use_nav_wp_takeoff:
            self.status_update.emit(
                f"📡 GPS 3D Fix ({self._drone_state.gps_num_sat} sats) & Sensors OK — "
                f"Sử dụng NAV WP Takeoff"
            )
        else:
            reason = "GPS yếu" if not has_3d_fix else "Thiếu OptFlow/LiDAR"
            self.status_update.emit(
                f"⚠️ {reason} — Fallback ALTHOLD Takeoff (GCS điều khiển ga)"
            )

        if self._drone_state.is_armed:
            # Đã ARM sẵn → skip thẳng sang cất cánh
            self.status_update.emit("Drone đã ARM — bắt đầu cất cánh...")
            self._channels[self.CH_ROLL] = self.RC_CENTER
            self._channels[self.CH_PITCH] = self.RC_CENTER
            self._channels[self.CH_YAW] = self.RC_CENTER
            self._channels[self.CH_AUX1] = self.AUX_ARM
            self._channels[self.CH_AUX2] = self.AUX_NAV_ALTHOLD_POSHOLD
            self._channels[self.CH_AUX3] = self.AUX_SAFE_LAND_OFF
            self._channels[self.CH_AUX4] = self.AUX_RTH_OFF
            self._channels[self.CH_THROTTLE] = self.RC_CENTER
            self._send_rc()

            if self._use_nav_wp_takeoff:
                self._wp_upload_time = time.time()
                self._transition("WP_UPLOAD")
            else:
                self._climb_start_time = time.time()
                self._transition("NAV_CLIMB")
            return

        # Đặt throttle thấp, tất cả kênh center — chuẩn bị ARM
        self._channels = self._safe_channels()
        self._send_rc()
        self._arm_start_time = time.time()
        self._transition("WAIT_RC_LINK")
        self.status_update.emit("Khởi tạo kết nối RC...")

    def _handle_wait_rc_link(self):
        """
        Gửi RC DISARM liên tục trong RC_LINK_WAIT_S giây.

        INAV yêu cầu thấy AUX1 ở vị trí DISARM trong ít nhất 1 giây
        trước khi cho phép ARM (cơ chế bảo vệ ARMING_DISABLED_ARM_SWITCH).
        Nếu không có bước này, INAV sẽ báo lỗi ARM_SWITCH và từ chối ARM.

        Tương tự như khi người dùng gạt cần gạt từ từ:
        - Khởi động → cần gạt ở mức thấp (DISARM)
        - Chờ 1.5 giây
        - Gạt lên mức cao (ARM)
        """
        self._send_rc()
        elapsed = time.time() - self._arm_start_time
        remaining = max(0.0, self.RC_LINK_WAIT_S - elapsed)
        self.status_update.emit(f"Khởi tạo RC link... ({remaining:.1f}s)")
        if elapsed > self.RC_LINK_WAIT_S:
            self._arm_start_time = time.time()
            self._transition("ARMING")
            self.status_update.emit("Đang gửi lệnh ARM...")

    def _handle_wait_rc_link_arm_only(self):
        """
        Gửi RC DISARM liên tục trong RC_LINK_WAIT_S giây (nút ARM lẻ).

        Xem giải thích tại _handle_wait_rc_link.
        Cùng cơ chế an toàn: phải "gạt từ từ" — gửi DISARM trước rồi mới ARM.
        """
        self._send_rc()
        elapsed = time.time() - self._arm_start_time
        remaining = max(0.0, self.RC_LINK_WAIT_S - elapsed)
        self.status_update.emit(f"Khởi tạo RC link... ({remaining:.1f}s)")
        if elapsed > self.RC_LINK_WAIT_S:
            self._arm_start_time = time.time()
            self._transition("ARMING_ONLY")
            self.status_update.emit("Đang gửi lệnh ARM...")

    def _handle_arming(self):
        """Gửi AUX1=HIGH liên tục và chờ FC xác nhận ARM qua MSP_STATUS."""
        self._channels[self.CH_AUX1] = self.AUX_ARM
        self._send_rc()

        if self._drone_state.is_armed:
            self.status_update.emit("ARM thành công — chờ ổn định...")
            self._armed_wait_start = time.time()
            self._transition("ARMED_WAIT")
        elif time.time() - self._arm_start_time > self.ARM_TIMEOUT_S:
            self._abort(
                f"ARM thất bại — timeout {self.ARM_TIMEOUT_S:.0f}s.\n"
                "Nguyên nhân thường gặp:\n"
                "• ARMING_DISABLED_ARM_SWITCH: Chờ thêm 2s rồi thử lại\n"
                "• ARMING_DISABLED_DSHOT_BEEPER: Tắt DShot Beeper trong INAV Configurator\n"
                "• Kiểm tra tab Modes: ARM phải gán đúng CH5, dải kích hoạt"
            )

    def _handle_arming_only(self):
        """ARM riêng lẻ (không takeoff) — chờ xác nhận rồi dừng timer."""
        self._channels[self.CH_AUX1] = self.AUX_ARM
        self._send_rc()

        if self._drone_state.is_armed:
            self._timer.stop()
            self._transition("IDLE")
            self.status_update.emit("ARM thành công ✓")
            self.mode_activated.emit("Armed")  # Hiện overlay vì motor đang quay
        elif time.time() - self._arm_start_time > self.ARM_TIMEOUT_S:
            self._timer.stop()
            self._channels = self._safe_channels()
            self._send_rc()
            self._transition("IDLE")
            self.mode_activated.emit("")  # Tắt overlay
            self.error_occurred.emit(
                f"ARM thất bại — timeout {self.ARM_TIMEOUT_S:.0f}s.\n"
                "Nguyên nhân thường gặp:\n"
                "• ARMING_DISABLED_ARM_SWITCH: Chờ thêm 2s rồi thử lại\n"
                "• ARMING_DISABLED_DSHOT_BEEPER: Tắt DShot Beeper trong INAV Configurator\n"
                "• Kiểm tra tab Modes: ARM phải gán đúng CH5, dải kích hoạt"
            )

    def _handle_armed_wait(self):
        """Chờ drone ổn định 500ms sau khi ARM trước khi cất cánh."""
        # Duy trì AUX1=ARM liên tục trong lúc chờ ổn định
        self._channels[self.CH_AUX1] = self.AUX_ARM
        self._send_rc()

        if time.time() - self._armed_wait_start > self.ARMED_WAIT_S:
            # Bật ALTHOLD + POSHOLD
            self._channels[self.CH_AUX2] = self.AUX_NAV_ALTHOLD_POSHOLD
            self._channels[self.CH_THROTTLE] = self.RC_CENTER
            self._send_rc()

            if self._use_nav_wp_takeoff:
                # ======= CHẾ ĐỘ MỚI: NAV WP TAKEOFF =======
                # Upload 1 WP tại vị trí hiện tại với độ cao mục tiêu
                self._wp_upload_time = time.time()
                self._transition("WP_UPLOAD")
                self.status_update.emit("📤 Đang upload WP cất cánh lên FC...")
            else:
                # ======= FALLBACK: ALTHOLD THROTTLE RAMP =======
                # Không có GPS → dùng cách cũ (đẩy ga thủ công)
                self._climb_start_time = time.time()
                self._transition("NAV_CLIMB")
                self.status_update.emit("ALTHOLD bật — GCS điều khiển cất cánh (fallback)...")

    def _handle_wp_upload(self):
        """
        TRẠNG THÁI MỚI: Upload 1 Waypoint tại vị trí hiện tại + độ cao mục tiêu.

        INAV nhận WP với flag 0xA5 (last WP) → khi bật NAV WP, INAV sẽ
        tự động cất cánh và leo lên đúng độ cao đã chỉ định, sau đó hover tại chỗ.
        Toàn bộ điều khiển bay vật lý do C++ của INAV xử lý ở 1000Hz.
        """
        # Duy trì ARM + ALTHOLD
        self._channels[self.CH_AUX1] = self.AUX_ARM
        self._channels[self.CH_THROTTLE] = self.RC_CENTER
        self._send_rc()

        # Lấy tọa độ GPS đã được làm mượt qua Moving Average Box (Chống trôi)
        stable_gps = self._drone_state.get_stable_gps()

        if not stable_gps:
            self.status_update.emit("⏳ Đang thu thập mốc GPS ổn định (Anti-drift)...")
            return
            
        lat, lon = stable_gps
        alt_cm = int(self._target_altitude * 100)

        if lat == 0.0 and lon == 0.0:
            # GPS chưa có dữ liệu → fallback ALTHOLD
            self.status_update.emit("⚠️ GPS chưa có tọa độ — fallback ALTHOLD")
            self._use_nav_wp_takeoff = False
            self._climb_start_time = time.time()
            self._transition("NAV_CLIMB")
            return

        # Tạo WP: action=WAYPOINT(1), flag=0xA5 (last WP)
        frame = self._parser.pack_set_wp(
            wp_no=1, lat=lat, lon=lon, alt=alt_cm, p1=0, p2=0, p3=0xA5
        )
        self._worker.send_command(frame)

        self.status_update.emit(
            f"✅ WP uploaded: ({lat:.6f}, {lon:.6f}) alt={self._target_altitude:.0f}m — "
            f"Chờ kích hoạt NAV WP..."
        )
        self._wp_upload_time = time.time()
        self._transition("WP_ACTIVATE")

    def _handle_wp_activate(self):
        """
        TRẠNG THÁI MỚI: Chờ 1s rồi bật NAV WP mode.

        Sau khi upload WP, cần chờ để INAV xử lý xong WP vào bộ nhớ.
        Sau đó gạt AUX2=2000 (NAV WP) → INAV tự cất cánh.
        """
        self._channels[self.CH_AUX1] = self.AUX_ARM
        self._channels[self.CH_THROTTLE] = self.RC_CENTER
        self._send_rc()

        if time.time() - self._wp_upload_time > self.NAV_WP_ACTIVATE_WAIT_S:
            # Bật NAV WP → INAV tự động cất cánh
            self._channels[self.CH_AUX2] = self.AUX_NAV_ALTHOLD_POSHOLD  # NAV WP cùng dải
            self._channels[self.CH_THROTTLE] = self.RC_CENTER  # INAV tự điều khiển ga
            self._climb_start_time = time.time()
            self._send_rc()
            self._transition("NAV_CLIMB")
            self.status_update.emit(
                f"🚀 NAV WP đã kích hoạt — INAV tự động cất cánh lên {self._target_altitude:.0f}m"
            )

    def _handle_nav_climb(self):
        """
        Trạng thái leo cao.

        Nếu dùng NAV WP: INAV đã tự điều khiển hoàn toàn. GCS chỉ giám sát
        độ cao từ telemetry và gửi RC giữ nguyên (throttle=1500) để duy trì
        MSP override. INAV C++ PID 1000Hz tự tính toán ga, bù gió, phanh mượt.

        Nếu fallback ALTHOLD: GCS đẩy throttle > 1500 để ra lệnh leo,
        INAV PID vẫn điều khiển motor nhưng GCS chỉ định climb rate.
        """
        # An toàn: nếu FC tự DISARM giữa chừng (failsafe hardware, lỗi sensor)
        if not self._drone_state.is_armed:
            self._timer.stop()
            self._channels = self._safe_channels()
            self._send_rc()
            self._transition("IDLE")
            self.status_update.emit("Drone DISARM giữa chừng — dừng cất cánh")
            self.mode_activated.emit("")
            return

        # Đọc độ cao tốt nhất: LiDAR MTF-02 nếu trong tầm, Barometer nếu không
        alt = self._effective_altitude

        if self._use_nav_wp_takeoff:
            # ======= NAV WP MODE: INAV TỰ ĐIỀU KHIỂN =======
            # GCS chỉ gửi RC center để duy trì override, KHÔNG can thiệp ga
            self._channels[self.CH_AUX2] = self.AUX_NAV_ALTHOLD_POSHOLD
            self._channels[self.CH_THROTTLE] = self.RC_CENTER
            self._send_rc()

            self.status_update.emit(
                f"INAV đang cất cánh (NAV WP) — Alt: {alt:.1f}m / {self._target_altitude:.0f}m | "
                f"GPS: {self._drone_state.gps_num_sat} sats"
            )
        else:
            # ======= FALLBACK ALTHOLD: GCS ĐIỀU KHIỂN GA =======
            # Vẫn giữ logic đẩy ga cũ cho bay indoor không GPS
            distance_to_target = max(0.0, self._target_altitude - alt)

            if distance_to_target < 1.0:
                # Phanh mềm: Gần đích → giảm ga tỉ lệ
                progress = distance_to_target / 1.0
                target_throttle = int(self.RC_CENTER + (self.THROTTLE_CLIMB_CMD - self.RC_CENTER) * progress)
            else:
                # Khởi hành mềm: 1.5s đầu tiên tăng ga dần
                elapsed_climb = time.time() - self._climb_start_time
                ramp_duration = 1.5
                if elapsed_climb < ramp_duration:
                    progress = elapsed_climb / ramp_duration
                    target_throttle = int(self.RC_CENTER + (self.THROTTLE_CLIMB_CMD - self.RC_CENTER) * progress)
                else:
                    target_throttle = self.THROTTLE_CLIMB_CMD

            smoothed_throttle = max(1000, min(target_throttle, 2000))
            self._channels[self.CH_AUX2] = self.AUX_NAV_ALTHOLD_POSHOLD
            self._channels[self.CH_THROTTLE] = smoothed_throttle
            self._send_rc()

            self.status_update.emit(
                f"ALTHOLD leo (fallback) — Alt: {alt:.1f}m / {self._target_altitude:.0f}m | Ga: {smoothed_throttle}μs"
            )

        # Kiểm tra đạt độ cao mục tiêu
        if alt >= self._target_altitude - self.ALTITUDE_TOLERANCE:
            self._transition("ALTITUDE_REACHED")
        elif time.time() - self._climb_start_time > self.CLIMB_TIMEOUT_S:
            self._abort(
                f"Không đạt độ cao mục tiêu ({alt:.1f}m / "
                f"{self._target_altitude:.0f}m) — timeout {self.CLIMB_TIMEOUT_S:.0f}s"
            )

    def _handle_altitude_reached(self):
        """
        Đạt độ cao mục tiêu — đưa throttle về center để INAV giữ.

        Throttle = 1500 (center) trong ALTHOLD = lệnh "giữ độ cao hiện tại".
        INAV PID tự lock độ cao + GPS BZ 251 lock vị trí (POSHOLD).
        LiDAR MTF-02 hỗ trợ giữ chính xác hơn (INAV tự xử lý).
        """
        self._channels[self.CH_THROTTLE] = self.RC_CENTER
        self._channels[self.CH_AUX2] = self.AUX_NAV_ALTHOLD_POSHOLD
        self._send_rc()

        alt = self._drone_state.altitude
        lat = self._drone_state.latitude
        lon = self._drone_state.longitude

        self.status_update.emit(
            f"Giữ độ cao tại {alt:.1f}m ✓ — ALTHOLD+POSHOLD đã bật "
            f"| GPS: ({lat:.6f}, {lon:.6f})"
        )
        self._transition("HOLDING")
        self.takeoff_complete.emit()
        self.mode_activated.emit(f"Holding {alt:.1f}m")

    def _handle_holding(self):
        """
        Duy trì trạng thái hover — gửi RC liên tục để giữ MSP override.

        INAV ALTHOLD + POSHOLD đang hoạt động, FC tự giữ độ cao và vị trí.
        GCS chỉ cần duy trì tín hiệu RC và giám sát trạng thái ARM.
        """
        self._send_rc()

        # Theo dõi an toàn: nếu FC tự DISARM → dừng
        if not self._drone_state.is_armed:
            self._timer.stop()
            self._transition("IDLE")
            self.status_update.emit("Drone đã DISARM — kết thúc hold")
            self.mode_activated.emit("")
            return

    def _handle_safe_landing(self):
        """
        Trạng thái Safe Landing — gửi AUX3=2000 liên tục.

        FC chiếm quyền điều khiển và hạ cánh. Khi chạm đất, FC tự DISARM.
        State machine chờ FC báo DISARMED rồi chuyển về IDLE.

        INAV tự sử dụng LiDAR MTF-02 (nav_surface_control = AUTO) để hạ cánh
        mượt mà — GCS không cần giám sát LiDAR thủ công.
        """
        # Duy trì tín hiệu Safe Land liên tục
        self._channels[self.CH_AUX3] = self.AUX_SAFE_LAND_ON
        self._channels[self.CH_AUX2] = self.AUX_ANGLE  # Tránh xung đột với POSHOLD
        self._send_rc()

        # Theo dõi: khi FC tự DISARM (đã chạm đất) → kết thúc
        if not self._drone_state.is_armed:
            self._timer.stop()
            self._channels = self._safe_channels()
            self._send_rc()
            self._transition("IDLE")
            self.status_update.emit("Hạ cánh thành công ✓ — Đã DISARM")
            self.mode_activated.emit("")

    def _handle_rth_active(self):
        """
        Trạng thái RTH đang hoạt động — gửi AUX4=2000 liên tục.

        FC sử dụng GPS BZ 251 + la bàn để bay về vị trí Home.
        Khi đến Home, FC tự hạ cánh và DISARM.
        """
        # Duy trì tín hiệu RTH liên tục
        self._channels[self.CH_AUX4] = self.AUX_RTH_ON
        self._send_rc()

        # Theo dõi: khi FC tự DISARM (đã về Home và hạ cánh) → kết thúc
        if not self._drone_state.is_armed:
            self._timer.stop()
            self._channels = self._safe_channels()
            self._send_rc()
            self._transition("IDLE")
            self.status_update.emit("RTH hoàn tất ✓ — Đã về Home và DISARM")
            self.mode_activated.emit("")

    # ══════════════════════════════════════════════
    # HÀM NỘI BỘ
    # ══════════════════════════════════════════════

    def _send_rc(self):
        """Gửi MSP_SET_RAW_RC với giá trị kênh hiện tại qua WifiWorker."""
        if not self._worker:
            return
        payload = struct.pack('<8H', *self._channels)
        frame = self._parser.pack_msg(MSP_SET_RAW_RC, payload)
        self._worker.send_command(frame)

    def _send_rc_emergency(self):
        """
        Gửi MSP_SET_RAW_RC qua emergency queue (ưu tiên cao nhất).

        Xóa toàn bộ command queue thường và gửi ngay lập tức.
        Dùng cho force_disarm() và force_safe_land().

        Frame được gắn prefix "EM:" để ESP32 nhận diện lệnh khẩn cấp:
        - ESP32 xóa UART input buffer (discard response cũ)
        - Chuyển tiếp MSP frame ngay lập tức xuống FC
        - Tắt failsafe state để không ghi đè lệnh emergency
        """
        if not self._worker:
            return
        payload = struct.pack('<8H', *self._channels)
        frame = self._parser.pack_msg(MSP_SET_RAW_RC, payload)
        # Gắn prefix EM: để ESP32 ưu tiên xử lý lệnh khẩn cấp
        emergency_frame = b'EM:' + frame
        if hasattr(self._worker, 'send_emergency_command'):
            self._worker.send_emergency_command(emergency_frame)
        else:
            self._worker.send_command(emergency_frame)

    def _safe_channels(self) -> list[int]:
        """
        Trả về bộ kênh RC an toàn: throttle thấp, DISARM, ANGLE mode bật.

        QUAN TRỌNG:
        - AUX1 (ARM) = 1000 → DISARM (motor tắt)
        - AUX2 (Mode) = 1500 → ANGLE mode (tự cân bằng, tránh lật)
        - AUX3 (Safe Land) = 1000 → OFF
        - AUX4 (RTH) = 1000 → OFF

        Thứ tự: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]
        """
        return [
            self.RC_CENTER,         # Roll
            self.RC_CENTER,         # Pitch
            self.THROTTLE_MIN,      # Throttle (ga thấp nhất)
            self.RC_CENTER,         # Yaw
            self.AUX_DISARM,        # AUX1 = 1000 → DISARM
            self.AUX_ANGLE,         # AUX2 = 1500 → ANGLE mode (an toàn)
            self.AUX_SAFE_LAND_OFF, # AUX3 = 1000 → Safe Land OFF
            self.AUX_RTH_OFF,       # AUX4 = 1000 → RTH OFF
        ]

    def _transition(self, new_state: str):
        """Chuyển trạng thái state machine và thông báo UI."""
        self._state = new_state
        self.state_changed.emit(new_state)

    def _abort(self, reason: str):
        """Dừng khẩn cấp: cắt ga, DISARM, dừng timer, báo lỗi."""
        self._timer.stop()
        self._channels = self._safe_channels()
        self._send_rc()
        self._transition("IDLE")
        self.mode_activated.emit("")  # Tắt overlay
        self.error_occurred.emit(reason)
