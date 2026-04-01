"""
flight_controller.py - Điều phối lệnh bay tự động.

Module này quản lý các chuỗi lệnh bay phức tạp:
- ARM / DISARM
- Takeoff & Hold (cất cánh và giữ độ cao)

Sử dụng QTimer để chạy state machine không chặn UI.
Gửi lệnh MSP qua WifiWorker.send_command().
Đọc trạng thái từ DroneState.

KHÔNG truy cập trực tiếp socket hay widget UI.
"""

import time
import struct
from PySide6.QtCore import QObject, QTimer, Signal

from comm.msp_parser import MSPParser, MSP_SET_RAW_RC


class FlightController(QObject):
    """
    Điều phối bay tự động: ARM, Takeoff, Hold.

    State machine chạy trên Main Thread qua QTimer (10Hz).
    Gửi MSP_SET_RAW_RC liên tục để duy trì quyền điều khiển RC trên FC.

    Signals:
        status_update(str): Cập nhật trạng thái cho UI
        takeoff_complete(): Báo hiệu đã đạt độ cao mục tiêu
        error_occurred(str): Thông báo lỗi
        state_changed(str): Thay đổi trạng thái state machine
    """

    # ── Signals ──
    status_update = Signal(str)
    takeoff_complete = Signal()
    error_occurred = Signal(str)
    state_changed = Signal(str)

    # ── Hằng số bay ──
    ALTITUDE_TOLERANCE = 0.3        # ±0.3m (chấp nhận được với barometer)
    THROTTLE_MIN = 1000             # Ga tối thiểu (μs)
    THROTTLE_HOVER = 1500           # Ga hover ước lượng (μs)
    THROTTLE_MAX_CLIMB = 1600       # Ga tối đa khi leo (μs) — giới hạn an toàn
    THROTTLE_RAMP_STEP = 10         # Tăng ga mỗi tick nhanh (μs)
    THROTTLE_FINE_STEP = 5          # Tăng ga mỗi tick chậm (μs)
    RC_CENTER = 1500                # Giá trị trung tâm kênh RC

    # ── Hằng số kênh AUX (theo cấu hình INAV Modes tab) ──
    # ARM:          CH5 (AUX1), dải kích hoạt 1750 - 2100
    # ANGLE:        CH6 (AUX2), dải kích hoạt 1400 - 1600
    # NAV ALTHOLD:  CH6 (AUX2), dải kích hoạt 1900 - 2100
    AUX_DISARM = 1000               # AUX1 tắt → DISARM (dưới 1750)
    AUX_ARM = 1900                  # AUX1 bật → ARM (giữa dải 1750-2100)
    AUX_ANGLE = 1500                # AUX2 → ANGLE mode (giữa dải 1400-1600)
    AUX_NAV_ALTHOLD = 2000          # AUX2 → NAV ALTHOLD (giữa dải 1900-2100)

    # ── Timeout ──
    TICK_INTERVAL_MS = 100          # Tần số state machine (10Hz)
    RC_LINK_WAIT_S = 1.5            # Chờ INAV reset ARM_SWITCH safety (giây)
                                    # INAV cần thấy AUX1=DISARM liên tục ≥1s trước khi cho phép ARM
    ARM_TIMEOUT_S = 5.0             # Timeout chờ ARM (giây) — tăng lên để chắc chắn
    CLIMB_TIMEOUT_S = 15.0          # Timeout đạt độ cao (giây)
    ARMED_WAIT_S = 0.5              # Chờ ổn định sau ARM (giây)

    # ── Chỉ số kênh RC (thứ tự AETR của INAV) ──
    CH_ROLL = 0        # Aileron
    CH_PITCH = 1       # Elevator
    CH_THROTTLE = 2    # Throttle
    CH_YAW = 3         # Rudder
    CH_AUX1 = 4        # ARM (CH5)
    CH_AUX2 = 5        # ANGLE / NAV_ALTHOLD (CH6)

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
        """ARM drone bằng cách bật AUX1 (đợi RC link ổn định rồi mới bật)."""
        if not self._worker:
            self.error_occurred.emit("Chưa kết nối — không thể ARM")
            return
        if self._drone_state.is_armed:
            self.status_update.emit("Drone đã ARM sẵn")
            return

        self._channels = self._safe_channels() # Bắt đầu với DISARM
        self._send_rc()
        self._arm_start_time = time.time()
        self._transition("WAIT_RC_LINK_ARM_ONLY")
        self._timer.start(self.TICK_INTERVAL_MS)
        self.status_update.emit("Khởi tạo kết nối RC...")

    def disarm(self):
        """DISARM drone — tắt AUX, hạ ga, dừng state machine."""
        self._timer.stop()
        self._channels = self._safe_channels()
        self._send_rc()
        self._transition("IDLE")
        self.status_update.emit("Đã gửi lệnh DISARM")

    def takeoff_and_hold(self, target_alt: float = 3.0):
        """
        Chuỗi tự động: ARM → Tăng ga từ từ → Giữ độ cao.

        Args:
            target_alt: Độ cao mục tiêu (mét). Mặc định 3.0m.
        """
        if not self._worker:
            self.error_occurred.emit("Chưa kết nối — không thể cất cánh")
            return

        self._target_altitude = target_alt
        self._channels = self._safe_channels()
        self._transition("PRE_ARM_CHECK")
        self._timer.start(self.TICK_INTERVAL_MS)

    def abort(self):
        """Dừng khẩn cấp — cắt ga, DISARM, dừng timer."""
        self._abort("Người dùng yêu cầu dừng khẩn cấp")

    @property
    def state(self) -> str:
        """Trạng thái hiện tại của state machine."""
        return self._state

    @property
    def is_active(self) -> bool:
        """True nếu đang thực hiện chuỗi bay (không phải IDLE)."""
        return self._state != "IDLE"

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

        elif self._state == "THROTTLE_RAMP":
            self._handle_throttle_ramp()

        elif self._state == "ALT_HOLD_ENGAGE":
            self._handle_alt_hold_engage()

        elif self._state == "HOLDING":
            self._handle_holding()

    # ══════════════════════════════════════════════
    # XỬ LÝ TỪNG TRẠNG THÁI
    # ══════════════════════════════════════════════

    def _handle_pre_arm_check(self):
        """Kiểm tra điều kiện an toàn trước khi ARM."""
        if self._drone_state.is_armed:
            # Đã ARM sẵn → skip thẳng sang cất cánh
            self.status_update.emit("Drone đã ARM — bắt đầu cất cánh")
            self._channels[self.CH_AUX1] = self.AUX_ARM
            self._climb_start_time = time.time()
            self._transition("THROTTLE_RAMP")
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
                "• Kiểm tra tab Modes: ARM phải gán đúng CH5, dải 1750-2100"
            )

    def _handle_arming_only(self):
        """ARM riêng lẻ (không takeoff) — chờ xác nhận rồi dừng timer."""
        self._channels[self.CH_AUX1] = self.AUX_ARM
        self._send_rc()

        if self._drone_state.is_armed:
            self._timer.stop()
            self._transition("IDLE")
            self.status_update.emit("ARM thành công ✓")
        elif time.time() - self._arm_start_time > self.ARM_TIMEOUT_S:
            self._timer.stop()
            self._channels = self._safe_channels()
            self._send_rc()
            self._transition("IDLE")
            self.error_occurred.emit(
                f"ARM thất bại — timeout {self.ARM_TIMEOUT_S:.0f}s.\n"
                "Nguyên nhân thường gặp:\n"
                "• ARMING_DISABLED_ARM_SWITCH: Chờ thêm 2s rồi thử lại\n"
                "• ARMING_DISABLED_DSHOT_BEEPER: Tắt DShot Beeper trong INAV Configurator\n"
                "• Kiểm tra tab Modes: ARM phải gán đúng CH5, dải 1750-2100"
            )

    def _handle_armed_wait(self):
        """Chờ drone ổn định 500ms sau khi ARM trước khi tăng ga."""
        self._send_rc()  # Duy trì tín hiệu RC

        if time.time() - self._armed_wait_start > self.ARMED_WAIT_S:
            self._climb_start_time = time.time()
            self._transition("THROTTLE_RAMP")
            self.status_update.emit("Bắt đầu tăng ga — cất cánh...")

    def _handle_throttle_ramp(self):
        """Tăng dần throttle và theo dõi altitude từ barometer."""
        current_throttle = self._channels[self.CH_THROTTLE]

        # Tăng ga từ từ
        if current_throttle < self.THROTTLE_HOVER:
            # Giai đoạn 1: Ramp nhanh tới hover point (10μs/tick)
            self._channels[self.CH_THROTTLE] = min(
                current_throttle + self.THROTTLE_RAMP_STEP,
                self.THROTTLE_HOVER
            )
        elif current_throttle < self.THROTTLE_MAX_CLIMB:
            # Giai đoạn 2: Tăng chậm nếu chưa đạt độ cao (5μs/tick)
            self._channels[self.CH_THROTTLE] = min(
                current_throttle + self.THROTTLE_FINE_STEP,
                self.THROTTLE_MAX_CLIMB
            )

        self._send_rc()

        # Cập nhật UI với thông tin thời gian thực
        alt = self._drone_state.altitude
        throttle = self._channels[self.CH_THROTTLE]
        self.status_update.emit(
            f"Đang leo — Alt: {alt:.1f}m / {self._target_altitude:.0f}m | "
            f"Throttle: {throttle}μs"
        )

        # Kiểm tra đạt độ cao mục tiêu
        if alt >= self._target_altitude - self.ALTITUDE_TOLERANCE:
            self._transition("ALT_HOLD_ENGAGE")
        elif time.time() - self._climb_start_time > self.CLIMB_TIMEOUT_S:
            self._abort(
                f"Không đạt độ cao mục tiêu ({alt:.1f}m / "
                f"{self._target_altitude:.0f}m) — timeout {self.CLIMB_TIMEOUT_S:.0f}s"
            )

    def _handle_alt_hold_engage(self):
        """Bật NAV_ALTHOLD khi đạt độ cao mục tiêu."""
        # Bật ALTHOLD qua AUX2 (chuyển từ ANGLE → ALTHOLD), throttle về center
        self._channels[self.CH_THROTTLE] = self.RC_CENTER
        self._channels[self.CH_AUX2] = self.AUX_NAV_ALTHOLD
        self._send_rc()

        alt = self._drone_state.altitude
        self.status_update.emit(
            f"Giữ độ cao tại {alt:.1f}m ✓ — NAV_ALTHOLD đã bật"
        )
        self._transition("HOLDING")
        self.takeoff_complete.emit()

    def _handle_holding(self):
        """Duy trì trạng thái hover — gửi RC liên tục để giữ MSP override."""
        self._send_rc()

        # Theo dõi an toàn: nếu FC tự DISARM → dừng
        if not self._drone_state.is_armed:
            self._timer.stop()
            self._transition("IDLE")
            self.status_update.emit("Drone đã DISARM — kết thúc hold")

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

    def _safe_channels(self) -> list[int]:
        """
        Trả về bộ kênh RC an toàn: throttle thấp, DISARM, ANGLE mode bật.

        QUAN TRỌNG: AUX2 (CH6) mặc định ở AUX_ANGLE (1500) để drone
        luôn ở chế độ tự cân bằng. Nếu để 1000 (ACRO) drone sẽ lật!
        """
        return [
            self.RC_CENTER,    # Roll
            self.RC_CENTER,    # Pitch
            self.THROTTLE_MIN, # Throttle
            self.RC_CENTER,    # Yaw
            self.AUX_DISARM,   # AUX1 (ARM off — dưới 1750)
            self.AUX_ANGLE,    # AUX2 (ANGLE mode — 1500, trong dải 1400-1600)
            self.RC_CENTER,    # AUX3
            self.RC_CENTER,    # AUX4
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
        self.error_occurred.emit(reason)
