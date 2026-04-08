"""
drone_state.py - Lưu trữ trạng thái hiện tại của Drone.

Module này là trung tâm chia sẻ dữ liệu giữa:
- WifiWorker (ghi dữ liệu telemetry mới nhất)
- FlightController (đọc trạng thái để ra quyết định bay)
- GCSApp (đọc trạng thái để cập nhật UI)

Tất cả truy cập diễn ra trên Main Thread (qua Signal/Slot), nên không cần lock.
"""


class DroneState:
    """
    Lưu trữ toàn bộ trạng thái drone nhận từ Flight Controller.

    Được cập nhật bởi GCSApp.update_telemetry_ui() mỗi khi nhận telemetry mới.
    Được đọc bởi FlightController để ra quyết định bay tự động.
    """

    def __init__(self):
        """Khởi tạo trạng thái mặc định (chưa có dữ liệu)."""
        # ── Từ MSP_ANALOG ──
        self.voltage: float = 0.0       # Điện áp pin (Volt)
        self.current: float = 0.0       # Dòng điện (Ampere)

        # ── Từ MSP_ATTITUDE ──
        self.roll: float = 0.0          # Góc Roll (độ)
        self.pitch: float = 0.0         # Góc Pitch (độ)
        self.yaw: float = 0.0           # Góc Yaw (0-360°)

        # ── Từ MSP_ALTITUDE ──
        self.altitude: float = 0.0      # Độ cao ước lượng (mét, từ barometer)
        self.vario: float = 0.0         # Tốc độ thẳng đứng (m/s)

        # ── Từ MSP_STATUS ──
        self.is_armed: bool = False     # Trạng thái ARM
        self.flight_mode_flags: int = 0 # Cờ chế độ bay (bitfield)

        # ── Từ MSP_MOTOR (chưa implement) ──
        self.motors: list[int] = [1000, 1000, 1000, 1000]

        # ── Từ MSP_RAW_GPS ── (Cảm biến GPS BZ 251)
        self.gps_fix_type: int = 0          # 0=No fix, 1=2D, 2=3D
        self.gps_num_sat: int = 0           # Số vệ tinh
        self.latitude: float = 0.0          # Vĩ độ (độ thập phân, VD: 21.0285)
        self.longitude: float = 0.0         # Kinh độ (độ thập phân, VD: 105.8542)
        self.gps_altitude: float = 0.0      # Độ cao GPS (mét)
        self.ground_speed: float = 0.0      # Tốc độ mặt đất (m/s)
        self.ground_course: float = 0.0     # Hướng di chuyển (độ, 0-360)
        self.gps_hdop: float = 0.0          # Độ chính xác ngang (HDOP)

        # ── Vị trí Home (Read-only từ FC, INAV tự chốt khi ARM) ──
        self.home_lat: float = 0.0          # Vĩ độ Home
        self.home_lon: float = 0.0          # Kinh độ Home
        self.has_home: bool = False         # Đã chốt Home chưa

        # ── Trạng thái kết nối ──
        self.is_connected: bool = False

        # ── Trạng thái mode bay hiện tại (cho emergency overlay) ──
        self.active_mode_name: str = ""     # Tên mode đang chạy (ARM, Takeoff, Mission...)

    def reset(self):
        """Reset toàn bộ trạng thái về mặc định (khi mất kết nối).

        ★ TASK-17: Viết reset thủ công thay vì gọi self.__init__() để:
        - Tránh stale reference nếu ai đó giữ ref cũ tới self.motors
        - Tránh bug nếu __init__ được thêm tham số trong tương lai
        - An toàn với sub-class override
        """
        # ── Từ MSP_ANALOG ──
        self.voltage = 0.0
        self.current = 0.0

        # ── Từ MSP_ATTITUDE ──
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # ── Từ MSP_ALTITUDE ──
        self.altitude = 0.0
        self.vario = 0.0

        # ── Từ MSP_STATUS ──
        self.is_armed = False
        self.flight_mode_flags = 0

        # ── Từ MSP_MOTOR ──
        self.motors[:] = [1000, 1000, 1000, 1000]  # In-place reset

        # ── Từ MSP_RAW_GPS ──
        self.gps_fix_type = 0
        self.gps_num_sat = 0
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_altitude = 0.0
        self.ground_speed = 0.0
        self.ground_course = 0.0
        self.gps_hdop = 0.0

        # ── Home ──
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.has_home = False

        # ── Kết nối ──
        self.is_connected = False

        # ── Mode bay ──
        self.active_mode_name = ""
