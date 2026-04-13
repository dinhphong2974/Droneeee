"""
drone_state.py - Lưu trữ trạng thái hiện tại của Drone.

Module này là trung tâm chia sẻ dữ liệu giữa:
- WifiWorker (ghi dữ liệu telemetry mới nhất)
- FlightController (đọc trạng thái để ra quyết định bay)
- GCSApp (đọc trạng thái để cập nhật UI)

Tất cả truy cập diễn ra trên Main Thread (qua Signal/Slot), nên không cần lock.
"""
import collections


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

        # ── Từ MSP_SONAR_ALTITUDE ── (Cảm biến LiDAR MTF-02 AIO)
        self.surface_altitude: float = -1.0     # Khoảng cách tới mặt đất (m), -1 = ngoài tầm
        self.surface_quality: int = 0           # Chất lượng đo rangefinder (0-255)
        self.has_valid_surface: bool = False     # True khi LiDAR có dữ liệu hợp lệ (≤2.5m)

        # ── Từ MSP_STATUS_EX ── (Sensor Health — Micoair MTF-02 AIO)
        self.sensor_opflow: bool = False        # True khi Optical Flow sensor hoạt động
        self.sensor_rangefinder: bool = False    # True khi Rangefinder/LiDAR hoạt động
        self.sensor_mag: bool = False            # True khi Compass (la bàn) hoạt động
        self.sensor_gps: bool = False            # True khi GPS module phát hiện
        self.sensor_baro: bool = False           # True khi Barometer hoạt động
        self.system_load: int = 0                # CPU load FC (%)

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
        self.ping_rtt_ms: int = -1          # RTT GCS↔ESP32 (ms), -1 = chưa đo

        # ── Trạng thái mode bay hiện tại (cho emergency overlay) ──
        self.active_mode_name: str = ""     # Tên mode đang chạy (ARM, Takeoff, Mission...)
        # ── Lịch sử GPS ──
        self._gps_history: collections.deque = collections.deque(maxlen=20)

    def record_gps_history(self):
        """Lưu tọa độ hiện tại vào hàng đợi để tính trung bình, loại bỏ sai số nhấp nháy."""
        if self.gps_fix_type >= 2 and self.gps_num_sat >= 6 and self.latitude != 0.0 and self.longitude != 0.0:
            self._gps_history.append((self.latitude, self.longitude))

    def get_stable_gps(self) -> tuple[float, float] | None:
        """Trả về tọa độ trung bình, hoặc None nếu chưa đủ mẫu (ít nhất 10 mẫu)."""
        if len(self._gps_history) < 10:
            return None
        
        avg_lat = sum(p[0] for p in self._gps_history) / len(self._gps_history)
        avg_lon = sum(p[1] for p in self._gps_history) / len(self._gps_history)
        return (avg_lat, avg_lon)

    def reset(self):
        """Reset toàn bộ trạng thái về mặc định (khi mất kết nối)."""
        self.__init__()
