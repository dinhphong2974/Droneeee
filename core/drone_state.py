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

        # ── Trạng thái kết nối ──
        self.is_connected: bool = False

    def reset(self):
        """Reset toàn bộ trạng thái về mặc định (khi mất kết nối)."""
        self.__init__()
