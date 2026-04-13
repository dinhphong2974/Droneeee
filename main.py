"""
main.py - Điểm khởi chạy ứng dụng Drone Ground Control Station.

File này CHỈ đóng vai trò:
1. Khởi tạo cửa sổ giao diện (MainWindow)
2. Kết nối Signal/Slot giữa các module (WifiWorker ↔ UI)
3. Quản lý vòng đời ứng dụng (mở/đóng kết nối)

Kiến trúc module:
    main.py → khởi tạo + điều phối Signal/Slot
    ui/main_window.py → cửa sổ chính (Nav Rail + Stacked Pages)
    ui/dashboard_tab.py → trang Attitude 3D + Telemetry + Motors
    ui/manual_control_tab.py → trang điều khiển thủ công
    ui/mission_tab.py → trang lộ trình bay + OpenStreetMap
    ui/config_tab.py → trang cấu hình
    ui/emergency_overlay.py → overlay cảnh báo khẩn cấp
    comm/wifi_client.py → kết nối TCP thô
    comm/wifi_worker.py → QThread chạy ngầm
    comm/msp_parser.py → giải mã giao thức MSP
    core/drone_state.py → trạng thái drone chia sẻ
    core/flight_controller.py → state machine bay tự động
"""

import sys
from PySide6.QtWidgets import (QApplication, QDialog, QVBoxLayout,
                                QHBoxLayout, QLineEdit, QPushButton, QLabel,
                                QMessageBox, QDoubleSpinBox)
from ui.main_window import MainWindow
from ui.emergency_overlay import EmergencyOverlay
from comm.wifi_worker import WifiWorker
from core.drone_state import DroneState
from core.flight_controller import FlightController

# ── Thông số pin Lipo 6S ──
LIPO_6S_MIN_VOLTAGE = 19.8  # Điện áp rỗng (V)
LIPO_6S_MAX_VOLTAGE = 25.2  # Điện áp đầy (V)


class ConnectionDialog(QDialog):
    """Cửa sổ cấu hình kết nối Wifi tới ESP32."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Kết nối ESP32 - Wifi")
        self.setFixedSize(320, 150)
        self.setup_ui()

        self.selected_ip = None
        self.selected_port = None
        self.is_mock_selected = False

    def setup_ui(self):
        """Xây dựng giao diện dialog nhập IP/Port."""
        layout = QVBoxLayout(self)

        # Nhập IP của ESP32
        h_ip = QHBoxLayout()
        h_ip.addWidget(QLabel("IP ESP32:"))
        self.input_ip = QLineEdit("192.168.4.1")  # IP mặc định của ESP32 AP
        h_ip.addWidget(self.input_ip)
        layout.addLayout(h_ip)

        # Nhập Port giao tiếp
        h_port = QHBoxLayout()
        h_port.addWidget(QLabel("Port TCP:"))
        self.input_port = QLineEdit("8080")
        h_port.addWidget(self.input_port)
        layout.addLayout(h_port)

        # Các nút bấm
        h_btns = QHBoxLayout()
        self.btn_connect = QPushButton("🛜 Kết Nối Wifi")
        self.btn_connect.clicked.connect(self.accept_connection)

        self.btn_mock = QPushButton("🧪 Mock Test")
        self.btn_mock.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.btn_mock.clicked.connect(self.accept_mock)

        h_btns.addWidget(self.btn_connect)
        h_btns.addWidget(self.btn_mock)
        layout.addLayout(h_btns)

    def accept_connection(self):
        """Xác nhận kết nối thật: validate IP và Port trước khi đóng dialog."""
        ip = self.input_ip.text().strip()
        port = self.input_port.text().strip()
        if ip and port.isdigit():
            self.selected_ip = ip
            self.selected_port = port
            self.is_mock_selected = False
            self.accept()
        else:
            QMessageBox.warning(self, "Lỗi", "Vui lòng nhập IP và Port hợp lệ!")

    def accept_mock(self):
        """Chọn chế độ giả lập (Mock Test) để test UI không cần phần cứng."""
        self.is_mock_selected = True
        self.accept()


class TakeoffDialog(QDialog):
    """
    Dialog nhập độ cao mong muốn khi cất cánh.

    Cho phép người dùng chọn độ cao (1-50m) trước khi drone cất cánh.
    Kết hợp GPS BZ 251 để giữ vị trí chính xác (Position Hold).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("🚀 Cấu hình Takeoff")
        self.setFixedSize(380, 200)
        self.target_altitude = 3.0  # Mặc định 3m
        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng giao diện dialog nhập độ cao."""
        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        # Tiêu đề
        lbl_title = QLabel("⚠️ Drone sẽ tự động ARM, cất cánh và giữ vị trí.\n"
                          "Đảm bảo khu vực an toàn trước khi tiếp tục!")
        lbl_title.setWordWrap(True)
        lbl_title.setStyleSheet("color: #FFD54F; font-weight: bold; font-size: 12px; border: none;")
        layout.addWidget(lbl_title)

        # Nhập độ cao
        alt_layout = QHBoxLayout()
        lbl_alt = QLabel("Độ cao mục tiêu:")
        lbl_alt.setStyleSheet("font-weight: bold; font-size: 13px;")
        alt_layout.addWidget(lbl_alt)

        self.spin_altitude = QDoubleSpinBox()
        self.spin_altitude.setRange(1.0, 50.0)
        self.spin_altitude.setValue(3.0)
        self.spin_altitude.setSuffix(" mét")
        self.spin_altitude.setDecimals(1)
        self.spin_altitude.setSingleStep(0.5)
        self.spin_altitude.setStyleSheet(
            "QDoubleSpinBox { background-color: #252540; color: #d0d0e8; "
            "border: 1px solid #2a2a4a; border-radius: 6px; padding: 8px; "
            "font-size: 16px; font-weight: bold; }"
        )
        alt_layout.addWidget(self.spin_altitude)
        layout.addLayout(alt_layout)

        # Thông tin GPS
        self.lbl_gps_info = QLabel("📡 GPS: Đang chờ dữ liệu...")
        self.lbl_gps_info.setStyleSheet("color: #808098; font-size: 11px;")
        layout.addWidget(self.lbl_gps_info)

        # Nút xác nhận
        btn_layout = QHBoxLayout()

        btn_cancel = QPushButton("❌ Hủy")
        btn_cancel.setStyleSheet(
            "QPushButton { background-color: #F44336; color: white; font-weight: bold; "
            "border-radius: 6px; padding: 10px; font-size: 13px; }"
            "QPushButton:hover { background-color: #E53935; }"
        )
        btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(btn_cancel)

        btn_confirm = QPushButton("🚀 Cất cánh")
        btn_confirm.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; "
            "border-radius: 6px; padding: 10px; font-size: 13px; }"
            "QPushButton:hover { background-color: #43A047; }"
        )
        btn_confirm.clicked.connect(self._on_confirm)
        btn_layout.addWidget(btn_confirm)

        layout.addLayout(btn_layout)

    def _on_confirm(self):
        """Lưu độ cao đã chọn và đóng dialog."""
        self.target_altitude = self.spin_altitude.value()
        self.accept()

    def update_gps_info(self, fix_type: int, num_sat: int, lat: float, lon: float):
        """
        Cập nhật thông tin GPS hiện tại trên dialog.

        Args:
            fix_type: 0=No fix, 1=2D, 2=3D
            num_sat: Số vệ tinh
            lat, lon: Tọa độ hiện tại
        """
        fix_text = "No Fix" if fix_type == 0 else ("2D" if fix_type == 1 else "3D ✓")
        color = "#4CAF50" if fix_type >= 2 else "#F44336"
        self.lbl_gps_info.setText(
            f"📡 GPS: {fix_text} | {num_sat} sats | ({lat:.6f}, {lon:.6f})"
        )
        self.lbl_gps_info.setStyleSheet(f"color: {color}; font-size: 11px;")


class GCSApp(MainWindow):
    """
    Lớp ứng dụng chính — kế thừa MainWindow và thêm logic điều phối.

    MainWindow xây dựng UI layout, GCSApp kết nối Signal/Slot:
    - WifiWorker.connection_status → handle_connection_status
    - WifiWorker.telemetry_data → update_telemetry_ui
    - FlightController.state_changed → _on_flight_state_changed
    - FlightController.mode_activated → emergency overlay
    - btn_disconnect.clicked → toggle_connection
    """

    def __init__(self):
        super().__init__()

        # Gắn sự kiện cho nút kết nối/ngắt kết nối ở Top Bar
        self.btn_disconnect.clicked.connect(self.toggle_connection)

        self.worker: WifiWorker | None = None

        # ── Trạng thái drone chia sẻ ──
        self.drone_state = DroneState()

        # ── Flight Controller (state machine bay tự động) ──
        self.flight_controller = FlightController(self.drone_state, parent=self)
        self.flight_controller.status_update.connect(self._on_flight_status)
        self.flight_controller.takeoff_complete.connect(self._on_takeoff_complete)
        self.flight_controller.error_occurred.connect(self._on_flight_error)
        self.flight_controller.state_changed.connect(self._on_flight_state_changed)
        self.flight_controller.mode_activated.connect(self._on_mode_activated)

        # ── Emergency Overlay (cảnh báo khẩn cấp) ──
        self.emergency_overlay = EmergencyOverlay(self)
        self.emergency_overlay.btn_emergency_disarm.clicked.connect(self._emergency_force_disarm)
        self.emergency_overlay.btn_emergency_safe_land.clicked.connect(self._emergency_force_safe_land)

        # ── Kết nối nút điều khiển bay (từ ManualControlTab) ──
        mc = self.manual_control_tab
        mc.btn_arm.clicked.connect(self.flight_controller.arm)
        mc.btn_disarm.clicked.connect(self.flight_controller.disarm)
        mc.btn_takeoff_hold.clicked.connect(self._confirm_takeoff)
        mc.btn_rth.clicked.connect(self.flight_controller.rth)

        # ── Kết nối nút mission (từ MissionTab) ──
        mt = self.mission_tab
        mt.btn_upload.clicked.connect(self._upload_mission)
        mt.btn_start_mission_tab.clicked.connect(self._start_mission)
        mt.btn_stop_mission.clicked.connect(self._stop_mission)

        # ── Cờ cảnh báo khoảng cách (tránh hiện dialog lặp) ──
        self._distance_warning_shown = False

        # Khởi chạy ở trạng thái chưa có mạng
        self.set_ui_state_na()

    # ══════════════════════════════════════════════
    # QUẢN LÝ KẾT NỐI
    # ══════════════════════════════════════════════

    def toggle_connection(self):
        """Xử lý nút bấm Top Bar: Mở bảng kết nối hoặc ngắt kết nối an toàn."""
        if self.worker is not None:
            # Đang có kết nối → NGẮT KẾT NỐI
            self.flight_controller.abort()  # Dừng bay tự động nếu đang chạy
            self.flight_controller.set_worker(None)
            self.worker.stop()
            self.worker = None
            self.drone_state.reset()
            self.set_ui_state_na()
            self.emergency_overlay.hide_overlay()
            self.setWindowTitle("Drone Ground Station - Đã ngắt kết nối")
            QMessageBox.information(self, "Thông báo", "Đã ngắt kết nối với thiết bị an toàn.")
        else:
            # Chưa có kết nối → MỞ DIALOG KẾT NỐI
            dialog = ConnectionDialog(self)
            if dialog.exec() == QDialog.Accepted:
                self.start_connection(dialog.selected_ip, dialog.selected_port, dialog.is_mock_selected)

    def start_connection(self, ip, port, is_mock):
        """
        Khởi tạo WifiWorker và kết nối Signal/Slot.

        Đây là điểm duy nhất kết nối các module lại với nhau.
        """
        self.worker = WifiWorker(ip=ip, port=port, is_mock=is_mock)

        # Kết nối Signal từ worker tới các slot xử lý trên UI
        self.worker.connection_status.connect(self.handle_connection_status)
        self.worker.telemetry_data.connect(self.update_telemetry_ui)
        self.worker.ping_updated.connect(self._on_ping_updated)
        self.worker.command_acked.connect(self._on_command_acked)

        # Cấp worker cho FlightController để gửi lệnh
        self.flight_controller.set_worker(self.worker)

        # Khởi chạy thread ngầm
        self.worker.start()

        self._distance_warning_shown = False

    # ══════════════════════════════════════════════
    # SLOT: XỬ LÝ TÍN HIỆU TỪ WORKER
    # ══════════════════════════════════════════════

    def handle_connection_status(self, success: bool, message: str):
        """Slot xử lý tín hiệu trạng thái mạng từ WifiWorker."""
        if success:
            self.setWindowTitle(f"Drone Ground Station - {message}")
            self.drone_state.is_connected = True
            self.enable_ui_components()
        else:
            # Kết nối thất bại HOẶC bị đứt giữa chừng
            self.flight_controller.abort()
            self.flight_controller.set_worker(None)
            self.worker = None
            self.drone_state.reset()
            self.set_ui_state_na()
            self.emergency_overlay.hide_overlay()
            self.setWindowTitle("Drone Ground Station - Mất kết nối")
            QMessageBox.warning(self, "Cảnh báo Mạng", message)

    def update_telemetry_ui(self, data: dict):
        """
        Slot cập nhật dữ liệu telemetry từ FC lên giao diện.

        Nhận dict đã giải mã từ WifiWorker qua Signal,
        chỉ cập nhật các trường có dữ liệu mới.
        """
        # Shortcut truy cập tab dashboard
        dash = self.dashboard_tab

        # ── Cập nhật điện áp pin Lipo 6S ──
        if "voltage" in data:
            v = data["voltage"]
            self.drone_state.voltage = v
            percent = int(((v - LIPO_6S_MIN_VOLTAGE) / (LIPO_6S_MAX_VOLTAGE - LIPO_6S_MIN_VOLTAGE)) * 100)
            percent = max(0, min(100, percent))

            self.lbl_batt_volt.setText(f"{v:.2f} V")
            self.lbl_batt_perc.setText(f"{percent} %")
            self.bar_battery_volt.setValue(percent)

            color = "#4CAF50" if percent > 50 else ("#FFC107" if percent > 20 else "#F44336")
            self.bar_battery_volt.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; }}")

        if "current" in data:
            self.drone_state.current = data["current"]

        # ── Cập nhật góc nghiêng + đồng bộ vào drone_state ──
        if "roll" in data:
            self.drone_state.roll = data["roll"]
            dash.val_roll.setText(f"{data['roll']:.1f}°")
        if "pitch" in data:
            self.drone_state.pitch = data["pitch"]
            dash.val_pitch.setText(f"{data['pitch']:.1f}°")
        if "yaw" in data:
            self.drone_state.yaw = data["yaw"]
            dash.val_yaw.setText(f"{data['yaw']:.1f}°")

        # ── Cập nhật Attitude 3D Widget (Panda3D) ──
        if any(k in data for k in ("roll", "pitch", "yaw")):
            dash.widget_3d_attitude.update_attitude(
                self.drone_state.roll,
                self.drone_state.pitch,
                self.drone_state.yaw,
            )

        # ── Cập nhật vòng tua động cơ 1960kv ──
        if "motor1" in data:
            dash.val_motor1.setText(str(data["motor1"]))
            dash.bar_motor1.setValue(data["motor1"])
            dash.val_motor2.setText(str(data["motor2"]))
            dash.bar_motor2.setValue(data["motor2"])
            dash.val_motor3.setText(str(data["motor3"]))
            dash.bar_motor3.setValue(data["motor3"])
            dash.val_motor4.setText(str(data["motor4"]))
            dash.bar_motor4.setValue(data["motor4"])

        # ── Cập nhật độ cao từ barometer (MSP_ALTITUDE) ──
        if "altitude" in data:
            alt = data["altitude"]
            self.drone_state.altitude = alt
            dash.val_alt.setText(f"{alt:.1f} m")
            dash.val_alt.setStyleSheet("color: #2196F3; font-weight: bold;")

        if "vario" in data:
            self.drone_state.vario = data["vario"]

        # ── Cập nhật trạng thái ARM từ FC (MSP_STATUS) ──
        if "is_armed" in data:
            self.drone_state.is_armed = data["is_armed"]
            if data["is_armed"]:
                dash.val_armed.setText("ARMED")
                dash.val_armed.setStyleSheet("color: red; font-weight: bold;")

                # Hiện emergency overlay khi drone ARMED
                if not self.emergency_overlay.isVisible():
                    self.emergency_overlay.show_with_mode("Armed")

                # Chốt Home position khi ARM lần đầu (read-only từ GPS)
                if not self.drone_state.has_home and self.drone_state.latitude != 0.0:
                    self.drone_state.home_lat = self.drone_state.latitude
                    self.drone_state.home_lon = self.drone_state.longitude
                    self.drone_state.has_home = True
                    self.mission_tab.update_home_position(
                        self.drone_state.home_lat, self.drone_state.home_lon
                    )
            else:
                dash.val_armed.setText("DISARMED")
                dash.val_armed.setStyleSheet("color: #4CAF50; font-weight: bold;")

                # Ẩn emergency overlay khi DISARMED (trừ khi FC đang idle)
                if self.emergency_overlay.isVisible() and not self.flight_controller.is_active:
                    self.emergency_overlay.hide_overlay()

        if "flight_mode_flags" in data:
            self.drone_state.flight_mode_flags = data["flight_mode_flags"]

        # ══════════════════════════════════════════════
        # CẬP NHẬT GPS DATA (từ GPS BZ 251 qua MSP_RAW_GPS)
        # ══════════════════════════════════════════════

        if "gps_fix_type" in data:
            self.drone_state.gps_fix_type = data["gps_fix_type"]
            fix_text = "No Fix" if data["gps_fix_type"] == 0 else (
                "2D" if data["gps_fix_type"] == 1 else "3D ✓"
            )
            fix_color = "#4CAF50" if data["gps_fix_type"] >= 2 else "#F44336"
            dash.val_gps_fix.setText(fix_text)
            dash.val_gps_fix.setStyleSheet(f"color: {fix_color}; font-weight: bold;")

        if "gps_num_sat" in data:
            self.drone_state.gps_num_sat = data["gps_num_sat"]
            dash.val_sats.setText(str(data["gps_num_sat"]))
            sat_color = "#4CAF50" if data["gps_num_sat"] >= 6 else "#FFC107"
            dash.val_sats.setStyleSheet(f"color: {sat_color}; font-weight: bold;")

        if "latitude" in data:
            self.drone_state.latitude = data["latitude"]
            dash.val_lat.setText(f"{data['latitude']:.6f}")
            dash.val_lat.setStyleSheet("color: #2196F3; font-weight: bold;")

        if "longitude" in data:
            self.drone_state.longitude = data["longitude"]
            dash.val_lon.setText(f"{data['longitude']:.6f}")
            dash.val_lon.setStyleSheet("color: #2196F3; font-weight: bold;")

        if "ground_speed" in data:
            self.drone_state.ground_speed = data["ground_speed"]
            dash.val_spd.setText(f"{data['ground_speed']:.1f} m/s")
            dash.val_spd.setStyleSheet("color: #2196F3; font-weight: bold;")

        if "gps_altitude" in data:
            self.drone_state.gps_altitude = data["gps_altitude"]

        # ── Cập nhật vị trí drone trên bản đồ mission (real-time, no reload) ──
        if "latitude" in data and "longitude" in data:
            lat = data["latitude"]
            lon = data["longitude"]
            self.drone_state.record_gps_history()
            heading = self.drone_state.yaw  # Heading từ MSP_ATTITUDE
            if lat != 0.0 or lon != 0.0:
                self.mission_tab.update_drone_position(lat, lon, heading)

                # Kiểm tra cảnh báo khoảng cách
                self._check_distance_safety()

        # ══════════════════════════════════════════════
        # CẬP NHẬT LiDAR DATA (từ MTF-02 qua MSP_SONAR_ALTITUDE)
        # ══════════════════════════════════════════════

        if "surface_altitude" in data:
            s_alt = data["surface_altitude"]
            self.drone_state.surface_altitude = s_alt
            self.drone_state.has_valid_surface = s_alt >= 0

            if s_alt >= 0:
                dash.val_surface_alt.setText(f"{s_alt:.2f} m")
                dash.val_surface_alt.setStyleSheet("color: #00E676; font-weight: bold;")
            else:
                dash.val_surface_alt.setText("Out of Range")
                dash.val_surface_alt.setStyleSheet("color: #808098; font-weight: bold;")

        if "surface_quality" in data:
            s_qual = data["surface_quality"]
            self.drone_state.surface_quality = s_qual
            qual_color = "#00E676" if s_qual > 100 else ("#FFC107" if s_qual > 0 else "#F44336")
            dash.val_lidar_qual.setText(str(s_qual))
            dash.val_lidar_qual.setStyleSheet(f"color: {qual_color}; font-weight: bold;")

        # ══════════════════════════════════════════════
        # CẬP NHẬT SENSOR HEALTH CARD (từ MSP_STATUS_EX)
        # ══════════════════════════════════════════════

        if "sensor_opflow" in data:
            self.drone_state.sensor_opflow = data["sensor_opflow"]
            if data["sensor_opflow"]:
                dash.val_opflow.setText("✅ Active")
                dash.val_opflow.setStyleSheet("color: #00E676; font-weight: bold;")
                dash.bar_sensor_opflow.setValue(100)
                dash.bar_sensor_opflow.setStyleSheet(
                    "QProgressBar { border: 1px solid #2a2a4a; border-radius: 4px; background-color: #252540; }"
                    "QProgressBar::chunk { border-radius: 3px; background-color: #00E676; }"
                )
                dash.val_sensor_opflow.setText("Active")
                dash.val_sensor_opflow.setStyleSheet("color: #00E676; font-weight: bold;")
            else:
                dash.val_opflow.setText("⚠️ Inactive")
                dash.val_opflow.setStyleSheet("color: #FFC107; font-weight: bold;")
                dash.bar_sensor_opflow.setValue(0)
                dash.val_sensor_opflow.setText("Inactive")
                dash.val_sensor_opflow.setStyleSheet("color: #FFC107; font-weight: bold;")

        if "sensor_rangefinder" in data:
            self.drone_state.sensor_rangefinder = data["sensor_rangefinder"]
            if data["sensor_rangefinder"]:
                dash.bar_sensor_lidar.setValue(100)
                dash.bar_sensor_lidar.setStyleSheet(
                    "QProgressBar { border: 1px solid #2a2a4a; border-radius: 4px; background-color: #252540; }"
                    "QProgressBar::chunk { border-radius: 3px; background-color: #00E676; }"
                )
                dash.val_sensor_lidar.setText("Active")
                dash.val_sensor_lidar.setStyleSheet("color: #00E676; font-weight: bold;")
            else:
                dash.bar_sensor_lidar.setValue(0)
                dash.val_sensor_lidar.setText("Off")
                dash.val_sensor_lidar.setStyleSheet("color: #808098; font-weight: bold;")

        if "sensor_mag" in data:
            self.drone_state.sensor_mag = data["sensor_mag"]
            if data["sensor_mag"]:
                dash.bar_sensor_mag.setValue(100)
                dash.bar_sensor_mag.setStyleSheet(
                    "QProgressBar { border: 1px solid #2a2a4a; border-radius: 4px; background-color: #252540; }"
                    "QProgressBar::chunk { border-radius: 3px; background-color: #00E676; }"
                )
                dash.val_sensor_mag.setText("OK")
                dash.val_sensor_mag.setStyleSheet("color: #00E676; font-weight: bold;")
            else:
                dash.bar_sensor_mag.setValue(0)
                dash.val_sensor_mag.setText("Off")
                dash.val_sensor_mag.setStyleSheet("color: #808098; font-weight: bold;")

        if "sensor_gps" in data:
            self.drone_state.sensor_gps = data["sensor_gps"]

        if "sensor_baro" in data:
            self.drone_state.sensor_baro = data["sensor_baro"]

        # Cập nhật GPS sensor health bar dựa trên số vệ tinh
        if "gps_num_sat" in data:
            num_sat = data["gps_num_sat"]
            gps_quality = min(100, int(num_sat / 12 * 100))
            dash.bar_sensor_gps.setValue(gps_quality)
            gps_color = "#00E676" if num_sat >= 8 else ("#FFC107" if num_sat >= 4 else "#F44336")
            dash.bar_sensor_gps.setStyleSheet(
                f"QProgressBar {{ border: 1px solid #2a2a4a; border-radius: 4px; background-color: #252540; }}"
                f"QProgressBar::chunk {{ border-radius: 3px; background-color: {gps_color}; }}"
            )
            fix_text = "3D" if self.drone_state.gps_fix_type >= 2 else ("2D" if self.drone_state.gps_fix_type == 1 else "No")
            dash.val_sensor_gps.setText(f"{num_sat} sats ({fix_text})")
            dash.val_sensor_gps.setStyleSheet(f"color: {gps_color}; font-weight: bold;")

    # ══════════════════════════════════════════════
    # QUẢN LÝ TRẠNG THÁI UI
    # ══════════════════════════════════════════════

    def set_ui_state_na(self):
        """Trạng thái mất mạng: Khóa giao diện và chuyển nút thành KẾT NỐI."""
        dash = self.dashboard_tab

        labels_to_na = [
            dash.val_batt_curr, dash.val_mode, dash.val_armed, dash.val_alt,
            dash.val_lat, dash.val_lon, dash.val_roll, dash.val_pitch,
            dash.val_yaw, dash.val_gps_fix, dash.val_sats, dash.val_spd,
            dash.val_motor1, dash.val_motor2, dash.val_motor3, dash.val_motor4,
            dash.val_surface_alt, dash.val_lidar_qual, dash.val_opflow
        ]
        for lbl in labels_to_na:
            lbl.setText("N/A")
            lbl.setStyleSheet("color: gray;")

        self.lbl_batt_volt.setText("-- V")
        self.lbl_batt_perc.setText("-- %")
        self.bar_battery_volt.setValue(0)

        self.lbl_wifi_icon.setText("📶 Mất kết nối")
        self.lbl_wifi_icon.setStyleSheet("color: gray;")

        # Reset ping label về trạng thái chưa đo
        self.lbl_ping.setText("🏓 ---ms")
        self.lbl_ping.setStyleSheet("color: #808098;")

        self.btn_disconnect.setText("Kết nối")
        self.btn_disconnect.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #45a049; }"
        )

        self.manual_control_tab.setEnabled(False)
        self.manual_control_tab.val_flight_status.setText("IDLE")
        self.manual_control_tab.val_flight_status.setStyleSheet("color: gray;")
        self.mission_tab.setEnabled(False)

    def enable_ui_components(self):
        """Trạng thái có mạng: Mở khóa giao diện và chuyển nút thành NGẮT KẾT NỐI."""
        dash = self.dashboard_tab

        self.manual_control_tab.setEnabled(True)
        self.mission_tab.setEnabled(True)
        dash.val_armed.setStyleSheet("color: red;")
        dash.val_gps_fix.setStyleSheet("color: #4CAF50;")

        self.lbl_wifi_icon.setText("📶 Đã kết nối")
        self.lbl_wifi_icon.setStyleSheet("color: #4CAF50;")

        self.btn_disconnect.setText("Ngắt kết nối")
        self.btn_disconnect.setStyleSheet(
            "QPushButton { background-color: #F44336; color: white; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #D32F2F; }"
        )

    # ══════════════════════════════════════════════
    # VÒNG ĐỜI ỨNG DỤNG
    # ══════════════════════════════════════════════

    def closeEvent(self, event):
        """Đảm bảo ngắt kết nối an toàn khi người dùng đóng cửa sổ."""
        if self.flight_controller.is_active:
            self.flight_controller.abort()
        if self.worker:
            self.worker.stop()
        event.accept()

    def resizeEvent(self, event):
        """Cập nhật vị trí emergency overlay khi resize cửa sổ."""
        super().resizeEvent(event)
        if hasattr(self, 'emergency_overlay') and self.emergency_overlay.isVisible():
            self.emergency_overlay._update_position()

    # ══════════════════════════════════════════════
    # SLOT: FLIGHT CONTROLLER
    # ══════════════════════════════════════════════

    def _confirm_takeoff(self):
        """Hiện dialog nhập độ cao trước khi cất cánh tự động."""
        dialog = TakeoffDialog(self)

        # Cập nhật thông tin GPS hiện tại lên dialog
        dialog.update_gps_info(
            self.drone_state.gps_fix_type,
            self.drone_state.gps_num_sat,
            self.drone_state.latitude,
            self.drone_state.longitude
        )

        if dialog.exec() == QDialog.Accepted:
            target_alt = dialog.target_altitude
            self.flight_controller.takeoff_and_hold(target_alt)

    def _on_flight_status(self, message: str):
        """Slot: Cập nhật trạng thái bay lên UI."""
        self.manual_control_tab.val_flight_status.setText(message)
        self.manual_control_tab.val_flight_status.setStyleSheet(
            "color: #2196F3; font-weight: bold;"
        )

    def _on_takeoff_complete(self):
        """Slot: Cất cánh thành công — cập nhật UI."""
        self.manual_control_tab.val_flight_status.setStyleSheet(
            "color: #4CAF50; font-weight: bold;"
        )

    def _on_flight_error(self, message: str):
        """Slot: Lỗi bay — hiện cảnh báo và cập nhật UI."""
        self.manual_control_tab.val_flight_status.setText(f"LỖI: {message}")
        self.manual_control_tab.val_flight_status.setStyleSheet(
            "color: red; font-weight: bold;"
        )
        QMessageBox.critical(self, "Lỗi Bay Tự Động", message)

    def _on_flight_state_changed(self, new_state: str):
        """Slot: State machine chuyển trạng thái — cập nhật nút Takeoff/Abort."""
        mc = self.manual_control_tab
        if new_state == "IDLE":
            mc.btn_takeoff_hold.setText("🚀 Takeoff")
            mc.btn_takeoff_hold.setStyleSheet(
                "QPushButton { background-color: #FF9800; color: white; font-weight: bold; "
                "font-size: 14px; border-radius: 6px; } "
                "QPushButton:hover { background-color: #F57C00; } "
                "QPushButton:disabled { background-color: #555; color: #888; }"
            )
            try:
                mc.btn_takeoff_hold.clicked.disconnect()
            except RuntimeError:
                pass
            mc.btn_takeoff_hold.clicked.connect(self._confirm_takeoff)
        else:
            mc.btn_takeoff_hold.setText("⛔ ABORT")
            mc.btn_takeoff_hold.setStyleSheet(
                "QPushButton { background-color: #F44336; color: white; font-weight: bold; "
                "font-size: 14px; border-radius: 6px; } "
                "QPushButton:hover { background-color: #D32F2F; }"
            )
            try:
                mc.btn_takeoff_hold.clicked.disconnect()
            except RuntimeError:
                pass
            mc.btn_takeoff_hold.clicked.connect(self.flight_controller.abort)

    def _on_mode_activated(self, mode_name: str):
        """
        Slot: Mode bay được kích hoạt — hiện/ẩn emergency overlay.

        Khi mode_name rỗng "" → ẩn overlay.
        Khi mode_name có nội dung → hiện overlay với tên mode.
        """
        if mode_name:
            self.emergency_overlay.show_with_mode(mode_name)
            self.drone_state.active_mode_name = mode_name
        else:
            self.emergency_overlay.hide_overlay()
            self.drone_state.active_mode_name = ""

    def _on_ping_updated(self, rtt_ms: int):
        """
        Slot: Nhận RTT mới từ WifiWorker, cập nhật ping label trên top bar.

        Màu sắc theo chất lượng:
          • Xanh lá  (≤50ms)  : Tốt
          • Vàng     (≤150ms) : Trung bình
          • Cam      (≤300ms) : Chậm
          • Đỏ       (>300ms) : Kém
        """
        self.drone_state.ping_rtt_ms = rtt_ms
        self.lbl_ping.setText(f"🏓 {rtt_ms}ms")
        if rtt_ms <= 50:
            color = "#4CAF50"   # xanh lá — tốt
        elif rtt_ms <= 150:
            color = "#FFC107"   # vàng — trung bình
        elif rtt_ms <= 300:
            color = "#FF9800"   # cam — chậm
        else:
            color = "#F44336"   # đỏ — kém
        self.lbl_ping.setStyleSheet(f"color: {color}; font-weight: bold;")

    def _on_command_acked(self, ack_type: str):
        """
        Slot: ESP32 đã xác nhận nhận lệnh — hiển thị trên flight status.

        ACK types:
          • RC  : Lệnh RC đã gửi tới FC
          • FS  : Cấu hình failsafe đã lưu
          • EM  : Lệnh khẩn cấp đã thực thi
        """
        type_labels = {
            "RC": "✓ RC",
            "FS": "✓ Failsafe config",
            "EM": "✓ Emergency cmd",
        }
        label = type_labels.get(ack_type, f"✓ ACK:{ack_type}")
        # Chỉ hiển thị trên status nếu không phải RC (RC quá nhiều sẽ gây nhấp nháy)
        if ack_type != "RC":
            mc = self.manual_control_tab
            mc.val_flight_status.setText(label)
            mc.val_flight_status.setStyleSheet("color: #00E676; font-weight: bold;")

    # ══════════════════════════════════════════════
    # EMERGENCY OVERLAY — NÚT KHẨN CẤP
    # ══════════════════════════════════════════════

    def _emergency_force_disarm(self):
        """Nút DISARM khẩn cấp từ overlay — FORCE tắt motor, bypass state machine."""
        self.flight_controller.force_disarm()
        self.emergency_overlay.hide_overlay()

    def _emergency_force_safe_land(self):
        """Nút Safe Land khẩn cấp từ overlay — FORCE hạ cánh, bypass state machine."""
        self.flight_controller.force_safe_land()

    # ══════════════════════════════════════════════
    # MISSION LOGIC
    # ══════════════════════════════════════════════

    def _upload_mission(self):
        """Upload waypoints từ MissionTab xuống FC qua MSP_SET_WP."""
        waypoints = self.mission_tab.get_waypoints()
        if not waypoints:
            QMessageBox.warning(self, "Cảnh báo", "Chưa có waypoint nào để upload!")
            return

        reply = QMessageBox.question(
            self,
            "Xác nhận Upload",
            f"Sẽ upload {len(waypoints)} waypoint xuống FC.\n\n"
            "Bạn có chắc chắn?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.flight_controller.upload_mission(waypoints)

    def _start_mission(self):
        """
        Bắt đầu bay mission — Hiện dialog xác nhận với cảnh báo an toàn.

        Logic:
        1. Kiểm tra có waypoint không
        2. Hỏi cấu hình failsafe (RTH hoặc Ignore)
        3. Gửi cấu hình failsafe xuống ESP32
        4. Thông báo kết quả
        """
        waypoints = self.mission_tab.get_waypoints()
        if not waypoints:
            QMessageBox.warning(self, "Cảnh báo", "Chưa có waypoint nào! Hãy click trên bản đồ để thêm.")
            return

        # Dialog hỏi bắt đầu bay hay hủy
        msg = QMessageBox(self)
        msg.setWindowTitle("🗺️ Bắt đầu Mission")
        msg.setText(
            f"Drone sẽ bay theo {len(waypoints)} waypoint đã thiết lập.\n\n"
            "Khi drone bay xa, bạn có muốn kích hoạt cơ chế\n"
            "Failsafe (RTH) khi mất kết nối WiFi không?"
        )
        msg.setIcon(QMessageBox.Question)

        btn_yes = msg.addButton("✅ Yes — RTH khi mất WiFi", QMessageBox.YesRole)
        btn_ignore = msg.addButton("⚠️ Ignore — Bay hết rồi Safe Land", QMessageBox.NoRole)
        btn_cancel = msg.addButton("❌ Hủy", QMessageBox.RejectRole)

        msg.exec()

        clicked = msg.clickedButton()
        if clicked == btn_cancel:
            return

        if clicked == btn_yes:
            # Cấu hình ESP32: Mất WiFi → RTH
            self.flight_controller.send_failsafe_config("rth")
            self.mission_tab.val_failsafe_status.setText("RTH khi mất WiFi")
            self.mission_tab.val_failsafe_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
        elif clicked == btn_ignore:
            # Cấu hình ESP32: Mất WiFi → Không can thiệp, drone bay hết WP rồi Safe Land
            self.flight_controller.send_failsafe_config("ignore")
            self.mission_tab.val_failsafe_status.setText("Ignore — Safe Land cuối")
            self.mission_tab.val_failsafe_status.setStyleSheet("color: #FFC107; font-weight: bold;")

        # Upload waypoints rồi thông báo sẵn sàng
        self.flight_controller.upload_mission(waypoints)
        self.emergency_overlay.show_with_mode("Mission")

        QMessageBox.information(
            self,
            "Mission Ready",
            "✅ Waypoints đã được upload lên FC.\n\n"
            "Để bắt đầu bay, hãy ARM drone rồi bật mode NAV WP\n"
            "trên remote hoặc qua INAV Configurator."
        )

    def _stop_mission(self):
        """Dừng mission — gửi lệnh Safe Land."""
        reply = QMessageBox.question(
            self,
            "Dừng Mission",
            "Bạn muốn dừng mission và hạ cánh tại chỗ?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.flight_controller.safe_land()

    def _check_distance_safety(self):
        """
        Kiểm tra khoảng cách drone → Home.
        Nếu vượt ngưỡng → hiện dialog cảnh báo failsafe.
        """
        if self._distance_warning_shown:
            return  # Đã cảnh báo rồi, không hỏi lại

        if not self.drone_state.has_home:
            return

        if self.mission_tab.check_distance_warning():
            self._distance_warning_shown = True

            reply = QMessageBox.question(
                self,
                "⚠️ Cảnh báo Khoảng cách",
                f"Drone đã bay xa hơn {self.mission_tab._distance_threshold}m "
                "so với vị trí Home!\n\n"
                "Bạn có muốn kích hoạt cơ chế Failsafe\n"
                "(RTH tự động) khi mất kết nối WiFi không?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            if reply == QMessageBox.Yes:
                self.flight_controller.send_failsafe_config("rth")
                self.mission_tab.val_failsafe_status.setText("RTH — Auto Failsafe")
                self.mission_tab.val_failsafe_status.setStyleSheet("color: #F44336; font-weight: bold;")




# ══════════════════════════════════════════════════
# KHỞI CHẠY ỨNG DỤNG
# ══════════════════════════════════════════════════

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # 1. Khởi chạy màn hình chính
    window = GCSApp()
    window.show()

    # 2. Tự động bật hộp thoại kết nối để tiện lợi
    dialog = ConnectionDialog(window)
    if dialog.exec() == QDialog.Accepted:
        window.start_connection(dialog.selected_ip, dialog.selected_port, dialog.is_mock_selected)

    sys.exit(app.exec())