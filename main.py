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
    ui/mission_tab.py → trang lộ trình bay
    ui/config_tab.py → trang cấu hình
    comm/wifi_client.py → kết nối TCP thô
    comm/wifi_worker.py → QThread chạy ngầm
    comm/msp_parser.py → giải mã giao thức MSP
    core/drone_state.py → trạng thái drone chia sẻ
    core/flight_controller.py → state machine bay tự động
"""

import sys
from PySide6.QtWidgets import (QApplication, QDialog, QVBoxLayout,
                                QHBoxLayout, QLineEdit, QPushButton, QLabel, QMessageBox)
from ui.main_window import MainWindow
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


class GCSApp(MainWindow):
    """
    Lớp ứng dụng chính — kế thừa MainWindow và thêm logic điều phối.

    MainWindow xây dựng UI layout, GCSApp kết nối Signal/Slot:
    - WifiWorker.connection_status → handle_connection_status
    - WifiWorker.telemetry_data → update_telemetry_ui
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

        # ── Kết nối nút điều khiển bay (từ ManualControlTab) ──
        mc = self.manual_control_tab
        mc.btn_arm.clicked.connect(self.flight_controller.arm)
        mc.btn_disarm.clicked.connect(self.flight_controller.disarm)
        mc.btn_takeoff_hold.clicked.connect(self._confirm_takeoff)

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

        Đây là điểm duy nhất kết nối các module lại với nhau:
        - WifiWorker.connection_status → self.handle_connection_status
        - WifiWorker.telemetry_data → self.update_telemetry_ui
        """
        self.worker = WifiWorker(ip=ip, port=port, is_mock=is_mock)

        # Kết nối Signal từ worker tới các slot xử lý trên UI
        self.worker.connection_status.connect(self.handle_connection_status)
        self.worker.telemetry_data.connect(self.update_telemetry_ui)

        # Cấp worker cho FlightController để gửi lệnh
        self.flight_controller.set_worker(self.worker)

        # Khởi chạy thread ngầm
        self.worker.start()

    # ══════════════════════════════════════════════
    # SLOT: XỬ LÝ TÍN HIỆU TỪ WORKER
    # ══════════════════════════════════════════════

    def handle_connection_status(self, success: bool, message: str):
        """
        Slot xử lý tín hiệu trạng thái mạng từ WifiWorker.

        Args:
            success: True = kết nối thành công, False = thất bại/đứt mạng
            message: Mô tả trạng thái hiện tại
        """
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
            self.setWindowTitle("Drone Ground Station - Mất kết nối")
            QMessageBox.warning(self, "Cảnh báo Mạng", message)

    def update_telemetry_ui(self, data: dict):
        """
        Slot cập nhật dữ liệu telemetry từ FC lên giao diện.

        Nhận dict đã giải mã từ WifiWorker qua Signal,
        chỉ cập nhật các trường có dữ liệu mới.

        Args:
            data: Dict chứa dữ liệu telemetry, VD: {"voltage": 24.5, "roll": 1.2}
        """
        # Shortcut truy cập tab dashboard
        dash = self.dashboard_tab

        # ── Cập nhật điện áp pin Lipo 6S ──
        if "voltage" in data:
            v = data["voltage"]
            # Quy đổi điện áp Lipo 6S (19.8V rỗng - 25.2V đầy) sang phần trăm
            percent = int(((v - LIPO_6S_MIN_VOLTAGE) / (LIPO_6S_MAX_VOLTAGE - LIPO_6S_MIN_VOLTAGE)) * 100)
            percent = max(0, min(100, percent))

            self.lbl_batt_volt.setText(f"{v:.2f} V")
            self.lbl_batt_perc.setText(f"{percent} %")
            self.bar_battery_volt.setValue(percent)

            # Đổi màu thanh pin theo mức: xanh > 50%, vàng > 20%, đỏ ≤ 20%
            color = "#4CAF50" if percent > 50 else ("#FFC107" if percent > 20 else "#F44336")
            self.bar_battery_volt.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; }}")

        # ── Cập nhật góc nghiêng (từ DashboardTab) ──
        if "pitch" in data:
            dash.val_pitch.setText(f"{data['pitch']:.1f}°")
        if "roll" in data:
            dash.val_roll.setText(f"{data['roll']:.1f}°")
        if "yaw" in data:
            dash.val_yaw.setText(f"{data['yaw']:.1f}°")

        # ── Cập nhật Attitude 3D Widget ──
        if any(k in data for k in ("roll", "pitch", "yaw")):
            dash.widget_3d_attitude.update_attitude(
                data.get("roll", dash.widget_3d_attitude._roll),
                data.get("pitch", dash.widget_3d_attitude._pitch),
                data.get("yaw", dash.widget_3d_attitude._yaw),
            )

        # ── Cập nhật vòng tua động cơ 1960kv (từ DashboardTab) ──
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
            else:
                dash.val_armed.setText("DISARMED")
                dash.val_armed.setStyleSheet("color: #4CAF50; font-weight: bold;")

        if "flight_mode_flags" in data:
            self.drone_state.flight_mode_flags = data["flight_mode_flags"]

    # ══════════════════════════════════════════════
    # QUẢN LÝ TRẠNG THÁI UI
    # ══════════════════════════════════════════════

    def set_ui_state_na(self):
        """Trạng thái mất mạng: Khóa giao diện và chuyển nút thành KẾT NỐI."""
        # Shortcut truy cập tab dashboard
        dash = self.dashboard_tab

        # Reset tất cả label giá trị về N/A
        labels_to_na = [
            dash.val_batt_curr, dash.val_mode, dash.val_armed, dash.val_alt,
            dash.val_lat, dash.val_lon, dash.val_roll, dash.val_pitch,
            dash.val_yaw, dash.val_gps_fix, dash.val_sats, dash.val_spd,
            dash.val_motor1, dash.val_motor2, dash.val_motor3, dash.val_motor4
        ]
        for lbl in labels_to_na:
            lbl.setText("N/A")
            lbl.setStyleSheet("color: gray;")

        # Reset Top Bar (Pin & Wifi)
        self.lbl_batt_volt.setText("-- V")
        self.lbl_batt_perc.setText("-- %")
        self.bar_battery_volt.setValue(0)

        self.lbl_wifi_icon.setText("📶 Mất kết nối")
        self.lbl_wifi_icon.setStyleSheet("color: gray;")

        # Biến nút thành "Kết nối" (Màu xanh)
        self.btn_disconnect.setText("Kết nối")
        self.btn_disconnect.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #45a049; }"
        )

        # Khóa vùng điều khiển để tránh gửi lệnh khi mất mạng
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

        # Cập nhật trạng thái WiFi
        self.lbl_wifi_icon.setText("📶 Đã kết nối")
        self.lbl_wifi_icon.setStyleSheet("color: #4CAF50;")

        # Biến nút thành "Ngắt kết nối" (Màu đỏ)
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

    # ══════════════════════════════════════════════
    # SLOT: FLIGHT CONTROLLER
    # ══════════════════════════════════════════════

    def _confirm_takeoff(self):
        """Hiện dialog xác nhận trước khi cất cánh tự động."""
        reply = QMessageBox.question(
            self,
            "Xác nhận Takeoff",
            "🚀 Drone sẽ tự động ARM, cất cánh và giữ ở 3 mét.\n\n"
            "⚠️ Đảm bảo khu vực an toàn trước khi tiếp tục!\n\n"
            "Bạn có muốn tiếp tục?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.flight_controller.takeoff_and_hold(3.0)

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
        """Slot: State machine chuyển trạng thái."""
        mc = self.manual_control_tab
        if new_state == "IDLE":
            mc.btn_takeoff_hold.setText("🚀 Takeoff & Hold 3m")
            mc.btn_takeoff_hold.setStyleSheet(
                "QPushButton { background-color: #FF9800; color: white; font-weight: bold; "
                "font-size: 14px; border-radius: 6px; } "
                "QPushButton:hover { background-color: #F57C00; } "
                "QPushButton:disabled { background-color: #555; color: #888; }"
            )
            mc.btn_takeoff_hold.clicked.disconnect()
            mc.btn_takeoff_hold.clicked.connect(self._confirm_takeoff)
        else:
            mc.btn_takeoff_hold.setText("⛔ ABORT")
            mc.btn_takeoff_hold.setStyleSheet(
                "QPushButton { background-color: #F44336; color: white; font-weight: bold; "
                "font-size: 14px; border-radius: 6px; } "
                "QPushButton:hover { background-color: #D32F2F; }"
            )
            mc.btn_takeoff_hold.clicked.disconnect()
            mc.btn_takeoff_hold.clicked.connect(self.flight_controller.abort)


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