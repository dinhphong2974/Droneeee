import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QDialog, QVBoxLayout, 
                               QHBoxLayout, QLineEdit, QPushButton, QLabel, QMessageBox)
from PySide6.QtCore import Qt
from ui.gcs_dashboard import Ui_MainWindow
from comm.wifi_worker import WifiWorker

class ConnectionDialog(QDialog):
    """Cửa sổ kết nối mạng Wifi tới ESP32"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Kết nối ESP32 - Wifi")
        self.setFixedSize(320, 150)
        self.setup_ui()
        
        self.selected_ip = None
        self.selected_port = None
        self.is_mock_selected = False

    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Nhập IP của ESP32
        h_ip = QHBoxLayout()
        h_ip.addWidget(QLabel("IP ESP32:"))
        self.input_ip = QLineEdit("192.168.4.1") # Gợi ý sẵn IP phổ biến của ESP32 AP
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
        self.is_mock_selected = True
        self.accept()

class GCSApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.worker = None
        self.set_ui_state_na() # Mặc định làm mờ tất cả khi chưa có mạng

    def set_ui_state_na(self):
        """Làm mờ/Khóa giao diện khi chưa lấy được data"""
        labels_to_na = [
            self.ui.val_batt_curr, self.ui.val_mode, self.ui.val_armed, self.ui.val_alt,
            self.ui.val_lat, self.ui.val_lon, self.ui.val_roll, self.ui.val_pitch,
            self.ui.val_yaw, self.ui.val_gps_fix, self.ui.val_sats, self.ui.val_spd,
            self.ui.val_motor1, self.ui.val_motor2, self.ui.val_motor3, self.ui.val_motor4
        ]
        for lbl in labels_to_na:
            lbl.setText("N/A")
            lbl.setStyleSheet("color: gray;")
            
        self.ui.lbl_batt_volt.setText("-- V")
        self.ui.lbl_batt_perc.setText("-- %")
        self.ui.bar_battery_volt.setValue(0)
        self.ui.lbl_wifi_icon.setText("📶 Mất mạng")
        self.ui.lbl_wifi_icon.setStyleSheet("color: gray;")
        
        # Khóa vùng ghi dữ liệu để tránh gửi lệnh sai khi mất sóng
        self.ui.grp_manual_control.setEnabled(False)
        self.ui.tab_mission.setEnabled(False)

    def enable_ui_components(self):
        """Mở khóa giao diện khi mạng ổn định"""
        self.ui.grp_manual_control.setEnabled(True)
        self.ui.tab_mission.setEnabled(True)
        self.ui.val_armed.setStyleSheet("color: red;")
        self.ui.val_gps_fix.setStyleSheet("color: #4CAF50;")
        self.ui.lbl_wifi_icon.setText("📶 Đã kết nối")
        self.ui.lbl_wifi_icon.setStyleSheet("color: #4CAF50;")

    def start_connection(self, ip, port, is_mock):
        """Kích hoạt luồng Wifi chạy ngầm"""
        self.worker = WifiWorker(ip=ip, port=port, is_mock=is_mock)
        self.worker.connection_status.connect(self.handle_connection_status)
        self.worker.telemetry_data.connect(self.update_telemetry_ui)
        self.worker.start()

    def handle_connection_status(self, success, message):
        if success:
            self.setWindowTitle(f"Drone Ground Station - {message}")
            self.enable_ui_components()
        else:
            QMessageBox.critical(self, "Lỗi Mạng", message)
            self.set_ui_state_na()

    def update_telemetry_ui(self, data):
        """Vẽ dữ liệu nhận được lên giao diện mượt mà nhất"""
        if "voltage" in data:
            v = data["voltage"]
            percent = int(((v - 19.8) / (25.2 - 19.8)) * 100)
            percent = max(0, min(100, percent))
            self.ui.lbl_batt_volt.setText(f"{v:.2f} V")
            self.ui.lbl_batt_perc.setText(f"{percent} %")
            self.ui.bar_battery_volt.setValue(percent)
            
            color = "#4CAF50" if percent > 50 else ("#FFC107" if percent > 20 else "#F44336")
            self.ui.bar_battery_volt.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; }}")

        if "pitch" in data: self.ui.val_pitch.setText(f"{data['pitch']:.1f}°")
        if "roll" in data: self.ui.val_roll.setText(f"{data['roll']:.1f}°")
        if "yaw" in data: self.ui.val_yaw.setText(f"{data['yaw']:.1f}°")
        
        if "motor1" in data:
            self.ui.val_motor1.setText(str(data["motor1"]))
            self.ui.bar_motor1.setValue(data["motor1"])
            self.ui.val_motor2.setText(str(data["motor2"]))
            self.ui.bar_motor2.setValue(data["motor2"])
            self.ui.val_motor3.setText(str(data["motor3"]))
            self.ui.bar_motor3.setValue(data["motor3"])
            self.ui.val_motor4.setText(str(data["motor4"]))
            self.ui.bar_motor4.setValue(data["motor4"])

    def closeEvent(self, event):
        if self.worker:
            self.worker.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    dialog = ConnectionDialog()
    if dialog.exec() == QDialog.Accepted:
        window = GCSApp()
        window.start_connection(dialog.selected_ip, dialog.selected_port, dialog.is_mock_selected)
        window.show()
        sys.exit(app.exec())