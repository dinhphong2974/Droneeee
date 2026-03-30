"""
dashboard_tab.py - Tab giám sát chính: Telemetry, Motors, Manual Control.

Chứa class DashboardTab(QWidget) bao gồm:
- Bảng Telemetry: Pin, Attitude (Roll/Pitch/Yaw), GPS, Tốc độ
- Bảng Motors: 4 thanh hiển thị vòng tua động cơ 1960kv (PWM 1000-2000)
- Bảng Manual Control: Slider điều khiển + nút ARM/DISARM/RTH
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QGridLayout, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QProgressBar, QSlider, QPushButton
)


class DashboardTab(QWidget):
    """Tab chính hiển thị toàn bộ thông số bay và điều khiển thủ công."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Font in đậm dùng chung cho các giá trị hiển thị
        self._bold_font = QFont()
        self._bold_font.setBold(True)

        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng layout chính dạng lưới 2x2."""
        layout = QGridLayout(self)

        # [0,0] Bảng thông số bay (Telemetry)
        self._create_telemetry_group()
        layout.addWidget(self.grp_telemetry, 0, 0)

        # [0,1] Bảng điều khiển thủ công (Manual Control)
        self._create_manual_control_group()
        layout.addWidget(self.grp_manual_control, 0, 1)

        # [1,0..1] Bảng vòng tua động cơ (Motors) — trải rộng 2 cột
        self._create_motors_group()
        layout.addWidget(self.grp_motors, 1, 0, 1, 2)

    # ══════════════════════════════════════════════
    # BẢNG TELEMETRY
    # ══════════════════════════════════════════════

    def _create_telemetry_group(self):
        """Tạo bảng hiển thị thông số bay: Pin, Mode, Attitude, GPS."""
        self.grp_telemetry = QGroupBox("Telemetry Dashboard")
        grid = QGridLayout(self.grp_telemetry)

        # ── Cột trái: Trạng thái chung ──

        # Dòng điện pin
        self.lbl_batt_curr = QLabel("Battery Current")
        self.val_batt_curr = QLabel("N/A")
        self.val_batt_curr.setFont(self._bold_font)
        grid.addWidget(self.lbl_batt_curr, 0, 0)
        grid.addWidget(self.val_batt_curr, 0, 1)

        # Chế độ bay
        self.lbl_mode = QLabel("Mode")
        self.val_mode = QLabel("N/A")
        self.val_mode.setFont(self._bold_font)
        grid.addWidget(self.lbl_mode, 1, 0)
        grid.addWidget(self.val_mode, 1, 1)

        # Trạng thái ARM
        self.lbl_armed = QLabel("Armed")
        self.val_armed = QLabel("N/A")
        self.val_armed.setFont(self._bold_font)
        grid.addWidget(self.lbl_armed, 2, 0)
        grid.addWidget(self.val_armed, 2, 1)

        # Độ cao
        self.lbl_alt = QLabel("Altitude")
        self.val_alt = QLabel("N/A")
        self.val_alt.setFont(self._bold_font)
        grid.addWidget(self.lbl_alt, 3, 0)
        grid.addWidget(self.val_alt, 3, 1)

        # Tọa độ GPS (nhóm con)
        self._create_gps_coords_group()
        grid.addWidget(self.grp_gps_coords, 4, 0, 1, 2)

        # ── Cột phải: Attitude + GPS Fix ──

        # Roll
        self.lbl_roll = QLabel("Roll")
        self.val_roll = QLabel("N/A")
        self.val_roll.setFont(self._bold_font)
        grid.addWidget(self.lbl_roll, 0, 4)
        grid.addWidget(self.val_roll, 0, 5)

        # Pitch
        self.lbl_pitch = QLabel("Pitch")
        self.val_pitch = QLabel("N/A")
        self.val_pitch.setFont(self._bold_font)
        grid.addWidget(self.lbl_pitch, 1, 4)
        grid.addWidget(self.val_pitch, 1, 5)

        # Yaw
        self.lbl_yaw = QLabel("Yaw")
        self.val_yaw = QLabel("N/A")
        self.val_yaw.setFont(self._bold_font)
        grid.addWidget(self.lbl_yaw, 2, 4)
        grid.addWidget(self.val_yaw, 2, 5)

        # GPS Fix
        self.lbl_gps_fix = QLabel("GPS Fix")
        self.val_gps_fix = QLabel("N/A")
        self.val_gps_fix.setFont(self._bold_font)
        grid.addWidget(self.lbl_gps_fix, 3, 4)
        grid.addWidget(self.val_gps_fix, 3, 5)

        # Số vệ tinh
        self.lbl_sats = QLabel("Satellites")
        self.val_sats = QLabel("N/A")
        self.val_sats.setFont(self._bold_font)
        grid.addWidget(self.lbl_sats, 4, 4)
        grid.addWidget(self.val_sats, 4, 5)

        # Tốc độ mặt đất
        self.lbl_spd = QLabel("Ground Speed")
        self.val_spd = QLabel("N/A")
        self.val_spd.setFont(self._bold_font)
        grid.addWidget(self.lbl_spd, 5, 4)
        grid.addWidget(self.val_spd, 5, 5)

    def _create_gps_coords_group(self):
        """Tạo nhóm hiển thị tọa độ GPS (Latitude/Longitude)."""
        self.grp_gps_coords = QGroupBox("GPS Coordinates")
        grid = QGridLayout(self.grp_gps_coords)

        self.lbl_lat = QLabel("Latitude")
        self.val_lat = QLabel("N/A")
        self.val_lat.setFont(self._bold_font)
        grid.addWidget(self.lbl_lat, 0, 0)
        grid.addWidget(self.val_lat, 0, 1)

        self.lbl_lon = QLabel("Longitude")
        self.val_lon = QLabel("N/A")
        self.val_lon.setFont(self._bold_font)
        grid.addWidget(self.lbl_lon, 0, 2)
        grid.addWidget(self.val_lon, 0, 3)

    # ══════════════════════════════════════════════
    # BẢNG ĐIỀU KHIỂN THỦ CÔNG (MANUAL CONTROL)
    # ══════════════════════════════════════════════

    def _create_manual_control_group(self):
        """Tạo bảng slider điều khiển + nút ARM/DISARM/RTH."""
        self.grp_manual_control = QGroupBox("Manual Control")
        grid = QGridLayout(self.grp_manual_control)

        # Cấu hình từng slider: (tên_label, text, tên_slider, tên_val, min, max, default, row)
        slider_configs = [
            ("lbl_throttle",    "Throttle", "slider_throttle", "val_throttle",    1000, 2000, 1000, 0),
            ("lbl_roll_input",  "Roll",     "slider_roll",     "val_roll_input",  1000, 2000, 1500, 1),
            ("lbl_pitch_input", "Pitch",    "slider_pitch",    "val_pitch_input", 1000, 2000, 1500, 2),
            ("lbl_yaw_input",   "Yaw",      "slider_yaw",      "val_yaw_input",   1000, 2000, 1500, 3),
            ("lbl_aux1",        "AUX1",     "slider_aux1",     "val_aux1",        1000, 2000, 1000, 4),
            ("lbl_aux2",        "AUX2",     "slider_aux2",     "val_aux2",        1000, 2000, 1000, 5),
        ]

        for lbl_name, lbl_text, slider_name, val_name, min_v, max_v, default, row in slider_configs:
            # Label mô tả
            lbl = QLabel(lbl_text)
            setattr(self, lbl_name, lbl)
            grid.addWidget(lbl, row, 0)

            # Slider điều khiển (PWM 1000-2000μs)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_v)
            slider.setMaximum(max_v)
            slider.setValue(default)
            setattr(self, slider_name, slider)
            grid.addWidget(slider, row, 1)

            # Giá trị hiển thị
            val = QLabel(str(default))
            setattr(self, val_name, val)
            grid.addWidget(val, row, 2)

        # ── Hàng nút bấm điều khiển ──
        btn_layout = QHBoxLayout()

        # Cấu hình nút: (tên_biến, text_hiển_thị)
        button_configs = [
            ("btn_send_manual",       "Send Manual"),
            ("btn_arm",               "ARM"),
            ("btn_disarm",            "DISARM"),
            ("btn_hold",              "HOLD"),
            ("btn_rth",               "RTH"),
            ("btn_start_mission_main", "Start Mission"),
        ]

        for btn_name, btn_text in button_configs:
            btn = QPushButton(btn_text)
            setattr(self, btn_name, btn)
            btn_layout.addWidget(btn)

        grid.addLayout(btn_layout, 6, 0, 1, 3)

    # ══════════════════════════════════════════════
    # BẢNG VÒNG TUA ĐỘNG CƠ (MOTORS)
    # ══════════════════════════════════════════════

    def _create_motors_group(self):
        """Tạo bảng 4 thanh hiển thị vòng tua động cơ 1960kv."""
        self.grp_motors = QGroupBox("Motors")
        h_layout = QHBoxLayout(self.grp_motors)

        # Tạo 4 cột động cơ giống nhau (Motor 1 → Motor 4)
        for i in range(1, 5):
            v_layout = QVBoxLayout()

            # Tên động cơ
            lbl = QLabel(f"Motor {i}")
            lbl.setAlignment(Qt.AlignCenter)
            setattr(self, f"lbl_motor{i}", lbl)
            v_layout.addWidget(lbl)

            # Giá trị PWM hiện tại
            val = QLabel("1000")
            val.setFont(self._bold_font)
            val.setAlignment(Qt.AlignCenter)
            setattr(self, f"val_motor{i}", val)
            v_layout.addWidget(val)

            # Thanh hiển thị vòng tua (PWM 1000-2000μs)
            bar = QProgressBar()
            bar.setMinimum(1000)
            bar.setMaximum(2000)
            bar.setValue(1000)
            bar.setOrientation(Qt.Vertical)
            bar.setTextVisible(False)
            bar.setStyleSheet("QProgressBar::chunk { background-color: #4CAF50; }")
            setattr(self, f"bar_motor{i}", bar)
            v_layout.addWidget(bar)

            h_layout.addLayout(v_layout)
