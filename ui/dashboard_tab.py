"""
dashboard_tab.py - Tab giám sát chính: Attitude 3D + Telemetry + Sensors + Motors.

Chứa class DashboardTab(QWidget) bao gồm:
- Bên trái (65%): Widget mô phỏng tư thế drone (Artificial Horizon)
- Bên phải (35%): Bảng Telemetry + Sensor Health + Bảng Motors
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QGroupBox, QLabel, QProgressBar, QSplitter
)

from ui.attitude_3d_widget import Attitude3DWidget


class DashboardTab(QWidget):
    """Tab chính hiển thị tư thế bay 3D và thông số telemetry."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._bold_font = QFont()
        self._bold_font.setBold(True)
        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng layout: Attitude 3D (65%) | Telemetry + Sensors + Motors (35%)."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Sử dụng QSplitter để người dùng có thể resize
        splitter = QSplitter(Qt.Horizontal)

        # ── BÊN TRÁI: Attitude 3D (65%) ──
        self.widget_3d_attitude = Attitude3DWidget()
        splitter.addWidget(self.widget_3d_attitude)

        # ── BÊN PHẢI: Telemetry + Sensor Health + Motors (35%) ──
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(4, 4, 4, 4)

        self._create_telemetry_group()
        right_layout.addWidget(self.grp_telemetry)

        self._create_sensor_health_group()
        right_layout.addWidget(self.grp_sensor_health)

        self._create_motors_group()
        right_layout.addWidget(self.grp_motors)

        right_layout.addStretch()
        splitter.addWidget(right_panel)

        # Tỷ lệ mặc định 65:35
        splitter.setSizes([650, 350])

        layout.addWidget(splitter)

    # ══════════════════════════════════════════════
    # BẢNG TELEMETRY
    # ══════════════════════════════════════════════

    def _create_telemetry_group(self):
        """Tạo bảng hiển thị thông số bay: Pin, Mode, Attitude, GPS."""
        self.grp_telemetry = QGroupBox("Telemetry Dashboard")
        grid = QGridLayout(self.grp_telemetry)
        grid.setSpacing(6)

        # ── Cột trái: Trạng thái chung ──
        row = 0
        fields_left = [
            ("lbl_batt_curr", "Battery Current", "val_batt_curr"),
            ("lbl_mode",      "Mode",            "val_mode"),
            ("lbl_armed",     "Armed",           "val_armed"),
            ("lbl_alt",       "Altitude",        "val_alt"),
        ]
        for lbl_name, lbl_text, val_name in fields_left:
            lbl = QLabel(lbl_text)
            val = QLabel("N/A")
            val.setFont(self._bold_font)
            setattr(self, lbl_name, lbl)
            setattr(self, val_name, val)
            grid.addWidget(lbl, row, 0)
            grid.addWidget(val, row, 1)
            row += 1

        # GPS Coordinates (nhóm con)
        self._create_gps_coords_group()
        grid.addWidget(self.grp_gps_coords, row, 0, 1, 2)
        row += 1

        # ── Cột phải: Attitude + GPS Fix + Sensors ──
        row_r = 0
        fields_right = [
            ("lbl_roll",        "Roll",         "val_roll"),
            ("lbl_pitch",       "Pitch",        "val_pitch"),
            ("lbl_yaw",         "Yaw",          "val_yaw"),
            ("lbl_gps_fix",     "GPS Fix",      "val_gps_fix"),
            ("lbl_sats",        "🛰 Satellites","val_sats"),
            ("lbl_spd",         "Ground Speed", "val_spd"),
            ("lbl_surface_alt", "📡 Surface Alt","val_surface_alt"),
            ("lbl_lidar_qual",  "LiDAR Quality","val_lidar_qual"),
            ("lbl_opflow",      "👁 Optical Flow","val_opflow"),
        ]
        for lbl_name, lbl_text, val_name in fields_right:
            lbl = QLabel(lbl_text)
            val = QLabel("N/A")
            val.setFont(self._bold_font)
            setattr(self, lbl_name, lbl)
            setattr(self, val_name, val)
            grid.addWidget(lbl, row_r, 3)
            grid.addWidget(val, row_r, 4)
            row_r += 1

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
    # SENSOR HEALTH CARD
    # ══════════════════════════════════════════════

    def _create_sensor_health_group(self):
        """Tạo bảng trạng thái cảm biến: GPS, LiDAR, OptFlow, Compass."""
        self.grp_sensor_health = QGroupBox("Sensor Health")
        self.grp_sensor_health.setStyleSheet("""
            QGroupBox {
                border: 1px solid #2a3a5a;
                border-radius: 8px;
                margin-top: 14px;
                padding-top: 18px;
                font-weight: bold;
                color: #80a0d8;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }
        """)
        grid = QGridLayout(self.grp_sensor_health)
        grid.setSpacing(8)

        sensors = [
            ("🛰", "GPS",          "sensor_gps"),
            ("📡", "LiDAR",        "sensor_lidar"),
            ("👁", "Optical Flow", "sensor_opflow"),
            ("🧭", "Compass",      "sensor_mag"),
        ]

        for row, (icon, name, attr_prefix) in enumerate(sensors):
            # Icon + Tên
            lbl_icon = QLabel(f"{icon}  {name}")
            lbl_icon.setStyleSheet("color: #c0c0e0; font-weight: bold; font-size: 12px;")
            grid.addWidget(lbl_icon, row, 0)

            # Thanh Progress (quality indicator)
            bar = QProgressBar()
            bar.setMinimum(0)
            bar.setMaximum(100)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setMaximumHeight(12)
            bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid #2a2a4a;
                    border-radius: 4px;
                    background-color: #252540;
                }
                QProgressBar::chunk {
                    border-radius: 3px;
                    background-color: #4a4a6a;
                }
            """)
            setattr(self, f"bar_{attr_prefix}", bar)
            grid.addWidget(bar, row, 1)

            # Giá trị text + trạng thái
            val = QLabel("—")
            val.setFont(self._bold_font)
            val.setStyleSheet("color: #808098;")
            val.setMinimumWidth(100)
            setattr(self, f"val_{attr_prefix}", val)
            grid.addWidget(val, row, 2)

    # ══════════════════════════════════════════════
    # BẢNG VÒNG TUA ĐỘNG CƠ (MOTORS)
    # ══════════════════════════════════════════════

    def _create_motors_group(self):
        """Tạo bảng 4 thanh hiển thị vòng tua động cơ 1960kv."""
        self.grp_motors = QGroupBox("Motors")
        h_layout = QHBoxLayout(self.grp_motors)

        for i in range(1, 5):
            v_layout = QVBoxLayout()

            lbl = QLabel(f"Motor {i}")
            lbl.setAlignment(Qt.AlignCenter)
            setattr(self, f"lbl_motor{i}", lbl)
            v_layout.addWidget(lbl)

            val = QLabel("1000")
            val.setFont(self._bold_font)
            val.setAlignment(Qt.AlignCenter)
            setattr(self, f"val_motor{i}", val)
            v_layout.addWidget(val)

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
