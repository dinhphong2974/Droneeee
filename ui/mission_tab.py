"""
mission_tab.py - Tab lập trình lộ trình bay (Mission Planning).

Chứa class MissionTab(QWidget) bao gồm:
- Form nhập Waypoint (Latitude, Longitude, Altitude)
- Bảng danh sách Waypoint đã thêm
- Các nút: Thêm, Xóa, Clear, Upload lên FC, Start/Stop Mission
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView
)


class MissionTab(QWidget):
    """Tab lập trình và quản lý lộ trình bay tự động."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng layout chính của tab Mission."""
        layout = QVBoxLayout(self)

        # Form nhập waypoint
        self._create_waypoint_input()
        layout.addWidget(self.grp_waypoint_input)

        # Bảng danh sách waypoint
        self._create_waypoint_list()
        layout.addWidget(self.grp_waypoint_list)

        # Hàng nút điều khiển mission
        self._create_action_buttons()
        layout.addLayout(self._btn_layout)

    # ══════════════════════════════════════════════
    # FORM NHẬP WAYPOINT
    # ══════════════════════════════════════════════

    def _create_waypoint_input(self):
        """Tạo form nhập tọa độ waypoint mới."""
        self.grp_waypoint_input = QGroupBox("Waypoint Input")
        h_layout = QHBoxLayout(self.grp_waypoint_input)

        # Latitude
        self.lbl_wp_lat = QLabel("Latitude")
        self.input_lat = QLineEdit()
        self.input_lat.setPlaceholderText("21.0")
        h_layout.addWidget(self.lbl_wp_lat)
        h_layout.addWidget(self.input_lat)

        # Longitude
        self.lbl_wp_lon = QLabel("Longitude")
        self.input_lon = QLineEdit()
        self.input_lon.setPlaceholderText("105.0")
        h_layout.addWidget(self.lbl_wp_lon)
        h_layout.addWidget(self.input_lon)

        # Altitude (mét)
        self.lbl_wp_alt = QLabel("Altitude (m)")
        self.input_alt = QLineEdit("10")
        h_layout.addWidget(self.lbl_wp_alt)
        h_layout.addWidget(self.input_alt)

        # Nút thêm waypoint
        self.btn_add_waypoint = QPushButton("Add Waypoint")
        h_layout.addWidget(self.btn_add_waypoint)

    # ══════════════════════════════════════════════
    # BẢNG DANH SÁCH WAYPOINT
    # ══════════════════════════════════════════════

    def _create_waypoint_list(self):
        """Tạo bảng hiển thị danh sách waypoint đã thêm."""
        self.grp_waypoint_list = QGroupBox("Waypoint List")
        v_layout = QVBoxLayout(self.grp_waypoint_list)

        # Bảng 4 cột: #, Lat, Lon, Alt
        self.table_waypoints = QTableWidget()
        self.table_waypoints.setColumnCount(4)
        self.table_waypoints.setHorizontalHeaderItem(0, QTableWidgetItem("#"))
        self.table_waypoints.setHorizontalHeaderItem(1, QTableWidgetItem("Lat"))
        self.table_waypoints.setHorizontalHeaderItem(2, QTableWidgetItem("Lon"))
        self.table_waypoints.setHorizontalHeaderItem(3, QTableWidgetItem("Alt (m)"))

        # Tự động giãn cột cho vừa bảng
        header = self.table_waypoints.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        v_layout.addWidget(self.table_waypoints)

    # ══════════════════════════════════════════════
    # NÚT ĐIỀU KHIỂN MISSION
    # ══════════════════════════════════════════════

    def _create_action_buttons(self):
        """Tạo hàng nút: Xóa, Clear, Upload, Start, Stop."""
        self._btn_layout = QHBoxLayout()

        # Cấu hình nút: (tên_biến, text_hiển_thị)
        button_configs = [
            ("btn_remove",            "Remove Selected"),
            ("btn_clear",             "Clear All"),
            ("btn_upload",            "Upload To FC"),
            ("btn_start_mission_tab", "Start Mission"),
            ("btn_stop_mission",      "Stop Mission"),
        ]

        for btn_name, btn_text in button_configs:
            btn = QPushButton(btn_text)
            setattr(self, btn_name, btn)
            self._btn_layout.addWidget(btn)
