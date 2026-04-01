"""
manual_control_tab.py - Tab điều khiển thủ công drone.

Chứa class ManualControlTab(QWidget) bao gồm:
- Slider điều khiển: Throttle, Roll, Pitch, Yaw, AUX1, AUX2
- Nút: Send Manual, ARM, DISARM, HOLD, RTH, Start Mission
- Nút Takeoff & Hold 3m (tự động cất cánh)
- Nhãn Flight Status
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QSlider, QPushButton
)


class ManualControlTab(QWidget):
    """Tab điều khiển thủ công — slider RC + nút lệnh bay."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._bold_font = QFont()
        self._bold_font.setBold(True)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        # ── Nhóm Slider RC ──
        self._create_rc_sliders()
        layout.addWidget(self.grp_rc_sliders)

        # ── Nhóm nút lệnh ──
        self._create_command_buttons()
        layout.addWidget(self.grp_commands)

        # ── Nút Takeoff & Hold ──
        self.btn_takeoff_hold = QPushButton("🚀 Takeoff & Hold 3m")
        self.btn_takeoff_hold.setMinimumHeight(44)
        self.btn_takeoff_hold.setStyleSheet(
            "QPushButton { background-color: #FF9800; color: white; font-weight: bold; "
            "font-size: 14px; border-radius: 6px; } "
            "QPushButton:hover { background-color: #F57C00; } "
            "QPushButton:disabled { background-color: #555; color: #888; }"
        )
        layout.addWidget(self.btn_takeoff_hold)

        # ── Flight Status ──
        status_layout = QHBoxLayout()
        self.lbl_flight_status = QLabel("Flight Status:")
        self.val_flight_status = QLabel("IDLE")
        self.val_flight_status.setFont(self._bold_font)
        self.val_flight_status.setStyleSheet("color: gray;")
        status_layout.addWidget(self.lbl_flight_status)
        status_layout.addWidget(self.val_flight_status)
        status_layout.addStretch()
        layout.addLayout(status_layout)

        layout.addStretch()

    def _create_rc_sliders(self):
        """Tạo 6 slider RC: Throttle, Roll, Pitch, Yaw, AUX1, AUX2."""
        self.grp_rc_sliders = QGroupBox("RC Channels")
        grid = QGridLayout(self.grp_rc_sliders)

        slider_configs = [
            ("lbl_throttle",    "Throttle", "slider_throttle", "val_throttle",    1000, 2000, 1000, 0),
            ("lbl_roll_input",  "Roll",     "slider_roll",     "val_roll_input",  1000, 2000, 1500, 1),
            ("lbl_pitch_input", "Pitch",    "slider_pitch",    "val_pitch_input", 1000, 2000, 1500, 2),
            ("lbl_yaw_input",   "Yaw",      "slider_yaw",      "val_yaw_input",   1000, 2000, 1500, 3),
            ("lbl_aux1",        "AUX1",     "slider_aux1",     "val_aux1",        1000, 2000, 1000, 4),
            ("lbl_aux2",        "AUX2",     "slider_aux2",     "val_aux2",        1000, 2000, 1000, 5),
        ]

        for lbl_name, lbl_text, slider_name, val_name, min_v, max_v, default, row in slider_configs:
            lbl = QLabel(lbl_text)
            setattr(self, lbl_name, lbl)
            grid.addWidget(lbl, row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_v)
            slider.setMaximum(max_v)
            slider.setValue(default)
            setattr(self, slider_name, slider)
            grid.addWidget(slider, row, 1)

            val = QLabel(str(default))
            setattr(self, val_name, val)
            grid.addWidget(val, row, 2)

    def _create_command_buttons(self):
        """Tạo nhóm nút lệnh bay."""
        self.grp_commands = QGroupBox("Commands")
        btn_layout = QHBoxLayout(self.grp_commands)

        button_configs = [
            ("btn_send_manual",        "Send Manual"),
            ("btn_arm",                "ARM"),
            ("btn_disarm",             "DISARM"),
            ("btn_hold",               "HOLD"),
            ("btn_rth",                "RTH"),
            ("btn_start_mission_main", "Start Mission"),
        ]

        for btn_name, btn_text in button_configs:
            btn = QPushButton(btn_text)
            setattr(self, btn_name, btn)
            btn_layout.addWidget(btn)
