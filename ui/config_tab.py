"""
config_tab.py - Tab cấu hình thông số bay (tương tự INAV Configurator).

Chứa class ConfigTab(QWidget) — hiện tại là placeholder,
sẽ mở rộng thêm: PID Tuning, Rate Profiles, Motor Mapping, Sensor Config...
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel


class ConfigTab(QWidget):
    """Tab cấu hình — placeholder cho các tính năng tương lai."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng giao diện placeholder."""
        layout = QVBoxLayout(self)

        # Thông báo placeholder
        lbl_placeholder = QLabel("⚙️ Tab Cấu Hình")
        lbl_placeholder.setAlignment(Qt.AlignCenter)
        placeholder_font = QFont()
        placeholder_font.setPointSize(16)
        placeholder_font.setBold(True)
        lbl_placeholder.setFont(placeholder_font)
        layout.addWidget(lbl_placeholder)

        lbl_desc = QLabel(
            "Tính năng đang phát triển...\n\n"
            "Dự kiến:\n"
            "• PID Tuning (Roll/Pitch/Yaw)\n"
            "• Rate Profiles\n"
            "• Motor Mapping & Direction\n"
            "• Sensor Calibration\n"
            "• Failsafe Configuration"
        )
        lbl_desc.setAlignment(Qt.AlignCenter)
        lbl_desc.setStyleSheet("color: gray; font-size: 12px;")
        layout.addWidget(lbl_desc)

        # Đẩy nội dung lên giữa màn hình
        layout.addStretch()
