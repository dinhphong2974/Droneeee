"""
main_window.py - Cửa sổ chính với Navigation Rail + Dark Theme.

Cấu trúc layout:
    MainWindow (QMainWindow)
    └── centralWidget (QVBoxLayout)
        ├── frame_topbar (Pin + Wifi + Nút kết nối)
        └── QHBoxLayout
            ├── NavRail (thanh điều hướng dọc bên trái)
            └── QStackedWidget (nội dung bên phải)
                ├── DashboardTab     (Attitude 3D + Telemetry + Motors)
                ├── ManualControlTab (Sliders + Buttons + Takeoff)
                ├── MissionTab       (Waypoint Planning)
                ├── ConfigTab        (Cấu hình)
                └── tab_log          (Log - placeholder)
"""

from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont, QColor
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFrame, QLabel, QProgressBar, QPushButton,
    QStackedWidget, QSpacerItem, QSizePolicy, QListWidget, QListWidgetItem
)

from ui.dashboard_tab import DashboardTab
from ui.manual_control_tab import ManualControlTab
from ui.mission_tab import MissionTab
from ui.config_tab import ConfigTab

# ══════════════════════════════════════════════
# DARK THEME STYLESHEET
# ══════════════════════════════════════════════

DARK_THEME = """
QMainWindow, QWidget {
    background-color: #1a1a2e;
    color: #d0d0e8;
    font-family: 'Segoe UI', 'Roboto', sans-serif;
}
QGroupBox {
    border: 1px solid #2a2a4a;
    border-radius: 8px;
    margin-top: 14px;
    padding-top: 18px;
    font-weight: bold;
    color: #a0a0c8;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}
QLabel {
    color: #c0c0e0;
    border: none;
}
QProgressBar {
    border: 1px solid #2a2a4a;
    border-radius: 4px;
    background-color: #252540;
    text-align: center;
}
QProgressBar::chunk {
    border-radius: 3px;
    background-color: #4CAF50;
}
QPushButton {
    background-color: #2d2d50;
    color: #d0d0e8;
    border: 1px solid #3a3a60;
    border-radius: 6px;
    padding: 6px 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #3a3a6a;
    border-color: #5a5a90;
}
QPushButton:pressed {
    background-color: #1a1a30;
}
QPushButton:disabled {
    background-color: #1a1a2a;
    color: #505070;
    border-color: #252540;
}
QSlider::groove:horizontal {
    border: 1px solid #2a2a4a;
    height: 6px;
    background: #252540;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #6c6cff;
    border: 1px solid #8a8aff;
    width: 16px;
    margin: -5px 0;
    border-radius: 8px;
}
QSlider::handle:horizontal:hover {
    background: #8a8aff;
}
QLineEdit {
    background-color: #252540;
    color: #d0d0e8;
    border: 1px solid #2a2a4a;
    border-radius: 4px;
    padding: 5px 8px;
    selection-background-color: #4a4a90;
}
QTableWidget {
    background-color: #1e1e38;
    color: #d0d0e8;
    border: 1px solid #2a2a4a;
    gridline-color: #2a2a4a;
    alternate-background-color: #252540;
}
QTableWidget::item { padding: 4px; }
QHeaderView::section {
    background-color: #2a2a4a;
    color: #c0c0e0;
    border: 1px solid #3a3a60;
    padding: 4px;
    font-weight: bold;
}
QScrollBar:vertical {
    background: #1a1a2e;
    width: 10px;
    border-radius: 5px;
}
QScrollBar::handle:vertical {
    background: #3a3a60;
    border-radius: 5px;
    min-height: 20px;
}
QSplitter::handle {
    background-color: #2a2a4a;
    width: 3px;
}
QFrame#frame_topbar {
    background-color: #14142a;
    border-bottom: 1px solid #2a2a4a;
}
"""


class MainWindow(QMainWindow):
    """
    Cửa sổ chính của ứng dụng GCS với Navigation Rail.

    Chỉ chứa UI layout:
    - Top Bar: Hiển thị pin, wifi, nút kết nối
    - Nav Rail: Điều hướng dọc bên trái
    - Stacked Widget: Nội dung trang bên phải

    Logic xử lý nằm ở GCSApp (main.py), KHÔNG nằm ở đây.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Ground Control Station - INAV MSP")
        self.resize(1400, 900)

        # Font in đậm dùng chung
        self._bold_font = QFont()
        self._bold_font.setBold(True)

        # Áp dụng Dark Theme
        self.setStyleSheet(DARK_THEME)

        # Xây dựng giao diện
        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng layout chính: Top Bar + [Nav Rail | Content]."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ── Top Bar ──
        self._create_topbar()
        main_layout.addWidget(self.frame_topbar)

        # ── Body: Nav Rail (trái) + Content (phải) ──
        body_layout = QHBoxLayout()
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(0)

        self._create_nav_rail()
        body_layout.addWidget(self.nav_rail)

        self._create_pages()
        body_layout.addWidget(self.stacked_widget)

        main_layout.addLayout(body_layout)

    # ══════════════════════════════════════════════
    # TOP BAR (Pin + WiFi + Nút kết nối)
    # ══════════════════════════════════════════════

    def _create_topbar(self):
        """Tạo thanh trạng thái trên cùng."""
        self.frame_topbar = QFrame()
        self.frame_topbar.setObjectName("frame_topbar")
        self.frame_topbar.setFixedHeight(48)

        h_layout = QHBoxLayout(self.frame_topbar)
        h_layout.setContentsMargins(16, 6, 16, 6)

        # ── Nhóm Pin ──
        self.lbl_batt_volt = QLabel("-- V")
        self.lbl_batt_volt.setFont(self._bold_font)
        self.lbl_batt_volt.setStyleSheet("color: #e0e0ff;")
        h_layout.addWidget(self.lbl_batt_volt)

        self.bar_battery_volt = QProgressBar()
        self.bar_battery_volt.setMinimumSize(QSize(150, 18))
        self.bar_battery_volt.setMaximumSize(QSize(200, 22))
        self.bar_battery_volt.setValue(0)
        self.bar_battery_volt.setTextVisible(False)
        h_layout.addWidget(self.bar_battery_volt)

        self.lbl_batt_perc = QLabel("-- %")
        self.lbl_batt_perc.setFont(self._bold_font)
        self.lbl_batt_perc.setStyleSheet("color: #e0e0ff;")
        h_layout.addWidget(self.lbl_batt_perc)

        # ── Khoảng trống giãn ──
        h_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum))

        # ── Ping (RTT GCS↔ESP32) ──
        self.lbl_ping_title = QLabel("Ping:")
        self.lbl_ping_title.setStyleSheet("color: #808098;")
        h_layout.addWidget(self.lbl_ping_title)

        self.lbl_ping = QLabel("🏓 ---ms")
        self.lbl_ping.setFont(self._bold_font)
        self.lbl_ping.setStyleSheet("color: #808098;")
        self.lbl_ping.setMinimumWidth(90)
        h_layout.addWidget(self.lbl_ping)

        h_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

        # ── Nhóm WiFi ──
        self.lbl_wifi_status = QLabel("WiFi Status:")
        self.lbl_wifi_status.setStyleSheet("color: #808098;")
        h_layout.addWidget(self.lbl_wifi_status)

        self.lbl_wifi_icon = QLabel("📶 Đang chờ")
        self.lbl_wifi_icon.setFont(self._bold_font)
        h_layout.addWidget(self.lbl_wifi_icon)

        h_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

        # ── Nút kết nối ──
        self.btn_disconnect = QPushButton("Ngắt kết nối")
        self.btn_disconnect.setMinimumSize(QSize(130, 32))
        self.btn_disconnect.setStyleSheet(
            "QPushButton { background-color: #F44336; color: white; "
            "font-weight: bold; border-radius: 4px; } "
            "QPushButton:hover { background-color: #D32F2F; }"
        )
        h_layout.addWidget(self.btn_disconnect)

    # ══════════════════════════════════════════════
    # NAVIGATION RAIL (thanh dọc bên trái)
    # ══════════════════════════════════════════════

    def _create_nav_rail(self):
        """Tạo thanh điều hướng dọc bên trái với icons + text."""
        self.nav_rail = QListWidget()
        self.nav_rail.setFixedWidth(160)
        self.nav_rail.setSpacing(2)
        self.nav_rail.setStyleSheet("""
            QListWidget {
                background-color: #14142a;
                border: none;
                border-right: 1px solid #2a2a4a;
                outline: none;
                padding-top: 8px;
            }
            QListWidget::item {
                color: #808098;
                padding: 12px 16px;
                border-radius: 8px;
                margin: 2px 8px;
                font-size: 13px;
                font-weight: bold;
            }
            QListWidget::item:selected {
                background-color: #2a2a55;
                color: #a0a0ff;
                border-left: 3px solid #6c6cff;
            }
            QListWidget::item:hover:!selected {
                background-color: #1e1e40;
                color: #c0c0e0;
            }
        """)

        nav_items = [
            ("📊  Dashboard",       0),
            ("🎮  Manual Control",  1),
            ("🗺️  Mission",         2),
            ("⚙️  Config",          3),
            ("📋  Log",             4),
        ]

        for text, idx in nav_items:
            item = QListWidgetItem(text)
            item.setSizeHint(QSize(140, 44))
            item.setData(Qt.UserRole, idx)
            self.nav_rail.addItem(item)

        # Mặc định chọn Dashboard
        self.nav_rail.setCurrentRow(0)

        # Kết nối chuyển trang
        self.nav_rail.currentRowChanged.connect(self._on_nav_changed)

    # ══════════════════════════════════════════════
    # PAGES (QStackedWidget)
    # ══════════════════════════════════════════════

    def _create_pages(self):
        """Tạo QStackedWidget chứa các trang nội dung."""
        self.stacked_widget = QStackedWidget()

        # Page 0: Dashboard (Attitude 3D + Telemetry + Motors)
        self.dashboard_tab = DashboardTab()
        self.stacked_widget.addWidget(self.dashboard_tab)

        # Page 1: Manual Control (Sliders + Buttons)
        self.manual_control_tab = ManualControlTab()
        self.stacked_widget.addWidget(self.manual_control_tab)

        # Page 2: Mission (Waypoint Planning)
        self.mission_tab = MissionTab()
        self.stacked_widget.addWidget(self.mission_tab)

        # Page 3: Config (placeholder)
        self.config_tab = ConfigTab()
        self.stacked_widget.addWidget(self.config_tab)

        # Page 4: Log (placeholder)
        self.tab_log = QWidget()
        log_layout = QVBoxLayout(self.tab_log)
        lbl_log = QLabel("📋 Log — Đang phát triển...")
        lbl_log.setAlignment(Qt.AlignCenter)
        lbl_log.setStyleSheet("color: #808098; font-size: 16px; font-weight: bold;")
        log_layout.addWidget(lbl_log)
        self.stacked_widget.addWidget(self.tab_log)

        # Mặc định hiển thị Dashboard
        self.stacked_widget.setCurrentIndex(0)

    def _on_nav_changed(self, index: int):
        """Chuyển trang khi chọn mục trên Nav Rail."""
        if 0 <= index < self.stacked_widget.count():
            self.stacked_widget.setCurrentIndex(index)
