"""
main_window.py - Cửa sổ chính chứa Top Bar và các Tab.

Xây dựng hoàn toàn bằng Pure Python (không phụ thuộc file .ui).

Cấu trúc layout:
    MainWindow (QMainWindow)
    └── centralWidget (QVBoxLayout)
        ├── frame_topbar (Pin + Wifi + Nút kết nối)
        └── tab_widget (QTabWidget)
            ├── DashboardTab  (Telemetry + Motors + Manual Control)
            ├── MissionTab    (Waypoint Planning)
            ├── ConfigTab     (Cấu hình - placeholder)
            └── tab_log       (Log - placeholder)
"""

from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFrame, QLabel, QProgressBar, QPushButton,
    QTabWidget, QSpacerItem, QSizePolicy
)

from ui.dashboard_tab import DashboardTab
from ui.mission_tab import MissionTab
from ui.config_tab import ConfigTab


class MainWindow(QMainWindow):
    """
    Cửa sổ chính của ứng dụng GCS.

    Chỉ chứa UI layout:
    - Top Bar: Hiển thị pin, wifi, nút kết nối
    - Tab Widget: Nhúng DashboardTab, MissionTab, ConfigTab

    Logic xử lý nằm ở GCSApp (main.py), KHÔNG nằm ở đây.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Ground Control Station - INAV MSP")
        self.resize(1200, 800)

        # Font in đậm dùng chung
        self._bold_font = QFont()
        self._bold_font.setBold(True)

        # Xây dựng giao diện
        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng layout chính: Top Bar + Tab Widget."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ── Top Bar ──
        self._create_topbar()
        main_layout.addWidget(self.frame_topbar)

        # ── Tab Widget chứa tất cả các tab ──
        self._create_tabs()
        main_layout.addWidget(self.tab_widget)

    # ══════════════════════════════════════════════
    # TOP BAR (Pin + WiFi + Nút kết nối)
    # ══════════════════════════════════════════════

    def _create_topbar(self):
        """Tạo thanh trạng thái trên cùng: Pin, WiFi, nút Connect/Disconnect."""
        self.frame_topbar = QFrame()
        self.frame_topbar.setFrameShape(QFrame.StyledPanel)
        self.frame_topbar.setFrameShadow(QFrame.Raised)

        h_layout = QHBoxLayout(self.frame_topbar)
        h_layout.setContentsMargins(10, 5, 10, 5)

        # ── Nhóm Pin ──

        # Điện áp (VD: "24.5 V")
        self.lbl_batt_volt = QLabel("-- V")
        self.lbl_batt_volt.setFont(self._bold_font)
        h_layout.addWidget(self.lbl_batt_volt)

        # Thanh hiển thị % pin
        self.bar_battery_volt = QProgressBar()
        self.bar_battery_volt.setMinimumSize(QSize(150, 20))
        self.bar_battery_volt.setMaximumSize(QSize(200, 25))
        self.bar_battery_volt.setValue(0)
        self.bar_battery_volt.setTextVisible(False)
        self.bar_battery_volt.setStyleSheet(
            "QProgressBar { border: 2px solid #8f8f91; border-radius: 5px; "
            "background-color: #e0e0e0; text-align: center; } "
            "QProgressBar::chunk { background-color: gray; border-radius: 3px; }"
        )
        h_layout.addWidget(self.bar_battery_volt)

        # Phần trăm pin (VD: "85 %")
        self.lbl_batt_perc = QLabel("-- %")
        self.lbl_batt_perc.setFont(self._bold_font)
        h_layout.addWidget(self.lbl_batt_perc)

        # ── Khoảng trống giãn ──
        h_layout.addItem(QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum))

        # ── Nhóm WiFi ──

        self.lbl_wifi_status = QLabel("WiFi Status:")
        h_layout.addWidget(self.lbl_wifi_status)

        self.lbl_wifi_icon = QLabel("📶 Đang chờ")
        self.lbl_wifi_icon.setFont(self._bold_font)
        h_layout.addWidget(self.lbl_wifi_icon)

        # Khoảng trống cố định
        h_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

        # ── Nút kết nối/ngắt kết nối ──

        self.btn_disconnect = QPushButton("Ngắt kết nối")
        self.btn_disconnect.setMinimumSize(QSize(120, 30))
        self.btn_disconnect.setStyleSheet(
            "QPushButton { background-color: #F44336; color: white; "
            "font-weight: bold; border-radius: 4px; } "
            "QPushButton:hover { background-color: #D32F2F; }"
        )
        h_layout.addWidget(self.btn_disconnect)

    # ══════════════════════════════════════════════
    # TAB WIDGET
    # ══════════════════════════════════════════════

    def _create_tabs(self):
        """Tạo QTabWidget và nhúng các tab module vào."""
        self.tab_widget = QTabWidget()

        # Tab 1: Dashboard (Telemetry + Motors + Manual Control)
        self.dashboard_tab = DashboardTab()
        self.tab_widget.addTab(self.dashboard_tab, "Dashboard")

        # Tab 2: Mission (Lập trình lộ trình bay)
        self.mission_tab = MissionTab()
        self.tab_widget.addTab(self.mission_tab, "Mission")

        # Tab 3: Config (Cấu hình — placeholder)
        self.config_tab = ConfigTab()
        self.tab_widget.addTab(self.config_tab, "Config")

        # Tab 4: Log (Placeholder — sẽ implement sau)
        self.tab_log = QWidget()
        self.tab_widget.addTab(self.tab_log, "Log")

        # Mặc định hiển thị tab Dashboard
        self.tab_widget.setCurrentIndex(0)
