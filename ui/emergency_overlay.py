"""
emergency_overlay.py - Widget cảnh báo khẩn cấp nổi (overlay) trên màn hình chính.

Hiển thị ở góc dưới phải khi drone đang hoạt động (ARM, Takeoff, Mission...).
Chứa 2 nút khẩn cấp:
- DISARM (đỏ): Tắt motor lập tức
- Safe Land (xanh): Hạ cánh an toàn tại chỗ

Luôn nổi lên trên (raise) để người dùng click kịp thời.
Thiết kế: Semi-transparent, glassmorphism, fade-in/out animation.
"""

from PySide6.QtCore import Qt, QPropertyAnimation, QEasingCurve, Property
from PySide6.QtGui import QFont, QColor
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGraphicsOpacityEffect
)


class EmergencyOverlay(QWidget):
    """
    Widget overlay cảnh báo nổi ở góc dưới phải của cửa sổ chính.

    Hiển thị tên mode đang chạy và 2 nút khẩn cấp.
    Tự động show/hide khi mode bay thay đổi.
    """

    # ── Kích thước và vị trí ──
    OVERLAY_WIDTH = 320
    OVERLAY_HEIGHT = 130
    MARGIN_RIGHT = 20       # Khoảng cách từ cạnh phải
    MARGIN_BOTTOM = 20      # Khoảng cách từ cạnh dưới

    def __init__(self, parent=None):
        """
        Khởi tạo overlay.

        Args:
            parent: Widget cha (MainWindow/GCSApp) — dùng để tính vị trí
        """
        super().__init__(parent)

        # ── Cấu hình widget ──
        self.setFixedSize(self.OVERLAY_WIDTH, self.OVERLAY_HEIGHT)
        # Không dùng WindowStaysOnTopHint vì overlay là con của MainWindow
        # Thay vào đó dùng raise_() để đưa lên trên cùng
        self.setAttribute(Qt.WA_TranslucentBackground, False)

        # ── Hiệu ứng mờ dần (opacity effect) cho animation ──
        self._opacity_effect = QGraphicsOpacityEffect(self)
        self._opacity_effect.setOpacity(0.0)
        self.setGraphicsEffect(self._opacity_effect)

        # ── Animation fade-in/out ──
        self._fade_anim = QPropertyAnimation(self._opacity_effect, b"opacity")
        self._fade_anim.setDuration(300)  # 300ms
        self._fade_anim.setEasingCurve(QEasingCurve.InOutQuad)
        self._fade_out_connected = False  # Theo dõi trạng thái connect signal

        # ── Xây dựng UI ──
        self._setup_ui()

        # ── Ẩn mặc định ──
        self.hide()

    def _setup_ui(self):
        """Xây dựng giao diện overlay: label mode + 2 nút khẩn cấp."""
        # ── Stylesheet tổng thể — glassmorphism dark ──
        self.setStyleSheet("""
            EmergencyOverlay {
                background-color: rgba(20, 20, 42, 230);
                border: 2px solid rgba(255, 80, 80, 150);
                border-radius: 12px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(10)

        # ── Label hiển thị tên mode đang chạy ──
        self.lbl_mode_running = QLabel("⚠️ [Mode] is running")
        self.lbl_mode_running.setAlignment(Qt.AlignCenter)
        self.lbl_mode_running.setFont(QFont("Segoe UI", 13, QFont.Bold))
        self.lbl_mode_running.setStyleSheet(
            "color: #FFD54F; background: transparent; border: none;"
        )
        layout.addWidget(self.lbl_mode_running)

        # ── Hàng nút khẩn cấp ──
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        # Nút DISARM (đỏ) — Tắt motor lập tức
        self.btn_emergency_disarm = QPushButton("⛔ DISARM")
        self.btn_emergency_disarm.setMinimumHeight(40)
        self.btn_emergency_disarm.setCursor(Qt.PointingHandCursor)
        self.btn_emergency_disarm.setStyleSheet("""
            QPushButton {
                background-color: #F44336;
                color: white;
                font-weight: bold;
                font-size: 14px;
                border-radius: 8px;
                border: 2px solid #D32F2F;
            }
            QPushButton:hover {
                background-color: #E53935;
                border-color: #FF5252;
            }
            QPushButton:pressed {
                background-color: #C62828;
            }
        """)
        btn_layout.addWidget(self.btn_emergency_disarm)

        # Nút Safe Land (xanh) — Hạ cánh an toàn tại chỗ
        self.btn_emergency_safe_land = QPushButton("🛬 Safe Land")
        self.btn_emergency_safe_land.setMinimumHeight(40)
        self.btn_emergency_safe_land.setCursor(Qt.PointingHandCursor)
        self.btn_emergency_safe_land.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                font-size: 14px;
                border-radius: 8px;
                border: 2px solid #388E3C;
            }
            QPushButton:hover {
                background-color: #43A047;
                border-color: #66BB6A;
            }
            QPushButton:pressed {
                background-color: #2E7D32;
            }
        """)
        btn_layout.addWidget(self.btn_emergency_safe_land)

        layout.addLayout(btn_layout)

    # ══════════════════════════════════════════════
    # API CÔNG KHAI
    # ══════════════════════════════════════════════

    def show_with_mode(self, mode_name: str):
        """
        Hiển thị overlay với tên mode bay đang chạy.

        Args:
            mode_name: Tên mode (VD: "Armed", "Takeoff 5m", "Mission", "RTH")
        """
        if not mode_name:
            self.hide_overlay()
            return

        self.lbl_mode_running.setText(f"⚠️ {mode_name} is running")

        # Đặt vị trí góc dưới phải của cửa sổ cha
        self._update_position()

        # Hiện widget và fade-in
        self.show()
        self.raise_()  # Đưa lên trên cùng z-order

        # Animation fade-in
        self._fade_anim.stop()
        self._fade_anim.setStartValue(self._opacity_effect.opacity())
        self._fade_anim.setEndValue(0.95)
        self._fade_anim.start()

    def hide_overlay(self):
        """Ẩn overlay với animation fade-out."""
        # Animation fade-out
        self._fade_anim.stop()
        self._fade_anim.setStartValue(self._opacity_effect.opacity())
        self._fade_anim.setEndValue(0.0)
        # Chỉ connect nếu chưa connect — tránh signal leak + RuntimeWarning
        if not self._fade_out_connected:
            self._fade_anim.finished.connect(self._on_fade_out_done)
            self._fade_out_connected = True
        self._fade_anim.start()

    def _on_fade_out_done(self):
        """Callback khi animation fade-out kết thúc — ẩn widget."""
        if self._fade_out_connected:
            self._fade_anim.finished.disconnect(self._on_fade_out_done)
            self._fade_out_connected = False
        if self._opacity_effect.opacity() < 0.05:
            self.hide()

    def _update_position(self):
        """Cập nhật vị trí overlay ở góc dưới phải của widget cha."""
        if self.parent():
            parent_rect = self.parent().rect()
            x = parent_rect.width() - self.OVERLAY_WIDTH - self.MARGIN_RIGHT
            y = parent_rect.height() - self.OVERLAY_HEIGHT - self.MARGIN_BOTTOM
            self.move(max(0, x), max(0, y))

    def resizeEvent(self, event):
        """Cập nhật vị trí khi cửa sổ cha thay đổi kích thước."""
        super().resizeEvent(event)
        self._update_position()
