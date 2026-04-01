"""
attitude_3d_widget.py - Widget placeholder cho mô phỏng 3D tư thế drone.

Hiện tại là placeholder với nền tối và hướng dẫn tích hợp 3D.

Để tích hợp 3D thực tế, sử dụng PyOpenGL hoặc vispy:
    pip install PyOpenGL vispy

Ví dụ tích hợp vispy (thư viện nhẹ, hỗ trợ PySide6):
─────────────────────────────────────────────────────
    from vispy import scene
    from vispy.scene import visuals
    from vispy.io import read_mesh

    class Attitude3DWidget(scene.SceneCanvas):
        def __init__(self):
            super().__init__(keys='interactive', size=(800, 600))
            self.unfreeze()
            view = self.central_widget.add_view()
            view.camera = scene.cameras.TurntableCamera(elevation=30, azimuth=45)

            # Tải mô hình drone .obj hoặc .stl
            vertices, faces, normals, _ = read_mesh('drone_model.obj')
            self.mesh = visuals.Mesh(
                vertices=vertices, faces=faces,
                shading='smooth', color=(0.5, 0.5, 1.0, 1.0)
            )
            view.add(self.mesh)
            self.freeze()

        def update_attitude(self, roll, pitch, yaw):
            from vispy.visuals.transforms import MatrixTransform
            tr = MatrixTransform()
            tr.rotate(roll, (1, 0, 0))
            tr.rotate(pitch, (0, 1, 0))
            tr.rotate(yaw, (0, 0, 1))
            self.mesh.transform = tr
            self.update()
─────────────────────────────────────────────────────

Ví dụ tích hợp PyOpenGL + QOpenGLWidget:
─────────────────────────────────────────────────────
    from PySide6.QtOpenGLWidgets import QOpenGLWidget
    from OpenGL.GL import *
    from stl import mesh  # numpy-stl

    class Attitude3DWidget(QOpenGLWidget):
        def __init__(self):
            super().__init__()
            self._roll = self._pitch = self._yaw = 0.0

        def initializeGL(self):
            glClearColor(0.1, 0.1, 0.15, 1.0)
            glEnable(GL_DEPTH_TEST)
            self._drone_mesh = mesh.Mesh.from_file('drone.stl')

        def paintGL(self):
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            glTranslatef(0, 0, -5)
            glRotatef(self._pitch, 1, 0, 0)
            glRotatef(self._roll, 0, 0, 1)
            glRotatef(self._yaw, 0, 1, 0)
            # Draw mesh triangles...

        def update_attitude(self, roll, pitch, yaw):
            self._roll, self._pitch, self._yaw = roll, pitch, yaw
            self.update()
─────────────────────────────────────────────────────
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QPainter, QColor, QPen, QBrush
from PySide6.QtWidgets import QWidget
import math


class Attitude3DWidget(QWidget):
    """
    Placeholder cho mô phỏng 3D tư thế drone (Roll/Pitch/Yaw).

    Hiện tại vẽ Artificial Horizon 2D (giống đồng hồ bay thật).
    Xem docstring module để biết cách tích hợp OpenGL 3D.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    def update_attitude(self, roll: float, pitch: float, yaw: float):
        """Cập nhật tư thế từ telemetry data."""
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self.update()

    def paintEvent(self, event):
        """Vẽ Artificial Horizon + Compass."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        radius = min(w, h) // 2 - 20

        # ── Nền tối ──
        painter.fillRect(self.rect(), QColor(20, 20, 35))

        # ── Vẽ Artificial Horizon ──
        self._draw_horizon(painter, cx, cy, radius)

        # ── Vẽ Compass (Yaw) ──
        self._draw_compass(painter, cx, h - 40, radius)

        # ── Vẽ thông số text ──
        self._draw_readouts(painter, w, h)

        painter.end()

    def _draw_horizon(self, painter: QPainter, cx: int, cy: int, radius: int):
        """Vẽ đường chân trời nhân tạo (Artificial Horizon)."""
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self._roll)

        pitch_offset = self._pitch * 3  # 3px per degree

        # Sky (xanh dương)
        sky_color = QColor(25, 118, 210)
        painter.setBrush(QBrush(sky_color))
        painter.setPen(Qt.NoPen)
        painter.drawRect(-radius, -radius + int(pitch_offset), radius * 2, radius)

        # Ground (nâu)
        ground_color = QColor(121, 85, 72)
        painter.setBrush(QBrush(ground_color))
        painter.drawRect(-radius, int(pitch_offset), radius * 2, radius)

        # Horizon line
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(-radius, int(pitch_offset), radius, int(pitch_offset))

        # Pitch ladder
        painter.setPen(QPen(QColor(255, 255, 255, 150), 1))
        small_font = QFont("Consolas", 8)
        painter.setFont(small_font)
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y = int(pitch_offset - deg * 3)
            line_w = 30 if abs(deg) % 20 == 0 else 15
            painter.drawLine(-line_w, y, line_w, y)
            if abs(deg) % 20 == 0:
                painter.drawText(line_w + 4, y + 4, f"{deg}°")

        painter.restore()

        # ── Center crosshair (cố định) ──
        painter.setPen(QPen(QColor(255, 200, 0), 3))
        painter.drawLine(cx - 30, cy, cx - 10, cy)
        painter.drawLine(cx + 10, cy, cx + 30, cy)
        painter.drawLine(cx, cy - 5, cx, cy + 5)

        # ── Roll indicator arc ──
        painter.save()
        painter.translate(cx, cy)
        painter.setPen(QPen(QColor(255, 255, 255, 120), 1))
        arc_r = radius - 10
        for deg in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            angle_rad = math.radians(deg - 90)
            x1 = int(arc_r * math.cos(angle_rad))
            y1 = int(arc_r * math.sin(angle_rad))
            tick_len = 10 if deg % 30 == 0 else 5
            x2 = int((arc_r - tick_len) * math.cos(angle_rad))
            y2 = int((arc_r - tick_len) * math.sin(angle_rad))
            painter.drawLine(x1, y1, x2, y2)
        painter.restore()

    def _draw_compass(self, painter: QPainter, cx: int, cy: int, radius: int):
        """Vẽ compass bar đơn giản cho Yaw."""
        painter.setPen(QPen(QColor(200, 200, 220), 1))
        bar_w = min(radius * 2, 300)
        bar_h = 24
        x0 = cx - bar_w // 2

        # Background
        painter.setBrush(QBrush(QColor(30, 30, 50, 200)))
        painter.drawRoundedRect(x0, cy - bar_h // 2, bar_w, bar_h, 4, 4)

        # Compass labels
        painter.setPen(QColor(200, 200, 220))
        compass_font = QFont("Consolas", 9, QFont.Bold)
        painter.setFont(compass_font)
        directions = {0: "N", 45: "NE", 90: "E", 135: "SE",
                      180: "S", 225: "SW", 270: "W", 315: "NW"}
        for deg, label in directions.items():
            offset = ((deg - self._yaw + 180) % 360 - 180)
            px = cx + int(offset * bar_w / 180)
            if x0 < px < x0 + bar_w:
                painter.drawText(px - 8, cy + 5, label)

        # Center triangle
        painter.setPen(QPen(QColor(255, 200, 0), 2))
        painter.drawLine(cx, cy - bar_h // 2 - 4, cx - 4, cy - bar_h // 2 + 2)
        painter.drawLine(cx, cy - bar_h // 2 - 4, cx + 4, cy - bar_h // 2 + 2)

    def _draw_readouts(self, painter: QPainter, w: int, h: int):
        """Vẽ giá trị Roll/Pitch/Yaw dạng text."""
        painter.setPen(QColor(200, 200, 220))
        readout_font = QFont("Consolas", 11, QFont.Bold)
        painter.setFont(readout_font)

        texts = [
            f"ROLL  {self._roll:+6.1f}°",
            f"PITCH {self._pitch:+6.1f}°",
            f"YAW   {self._yaw:6.1f}°",
        ]
        for i, txt in enumerate(texts):
            painter.drawText(10, 25 + i * 20, txt)
