"""
attitude_3d_widget.py
Mô phỏng 3D tư thế drone dùng Panda3D + PySide6 + file GLB.

Tính năng:
- Nhúng Panda3D vào QWidget của PySide6
- Tự động tìm model drone từ thư mục: ../assets/drone.glb
- Tự động convert đường dẫn Windows sang chuẩn Panda3D VFS
- Cập nhật realtime roll / pitch / yaw
"""

import os
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer, Qt

# Import thư viện Panda3D
from panda3d.core import (
    WindowProperties,
    loadPrcFileData,
    AmbientLight,
    DirectionalLight,
    LineSegs,
    TextNode,
    CardMaker,
    Filename,  # <--- THÊM CLASS NÀY ĐỂ XỬ LÝ ĐƯỜNG DẪN WINDOWS
)
from direct.showbase.ShowBase import ShowBase


class Panda3DEngine(ShowBase):
    """Lõi render Panda3D."""

    def __init__(self, parent_hwnd: int, width: int, height: int, model_path: str):
        # Không cho Panda3D tự mở cửa sổ riêng
        loadPrcFileData("", "window-type none")
        loadPrcFileData("", "sync-video true")
        loadPrcFileData("", "show-frame-rate-meter false")

        loadPrcFileData("", "notify-level-windisplay error")

        super().__init__()

        # Nhúng cửa sổ Panda3D vào QWidget
        props = WindowProperties()
        props.setParentWindow(parent_hwnd)
        props.setOrigin(0, 0)
        props.setSize(max(1, width), max(1, height))
        
        self.makeDefaultPipe()
        self.openDefaultWindow(props=props)
        self.setBackgroundColor(0.9, 0.9, 0.9, 1.0)

        # Node attitude: nhận yaw/pitch/roll realtime
        self.attitude_node = self.render.attachNewNode("attitude_node")

        # Node model: dùng để căn chỉnh trục model import
        self.model_node = self.attitude_node.attachNewNode("model_node")

        # Biến attitude hiện tại để smoothing
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Biến attitude đích
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

        # Load model drone
        self.drone_model = None
        self._load_drone_model(model_path)

        # Camera
        self.disableMouse()
        self.camera.setPos(0, -12, 4)
        self.camera.lookAt(0, 0, 0)

        # Ánh sáng
        self._setup_lighting()
        self._create_axes()

    def _load_drone_model(self, model_path: str):
        """Load model .glb và gắn vào scene."""
        try:
            self.drone_model = self.loader.loadModel(model_path)
            self.drone_model.reparentTo(self.model_node)

            # Đặt model tại gốc tọa độ
            self.drone_model.setPos(0, 0, 0)
            self.drone_model.setScale(8.0)
            self.drone_model.setColorScale(1.0, 1.0, 1.0, 1.0)

            # Cú pháp: setHpr(Heading, Pitch, Roll)
            # Heading chính là góc xoay quanh trục Z (Yaw).
            self.model_node.setHpr(90, 0, 0)


        except Exception as e:
            print(f"[ERROR] Không load được model '{model_path}': {e}")
            self.drone_model = None

    def _setup_lighting(self):
        alight = AmbientLight("ambient_light")
        alight.setColor((0.45, 0.45, 0.45, 1.0))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        dlight = DirectionalLight("directional_light")
        dlight.setColor((0.9, 0.9, 0.9, 1.0))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(45, -35, 0)
        self.render.setLight(dlnp)

    def _create_axes(self):
        segs = LineSegs()
        segs.setThickness(2.0)
        segs.setColor(1, 0, 0, 1)
        segs.moveTo(0, 0, 0); segs.drawTo(2, 0, 0)
        segs.setColor(0, 1, 0, 1)
        segs.moveTo(0, 0, 0); segs.drawTo(0, 2, 0)
        segs.setColor(0, 0, 1, 1)
        segs.moveTo(0, 0, 0); segs.drawTo(0, 0, 2)
        axes_node = segs.create()
        self.render.attachNewNode(axes_node)

        self._make_axis_label("X", (2.15, 0, 0), (1, 0, 0, 1))
        self._make_axis_label("Y", (0, 2.15, 0), (0, 1, 0, 1))
        self._make_axis_label("Z", (0, 0, 2.15), (0, 0, 1, 1))

    def _make_axis_label(self, text, pos, color):
        text_node = TextNode(f"label_{text}")
        text_node.setText(text)
        text_node.setTextColor(*color)
        text_node_path = self.render.attachNewNode(text_node)
        text_node_path.setScale(0.25)
        text_node_path.setPos(*pos)
        text_node_path.setBillboardPointEye()

    def _create_ground(self):
        cm = CardMaker("ground")
        cm.setFrame(-10, 10, -10, 10)
        ground = self.render.attachNewNode(cm.generate())
        ground.setP(-90)
        ground.setZ(-1.5)
        ground.setColor(0.15, 0.15, 0.18, 1.0)

    def set_attitude_target(self, roll: float, pitch: float, yaw: float):
        self.target_roll = roll
        self.target_pitch = pitch
        self.target_yaw = yaw

    def update_attitude_smooth(self, alpha: float = 0.2):
        self.current_roll = (1 - alpha) * self.current_roll + alpha * self.target_roll
        self.current_pitch = (1 - alpha) * self.current_pitch + alpha * self.target_pitch
        self.current_yaw = (1 - alpha) * self.current_yaw + alpha * self.target_yaw

        self.attitude_node.setHpr(
            -self.current_yaw,   # Yaw
            self.current_roll,    # Roll
            self.current_pitch  # Pitch
            
        )


class Attitude3DWidget(QWidget):
    """QWidget chứa Panda3D."""

    def __init__(self, parent=None, model_filename: str = "drone.glb"):
        super().__init__(parent)
        self.setMinimumSize(400, 300)

        # 1. Lấy thư mục chứa file attitude_3d_widget.py (thư mục 'ui')
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 2. Lùi lại một cấp để ra thư mục gốc
        root_dir = os.path.dirname(current_dir)
        
        # 3. Nối đường dẫn dạng OS thông thường (vd: C:\...\assets\drone.glb)
        os_path = os.path.join(root_dir, 'assets', model_filename)
        
        # 4. CHUYỂN ĐỔI: Ép đường dẫn Windows sang chuẩn VFS của Panda3D
        panda_path = Filename.fromOsSpecific(os_path)
        
        # Lấy chuỗi đường dẫn chuẩn (vd: /c/Test/.../assets/drone.glb)
        self.model_path = panda_path.getFullpath()
        
        self.panda_engine = None

        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self._step_panda)

    def showEvent(self, event):
        super().showEvent(event)
        if self.panda_engine is None:
            try:
                win_id = int(self.winId())
                self.panda_engine = Panda3DEngine(
                    parent_hwnd=win_id,
                    width=self.width(),
                    height=self.height(),
                    model_path=self.model_path
                )
                self.timer.start(16)
            except Exception as e:
                print(f"[Attitude3DWidget] Lỗi khởi tạo Panda3D: {e}")
                print(f"[Attitude3DWidget] Model path: {self.model_path}")
                self.panda_engine = None

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.panda_engine and self.panda_engine.win:
            props = WindowProperties()
            props.setSize(max(1, self.width()), max(1, self.height()))
            self.panda_engine.win.requestProperties(props)

    def closeEvent(self, event):
        self.timer.stop()
        if self.panda_engine:
            try:
                self.panda_engine.userExit()
            except Exception:
                pass
        super().closeEvent(event)

    def _step_panda(self):
        if self.panda_engine:
            self.panda_engine.update_attitude_smooth(alpha=0.2)
            self.panda_engine.taskMgr.step()

    def update_attitude(self, roll: float, pitch: float, yaw: float):
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw

        if self.panda_engine:
            self.panda_engine.set_attitude_target(roll, pitch, yaw)