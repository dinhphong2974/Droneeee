"""
mission_tab.py - Tab lập trình lộ trình bay tích hợp bản đồ OpenStreetMap.

Chứa class MissionTab(QWidget) bao gồm:
- Bản đồ OpenStreetMap nhúng qua QWebEngineView + Leaflet.js (local)
- Drone marker cập nhật vị trí real-time từ GPS BZ 251 (KHÔNG reload page)
- Path tracking — vẽ đường bay theo thời gian thực
- Click-to-add-waypoint — nhấp chuột lên bản đồ để tạo waypoint
- Bảng danh sách Waypoint + nút điều khiển
- Logic an toàn: Cảnh báo khoảng cách + cấu hình failsafe

Architecture:
    assets/map.html     ← Leaflet map (self-contained, local JS/CSS)
    ↕ QWebChannel       ← Cầu nối JavaScript ↔ Python
    mission_tab.py      ← Python backend (QWebEngineView)

Luồng dữ liệu:
    GPS data → update_drone_position() → runJavaScript("updateDronePosition(...)")
    Map click → JS bridge.on_map_clicked() → WebBridge.map_clicked signal → _handle_map_click()

KHÔNG CÒN dùng folium. Leaflet.js load local từ assets/leaflet/.
"""

import os
import json
import math
from PySide6.QtCore import Qt, Slot, QUrl, Signal, QObject
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QGroupBox, QLabel, QLineEdit, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QMessageBox, QSpinBox, QInputDialog
)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebEngineCore import QWebEngineSettings
from PySide6.QtWebChannel import QWebChannel


# ══════════════════════════════════════════════
# HÀM TIỆN ÍCH — TÍNH KHOẢNG CÁCH HAVERSINE
# ══════════════════════════════════════════════

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Tính khoảng cách giữa 2 điểm trên mặt đất bằng công thức Haversine.

    Args:
        lat1, lon1: Tọa độ điểm 1 (độ thập phân)
        lat2, lon2: Tọa độ điểm 2 (độ thập phân)

    Returns:
        float: Khoảng cách (mét)
    """
    R = 6371000  # Bán kính trái đất (mét)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


# ══════════════════════════════════════════════
# WebBridge — Cầu nối Python ↔ JavaScript
# ══════════════════════════════════════════════

class WebBridge(QObject):
    """
    Cầu nối giao tiếp giữa JavaScript (Leaflet map) và Python (MissionTab).

    Class RIÊNG BIỆT kế thừa QObject, được đăng ký vào QWebChannel
    thay vì MissionTab, tránh cảnh báo:
    "Property '...' of object has no notify signal, is not bindable..."

    QWebChannel quét tất cả property/signal/slot của QObject đăng ký.
    MissionTab (QWidget) có rất nhiều property Qt nội bộ không có notify signal
    → gây hàng trăm dòng warning. WebBridge chỉ expose đúng 1 Slot cần thiết.

    Signals:
        map_clicked(float, float): Phát khi user click trên bản đồ Leaflet
    """

    # Signal gửi tọa độ click từ JavaScript → MissionTab
    map_clicked = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)

    @Slot(float, float)
    def on_map_clicked(self, lat: float, lon: float):
        """
        Slot nhận tọa độ từ JavaScript khi user click trên bản đồ.

        JavaScript gọi: bridge.on_map_clicked(lat, lng)
        → Slot này phát signal map_clicked(lat, lon)
        → MissionTab._handle_map_click(lat, lon) xử lý UI

        Args:
            lat: Vĩ độ điểm click (độ thập phân)
            lon: Kinh độ điểm click (độ thập phân)
        """
        self.map_clicked.emit(lat, lon)


class MissionTab(QWidget):
    """
    Tab lập trình lộ trình bay tích hợp bản đồ OpenStreetMap.

    Signals:
        waypoints_updated(list): Danh sách waypoint đã thay đổi
        start_mission_requested(list): Yêu cầu bắt đầu bay mission
        stop_mission_requested(): Yêu cầu dừng mission
        failsafe_config_requested(str): Yêu cầu cấu hình failsafe ("rth" hoặc "ignore")
    """

    # ── Signals giao tiếp với GCSApp ──
    waypoints_updated = Signal(list)
    start_mission_requested = Signal(list)
    stop_mission_requested = Signal()
    failsafe_config_requested = Signal(str)

    # ── Ngưỡng khoảng cách cảnh báo mặc định (mét) ──
    DEFAULT_DISTANCE_THRESHOLD = 50

    def __init__(self, parent=None):
        super().__init__(parent)

        # ── Dữ liệu nội bộ ──
        self._waypoints = []            # List[dict]: {"lat", "lon", "alt"}
        self._drone_lat = 0.0           # Vĩ độ drone hiện tại
        self._drone_lon = 0.0           # Kinh độ drone hiện tại
        self._drone_heading = 0.0       # Hướng bay (yaw)
        self._home_lat = 0.0            # Vĩ độ Home
        self._home_lon = 0.0            # Kinh độ Home
        self._has_home = False          # Đã có vị trí Home chưa
        self._distance_threshold = self.DEFAULT_DISTANCE_THRESHOLD

        # ── Cờ chống race condition: JavaScript chỉ chạy khi map đã load xong ──
        self.map_is_ready = False

        # ── Đường dẫn tuyệt đối tới assets/map.html ──
        # os.path.abspath đảm bảo QUrl.fromLocalFile hoạt động đúng
        # bất kể working directory hiện tại là gì
        self._map_html_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'assets', 'map.html')
        )

        # ── WebBridge: Cầu nối riêng biệt cho QWebChannel ──
        self._web_bridge = WebBridge(self)
        self._web_bridge.map_clicked.connect(self._handle_map_click)

        # ── Xây dựng UI ──
        self._setup_ui()

    def _setup_ui(self):
        """Xây dựng layout: Bản đồ (70%) | Panel điều khiển (30%)."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        splitter = QSplitter(Qt.Horizontal)

        # ── BÊN TRÁI: Bản đồ OpenStreetMap (70%) ──
        self._create_map_view()
        splitter.addWidget(self.map_container)

        # ── BÊN PHẢI: Panel điều khiển (30%) ──
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(8, 8, 8, 8)
        right_layout.setSpacing(8)

        # Nhóm thông tin khoảng cách + cấu hình
        self._create_distance_info()
        right_layout.addWidget(self.grp_distance_info)

        # Bảng waypoint
        self._create_waypoint_list()
        right_layout.addWidget(self.grp_waypoint_list)

        # Nút điều khiển mission
        self._create_action_buttons()
        right_layout.addLayout(self._btn_layout)

        right_layout.addStretch()
        splitter.addWidget(right_panel)

        # Tỷ lệ mặc định 70:30
        splitter.setSizes([700, 300])

        layout.addWidget(splitter)

        # ── Tải bản đồ ban đầu ──
        self._load_map()

    # ══════════════════════════════════════════════
    # BẢN ĐỒ (QWebEngineView + Leaflet local)
    # ══════════════════════════════════════════════

    def _create_map_view(self):
        """Tạo QWebEngineView để nhúng bản đồ OpenStreetMap, với nút định vị drone."""
        # ── Container widget giữ map + nút overlay ──
        self.map_container = QWidget()
        map_container_layout = QVBoxLayout(self.map_container)
        map_container_layout.setContentsMargins(0, 0, 0, 0)

        self.map_view = QWebEngineView()
        self.map_view.setMinimumSize(500, 400)

        # ── Cấu hình WebEngine settings ──
        # LocalContentCanAccessRemoteUrls: Cho phép file:// load tile từ OSM
        # JavascriptEnabled: Bắt buộc cho Leaflet
        # LocalStorageEnabled: Cache tiles nếu cần
        settings = self.map_view.settings()
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True
        )
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.JavascriptEnabled, True
        )
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.LocalStorageEnabled, True
        )

        # ── Thiết lập QWebChannel với WebBridge RIÊNG BIỆT ──
        # KHÔNG đăng ký self (MissionTab) vào channel
        # → tránh cảnh báo "Property has no notify signal"
        self._web_channel = QWebChannel()
        self._web_channel.registerObject("bridge", self._web_bridge)
        self.map_view.page().setWebChannel(self._web_channel)

        # ── Kết nối loadFinished để biết khi nào map sẵn sàng ──
        # JavaScript chỉ chạy SAFE sau khi signal này fire với ok=True
        self.map_view.loadFinished.connect(self._on_map_loaded)

        map_container_layout.addWidget(self.map_view)

        # ── Nút định vị drone — nằm đè lên bản đồ (bottom-left) ──
        # Parent = map_container (KHÔNG phải map_view) vì QWebEngineView
        # render nội dung web trong process riêng, che mất child widget.
        self.btn_locate_drone = QPushButton("📍", self.map_container)
        self.btn_locate_drone.setFixedSize(42, 42)
        self.btn_locate_drone.setToolTip("Định vị drone — Chuyển về vị trí drone hiện tại")
        self.btn_locate_drone.setCursor(Qt.PointingHandCursor)
        self.btn_locate_drone.setStyleSheet("""
            QPushButton {
                background-color: rgba(37, 37, 64, 0.92);
                color: #4FC3F7;
                font-size: 20px;
                border: 2px solid #3a3a60;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: rgba(58, 58, 106, 0.95);
                border-color: #4FC3F7;
            }
            QPushButton:pressed {
                background-color: rgba(79, 195, 247, 0.3);
            }
        """)
        self.btn_locate_drone.clicked.connect(self._locate_drone)
        self.btn_locate_drone.raise_()  # Đảm bảo nút nổi lên trên map

    def _load_map(self):
        """
        Tải file map.html local vào QWebEngineView.

        Dùng QUrl.fromLocalFile với đường dẫn tuyệt đối để QWebEngine
        có thể resolve relative paths (leaflet/leaflet.js, leaflet/leaflet.css)
        từ vị trí thực tế của map.html trên disk.
        """
        self.map_is_ready = False

        if not os.path.isfile(self._map_html_path):
            self.map_view.setHtml(
                "<html><body style='background:#1a1a2e; color:#F44336; "
                "display:flex; align-items:center; justify-content:center;'>"
                f"<h2>⚠️ Không tìm thấy file: {self._map_html_path}</h2>"
                "</body></html>"
            )
            print(f"[MissionTab] ERROR: map.html not found at {self._map_html_path}")
            return

        url = QUrl.fromLocalFile(self._map_html_path)
        self.map_view.setUrl(url)
        print(f"[MissionTab] Loading map from: {url.toString()}")

    def _on_map_loaded(self, ok: bool):
        """
        Callback khi QWebEngineView tải xong HTML.

        window.onload trong map.html đã xử lý việc:
        - Kiểm tra typeof L !== 'undefined'
        - Tạo Leaflet map
        - Setup QWebChannel bridge
        - Bind map click event

        Python side CHỈ CẦN set cờ map_is_ready = True ở đây.
        Mọi runJavaScript() gọi sau thời điểm này là an toàn.

        Args:
            ok: True nếu tải thành công, False nếu lỗi
        """
        if not ok:
            print("[MissionTab] ERROR: Failed to load map.html")
            self.map_is_ready = False
            return

        self.map_is_ready = True
        print("[MissionTab] OK - map.html loaded successfully")

        # ── Đặt vị trí nút định vị (lần đầu) ──
        self._reposition_locate_button()

        # ── Sync lại dữ liệu hiện có lên map (nếu có) ──
        # Trường hợp: GPS data đã nhận trước khi map load xong
        if self._has_home:
            self._js_update_home()
        if self._drone_lat != 0.0 or self._drone_lon != 0.0:
            self._js_update_drone()
        if self._waypoints:
            self._js_update_waypoints()

    # ══════════════════════════════════════════════
    # JAVASCRIPT BRIDGE — Gọi JS từ Python
    # ══════════════════════════════════════════════
    # Tất cả hàm _js_*() chỉ gọi runJavaScript() khi map_is_ready.
    # KHÔNG BAO GIỜ reload page — chỉ thao tác DOM qua JS functions
    # đã được định nghĩa sẵn trong map.html.

    def _run_js(self, script: str):
        """
        Helper an toàn để chạy JavaScript — chỉ thực thi khi map sẵn sàng.

        Nếu map chưa ready, bỏ qua (không queue — dữ liệu sẽ được
        sync lại trong _on_map_loaded khi map load xong).

        Args:
            script: Đoạn JavaScript cần thực thi
        """
        if self.map_is_ready:
            self.map_view.page().runJavaScript(script)

    def _js_update_drone(self):
        """Gọi JS updateDronePosition() — di chuyển drone marker."""
        self._run_js(
            f"updateDronePosition({self._drone_lat}, {self._drone_lon}, {self._drone_heading});"
        )

    def _js_update_home(self):
        """Gọi JS updateHomePosition() — đặt/di chuyển home marker."""
        self._run_js(
            f"updateHomePosition({self._home_lat}, {self._home_lon});"
        )

    def _js_update_waypoints(self):
        """Gọi JS updateWaypoints() — vẽ lại tất cả waypoint markers + route."""
        # BUG-05 FIX: Dùng json.dumps() hai lần để tạo JS string literal an toàn.
        # json.dumps(self._waypoints) → chuỗi JSON (có thể chứa dấu ' nếu có comment)
        # json.dumps(wp_json)         → escape thành JS string literal hợp lệ: "..."
        # Trước đây: f"updateWaypoints('{wp_json}');" bị syntax error nếu wp_json có dấu '
        wp_json = json.dumps(self._waypoints)
        self._run_js(f"updateWaypoints({json.dumps(wp_json)});")

    def _js_set_view(self, lat: float, lon: float, zoom: int = 0):
        """Gọi JS setMapView() — di chuyển bản đồ tới vị trí."""
        self._run_js(f"setMapView({lat}, {lon}, {zoom});")

    def _js_clear_all(self):
        """Gọi JS clearAll() — xóa toàn bộ markers và polylines."""
        self._run_js("clearAll();")

    # ══════════════════════════════════════════════
    # NÚT ĐỊNH VỊ DRONE
    # ══════════════════════════════════════════════

    def _locate_drone(self):
        """
        Chuyển bản đồ ngay lập tức về vị trí GPS hiện tại của drone.

        Nếu chưa có GPS fix (lat=0, lon=0), hiển thị cảnh báo.
        """
        if self._drone_lat == 0.0 and self._drone_lon == 0.0:
            QMessageBox.warning(
                self,
                "Không có vị trí",
                "Chưa nhận được tọa độ GPS từ drone.\n"
                "Hãy đảm bảo drone đã kết nối và có GPS fix."
            )
            return

        self._js_set_view(self._drone_lat, self._drone_lon, 17)
        print(f"[MissionTab] Locate drone: ({self._drone_lat:.6f}, {self._drone_lon:.6f})")

    def resizeEvent(self, event):
        """
        Giữ nút định vị ở góc dưới-trái bản đồ khi resize.
        """
        super().resizeEvent(event)
        self._reposition_locate_button()

    def _reposition_locate_button(self):
        """Đặt nút 📍 ở góc dưới-trái map_container, phía trên attribution."""
        if hasattr(self, 'btn_locate_drone') and hasattr(self, 'map_container'):
            margin = 12
            x = margin
            y = self.map_container.height() - self.btn_locate_drone.height() - margin - 20
            self.btn_locate_drone.move(x, max(y, margin))
            self.btn_locate_drone.raise_()  # Luôn giữ trên cùng

    # ══════════════════════════════════════════════
    # REFRESH MAP (legacy compatibility)
    # ══════════════════════════════════════════════

    def refresh_map(self):
        """
        Cập nhật bản đồ với dữ liệu hiện tại.

        KHÁC VỚI BẢN CŨ: Không reload toàn bộ HTML.
        Chỉ gọi JS functions để cập nhật markers/polylines.
        → Không flicker, không mất state.
        """
        if not self.map_is_ready:
            return

        self._js_update_waypoints()

        if self._has_home:
            self._js_update_home()

        if self._drone_lat != 0.0 or self._drone_lon != 0.0:
            self._js_update_drone()

    # ══════════════════════════════════════════════
    # XỬ LÝ CLICK TỪ BẢN ĐỒ (qua WebBridge)
    # ══════════════════════════════════════════════

    def _handle_map_click(self, lat: float, lon: float):
        """
        Xử lý tọa độ khi user click trên bản đồ Leaflet.

        Nhận signal từ WebBridge.map_clicked → hiện dialog nhập độ cao → thêm waypoint.

        Args:
            lat: Vĩ độ điểm click
            lon: Kinh độ điểm click
        """
        alt, ok = QInputDialog.getDouble(
            self,
            "Thêm Waypoint",
            f"Tọa độ: ({lat:.6f}, {lon:.6f})\n\nNhập độ cao (mét):",
            value=10.0,
            minValue=1.0,
            maxValue=120.0,
            decimals=1
        )
        if ok:
            self._add_waypoint(lat, lon, alt)

    # ══════════════════════════════════════════════
    # THÔNG TIN KHOẢNG CÁCH + CẤU HÌNH
    # ══════════════════════════════════════════════

    def _create_distance_info(self):
        """Tạo nhóm hiển thị khoảng cách GCS-Drone và cấu hình ngưỡng cảnh báo."""
        self.grp_distance_info = QGroupBox("Distance & Safety")
        layout = QVBoxLayout(self.grp_distance_info)
        layout.setSpacing(6)

        # Khoảng cách hiện tại
        dist_layout = QHBoxLayout()
        self.lbl_distance = QLabel("Distance:")
        self.val_distance = QLabel("N/A")
        bold_font = QFont()
        bold_font.setBold(True)
        self.val_distance.setFont(bold_font)
        self.val_distance.setStyleSheet("color: #4CAF50;")
        dist_layout.addWidget(self.lbl_distance)
        dist_layout.addWidget(self.val_distance)
        dist_layout.addStretch()
        layout.addLayout(dist_layout)

        # Cấu hình ngưỡng cảnh báo (mét)
        threshold_layout = QHBoxLayout()
        self.lbl_threshold = QLabel("Ngưỡng cảnh báo:")
        self.spin_threshold = QSpinBox()
        self.spin_threshold.setRange(10, 500)
        self.spin_threshold.setValue(self.DEFAULT_DISTANCE_THRESHOLD)
        self.spin_threshold.setSuffix(" m")
        self.spin_threshold.setStyleSheet(
            "QSpinBox { background-color: #252540; color: #d0d0e8; "
            "border: 1px solid #2a2a4a; border-radius: 4px; padding: 4px; }"
        )
        self.spin_threshold.valueChanged.connect(self._on_threshold_changed)
        threshold_layout.addWidget(self.lbl_threshold)
        threshold_layout.addWidget(self.spin_threshold)
        layout.addLayout(threshold_layout)

        # Trạng thái failsafe
        fs_layout = QHBoxLayout()
        self.lbl_failsafe_status = QLabel("Failsafe:")
        self.val_failsafe_status = QLabel("RTH (mặc định)")
        self.val_failsafe_status.setFont(bold_font)
        self.val_failsafe_status.setStyleSheet("color: #4CAF50;")
        fs_layout.addWidget(self.lbl_failsafe_status)
        fs_layout.addWidget(self.val_failsafe_status)
        fs_layout.addStretch()
        layout.addLayout(fs_layout)

    def _on_threshold_changed(self, value):
        """Cập nhật ngưỡng khoảng cách cảnh báo."""
        self._distance_threshold = value

    # ══════════════════════════════════════════════
    # BẢNG DANH SÁCH WAYPOINT
    # ══════════════════════════════════════════════

    def _create_waypoint_list(self):
        """Tạo bảng hiển thị danh sách waypoint đã thêm."""
        self.grp_waypoint_list = QGroupBox("Waypoint List")
        v_layout = QVBoxLayout(self.grp_waypoint_list)

        self.table_waypoints = QTableWidget()
        self.table_waypoints.setColumnCount(4)
        self.table_waypoints.setHorizontalHeaderItem(0, QTableWidgetItem("#"))
        self.table_waypoints.setHorizontalHeaderItem(1, QTableWidgetItem("Lat"))
        self.table_waypoints.setHorizontalHeaderItem(2, QTableWidgetItem("Lon"))
        self.table_waypoints.setHorizontalHeaderItem(3, QTableWidgetItem("Alt (m)"))

        header = self.table_waypoints.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        v_layout.addWidget(self.table_waypoints)

    # ══════════════════════════════════════════════
    # NÚT ĐIỀU KHIỂN MISSION
    # ══════════════════════════════════════════════

    def _create_action_buttons(self):
        """Tạo hàng nút: Xóa, Clear, Upload, Start, Stop."""
        self._btn_layout = QVBoxLayout()

        # Hàng 1: Quản lý waypoint
        row1 = QHBoxLayout()
        self.btn_remove = QPushButton("🗑️ Remove")
        self.btn_remove.clicked.connect(self._remove_selected_waypoint)
        self.btn_remove.setStyleSheet(
            "QPushButton { background-color: #F44336; color: white; font-weight: bold; border-radius: 6px; }"
            "QPushButton:hover { background-color: #E53935; }"
        )
        row1.addWidget(self.btn_remove)

        self.btn_clear = QPushButton("🧹 Clear All")
        self.btn_clear.clicked.connect(self._clear_all_waypoints)
        self.btn_clear.setStyleSheet(
            "QPushButton { background-color: #F44336; color: white; font-weight: bold; border-radius: 6px; }"
            "QPushButton:hover { background-color: #E53935; }"
        )
        row1.addWidget(self.btn_clear)
        self._btn_layout.addLayout(row1)

        # Hàng 2: Điều khiển mission
        row2 = QHBoxLayout()
        self.btn_upload = QPushButton("⬆️ Upload To FC")
        self.btn_upload.setStyleSheet(
            "QPushButton { background-color: #2196F3; color: white; font-weight: bold; border-radius: 6px; }"
            "QPushButton:hover { background-color: #1E88E5; }"
        )
        row2.addWidget(self.btn_upload)

        self.btn_start_mission_tab = QPushButton("▶️ Start Mission")
        self.btn_start_mission_tab.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; border-radius: 6px; }"
            "QPushButton:hover { background-color: #43A047; }"
        )
        row2.addWidget(self.btn_start_mission_tab)
        self._btn_layout.addLayout(row2)

        # Hàng 3: Dừng mission
        self.btn_stop_mission = QPushButton("⏹️ Stop Mission")
        self.btn_stop_mission.setStyleSheet(
            "QPushButton { background-color: #F44336; color: white; font-weight: bold; border-radius: 6px; }"
            "QPushButton:hover { background-color: #E53935; }"
        )
        self._btn_layout.addWidget(self.btn_stop_mission)

    # ══════════════════════════════════════════════
    # QUẢN LÝ WAYPOINT
    # ══════════════════════════════════════════════

    def _add_waypoint(self, lat: float, lon: float, alt: float):
        """
        Thêm waypoint mới vào danh sách và cập nhật bảng + bản đồ.

        Args:
            lat: Vĩ độ
            lon: Kinh độ
            alt: Độ cao (mét)
        """
        wp = {"lat": lat, "lon": lon, "alt": alt}
        self._waypoints.append(wp)

        # Thêm vào bảng hiển thị
        row = self.table_waypoints.rowCount()
        self.table_waypoints.insertRow(row)
        self.table_waypoints.setItem(row, 0, QTableWidgetItem(str(row + 1)))
        self.table_waypoints.setItem(row, 1, QTableWidgetItem(f"{lat:.6f}"))
        self.table_waypoints.setItem(row, 2, QTableWidgetItem(f"{lon:.6f}"))
        self.table_waypoints.setItem(row, 3, QTableWidgetItem(f"{alt:.1f}"))

        # Cập nhật waypoint markers trên bản đồ (KHÔNG reload page)
        self._js_update_waypoints()

        # Phát signal
        self.waypoints_updated.emit(self._waypoints)

    def _remove_selected_waypoint(self):
        """Xóa waypoint đang chọn trong bảng."""
        row = self.table_waypoints.currentRow()
        if row >= 0 and row < len(self._waypoints):
            self._waypoints.pop(row)
            self.table_waypoints.removeRow(row)
            # Cập nhật lại số thứ tự
            for i in range(self.table_waypoints.rowCount()):
                self.table_waypoints.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self._js_update_waypoints()
            self.waypoints_updated.emit(self._waypoints)

    def _clear_all_waypoints(self):
        """Xóa toàn bộ waypoint."""
        self._waypoints.clear()
        self.table_waypoints.setRowCount(0)
        self._js_update_waypoints()
        self.waypoints_updated.emit(self._waypoints)

    # ══════════════════════════════════════════════
    # CẬP NHẬT VỊ TRÍ DRONE (gọi từ GCSApp)
    # ══════════════════════════════════════════════

    def update_drone_position(self, lat: float, lon: float, heading: float = 0.0):
        """
        Cập nhật vị trí drone hiện tại trên bản đồ.

        Được gọi từ GCSApp khi nhận GPS data từ telemetry.
        KHÔNG reload page — gọi JS updateDronePosition() để di chuyển marker.

        Args:
            lat: Vĩ độ drone hiện tại
            lon: Kinh độ drone hiện tại
            heading: Hướng bay (yaw, độ). Mặc định 0.
        """
        if lat == 0.0 and lon == 0.0:
            return  # Không có GPS fix

        self._drone_lat = lat
        self._drone_lon = lon
        self._drone_heading = heading

        # Cập nhật drone marker trên bản đồ (chỉ gọi JS, không reload)
        self._js_update_drone()

        # Cập nhật khoảng cách hiển thị
        self._update_distance_display()

    def update_home_position(self, lat: float, lon: float):
        """
        Cập nhật vị trí Home (read-only từ FC, chốt khi ARM).

        Args:
            lat: Vĩ độ Home
            lon: Kinh độ Home
        """
        if lat == 0.0 and lon == 0.0:
            return
        self._home_lat = lat
        self._home_lon = lon
        self._has_home = True

        # Cập nhật home marker trên bản đồ
        self._js_update_home()

    def update_telemetry(self, lat: float, lon: float, heading: float = 0.0):
        """
        Alias cho update_drone_position — tương thích với tên gọi trong spec.

        Args:
            lat: Vĩ độ
            lon: Kinh độ
            heading: Hướng bay (độ)
        """
        self.update_drone_position(lat, lon, heading)

    def _update_distance_display(self):
        """Cập nhật hiển thị khoảng cách drone → Home."""
        if not self._has_home or (self._drone_lat == 0.0 and self._drone_lon == 0.0):
            self.val_distance.setText("N/A")
            return

        dist = haversine_distance(
            self._home_lat, self._home_lon,
            self._drone_lat, self._drone_lon
        )

        # Cập nhật label khoảng cách
        if dist < 1000:
            self.val_distance.setText(f"{dist:.0f} m")
        else:
            self.val_distance.setText(f"{dist/1000:.2f} km")

        # Đổi màu theo ngưỡng cảnh báo
        if dist > self._distance_threshold:
            self.val_distance.setStyleSheet("color: #F44336; font-weight: bold;")  # Đỏ
        else:
            self.val_distance.setStyleSheet("color: #4CAF50; font-weight: bold;")  # Xanh

    def check_distance_warning(self) -> bool:
        """
        Kiểm tra khoảng cách drone có vượt ngưỡng không.

        Returns:
            True nếu vượt ngưỡng cảnh báo
        """
        if not self._has_home:
            return False

        dist = haversine_distance(
            self._home_lat, self._home_lon,
            self._drone_lat, self._drone_lon
        )
        return dist > self._distance_threshold

    def get_waypoints(self) -> list[dict]:
        """Trả về danh sách waypoint hiện tại."""
        return self._waypoints.copy()
