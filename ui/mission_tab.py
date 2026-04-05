"""
mission_tab.py - Tab lập trình lộ trình bay tích hợp bản đồ OpenStreetMap.

Chứa class MissionTab(QWidget) bao gồm:
- Bản đồ OpenStreetMap nhúng qua QWebEngineView + folium
- Drone marker cập nhật vị trí real-time từ GPS BZ 251
- Path tracking — vẽ đường bay theo thời gian thực
- Click-to-add-waypoint — nhấp chuột lên bản đồ để tạo waypoint
- Bảng danh sách Waypoint + nút điều khiển
- Logic an toàn: Cảnh báo khoảng cách + cấu hình failsafe

Bug fixes (2026-04-05):
- Fix QWebChannel warning spam: Tách WebBridge(QObject) riêng biệt,
  không đăng ký MissionTab trực tiếp vào QWebChannel.
- Fix "L is not defined": Dùng loadFinished signal + map_is_ready flag,
  JavaScript chỉ chạy sau khi Leaflet đã load xong.

Thư viện: PySide6-WebEngine, folium (đã cài đặt sẵn)
"""

import os
import json
import math
import tempfile
from PySide6.QtCore import Qt, Slot, QUrl, Signal, QObject
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QGroupBox, QLabel, QLineEdit, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QMessageBox, QSpinBox, QInputDialog
)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebChannel import QWebChannel

try:
    import folium
except ImportError:
    folium = None


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
        self._drone_path = []           # List[tuple]: [(lat, lon), ...] đường bay
        self._drone_lat = 0.0           # Vĩ độ drone hiện tại
        self._drone_lon = 0.0           # Kinh độ drone hiện tại
        self._home_lat = 0.0            # Vĩ độ Home
        self._home_lon = 0.0            # Kinh độ Home
        self._has_home = False          # Đã có vị trí Home chưa
        self._map_initialized = False   # Bản đồ đã load chưa
        self._distance_threshold = self.DEFAULT_DISTANCE_THRESHOLD

        # ── Cờ chống race condition: JavaScript chỉ chạy khi map đã load xong ──
        self.map_is_ready = False

        # ── File tạm cho bản đồ HTML ──
        self._map_file = os.path.join(tempfile.gettempdir(), "drone_gcs_map.html")

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
        splitter.addWidget(self.map_view)

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
        self._load_initial_map()

    # ══════════════════════════════════════════════
    # BẢN ĐỒ (QWebEngineView + folium)
    # ══════════════════════════════════════════════

    def _create_map_view(self):
        """Tạo QWebEngineView để nhúng bản đồ OpenStreetMap."""
        self.map_view = QWebEngineView()
        self.map_view.setMinimumSize(500, 400)

        # ── Thiết lập QWebChannel với WebBridge RIÊNG BIỆT ──
        # KHÔNG đăng ký self (MissionTab) vào channel
        # → tránh cảnh báo "Property has no notify signal"
        self._web_channel = QWebChannel()
        self._web_channel.registerObject("bridge", self._web_bridge)
        self.map_view.page().setWebChannel(self._web_channel)

        # ── Kết nối loadFinished để chạy JS sau khi Leaflet đã sẵn sàng ──
        # FIX: "L is not defined" — do runJavaScript() chạy trước khi Leaflet load
        self.map_view.loadFinished.connect(self._on_map_loaded)

    def _load_initial_map(self):
        """Tải bản đồ OpenStreetMap ban đầu với folium."""
        if folium is None:
            self.map_view.setHtml(
                "<html><body style='background:#1a1a2e; color:#c0c0e0; "
                "display:flex; align-items:center; justify-content:center;'>"
                "<h2>⚠️ Thư viện folium chưa được cài đặt</h2>"
                "</body></html>"
            )
            return

        # Reset cờ — map chưa sẵn sàng cho đến khi loadFinished
        self.map_is_ready = False

        # Tọa độ mặc định: Hà Nội
        default_lat, default_lon = 21.0285, 105.8542

        self._generate_map_html(default_lat, default_lon)
        self.map_view.setUrl(QUrl.fromLocalFile(self._map_file))
        self._map_initialized = True

    def _on_map_loaded(self, ok: bool):
        """
        Callback khi QWebEngineView tải xong HTML.

        FIX cho lỗi "L is not defined":
        - Leaflet (L) chỉ có sẵn SAU KHI HTML load xong hoàn toàn
        - Mọi logic JavaScript thao tác với L phải nằm trong hàm này
        - Set cờ map_is_ready = True để các hàm khác biết có thể gọi runJavaScript()

        Args:
            ok: True nếu tải thành công, False nếu lỗi
        """
        if not ok:
            print("[MissionTab] Lỗi tải bản đồ HTML")
            self.map_is_ready = False
            return

        self.map_is_ready = True

        # ── Inject JavaScript: Click listener cho Leaflet map ──
        # Chạy SAU KHI Leaflet đã sẵn sàng (loadFinished = True)
        click_handler_js = """
        (function() {
            // Khởi tạo QWebChannel để giao tiếp Python <-> JavaScript
            new QWebChannel(qt.webChannelTransport, function(channel) {
                var bridge = channel.objects.bridge;

                // Tìm đối tượng map Leaflet (folium tạo biến global theo ID của div)
                var mapContainers = document.querySelectorAll('.folium-map');
                if (mapContainers.length > 0) {
                    var mapId = mapContainers[0].id;
                    var map = window[mapId];
                    if (map) {
                        map.on('click', function(e) {
                            bridge.on_map_clicked(e.latlng.lat, e.latlng.lng);
                        });
                    }
                }
            });
        })();
        """
        self.map_view.page().runJavaScript(click_handler_js)

    def _generate_map_html(self, center_lat, center_lon, zoom=16):
        """
        Tạo file HTML chứa bản đồ OpenStreetMap bằng folium.

        Bản đồ có:
        - Home marker (nếu đã ARM)
        - Drone marker (vị trí hiện tại)
        - Waypoint markers (đánh số)
        - Mission route (đường nối waypoints)
        - Đường bay tracking (polyline xanh)

        KHÔNG inject JavaScript click listener ở đây.
        Click listener được inject trong _on_map_loaded() sau khi Leaflet sẵn sàng.

        Args:
            center_lat, center_lon: Tâm bản đồ
            zoom: Mức zoom ban đầu
        """
        # Tạo bản đồ folium với tile mặc định OpenStreetMap
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=zoom,
            tiles='OpenStreetMap'
        )

        # ── Thêm Home marker nếu có ──
        if self._has_home:
            folium.Marker(
                [self._home_lat, self._home_lon],
                popup="🏠 HOME",
                icon=folium.Icon(color='green', icon='home', prefix='fa'),
                tooltip="Home Position"
            ).add_to(m)

        # ── Thêm Drone marker (vị trí hiện tại) ──
        if self._drone_lat != 0.0 or self._drone_lon != 0.0:
            folium.Marker(
                [self._drone_lat, self._drone_lon],
                popup="🛸 Drone",
                icon=folium.Icon(color='red', icon='plane', prefix='fa'),
                tooltip="Drone Position"
            ).add_to(m)

        # ── Thêm Waypoint markers (đánh số) ──
        for i, wp in enumerate(self._waypoints):
            folium.Marker(
                [wp["lat"], wp["lon"]],
                popup=f"WP {i+1}: Alt {wp['alt']}m",
                icon=folium.DivIcon(html=f"""
                    <div style="
                        background-color: #FF9800;
                        color: white;
                        border-radius: 50%;
                        width: 28px;
                        height: 28px;
                        display: flex;
                        align-items: center;
                        justify-content: center;
                        font-weight: bold;
                        font-size: 13px;
                        border: 2px solid white;
                        box-shadow: 0 2px 6px rgba(0,0,0,0.4);
                    ">{i+1}</div>
                """),
                tooltip=f"Waypoint {i+1}"
            ).add_to(m)

        # ── Vẽ đường nối waypoints (mission route) ──
        if len(self._waypoints) >= 2:
            wp_coords = [[wp["lat"], wp["lon"]] for wp in self._waypoints]
            folium.PolyLine(
                wp_coords,
                color='#FF9800',
                weight=3,
                opacity=0.8,
                dash_array='10'
            ).add_to(m)

        # ── Vẽ đường bay (path tracking real-time) ──
        if len(self._drone_path) >= 2:
            folium.PolyLine(
                self._drone_path,
                color='#2196F3',
                weight=2,
                opacity=0.7
            ).add_to(m)

        # ── Thêm script qrc cho QWebChannel (chỉ load thư viện, KHÔNG thao tác DOM) ──
        # Click listener sẽ được inject bởi _on_map_loaded() sau khi loadFinished
        webchannel_script = '<script src="qrc:///qtwebchannel/qwebchannel.js"></script>'
        m.get_root().html.add_child(folium.Element(webchannel_script))

        # Lưu ra file HTML tạm
        m.save(self._map_file)

    def refresh_map(self):
        """Tải lại bản đồ với dữ liệu cập nhật (waypoints, drone position, path)."""
        if folium is None or not self._map_initialized:
            return

        # Reset cờ — map sẽ reload, Leaflet chưa sẵn sàng
        self.map_is_ready = False

        center_lat = self._drone_lat if self._drone_lat != 0.0 else 21.0285
        center_lon = self._drone_lon if self._drone_lon != 0.0 else 105.8542

        self._generate_map_html(center_lat, center_lon)
        self.map_view.setUrl(QUrl.fromLocalFile(self._map_file))
        # map_is_ready sẽ được set True trong _on_map_loaded() callback

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
            min=1.0,
            max=120.0,
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

        # Cập nhật bản đồ
        self.refresh_map()

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
            self.refresh_map()
            self.waypoints_updated.emit(self._waypoints)

    def _clear_all_waypoints(self):
        """Xóa toàn bộ waypoint."""
        self._waypoints.clear()
        self.table_waypoints.setRowCount(0)
        self.refresh_map()
        self.waypoints_updated.emit(self._waypoints)

    # ══════════════════════════════════════════════
    # CẬP NHẬT VỊ TRÍ DRONE (gọi từ GCSApp)
    # ══════════════════════════════════════════════

    def update_drone_position(self, lat: float, lon: float):
        """
        Cập nhật vị trí drone hiện tại trên bản đồ.

        Được gọi từ GCSApp khi nhận GPS data từ telemetry.

        Args:
            lat: Vĩ độ drone hiện tại
            lon: Kinh độ drone hiện tại
        """
        if lat == 0.0 and lon == 0.0:
            return  # Không có GPS fix

        self._drone_lat = lat
        self._drone_lon = lon

        # Thêm vào path tracking
        self._drone_path.append((lat, lon))

        # Giới hạn path length để tránh tốn bộ nhớ (giữ 500 điểm cuối)
        if len(self._drone_path) > 500:
            self._drone_path = self._drone_path[-500:]

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
