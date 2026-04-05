# DroneGCS — Nhật Ký Phát Triển (Development Log)

> File này được tự động cập nhật sau mỗi lần sửa đổi code.
> Mục đích: Theo dõi toàn bộ quá trình phát triển dự án.

---

## 2026-04-05 — Feature Update Lớn: Failsafe + Emergency UI + Mission Map

### Tổng quan thay đổi
Triển khai 5 nhóm tính năng lớn:

1. **Hệ thống tracking** — Tạo file `memories.md` (file này)
2. **ESP32 Failsafe chủ động** — ESP32 tự gửi RTH/Safe Land khi mất WiFi
3. **Emergency Notification Overlay** — Bảng cảnh báo nổi + nút DISARM/Safe Land
4. **Nâng cấp Takeoff** — Dialog nhập độ cao + GPS Position Hold
5. **Mission Tab + OpenStreetMap** — Bản đồ tương tác, waypoint click, logic an toàn

### Chi tiết thay đổi từng file

| File | Hành động | Mô tả |
|---|---|---|
| `memories.md` | NEW | File tracking log phát triển |
| `core/drone_state.py` | MODIFY | Thêm thuộc tính GPS (lat, lon, fix, sats, speed, home_lat, home_lon) |
| `comm/msp_parser.py` | MODIFY | Thêm MSP_RAW_GPS (106), MSP_SET_WP (209), MSP_WP_GETINFO (20) |
| `ESP32/main.py` | MODIFY | Thêm logic failsafe chủ động: build MSP frame, gửi RTH/Land qua UART |
| `core/flight_controller.py` | MODIFY | Cập nhật AUX mapping (ARM=2000, ALTHOLD+POSHOLD, Safe Land AUX3, RTH AUX4), thêm safe_land(), rth() |
| `ui/emergency_overlay.py` | NEW | Widget overlay cảnh báo khẩn cấp (DISARM + Safe Land) |
| `ui/manual_control_tab.py` | MODIFY | Đổi nút "Takeoff & Hold 3m" → "Takeoff" |
| `ui/mission_tab.py` | REWRITE | Tích hợp OpenStreetMap (folium + QWebEngineView), waypoint tương tác, logic an toàn |
| `comm/wifi_worker.py` | MODIFY | Thêm MSP_RAW_GPS vào polling, mock GPS data, gửi cấu hình failsafe |
| `main.py` | MODIFY | Kết nối emergency overlay, takeoff dialog, mission logic, GPS data flow |
| `AGENT.md` | MODIFY | Cập nhật tài liệu kiến trúc mới |

### Quyết định thiết kế quan trọng
- **AUX Channel Mapping**: AUX1=ARM, AUX2=Flight Mode, AUX3=Safe Land, AUX4=RTH
- **Mission**: Dùng MSP_WP nạp waypoint xuống FC (INAV tự bay)
- **Home Position**: Read-only từ FC (INAV tự chốt khi ARM)
- **Distance Threshold**: 50m mặc định, có thể cấu hình trên UI
- **ESP32 Failsafe**: Khi mất WiFi → gửi MSP_SET_RAW_RC với AUX4=2000 (RTH) liên tục

### Phần cứng
- GPS + Compass: BZ 251 (đã được thêm vào hệ thống)
- FC: SpeedyBee F405 AIO 40A (INAV firmware)
- Khung: OddityRC XI35 Pro O4 3.5inch Cinewhoop

---

## 2026-04-05 (2) — Bug Fix: QWebChannel Warning + Leaflet Race Condition

### Lỗi đã sửa

| Bug | Nguyên nhân gốc | Cách sửa |
|---|---|---|
| Cảnh báo rác "Property has no notify signal" | Đăng ký MissionTab (QWidget) trực tiếp vào QWebChannel → Qt quét hàng trăm property nội bộ | Tạo class `WebBridge(QObject)` riêng biệt, chỉ expose 1 Slot `on_map_clicked` |
| `js: Uncaught ReferenceError: L is not defined` | JavaScript chạy trước khi Leaflet load xong (race condition) | Dùng `loadFinished.connect(_on_map_loaded)` + cờ `map_is_ready`, inject JS qua `runJavaScript()` trong callback |

### Thay đổi phụ
- NAV POSHOLD: Dải kích hoạt điều chỉnh thành 1900-2100 trên INAV Configurator
- Comment `AUX_NAV_ALTHOLD_POSHOLD` cập nhật phản ánh dải mới

---

## 2026-04-05 (3) — Bug Fix: Mô hình 3D không đồng bộ góc nghiêng

### Nguyên nhân gốc
`update_telemetry_ui()` trong `main.py` cập nhật label text và gọi `widget_3d_attitude.update_attitude()`
nhưng **KHÔNG ghi** `roll/pitch/yaw` vào `drone_state`. Khi chỉ có 1 trong 3 trường được gửi
(ví dụ chỉ có `roll`), widget nhận giá trị cũ từ `_roll/_pitch/_yaw` nội bộ thay vì từ `drone_state`.

### Lỗi đã sửa

| File | Bug | Fix |
|---|---|---|
| `main.py` | `drone_state.roll/pitch/yaw` không bao giờ được ghi | Thêm `self.drone_state.roll = data["roll"]` etc. trước khi cập nhật widget |
| `main.py` | `drone_state.voltage/current` không bao giờ được ghi | Thêm `self.drone_state.voltage = v` và `self.drone_state.current` |
| `main.py` | Widget 3D dùng `data.get()` với fallback cũ thay vì `drone_state` | Đổi thành `self.drone_state.roll/pitch/yaw` — single source of truth |
| `attitude_3d_widget.py` | Panda3D init có thể fail silent (không try/except) | Bọc `showEvent` trong `try/except`, in lỗi ra console nếu fail |
