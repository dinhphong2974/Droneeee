# DroneGCS — Agent Context Document

> **Mục đích**: Cung cấp toàn bộ bối cảnh dự án cho AI agent. Đọc file này TRƯỚC khi code bất kỳ thứ gì.
>
> **Cập nhật lần cuối**: 2026-04-05

---

## 1. Tổng Quan Dự Án

**DroneGCS** là ứng dụng Ground Control Station (Trạm điều khiển mặt đất) viết bằng **Python + PySide6 (Qt 6)**, dùng để giám sát và điều khiển drone qua giao thức **MSP (MultiWii Serial Protocol)**.

### Chuỗi kết nối phần cứng

```
[PC chạy DroneGCS]  ←── TCP/Wifi ──→  [ESP32 (cầu nối)]  ←── UART ──→  [Flight Controller chạy INAV]
```

- **ESP32** hoạt động như Access Point Wifi, tạo mạng riêng (`Drone_GCS_Wifi`), IP mặc định `192.168.4.1:8080`
- ESP32 là **cầu nối + failsafe chủ động**: nhận TCP từ PC → chuyển UART xuống FC, nhận UART từ FC → chuyển TCP lên PC
- ESP32 có **Active Failsafe**: nếu mất tín hiệu PC > 1.5 giây → tự gửi MSP_SET_RAW_RC với AUX4=2000 (RTH) qua UART
- ESP32 nhận cấu hình failsafe từ PC qua prefix `FS:rth` hoặc `FS:ignore`
- **GPS + La bàn**: BZ 251 kết nối trực tiếp vào FC, cung cấp tọa độ GPS qua MSP_RAW_GPS

### Tech Stack

| Thành phần | Công nghệ |
|---|---|
| GUI Framework | PySide6 (Qt 6.11+) |
| Ngôn ngữ | Python 3.10+ |
| Giao thức | MSP (MultiWii Serial Protocol) |
| Kết nối | TCP Socket qua Wifi |
| ESP32 Firmware | MicroPython |
| Flight Controller | INAV firmware |

---

## 2. Cấu Trúc Thư Mục

```
DroneGCS/
├── main.py                      ← Entry point + Điều phối Signal/Slot (class GCSApp + TakeoffDialog)
├── AGENT.md                     ← File này
├── memories.md                  ← Nhật ký phát triển (Development Log)
├── _test_ui_refactor.py         ← Script test kiểm tra widget names
│
├── comm/                        ← Package giao tiếp mạng
│   ├── __init__.py
│   ├── wifi_client.py           ← Socket TCP thô (connect/send/receive/close)
│   ├── wifi_worker.py           ← QThread chạy ngầm (polling loop 20Hz, MSP_RAW_GPS)
│   └── msp_parser.py            ← Đóng gói/giải mã MSP (ANALOG, ATTITUDE, ALTITUDE, STATUS, RAW_GPS, SET_WP)
│
├── core/                        ← Package logic nghiệp vụ
│   ├── __init__.py
│   ├── drone_state.py           ← Trạng thái drone chia sẻ (GPS, altitude, ARM, home position)
│   └── flight_controller.py    ← State machine bay tự động (ARM, Takeoff, Safe Land, RTH, Mission)
│
├── ui/                          ← Package giao diện
│   ├── __init__.py
│   ├── main_window.py           ← Cửa sổ chính (Nav Rail + Stacked Pages) — Pure Python
│   ├── dashboard_tab.py         ← Tab Dashboard (Attitude 3D + Telemetry + Motors)
│   ├── manual_control_tab.py    ← Tab Manual Control (Sliders + ARM/DISARM/Takeoff)
│   ├── mission_tab.py           ← Tab Mission (OpenStreetMap + Waypoints + Safety Logic)
│   ├── emergency_overlay.py     ← Overlay cảnh báo khẩn cấp (DISARM + Safe Land)
│   ├── attitude_3d_widget.py    ← Widget mô phỏng tư thế drone (Artificial Horizon)
│   ├── config_tab.py            ← Tab Config (placeholder)
│   ├── gcs_dashboard.py         ← ⚠️ LEGACY — KHÔNG DÙNG NỮA
│   └── gcs_dashboard.ui         ← ⚠️ LEGACY — KHÔNG DÙNG NỮA
│
└── ESP32/
    └── main.py                  ← Firmware MicroPython (cầu nối + failsafe chủ động RTH/SafeLand)
```

---

## 3. Kiến Trúc Module & Phân Chia Trách Nhiệm

### 3.1 Sơ đồ kế thừa & quan hệ

```
QMainWindow
    └── MainWindow (ui/main_window.py)     ← Chỉ xây layout UI
            └── GCSApp (main.py)           ← Kế thừa, thêm logic Signal/Slot
```

### 3.2 Chi tiết từng module

#### `main.py` — Entry Point & Điều Phối

- **Class `ConnectionDialog`**: Dialog nhập IP/Port hoặc chọn Mock Test
- **Class `GCSApp(MainWindow)`**: Kế thừa MainWindow, thêm toàn bộ logic
  - `toggle_connection()`: Mở dialog kết nối hoặc ngắt kết nối
  - `start_connection(ip, port, is_mock)`: Tạo WifiWorker + kết nối Signal/Slot
  - `handle_connection_status(success, message)`: Slot xử lý trạng thái mạng
  - `update_telemetry_ui(data: dict)`: Slot cập nhật dữ liệu telemetry lên UI
  - `set_ui_state_na()`: Reset UI về trạng thái mất kết nối
  - `enable_ui_components()`: Mở khóa UI khi có kết nối
  - `closeEvent()`: Ngắt kết nối an toàn khi đóng app
- **Hằng số**: `LIPO_6S_MIN_VOLTAGE = 19.8`, `LIPO_6S_MAX_VOLTAGE = 25.2`
- **Quy trình khởi chạy**:
  1. Tạo `GCSApp` → hiện cửa sổ
  2. Tự động mở `ConnectionDialog`
  3. Nếu Accept → gọi `start_connection()`

#### `ui/main_window.py` — Layout Cửa Sổ Chính

- **CHỈ chứa UI layout**, KHÔNG chứa logic xử lý
- Cấu trúc:
  ```
  MainWindow (QMainWindow, 1200x800)
  └── centralWidget (QVBoxLayout)
      ├── frame_topbar (QFrame)
      │   ├── lbl_batt_volt      (QLabel — "-- V")
      │   ├── bar_battery_volt   (QProgressBar — 150-200px)
      │   ├── lbl_batt_perc      (QLabel — "-- %")
      │   ├── [spacer]
      │   ├── lbl_wifi_status    (QLabel — "WiFi Status:")
      │   ├── lbl_wifi_icon      (QLabel — "📶 Đang chờ")
      │   └── btn_disconnect     (QPushButton — "Ngắt kết nối")
      └── tab_widget (QTabWidget)
          ├── dashboard_tab  → DashboardTab
          ├── mission_tab    → MissionTab
          ├── config_tab     → ConfigTab
          └── tab_log        → QWidget (placeholder)
  ```

#### `ui/dashboard_tab.py` — Tab Giám Sát Chính

- Layout dạng **Grid 2x2**:
  - `[0,0]` — **Telemetry**: Pin, Mode, Armed, Altitude, GPS, Attitude, Speed
  - `[0,1]` — **Manual Control**: 6 slider (Throttle, Roll, Pitch, Yaw, AUX1, AUX2) + 6 nút
  - `[1,0..1]` — **Motors**: 4 thanh hiển thị vòng tua (PWM 1000-2000μs, dọc)

#### `ui/mission_tab.py` — Tab Lập Trình Lộ Trình Bay

- Form nhập Waypoint: Latitude, Longitude, Altitude
- Bảng QTableWidget hiển thị danh sách waypoint (4 cột: #, Lat, Lon, Alt)
- 5 nút: Remove, Clear, Upload To FC, Start Mission, Stop Mission

#### `ui/config_tab.py` — Tab Cấu Hình (Placeholder)

- Hiện tại chỉ hiển thị text placeholder
- **Dự kiến**: PID Tuning, Rate Profiles, Motor Mapping, Sensor Calibration, Failsafe Config

#### `comm/wifi_client.py` — Kết Nối TCP Thô

- Class `WifiClient` — transport layer thuần túy
- **4 phương thức**: `connect()`, `send(bytes)`, `receive() → bytes`, `close()`
- Timeout đọc: `0.2s`, Buffer: `1024 bytes`
- Property `is_connected` kiểm tra trạng thái

#### `comm/wifi_worker.py` — Luồng Ngầm QThread

- Class `WifiWorker(QThread)`
- **2 Signal**:
  - `connection_status(bool, str)` → GCSApp.handle_connection_status
  - `telemetry_data(dict)` → GCSApp.update_telemetry_ui
- **2 chế độ chạy**:
  - `_run_real_mode()`: Kết nối ESP32 thật, polling MSP_ANALOG + MSP_ATTITUDE luân phiên
  - `_run_mock_mode()`: Giả lập dữ liệu telemetry (pin tụt dần, góc nghiêng dao động, motor ngẫu nhiên)
- Tần số polling: **20Hz** (`POLLING_INTERVAL = 0.05s`)
- `stop()`: Dừng thread an toàn bằng cờ `is_running = False`

#### `comm/msp_parser.py` — Giao Thức MSP

- Class `MSPParser`
- **Lệnh hỗ trợ**:
  - `MSP_ANALOG (110)`: Điện áp pin (`vbat/10.0 → Volt`), dòng điện (`amp/100.0 → Ampere`)
  - `MSP_ATTITUDE (108)`: Roll (`/10.0°`), Pitch (`/10.0°`), Yaw (`0-360°`)
- **Đóng gói**: `pack_msg(cmd, payload) → bytes` : Frame = `$M<` + Size + Cmd + Payload + Checksum
- **Giải mã**: `parse_buffer(data) → dict` : Gom buffer chống đứt gói, tìm `$M>`, verify checksum XOR
- **Struct format**:
  - MSP_ANALOG: `'<B H H h'` (7 bytes: vbat, mah_drawn, rssi, amperage)
  - MSP_ATTITUDE: `'<h h h'` (6 bytes: roll, pitch, yaw)

#### `ESP32/main.py` — Firmware MicroPython

- Wifi AP: SSID `Drone_GCS_Wifi`, password `password123`
- UART1: baudrate=115200, TX=17, RX=16
- TCP Server: port 8080, non-blocking
- 3 luồng xử lý trong vòng lặp chính:
  1. **Quản lý kết nối**: Accept client mới
  2. **PC → FC**: recv TCP → write UART
  3. **FC → PC**: read UART → send TCP
- **Failsafe**: Mất tín hiệu > 1000ms → ngắt socket, chờ kết nối lại

---

## 4. Luồng Dữ Liệu Chi Tiết

### 4.1 Telemetry (FC → UI)

```
[Flight Controller]
        │ UART (MSP Response: $M> + payload)
        ▼
[ESP32] ──── TCP ────→ [WifiClient.receive()]
                              │ raw bytes
                              ▼
                        [MSPParser.parse_buffer()]
                              │ dict: {"voltage": 24.5, "roll": 1.2, ...}
                              ▼
                        [WifiWorker] ──Signal──→ [GCSApp.update_telemetry_ui()]
                                                        │
                                                        ▼
                                                 [Cập nhật QLabel/QProgressBar]
```

### 4.2 Lệnh điều khiển (UI → FC)  *(chưa implement đầy đủ)*

```
[UI Button/Slider]
        │ Slide change / click
        ▼
[GCSApp] → [WifiWorker] → [MSPParser.pack_msg()] → [WifiClient.send()] → TCP → [ESP32] → UART → [FC]
```

### 4.3 Mock Mode

```
[WifiWorker._run_mock_mode()]
        │ random.uniform() + giảm điện áp dần
        │ dict: {"voltage", "pitch", "roll", "yaw", "motor1-4"}
        │
        ▼ Signal: telemetry_data
[GCSApp.update_telemetry_ui()] → Cập nhật UI y hệt chế độ thật
```

---

## 5. Dict Telemetry Data — Bảng Key/Value

Đây là format dict được truyền qua Signal `telemetry_data` và xử lý trong `update_telemetry_ui()`:

| Key | Type | Unit | Nguồn MSP | Cách tính từ raw |
|---|---|---|---|---|
| `voltage` | float | Volt | MSP_ANALOG | `vbat / 10.0` |
| `current` | float | Ampere | MSP_ANALOG | `amperage / 100.0` |
| `roll` | float | Degree (°) | MSP_ATTITUDE | `raw_roll / 10.0` |
| `pitch` | float | Degree (°) | MSP_ATTITUDE | `raw_pitch / 10.0` |
| `yaw` | float/int | Degree (0-360°) | MSP_ATTITUDE | Giá trị trực tiếp |
| `motor1` | int | PWM μs (1000-2000) | *(chưa implement)* | — |
| `motor2` | int | PWM μs (1000-2000) | *(chưa implement)* | — |
| `motor3` | int | PWM μs (1000-2000) | *(chưa implement)* | — |
| `motor4` | int | PWM μs (1000-2000) | *(chưa implement)* | — |

---

## 6. Naming Convention — Quy Tắc Đặt Tên Widget

### Prefix

| Prefix | Loại widget | Ví dụ |
|---|---|---|
| `lbl_` | QLabel (mô tả) | `lbl_batt_volt`, `lbl_roll` |
| `val_` | QLabel (giá trị hiển thị, **font bold**) | `val_roll`, `val_motor1` |
| `bar_` | QProgressBar | `bar_battery_volt`, `bar_motor1` |
| `btn_` | QPushButton | `btn_disconnect`, `btn_arm` |
| `slider_` | QSlider | `slider_throttle`, `slider_roll` |
| `input_` | QLineEdit | `input_ip`, `input_lat` |
| `grp_` | QGroupBox | `grp_telemetry`, `grp_motors` |
| `tab_` | QWidget (tab page) | `tab_log` |
| `table_` | QTableWidget | `table_waypoints` |

### Widget thuộc về đâu?

| Widget | Thuộc class | Truy cập từ GCSApp |
|---|---|---|
| `lbl_batt_volt`, `bar_battery_volt`, `lbl_batt_perc` | MainWindow | `self.lbl_batt_volt` |
| `lbl_wifi_icon`, `btn_disconnect` | MainWindow | `self.lbl_wifi_icon` |
| `val_roll`, `val_pitch`, `val_yaw` | DashboardTab | `self.dashboard_tab.val_roll` |
| `val_motor1`, `bar_motor1` | DashboardTab | `self.dashboard_tab.val_motor1` |
| `grp_manual_control` | DashboardTab | `self.dashboard_tab.grp_manual_control` |
| `table_waypoints`, `btn_upload` | MissionTab | `self.mission_tab.table_waypoints` |

---

## 7. Thông Số Phần Cứng

### Pin Lipo 6S

| Thông số | Giá trị |
|---|---|
| Số cell | 6S |
| Điện áp rỗng | 19.8V (3.3V/cell) |
| Điện áp đầy | 25.2V (4.2V/cell) |
| Công thức % | `(V - 19.8) / (25.2 - 19.8) * 100` |
| FC gửi raw | `vbat * 10` (VD: 245 = 24.5V) |

### Động cơ

| Thông số | Giá trị |
|---|---|
| KV | 1960kv |
| PWM Range | 1000μs (idle) → 2000μs (full throttle) |
| Số lượng | 4 (quadcopter) |

### ESP32

| Thông số | Giá trị |
|---|---|
| Wifi Mode | Access Point |
| SSID | `Drone_GCS_Wifi` |
| Password | `password123` |
| IP | `192.168.4.1` |
| TCP Port | `8080` |
| UART | Baudrate 115200, TX=GPIO17, RX=GPIO16 |
| Failsafe timeout | 1000ms |

---

## 8. Quy Tắc Code — PHẢI TUÂN THỦ

### 8.1 Phân tách trách nhiệm (SoC)

| Module | CHỈ ĐƯỢC LÀM | KHÔNG ĐƯỢC LÀM |
|---|---|---|
| `main.py` | Khởi tạo, kết nối Signal/Slot, cập nhật UI | Gọi socket trực tiếp, giải mã MSP |
| `wifi_client.py` | Mở/đóng socket, gửi/nhận bytes | Import QThread, biết về MSP |
| `wifi_worker.py` | Chạy vòng lặp ngầm, dùng WifiClient + MSPParser | Cập nhật UI trực tiếp |
| `msp_parser.py` | Đóng gói/giải mã MSP | Import socket, biết về thread |
| `ui/main_window.py` | Xây dựng layout UI | Chứa logic nghiệp vụ |
| `ui/dashboard_tab.py` | Xây dựng nhóm widget | Kết nối Signal/Slot nghiệp vụ |
| `ui/mission_tab.py` | Xây dựng form waypoint | Xử lý upload FC |

### 8.2 Quy tắc UI

1. **`ui/*.py` CHỈ tạo widget** — Không chứa Signal/Slot logic
2. **GCSApp kết nối Signal/Slot** — Tất cả logic nằm ở `main.py`
3. **Widget Top Bar** (pin, wifi, btn_disconnect) thuộc `MainWindow` — truy cập trực tiếp qua `self`
4. **Widget trong Tab** thuộc từng class riêng — truy cập qua `self.dashboard_tab`, `self.mission_tab`
5. **Thêm widget mới** phải đặt tên theo prefix convention (mục 6)

### 8.3 Quy tắc Comm

1. **WifiWorker phát Signal**, không bao giờ gọi `widget.setText()` trực tiếp
2. **MSPParser là stateless** (ngoại trừ buffer) — không biết đang kết nối ở đâu
3. **Thêm lệnh MSP mới**: thêm constant + thêm case trong `_decode_payload()` + thêm cmd vào `cmds_to_request` trong WifiWorker

### 8.4 File Legacy — KHÔNG CHẠM VÀO

- `ui/gcs_dashboard.py` — Auto-generated từ Qt Designer, **KHÔNG DÙNG NỮA**
- `ui/gcs_dashboard.ui` — File .ui gốc, giữ để tham khảo

---

## 9. Trạng Thái Tính Năng

### ✅ Đã hoàn thành

- [x] Kiến trúc module sạch (tách UI / Comm / Logic)
- [x] Kết nối TCP Wifi tới ESP32
- [x] Giải mã MSP_ANALOG (Voltage, Current)
- [x] Giải mã MSP_ATTITUDE (Roll, Pitch, Yaw)
- [x] Giải mã MSP_ALTITUDE (Altitude, Vario)
- [x] Giải mã MSP_STATUS (ARM status, flight mode flags)
- [x] Giải mã MSP_RAW_GPS (GPS BZ 251: lat, lon, sats, speed, fix)
- [x] Hiển thị Telemetry trên Dashboard (pin, góc, GPS, altitude)
- [x] Hiển thị PWM 4 Motor (thanh dọc)
- [x] Mock Test mode (giả lập dữ liệu + GPS)
- [x] Connection Dialog (nhập IP/Port hoặc Mock)
- [x] Quản lý trạng thái kết nối/mất kết nối trên UI
- [x] ESP32 firmware failsafe chủ động (RTH/SafeLand khi mất WiFi)
- [x] ARM / DISARM (state machine + RC link safety)
- [x] Takeoff & Hold (dialog nhập độ cao + ALTHOLD+POSHOLD)
- [x] Safe Land (AUX3=2000 → FC hạ cánh tại chỗ)
- [x] RTH — Return To Home (AUX4=2000 → FC bay về home)
- [x] Emergency Overlay (nút DISARM + Safe Land nổi trên UI)
- [x] Tab Mission — OpenStreetMap + Waypoints tương tác + Logic an toàn
- [x] MSP_SET_WP — Upload waypoints xuống FC
- [x] Cấu hình failsafe từ PC xuống ESP32 (FS:rth / FS:ignore)
- [x] DroneState — Centralized state management (GPS + Home position)
- [x] Dark theme + animations + responsive layout
- [x] Test script kiểm tra widget names (`_test_ui_refactor.py`)

### 🚧 Đang phát triển / Chưa implement

- [ ] **Gửi lệnh Manual Control** (slider → MSP_SET_RAW_RC → FC)
- [ ] **Tab Config**: PID Tuning, Rate Profiles, Motor Mapping
- [ ] **Tab Log**: Ghi log telemetry lịch sử
- [ ] **Motor data từ FC** (hiện chỉ có trong Mock, chưa parse MSP_MOTOR)

---

## 10. Hướng Dẫn Thêm Tính Năng Mới

### Thêm lệnh MSP mới (VD: MSP_RAW_GPS = 106)

1. **`msp_parser.py`**: Thêm constant `MSP_RAW_GPS = 106` + thêm `elif` trong `_decode_payload()`
2. **`wifi_worker.py`**: Import constant mới, thêm vào list `cmds_to_request`
3. **`main.py`**: Xử lý key mới trong `update_telemetry_ui(data)`
4. **`dashboard_tab.py`**: Thêm widget (nếu cần hiển thị)

### Thêm Tab mới

1. Tạo file `ui/new_tab.py` với class kế thừa `QWidget`
2. Import trong `ui/main_window.py`, thêm vào `_create_tabs()`
3. Xử lý logic Signal/Slot trong `main.py` (class GCSApp)

### Thêm nút điều khiển

1. Thêm widget trong `dashboard_tab.py` hoặc tab tương ứng
2. Kết nối `btn.clicked.connect(handler)` trong `main.py`
3. Trong handler: tạo MSP payload bằng `MSPParser.pack_msg()` → gửi qua `WifiWorker`

---

## 11. Chạy & Test

```bash
# Chạy ứng dụng chính
python main.py

# Chạy test kiểm tra widget names
python _test_ui_refactor.py
```

- Khi chạy `main.py`, dialog kết nối hiện ra đầu tiên
- Chọn **"🧪 Mock Test"** để test UI không cần phần cứng
- Chọn **"🛜 Kết Nối Wifi"** để kết nối ESP32 thật

---

## 12. Ánh Xạ Kênh AUX (MSP_SET_RAW_RC)

Thứ tự kênh: `[Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]`

| Kênh | FC Channel | Chức năng | 1000 | 1500 | 2000 |
|---|---|---|---|---|---|
| AUX1 | CH5 | ARM/DISARM | DISARM | — | ARM |
| AUX2 | CH6 | Flight Mode | Acro | ANGLE | ALTHOLD+POSHOLD |
| AUX3 | CH7 | Safe Land | OFF | — | Kích hoạt hạ cánh |
| AUX4 | CH8 | RTH | OFF | — | Kích hoạt RTH |

---

## 13. Lịch Sử Thay Đổi

| Ngày | Mô tả |
|---|---|
| 2026-03-30 | **Refactoring kiến trúc lớn**: Tách monolithic code thành module riêng biệt. Tạo `MainWindow`, `DashboardTab`, `MissionTab`, `ConfigTab`. GCSApp kế thừa MainWindow. |
| 2026-03-30 | Cập nhật AGENT.md toàn diện với đầy đủ context cho AI agent. |
| 2026-04-05 | **Feature Update lớn**: ESP32 failsafe chủ động (RTH/SafeLand), Emergency Overlay, Takeoff Dialog (nhập độ cao + GPS), Mission Tab với OpenStreetMap (folium + QWebEngineView), MSP_RAW_GPS + MSP_SET_WP, AUX channel mapping mới (4 AUX), DroneState GPS fields, cấu hình failsafe từ PC. |