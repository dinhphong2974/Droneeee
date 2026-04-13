# DroneGCS — Agent Context Document

> **Mục đích**: Cung cấp toàn bộ bối cảnh dự án cho AI agent. Đọc file này TRƯỚC khi code bất kỳ thứ gì.
>
> **Cập nhật lần cuối**: 2026-04-13

---

## 1. Tổng Quan Dự Án

**DroneGCS** là ứng dụng Ground Control Station (GCS) viết bằng **Python + PySide6 (Qt 6)**, điều khiển drone qua giao thức **MSP (MultiWii Serial Protocol)**.

### Chuỗi kết nối phần cứng

```
[PC DroneGCS] ←── TCP/WiFi ──→ [ESP32 AP] ←── UART ──→ [FC INAV (SpeedyBee F405)]
```

| Thiết bị | Vai trò |
|---|---|
| PC / GCS | Giao diện điều khiển, lập trình mission, giám sát |
| ESP32 | WiFi Access Point + cầu nối TCP↔UART + failsafe chủ động |
| FC INAV | Bay tự động, ARM/DISARM, GPS navigation, sensor fusion |
| GPS BZ 251 | UART5 của FC, cung cấp tọa độ qua MSP_RAW_GPS |
| LiDAR+OptFlow MTF-02 | UART6 của FC (giao thức MSP), rangefinder + optical flow |

### Tech Stack

| Thành phần | Công nghệ |
|---|---|
| GUI | PySide6 (Qt 6.11+) |
| Ngôn ngữ | Python 3.10+ |
| Giao thức | MSP (MultiWii Serial Protocol) |
| Kết nối | TCP Socket qua WiFi |
| ESP32 Firmware | MicroPython |
| Flight Controller | INAV firmware |

---

## 2. Cấu Trúc Thư Mục

```
DroneGCS/
├── main.py                    ← Entry point + Điều phối Signal/Slot (class GCSApp)
├── AGENT.md                   ← File này — bối cảnh dự án cho AI
├── memories.md                ← Nhật ký phát triển (Development Log)
│
├── comm/
│   ├── wifi_client.py         ← Socket TCP thô (connect/send/receive/close)
│   ├── wifi_worker.py         ← QThread polling 20Hz: MSP polling + PING + ACK
│   └── msp_parser.py          ← Đóng gói/giải mã MSP frames
│
├── core/
│   ├── drone_state.py         ← Trạng thái chia sẻ (GPS, altitude, ARM, ping)
│   └── flight_controller.py  ← State machine (IDLE→ARM→TAKEOFF→HOLD→DISARM...)
│
├── ui/
│   ├── main_window.py         ← Cửa sổ chính (Nav Rail + Top Bar + Pages)
│   ├── dashboard_tab.py       ← Tab Dashboard (Attitude 3D + Telemetry + Motors)
│   ├── manual_control_tab.py  ← Tab Manual Control (ARM/DISARM/Takeoff/RTH)
│   ├── mission_tab.py         ← Tab Mission (OpenStreetMap + Waypoints)
│   ├── emergency_overlay.py   ← Overlay khẩn cấp (DISARM + Safe Land)
│   ├── attitude_3d_widget.py  ← Widget mô phỏng tư thế drone (Artificial Horizon)
│   ├── config_tab.py          ← Tab Config (placeholder)
│   ├── gcs_dashboard.py       ← ⚠️ LEGACY — KHÔNG DÙNG NỮA
│   └── gcs_dashboard.ui       ← ⚠️ LEGACY — KHÔNG DÙNG NỮA
│
├── ESP32/
│   └── main.py                ← Firmware MicroPython (bridge + failsafe + PING/ACK)
│
└── tests/
    ├── test_flight_logic.py   ← Unit tests cho FlightController + MSPParser
    └── test_msp_noise_robustness.py ← 100 unit tests robustness MSP parser
```

---

## 3. Kiến Trúc Module

### 3.1 Phân cấp kế thừa UI

```
QMainWindow
    └── MainWindow (ui/main_window.py)   ← Chỉ xây layout UI
            └── GCSApp (main.py)         ← Kế thừa, thêm toàn bộ Signal/Slot logic
```

### 3.2 Sơ đồ luồng dữ liệu

```
┌─────────────────────────────────────────────────────────────┐
│                         GCS (PC)                             │
│                                                               │
│  FlightController ──send_command()──→ WifiWorker             │
│       ↑ state_changed                     │ TCP              │
│       │ status_update              ┌──────▼──────┐           │
│  GCSApp (main.py)                 │   ESP32 AP   │           │
│       ↑ telemetry_data            └──────┬──────┘           │
│       │ ping_updated                     │ UART              │
│       │ command_acked             ┌──────▼──────┐           │
│  UI Widgets                      │ FC (INAV)    │           │
│                                  └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. Chi Tiết Từng Module

### 4.1 `main.py` — Entry Point & Điều Phối

**Classes:**
- `ConnectionDialog`: Dialog nhập IP/Port hoặc chọn Mock Test
- `TakeoffDialog`: Dialog nhập độ cao và hiển thị GPS status trước khi cất cánh
- `GCSApp(MainWindow)`: Kế thừa MainWindow, thêm toàn bộ Signal/Slot logic

**Signals connections trong `start_connection()`:**
```python
self.worker.connection_status.connect(self.handle_connection_status)
self.worker.telemetry_data.connect(self.update_telemetry_ui)
self.worker.ping_updated.connect(self._on_ping_updated)      # MỚI 2026-04-13
self.worker.command_acked.connect(self._on_command_acked)    # MỚI 2026-04-13
```

**Slots quan trọng:**
- `_on_ping_updated(rtt_ms)`: Cập nhật `lbl_ping` với màu sắc (xanh/vàng/cam/đỏ)
- `_on_command_acked(ack_type)`: Hiển thị "✓ Failsafe config" / "✓ Emergency cmd" trên status
- `_emergency_force_disarm()`: Gọi `force_disarm()` trên FlightController
- `update_telemetry_ui(data)`: Cập nhật toàn bộ UI từ dict telemetry

**Hằng số:**
- `LIPO_6S_MIN_VOLTAGE = 19.8V`, `LIPO_6S_MAX_VOLTAGE = 25.2V`

---

### 4.2 `ui/main_window.py` — Layout Cửa Sổ Chính

**Widget `frame_topbar` (từ trái → phải):**
```
[lbl_batt_volt] [bar_battery_volt] [lbl_batt_perc] [spacer]
[lbl_ping_title] [lbl_ping]  [spacer]
[lbl_wifi_status] [lbl_wifi_icon] [spacer]
[btn_disconnect]
```

> **`lbl_ping`** (MỚI 2026-04-13): Hiển thị "🏓 28ms" với màu:
> - `#4CAF50` xanh ≤50ms · `#FFC107` vàng ≤150ms · `#FF9800` cam ≤300ms · `#F44336` đỏ >300ms

**Nav Rail** (bên trái, QListWidget): Dashboard / Manual Control / Mission / Config / Log

**Widget ownership:**

| Widget | Class | Truy cập từ GCSApp |
|---|---|---|
| `lbl_batt_volt`, `bar_battery_volt`, `lbl_batt_perc` | MainWindow | `self.lbl_batt_volt` |
| `lbl_ping`, `lbl_ping_title` | MainWindow | `self.lbl_ping` |
| `lbl_wifi_icon`, `btn_disconnect` | MainWindow | `self.lbl_wifi_icon` |
| `val_roll`, `val_pitch`, `val_yaw` | DashboardTab | `self.dashboard_tab.val_roll` |
| `val_motor1`, `bar_motor1` | DashboardTab | `self.dashboard_tab.val_motor1` |
| `val_flight_status` | ManualControlTab | `self.manual_control_tab.val_flight_status` |
| `table_waypoints`, `btn_upload` | MissionTab | `self.mission_tab.table_waypoints` |

---

### 4.3 `comm/wifi_worker.py` — QThread Polling

**Signals:**
```python
connection_status = Signal(bool, str)   # (success, message)
telemetry_data    = Signal(dict)        # dữ liệu đã decode
ping_updated      = Signal(int)         # RTT ms (MỚI 2026-04-13)
command_acked     = Signal(str)         # "RC" | "FS" | "EM" (MỚI 2026-04-13)
```

**Command Queues:**
- `_command_queue`: Queue thường (telemetry poll order, RC frames...)
- `_emergency_queue`: Queue ưu tiên cao nhất (force_disarm, force_safe_land)

**`send_emergency_command(data)`**:
1. Set cờ `_emergency_abort`
2. Xóa toàn bộ `_command_queue`
3. Đưa vào `_emergency_queue`

**Tách luồng text/binary trong `_extract_text_responses()`:**
- `PONG:<ts>\n` → tính RTT → `ping_updated.emit(rtt)`
- `ACK:<type>\n` → `command_acked.emit(type)`
- Bytes còn lại (MSP `$M>...`) → MSP parser

**MSP polling 20Hz** (luân phiên):
```
ANALOG → ATTITUDE → ALTITUDE → STATUS → RAW_GPS → SONAR_ALTITUDE → STATUS_EX
```

**PING gửi mỗi 1 giây** (trong vòng lặp real mode):
```python
ping_msg = f"PING:{int(now*1000)}".encode()
```

---

### 4.4 `comm/msp_parser.py` — Giao Thức MSP

**Lệnh hỗ trợ:**

| Constant | ID | Mô tả |
|---|---|---|
| `MSP_WP_GETINFO` | 20 | Thông tin mission |
| `MSP_SONAR_ALTITUDE` | 58 | Độ cao LiDAR MTF-02 (cm, âm = OOR) |
| `MSP_STATUS` | 101 | ARM status + flight mode flags |
| `MSP_RAW_GPS` | 106 | GPS BZ 251: lat, lon, sats, speed, fix |
| `MSP_ATTITUDE` | 108 | Roll, Pitch, Yaw |
| `MSP_ALTITUDE` | 109 | Barometer + vario |
| `MSP_ANALOG` | 110 | Voltage, current |
| `MSP_STATUS_EX` | 150 | Sensor health (OptFlow, LiDAR, Compass) |
| `MSP_SET_RAW_RC` | 200 | Gửi 8 kênh RC xuống FC |
| `MSP_SET_WP` | 209 | Upload waypoint |

**Buffer Protection**: `MAX_BUFFER_SIZE = 4096 bytes`

---

### 4.5 `core/drone_state.py` — Trạng Thái Chia Sẻ

Các field quan trọng:

```python
# ARM
is_armed: bool

# Altitude
altitude: float          # Barometer (mét)
surface_altitude: float  # LiDAR MTF-02 (mét, -1.0 nếu OOR)
surface_quality: int     # 0-255

# GPS
latitude, longitude: float
gps_fix_type: int        # 0=No Fix, 1=2D, 2=3D
gps_num_sat: int

# Attitude
roll, pitch, yaw: float

# Home position
home_lat, home_lon: float
has_home: bool

# Sensor health
sensor_opflow: bool
sensor_rangefinder: bool
sensor_mag: bool

# Network quality (MỚI 2026-04-13)
ping_rtt_ms: int         # RTT GCS↔ESP32 (ms)
```

---

### 4.6 `core/flight_controller.py` — State Machine Bay

**States:**

```
IDLE
├── arm()         → WAIT_RC_LINK_ARM_ONLY → ARMING_ONLY → IDLE
├── takeoff()     → PRE_ARM_CHECK → WAIT_RC_LINK → ARMING → ARMED_WAIT
│                   → WP_UPLOAD → WP_ACTIVATE → NAV_CLIMB → ALTITUDE_REACHED → HOLDING
├── safe_land()   → SAFE_LANDING → IDLE (khi FC disarm)
├── rth()         → RTH_ACTIVE → IDLE (khi FC disarm)
├── disarm()      → DISARMING → IDLE (khi is_armed=False)       [MỚI 2026-04-13]
└── force_disarm()→ FORCE_DISARMING → IDLE (khi is_armed=False) [MỚI 2026-04-13]
```

**AUX Channel Mapping (chuẩn AETR, index trong MSP frame):**

```
Index: [0=Roll, 1=Pitch, 2=Throttle, 3=Yaw, 4=AUX1, 5=AUX2, 6=AUX3, 7=AUX4]

AUX1 (CH5): ARM/DISARM    — 1000=DISARM, 2000=ARM
AUX2 (CH6): Flight Mode   — 1000=Acro, 1500=ANGLE, 2000=ALTHOLD+POSHOLD
AUX3 (CH7): Safe Land     — 1000=OFF, 2000=ON
AUX4 (CH8): RTH           — 1000=OFF, 2000=ON
```

**`_safe_channels()` — bộ kênh mặc định an toàn:**
```python
[RC_CENTER, RC_CENTER, THROTTLE_MIN, RC_CENTER,
 AUX_DISARM, AUX_ANGLE, AUX_SAFE_LAND_OFF, AUX_RTH_OFF]
 # [Roll,    Pitch,     Throttle,    Yaw,
 #  AUX1=DISARM, AUX2=ANGLE, AUX3=OFF, AUX4=OFF]
```

**Hằng số quan trọng:**
- `TICK_INTERVAL_MS = 100` (10Hz)
- `RC_LINK_WAIT_S = 2.0` — INAV cần thấy DISARM ≥1s trước khi cho phép ARM
- `ARM_TIMEOUT_S = 5.0`
- `DISARM_TIMEOUT_S = 5.0` — timeout chờ FC xác nhận DISARM
- `LIDAR_MAX_RANGE_M = 2.5`

---

### 4.7 `ESP32/main.py` — Firmware MicroPython

**Cấu hình:**
- WiFi AP: SSID `Drone_GCS_Wifi`, password `DroneGCS@2026!`, IP `192.168.4.1`
- TCP Server: port 8080, non-blocking
- UART1: baudrate=115200, TX=GPIO17, RX=GPIO16, rxbuf=512

**Prefix lệnh đặc biệt (từ GCS → ESP32):**

| Prefix | Xử lý |
|---|---|
| `EM:<msp_frame>` | Emergency: xóa UART buffer → gửi ngay → tắt failsafe → reply `ACK:EM\n` |
| `FS:rth` / `FS:ignore` | Cấu hình failsafe behavior → reply `ACK:FS\n` |
| `PING:<timestamp_ms>` | Đo latency → reply `PONG:<timestamp_ms>\n` |
| `HB:` | Heartbeat (reset failsafe timer, không gửi xuống FC) |
| `$M<...` (MSP frame) | Chuyển tiếp xuống FC qua UART. Nếu là MSP_SET_RAW_RC → reply `ACK:RC\n` |

**Failsafe watchdog:**
- Timeout: `2000ms` không nhận gì từ PC → kích hoạt
- Behavior `rth`: gửi RTH frame 10Hz qua UART liên tục
- Behavior `ignore`: không can thiệp (FC bay hết mission)
- **QUAN TRỌNG: KHÔNG đóng socket khi failsafe** — giữ mở để GCS override

---

## 5. Giao Thức PING/ACK (MỚI 2026-04-13)

### Đo latency GCS ↔ ESP32

```
GCS gửi: "PING:1713011234567\n"   (timestamp ms)
ESP32 phản hồi: "PONG:1713011234567\n"
GCS tính: RTT = time.now_ms - 1713011234567
```

### ACK confirmation

```
ESP32 → GCS:
  ACK:EM\n   — Emergency command đã xử lý
  ACK:FS\n   — Failsafe config đã lưu
  ACK:RC\n   — MSP_SET_RAW_RC đã chuyển tiếp xuống FC
```

### WifiWorker stream parsing

PONG/ACK là text kết thúc bằng `\n`, MSP response là binary bắt đầu bằng `$M>`.
`_extract_text_responses()` tách 2 loại trên cùng TCP stream bằng cách:
1. Quét từng byte tìm prefix `PONG:` hoặc `ACK:`
2. Tìm `\n` kết thúc → xử lý
3. Bytes còn lại → trả về cho MSP parser

---

## 6. FIX: DISARM Delay 10-20s (2026-04-13)

### Root cause
`disarm()` và `force_disarm()` cũ là **fire-and-forget**: gửi 1-3 frame rồi `timer.stop()`.
Khi INAV đang ở NAV modes (takeoff, mission), FC **từ chối DISARM** cho đến khi
thoát NAV mode tự nhiên → đợi 10-20s.

### Fix: Repeated-send State Machine

**`disarm()` mới:**
- Gửi frame đầu tiên ngay
- Chuyển state → `DISARMING`
- Timer 10Hz tiếp tục gửi DISARM qua **normal queue**
- Kết thúc khi: `is_armed == False` **hoặc** timeout 5s

**`force_disarm()` mới:**
1. Frame 1: Tắt NAV modes (AUX2=ANGLE, AUX1=ARM-giữ) qua **emergency queue**
2. Frame 2-3: DISARM (AUX1=1000) qua **emergency queue**
3. Chuyển state → `FORCE_DISARMING`
4. Timer 10Hz tiếp tục gửi DISARM qua **emergency queue**
5. Kết thúc khi: `is_armed == False` **hoặc** timeout 5s

---

## 7. Luồng Dữ Liệu Telemetry (dict keys)

| Key | Type | Unit | MSP nguồn |
|---|---|---|---|
| `voltage` | float | V | MSP_ANALOG (`vbat/10`) |
| `current` | float | A | MSP_ANALOG (`amperage/100`) |
| `roll` | float | ° | MSP_ATTITUDE (`raw/10`) |
| `pitch` | float | ° | MSP_ATTITUDE (`raw/10`) |
| `yaw` | float/int | ° (0-360) | MSP_ATTITUDE |
| `altitude` | float | m | MSP_ALTITUDE |
| `vario` | float | m/s | MSP_ALTITUDE |
| `is_armed` | bool | — | MSP_STATUS |
| `flight_mode_flags` | int | bitmask | MSP_STATUS |
| `gps_fix_type` | int | 0/1/2 | MSP_RAW_GPS |
| `gps_num_sat` | int | — | MSP_RAW_GPS |
| `latitude`, `longitude` | float | ° | MSP_RAW_GPS |
| `ground_speed` | float | m/s | MSP_RAW_GPS |
| `surface_altitude` | float | m | MSP_SONAR_ALTITUDE (-1 = OOR) |
| `surface_quality` | int | 0-255 | MSP_SONAR_ALTITUDE |
| `sensor_opflow` | bool | — | MSP_STATUS_EX |
| `sensor_rangefinder` | bool | — | MSP_STATUS_EX |
| `sensor_mag` | bool | — | MSP_STATUS_EX |

---

## 8. Quy Tắc Code — PHẢI TUÂN THỦ

### 8.1 Phân tách trách nhiệm (SoC)

| Module | CHỈ ĐƯỢC LÀM |
|---|---|
| `main.py` | Khởi tạo, kết nối Signal/Slot, cập nhật UI |
| `wifi_client.py` | Mở/đóng socket, gửi/nhận bytes |
| `wifi_worker.py` | Vòng lặp ngầm, dùng WifiClient + MSPParser |
| `msp_parser.py` | Đóng gói/giải mã MSP |
| `ui/main_window.py` | Xây dựng layout UI (KHÔNG chứa logic) |
| `flight_controller.py` | State machine, gửi lệnh qua WifiWorker |

### 8.2 Quy tắc UI

1. `ui/*.py` **CHỈ tạo widget** — KHÔNG chứa Signal/Slot logic nghiệp vụ
2. **GCSApp** kết nối Signal/Slot — tất cả logic nằm ở `main.py`
3. **Widget Top Bar** thuộc `MainWindow` — truy cập qua `self`
4. **Widget trong Tab** thuộc class riêng — truy cập qua `self.dashboard_tab` v.v.
5. **Thêm widget** phải đặt tên theo prefix convention (mục 9)

### 8.3 Quy tắc Comm

1. `WifiWorker` phát **Signal**, không bao giờ gọi `widget.setText()` trực tiếp
2. **Thêm lệnh MSP mới**:
   - `msp_parser.py`: Thêm constant + case trong `_decode_payload()`
   - `wifi_worker.py`: Thêm vào `cmds_to_request`
   - `main.py`: Xử lý key mới trong `update_telemetry_ui()`

### 8.4 Thread Safety

- `FlightController` chạy trên **Main Thread** (QTimer)
- `WifiWorker` chạy trên **Worker Thread** (QThread)
- Chỉ giao tiếp qua: `send_command()`, `send_emergency_command()` (Queue), Signals
- **KHÔNG truy cập `_client` hoặc `_parser` từ bên ngoài WifiWorker**

---

## 9. Naming Convention Widgets

| Prefix | Loại | Ví dụ |
|---|---|---|
| `lbl_` | QLabel (mô tả) | `lbl_batt_volt`, `lbl_ping` |
| `val_` | QLabel (giá trị, bold) | `val_roll`, `val_motor1` |
| `bar_` | QProgressBar | `bar_battery_volt`, `bar_motor1` |
| `btn_` | QPushButton | `btn_disconnect`, `btn_arm` |
| `slider_` | QSlider | `slider_throttle` |
| `input_` | QLineEdit | `input_ip`, `input_lat` |
| `grp_` | QGroupBox | `grp_telemetry` |
| `tab_` | QWidget (tab page) | `tab_log` |
| `table_` | QTableWidget | `table_waypoints` |

---

## 10. Thông Số Phần Cứng

### Pin Lipo 6S
- Rỗng: 19.8V (3.3V/cell), Đầy: 25.2V (4.2V/cell)
- Công thức %: `(V - 19.8) / (25.2 - 19.8) * 100`
- FC gửi raw: `vbat * 10` (VD: 245 = 24.5V)

### Động cơ & FC
- KV: 1960kv, PWM: 1000-2000μs, Số lượng: 4
- FC: SpeedyBee F405 AIO, Khung: OddityRC XI35 Pro O4 3.5" Cinewhoop

### ESP32
| | |
|---|---|
| WiFi Mode | Access Point |
| SSID | `Drone_GCS_Wifi` |
| Password | `DroneGCS@2026!` |
| IP | `192.168.4.1` |
| TCP Port | `8080` |
| UART | baudrate=115200, TX=GPIO17, RX=GPIO16 |
| Failsafe timeout | 2000ms |

### INAV CLI cần thiết
```
nav_use_midthr_for_althold = 1   (50% throttle = hover)
nav_mc_hover_thr = 1500
nav_mc_auto_climb_rate = 300     (climb rate tối đa cm/s)
nav_surface_control = AUTO       (INAV tự dùng LiDAR MTF-02)
```

---

## 11. Trạng Thái Tính Năng

### ✅ Đã hoàn thành

- [x] Kiến trúc module sạch (tách UI / Comm / Logic)
- [x] Kết nối TCP WiFi tới ESP32
- [x] Giải mã đầy đủ: MSP_ANALOG, ATTITUDE, ALTITUDE, STATUS, RAW_GPS, SONAR_ALTITUDE, STATUS_EX
- [x] Dashboard: Pin, góc nghiêng, GPS, altitude, motor bars, LiDAR quality
- [x] Attitude 3D Widget (Artificial Horizon)
- [x] Mock Test mode (giả lập toàn bộ dữ liệu + GPS + LiDAR)
- [x] Connection Dialog (IP/Port hoặc Mock)
- [x] ESP32 failsafe chủ động (RTH/SafeLand khi mất WiFi, giữ socket mở)
- [x] Emergency Protocol (prefix `EM:` bypass queue)
- [x] ARM / DISARM (state machine + INAV ARM_SWITCH safety)
- [x] **DISARM repeated-send** (không còn delay 10-20s) ← MỚI 2026-04-13
- [x] **FORCE DISARM emergency repeated-send** ← MỚI 2026-04-13
- [x] Takeoff & Hold (ALTHOLD+POSHOLD, NAV WP hoặc ALTHOLD fallback)
- [x] Safe Land (AUX3=2000, FC tự hạ)
- [x] RTH (AUX4=2000, FC bay về Home)
- [x] Emergency Overlay (nút DISARM + Safe Land nổi)
- [x] Tab Mission: OpenStreetMap + Waypoints + Upload MSP_SET_WP
- [x] Failsafe config từ PC (FS:rth / FS:ignore)
- [x] MTF-02 LiDAR: MSP_SONAR_ALTITUDE + Dashboard display
- [x] Sensor health: OptFlow, LiDAR, Compass (MSP_STATUS_EX)
- [x] **PING/PONG latency measurement** ← MỚI 2026-04-13
- [x] **ACK confirmation (EM/FS/RC)** ← MỚI 2026-04-13
- [x] **Ping label trên Top Bar** (màu sắc theo chất lượng) ← MỚI 2026-04-13
- [x] Unit tests (116 passed, 3 xfailed)

### 🚧 Chưa implement

- [ ] Manual Control sliders → MSP_SET_RAW_RC → FC
- [ ] Tab Config: PID Tuning, Rate Profiles, Motor Mapping
- [ ] Tab Log: ghi log telemetry lịch sử
- [ ] MSP_MOTOR — Motor PWM thật từ FC (hiện chỉ có trong Mock)

---

## 12. Thêm Tính Năng Mới

### Thêm lệnh MSP mới

1. `msp_parser.py`: `MSP_XXX = <id>` + `elif cmd == MSP_XXX:` trong `_decode_payload()`
2. `wifi_worker.py`: Thêm `MSP_XXX` vào `cmds_to_request`
3. `main.py`: Xử lý key mới trong `update_telemetry_ui(data)`
4. `dashboard_tab.py` (nếu cần hiển thị): Thêm widget

### Thêm Tab mới

1. Tạo `ui/new_tab.py` kế thừa `QWidget`
2. Import + thêm vào `_create_pages()` trong `main_window.py`
3. Logic Signal/Slot trong `GCSApp` (`main.py`)

---

## 13. Chạy & Test

```powershell
# Chạy ứng dụng
python main.py

# Chạy unit tests
python -m pytest tests/ -v --tb=short

# Kết quả mong đợi: 116 passed, 3 xfailed
```

---

## 14. Lịch Sử Thay Đổi

| Ngày | Mô tả |
|---|---|
| 2026-03-30 | **Refactoring kiến trúc lớn**: Tách monolithic code thành module. Tạo `MainWindow`, `DashboardTab`, `MissionTab`, `ConfigTab`. GCSApp kế thừa MainWindow. |
| 2026-04-05 | **Feature Update lớn**: ESP32 failsafe chủ động, Emergency Overlay, Takeoff Dialog, Mission Tab + OpenStreetMap, MSP_RAW_GPS + MSP_SET_WP, AUX channel mapping mới. |
| 2026-04-09 | **MTF-02 LiDAR Integration**: MSP_SONAR_ALTITUDE, sensor fusion, LiDAR display, buffer overflow protection, 13 unit tests. |
| 2026-04-10 | **Critical Bug Fixes**: Sửa failsafe đóng socket gây mất quyền Emergency. Sửa channel order thành AETR. Thêm `EM:` emergency protocol. Sửa Leaflet race condition. |
| 2026-04-13 | **DISARM Fix + PING/ACK System**: Thay fire-and-forget bằng repeated-send state machine (DISARMING/FORCE_DISARMING). Thêm PING/PONG, ACK:EM/FS/RC. Thêm ping label top bar. 116 tests passed. |

---

## 15. Skills Khuyến Nghị

Xem chi tiết tại [`RECOMMENDED_SKILLS.md`](file:///c:/DroneGCS/RECOMMENDED_SKILLS.md).

**Tóm tắt nhanh:**
- Debug FC behavior → `@firmware-analyst`, `@systematic-debugging`
- Viết code → `@python-pro`, `@react-patterns` (PySide6 tương tự)
- Review thay đổi lớn → `@code-reviewer`, `@architect-review`
- Unit tests → `@tdd-workflow`