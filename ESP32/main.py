import machine
import network
import socket
import time
import struct
import gc

# ══════════════════════════════════════════════════════════════
# 1. CẤU HÌNH UART (Giao tiếp với mạch FC SpeedyBee F405)
# ══════════════════════════════════════════════════════════════
# Sử dụng UART1, chân TX=17, RX=16 (theo sơ đồ chân ESP32)
# rxbuf=512: tăng buffer để chứa nhiều MSP response hơn
# (GCS poll 6 lệnh: ANALOG, ATTITUDE, ALTITUDE, STATUS, RAW_GPS, SONAR_ALTITUDE)
uart = machine.UART(1, baudrate=115200, tx=17, rx=16, rxbuf=512)

# ══════════════════════════════════════════════════════════════
# 2. CẤU HÌNH WIFI ACCESS POINT
# ══════════════════════════════════════════════════════════════
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="Drone_GCS_Wifi", password="DroneGCS@2026!")

print("Đang khởi tạo Wifi...")
while not ap.active():
    pass
print("Wifi AP đã sẵn sàng. IP:", ap.ifconfig()[0])

# ══════════════════════════════════════════════════════════════
# 3. CẤU HÌNH TCP SOCKET SERVER
# ══════════════════════════════════════════════════════════════
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bật SO_REUSEADDR giúp cổng 8080 rảnh ngay lập tức nếu ESP32 bị reset
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 8080))
s.listen(1)
s.setblocking(False)  # Ép server không được khóa luồng

print("Đang chờ PC kết nối tới cổng 8080...")

# ══════════════════════════════════════════════════════════════
# 4. HẰNG SỐ VÀ BIẾN FAILSAFE
# ══════════════════════════════════════════════════════════════

# ── Thời gian mất tín hiệu trước khi kích hoạt failsafe (ms) ──
# Tăng từ 1500ms lên 2000ms để tránh false-positive khi WiFi nhiễu tạm thời
FAILSAFE_TIMEOUT_MS = 2000

# ── Chu kỳ gửi lệnh RC khi failsafe đang hoạt động (ms) ──
# FC cần nhận tín hiệu RC liên tục, nếu ngừng gửi FC sẽ tự DISARM
FAILSAFE_RC_INTERVAL_MS = 100

# ── Giá trị kênh RC mặc định (PWM μs) ──
RC_CENTER = 1500     # Giá trị trung tâm (Roll, Pitch, Yaw)
RC_LOW = 1000        # Giá trị thấp (Throttle min, AUX tắt)
RC_HIGH = 2000       # Giá trị cao (AUX bật)

# ══════════════════════════════════════════════════════════════
# THỨ TỰ KÊNH RC — AETR (Chuẩn INAV mặc định)
# ══════════════════════════════════════════════════════════════
# MSP_SET_RAW_RC gửi 8 kênh theo thứ tự:
# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
#
# ĐÃ SỬA: Phiên bản cũ dùng RPYT (Yaw ở index 2, Throttle ở index 3)
# → không nhất quán với GCS FlightController dùng AETR
# → có thể gây hoán đổi Yaw/Throttle khi failsafe
#
# Ánh xạ kênh AUX theo cấu hình INAV Modes tab:
# AUX1 (CH5): ARM/DISARM — 2000=ARM, 1000=DISARM
# AUX2 (CH6): Flight Mode — 1000=Acro, 1500=ANGLE, 2000=ALTHOLD+POSHOLD
# AUX3 (CH7): Safe Land — 2000=Kích hoạt hạ cánh, 1000=Bình thường
# AUX4 (CH8): RTH — 2000=Kích hoạt Return To Home, 1000=Bình thường

CH_ROLL = 0
CH_PITCH = 1
CH_THROTTLE = 2   # ĐÃ SỬA: Index 2 = Throttle (chuẩn AETR)
CH_YAW = 3        # ĐÃ SỬA: Index 3 = Yaw (chuẩn AETR)
CH_AUX1 = 4
CH_AUX2 = 5
CH_AUX3 = 6
CH_AUX4 = 7

# ── MSP Protocol Constants ──
MSP_SET_RAW_RC = 200  # Mã lệnh gửi 8 kênh RC xuống FC

# ── Prefix lệnh đặc biệt từ PC ──
# PC gửi "FS:rth" hoặc "FS:ignore" để cấu hình hành vi failsafe
# PC gửi "EM:<MSP frame>" cho lệnh khẩn cấp ưu tiên cao nhất
# PC gửi "HB:" như heartbeat giữ kết nối sống
FS_PREFIX = b'FS:'
EMERGENCY_PREFIX = b'EM:'
HEARTBEAT_PREFIX = b'HB:'

# ── Biến trạng thái ──
conn = None                    # Kết nối TCP hiện tại
last_msg_time = time.ticks_ms()  # Thời điểm nhận tin nhắn cuối từ PC
failsafe_triggered = False     # Failsafe đã kích hoạt chưa
last_failsafe_rc_time = 0      # Thời điểm gửi RC failsafe lần cuối
conn_error_count = 0           # Đếm lỗi send/recv liên tiếp

# ── Cấu hình hành vi failsafe (có thể thay đổi từ PC) ──
# "rth" = Mất WiFi → gửi RTH (AUX4=2000), drone bay về Home
# "ignore" = Mất WiFi → không can thiệp, drone tiếp tục bay mission từ FC
failsafe_behavior = "rth"

# ── Bộ đếm cho gc.collect() định kỳ ──
gc_counter = 0
GC_INTERVAL = 200  # Gọi gc.collect() mỗi 200 vòng lặp (tăng từ 100 để giảm jitter)

# ── Ngưỡng lỗi liên tiếp trước khi đóng socket ──
CONN_ERROR_THRESHOLD = 10  # Đóng socket chỉ khi send/recv lỗi liên tiếp 10 lần


# ══════════════════════════════════════════════════════════════
# 5. HÀM XÂY DỰNG FRAME MSP (Giao thức MultiWii Serial Protocol)
# ══════════════════════════════════════════════════════════════

def build_msp_set_raw_rc(channels):
    """
    Xây dựng frame MSP_SET_RAW_RC hoàn chỉnh.

    Cấu trúc frame MSP:
        Header: $M< (3 bytes)
        Size: độ dài payload (1 byte) = 16 (8 kênh x 2 bytes)
        Cmd: mã lệnh (1 byte) = 200 (MSP_SET_RAW_RC)
        Payload: 8 kênh RC, mỗi kênh uint16 Little-Endian
        Checksum: XOR(size, cmd, tất cả byte payload) (1 byte)

    QUAN TRỌNG: Checksum phải tính chính xác bằng phép XOR.
    FC SpeedyBee sẽ reject ngay lập tức nếu sai checksum.

    Args:
        channels: List/tuple 8 giá trị kênh RC (1000-2000 μs)
                  Thứ tự AETR: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]

    Returns:
        bytes: Frame MSP hoàn chỉnh sẵn sàng gửi qua UART
    """
    # Header MSP request: $M<
    header = b'$M<'

    # Payload: 8 kênh x uint16 LE = 16 bytes
    payload = struct.pack('<8H', *channels)

    size = len(payload)  # = 16
    cmd = MSP_SET_RAW_RC  # = 200

    # ═══ Tính Checksum bằng phép XOR ═══
    # checksum = size XOR cmd XOR payload[0] XOR payload[1] XOR ...
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b

    # Ghép frame hoàn chỉnh: header + size + cmd + payload + checksum
    frame = bytearray(header)
    frame.append(size)
    frame.append(cmd)
    frame.extend(payload)
    frame.append(checksum & 0xFF)  # Đảm bảo checksum nằm trong 0-255

    return bytes(frame)


def send_failsafe_rth():
    """
    Gửi lệnh RTH (Return To Home) qua UART tới FC.

    Kích hoạt AUX4=2000 (CH8) để bật mode NAV RTH trên INAV.
    Đồng thời giữ ARM (AUX1=2000) và ANGLE mode (AUX2=1500)
    để đảm bảo drone không bị DISARM giữa chừng.

    Thứ tự kênh: AETR [Roll, Pitch, Throttle, Yaw, AUX1-4]
    """
    channels = [0] * 8
    channels[CH_ROLL]     = RC_CENTER   # Roll — giữ trung tâm
    channels[CH_PITCH]    = RC_CENTER   # Pitch — giữ trung tâm
    channels[CH_THROTTLE] = RC_CENTER   # Throttle — FC tự điều khiển khi RTH
    channels[CH_YAW]      = RC_CENTER   # Yaw — giữ trung tâm
    channels[CH_AUX1]     = RC_HIGH     # AUX1 (CH5) = 2000 → Duy trì ARM
    channels[CH_AUX2]     = RC_CENTER   # AUX2 (CH6) = 1500 → ANGLE mode (ổn định)
    channels[CH_AUX3]     = RC_LOW      # AUX3 (CH7) = 1000 → Safe Land OFF
    channels[CH_AUX4]     = RC_HIGH     # AUX4 (CH8) = 2000 → KÍCH HOẠT RTH

    frame = build_msp_set_raw_rc(channels)
    uart.write(frame)


def send_failsafe_land():
    """
    Gửi lệnh Safe Land (hạ cánh tại chỗ) qua UART tới FC.

    Kích hoạt AUX3=2000 (CH7) để bật mode Emergency Landing.
    FC sẽ tự động chiếm quyền điều khiển và hạ cánh tại vị trí hiện tại.

    Thứ tự kênh: AETR [Roll, Pitch, Throttle, Yaw, AUX1-4]
    """
    channels = [0] * 8
    channels[CH_ROLL]     = RC_CENTER   # Roll — giữ trung tâm
    channels[CH_PITCH]    = RC_CENTER   # Pitch — giữ trung tâm
    channels[CH_THROTTLE] = RC_CENTER   # Throttle — FC tự điều khiển khi Landing
    channels[CH_YAW]      = RC_CENTER   # Yaw — giữ trung tâm
    channels[CH_AUX1]     = RC_HIGH     # AUX1 (CH5) = 2000 → Duy trì ARM
    channels[CH_AUX2]     = RC_CENTER   # AUX2 (CH6) = 1500 → ANGLE mode
    channels[CH_AUX3]     = RC_HIGH     # AUX3 (CH7) = 2000 → KÍCH HOẠT SAFE LAND
    channels[CH_AUX4]     = RC_LOW      # AUX4 (CH8) = 1000 → RTH OFF

    frame = build_msp_set_raw_rc(channels)
    uart.write(frame)


def check_failsafe_config(data):
    """
    Kiểm tra và xử lý lệnh cấu hình failsafe từ PC.

    PC gửi prefix "FS:" kèm hành vi mong muốn:
    - "FS:rth" → Mất WiFi sẽ kích hoạt RTH
    - "FS:ignore" → Mất WiFi không can thiệp (drone bay tiếp mission)

    Lệnh cấu hình KHÔNG được chuyển tiếp xuống FC qua UART
    vì đây là lệnh nội bộ của ESP32.

    Args:
        data: Bytes dữ liệu nhận từ TCP

    Returns:
        bool: True nếu đây là lệnh cấu hình (đã xử lý), False nếu không phải
    """
    global failsafe_behavior

    if data.startswith(FS_PREFIX):
        config = data[len(FS_PREFIX):].strip()
        if config == b'rth':
            failsafe_behavior = "rth"
            print("[CONFIG] Failsafe behavior: RTH khi mất WiFi")
        elif config == b'ignore':
            failsafe_behavior = "ignore"
            print("[CONFIG] Failsafe behavior: IGNORE khi mất WiFi (mission tiếp tục)")
        else:
            print("[CONFIG] Không nhận dạng:", config)
        return True

    return False  # Không phải lệnh cấu hình → chuyển tiếp xuống FC


def handle_emergency(data):
    """
    Xử lý lệnh KHẨN CẤP từ PC — ưu tiên cao nhất.

    Khi nhận prefix "EM:", ESP32 sẽ:
    1. Xóa toàn bộ UART input buffer (discard response đang chờ)
    2. Chuyển tiếp MSP frame ngay lập tức xuống FC
    3. Reset failsafe state để không ghi đè lệnh emergency

    Dùng cho force_disarm() và force_safe_land() từ GCS.

    Args:
        data: Bytes dữ liệu nhận từ TCP, bắt đầu bằng "EM:"

    Returns:
        bool: True nếu đây là lệnh emergency (đã xử lý)
    """
    global failsafe_triggered

    if data.startswith(EMERGENCY_PREFIX):
        msp_frame = data[len(EMERGENCY_PREFIX):]

        # 1. Xóa UART input buffer — loại bỏ response cũ đang chờ
        #    Giúp FC nhận lệnh emergency nhanh hơn, không bị chậm bởi queue cũ
        uart.read()

        # 2. Gửi MSP frame xuống FC ngay lập tức
        if msp_frame:
            uart.write(msp_frame)
            print("[!] EMERGENCY — Đã chuyển tiếp lệnh khẩn cấp xuống FC")

        # 3. Tắt failsafe state — lệnh emergency có quyền cao hơn failsafe
        #    Nếu ESP32 đang gửi RTH liên tục, lệnh DISARM sẽ bị ghi đè
        #    → Phải tắt failsafe để lệnh DISARM có hiệu lực
        if failsafe_triggered:
            failsafe_triggered = False
            print("[!] EMERGENCY — Failsafe bị tắt bởi lệnh khẩn cấp")

        return True

    return False


def close_connection():
    """
    Đóng kết nối TCP an toàn.

    Chỉ được gọi khi kết nối thực sự không thể sử dụng được nữa
    (send/recv lỗi liên tiếp vượt ngưỡng CONN_ERROR_THRESHOLD).

    KHÔNG gọi trong failsafe handler — giữ socket để GCS có thể
    gửi lệnh emergency override.
    """
    global conn, conn_error_count
    if conn is not None:
        try:
            conn.close()
        except:
            pass
        conn = None
        conn_error_count = 0
        print("[!] TCP connection closed")


# ══════════════════════════════════════════════════════════════
# 6. VÒNG LẶP CHÍNH (Cầu nối + Failsafe chủ động)
# ══════════════════════════════════════════════════════════════
#
# KIẾN TRÚC AN TOÀN (ĐÃ SỬA):
#
# Phiên bản cũ: Failsafe → đóng socket ngay → GCS mất quyền emergency
# Phiên bản mới: Failsafe → GIỮ socket mở → GCS vẫn gửi được DISARM/Land
#
# Socket chỉ bị đóng khi:
# - send() lỗi liên tiếp vượt ngưỡng (kết nối thật sự đứt)
# - recv() nhận empty bytes (PC chủ động ngắt)
#
# Lệnh Emergency (prefix EM:) được ưu tiên:
# - Xóa UART buffer trước khi gửi
# - Tắt failsafe state để không bị ghi đè
# ══════════════════════════════════════════════════════════════

while True:

    # ─── LUỒNG 0: Quản lý kết nối TCP ───
    if conn is None:
        try:
            conn, addr = s.accept()
            conn.setblocking(False)
            print("\n[+] PC đã kết nối từ:", addr)

            # Reset đồng hồ đếm giờ an toàn
            last_msg_time = time.ticks_ms()
            failsafe_triggered = False
            conn_error_count = 0
        except OSError:
            pass

    else:
        # ─── LUỒNG 1: Đọc lệnh từ PC (WiFi) → Gửi xuống FC (UART) ───
        try:
            data_from_pc = conn.recv(1024)
            if data_from_pc == b'':
                # Nhận mảng byte rỗng = PC đã chủ động ngắt mạng
                print("[!] PC đã ngắt kết nối (empty recv)")
                close_connection()
                continue

            if data_from_pc:
                # Reset bộ đếm lỗi vì recv thành công
                conn_error_count = 0

                # ═══ KIỂM TRA LỆNH ĐẶC BIỆT (ưu tiên từ cao → thấp) ═══

                # Priority 1: EMERGENCY — Lệnh khẩn cấp (DISARM/Safe Land)
                if handle_emergency(data_from_pc):
                    last_msg_time = time.ticks_ms()

                # Priority 2: FAILSAFE CONFIG — Cấu hình hành vi mất WiFi
                elif check_failsafe_config(data_from_pc):
                    last_msg_time = time.ticks_ms()

                # Priority 3: HEARTBEAT — Giữ kết nối sống
                elif data_from_pc.startswith(HEARTBEAT_PREFIX):
                    last_msg_time = time.ticks_ms()
                    # Heartbeat không cần chuyển tiếp xuống FC

                # Priority 4: MSP DATA — Chuyển tiếp bình thường xuống FC
                else:
                    uart.write(data_from_pc)
                    last_msg_time = time.ticks_ms()

                # ═══ RESET FAILSAFE KHI NHẬN TÍN HIỆU TỪ PC ═══
                # Bất kỳ data nào từ PC (kể cả heartbeat) đều chứng minh
                # kết nối WiFi còn hoạt động → tắt failsafe
                if failsafe_triggered:
                    print("[+] PC đã kết nối lại — tắt Failsafe")
                    failsafe_triggered = False

        except OSError:
            # Lỗi recv tạm thời do non-blocking socket — KHÔNG đóng socket
            # Đây KHÔNG phải mất kết nối, chỉ là chưa có data để đọc
            pass

        # ─── LUỒNG 2: Đọc phản hồi từ FC (UART) → Gửi lên PC (WiFi) ───
        if uart.any():
            # Giới hạn đọc 256 bytes/lần để tránh fragment lớn trên WiFi
            data_from_fc = uart.read(256)
            if data_from_fc and conn is not None:
                try:
                    conn.send(data_from_fc)
                    conn_error_count = 0  # Reset counter khi send OK
                except OSError:
                    # Send thất bại — kết nối có thể bị đứt
                    conn_error_count += 1
                    if conn_error_count >= CONN_ERROR_THRESHOLD:
                        # Lỗi liên tiếp vượt ngưỡng → kết nối thật sự đứt
                        print("[!] Send lỗi liên tiếp — đóng kết nối")
                        close_connection()
                        continue
                    # Dưới ngưỡng → bỏ qua, thử lại lần sau

        # ─── LUỒNG 3: Watchdog Failsafe (chủ động gửi lệnh an toàn) ───
        elapsed = time.ticks_diff(time.ticks_ms(), last_msg_time)

        if elapsed > FAILSAFE_TIMEOUT_MS:
            if not failsafe_triggered:
                # ═══ LẦN ĐẦU PHÁT HIỆN MẤT KẾT NỐI ═══
                failsafe_triggered = True
                print("\n[!] MẤT KẾT NỐI VỚI PC!")

                if failsafe_behavior == "rth":
                    print("[FAILSAFE] Kích hoạt RTH — Drone bay về Home")
                    send_failsafe_rth()
                elif failsafe_behavior == "ignore":
                    print("[FAILSAFE] IGNORE — Drone tiếp tục bay mission")
                    # Không gửi lệnh can thiệp, FC tự bay theo mission đã nạp
                else:
                    print("[FAILSAFE] Mặc định: Kích hoạt RTH")
                    send_failsafe_rth()

                last_failsafe_rc_time = time.ticks_ms()

            else:
                # ═══ FAILSAFE ĐANG HOẠT ĐỘNG — GỬI RC LIÊN TỤC ═══
                # FC cần nhận tín hiệu RC liên tục (mỗi 100ms).
                # Nếu ngừng gửi, FC sẽ timeout và tự DISARM → drone rơi!
                now = time.ticks_ms()
                if time.ticks_diff(now, last_failsafe_rc_time) > FAILSAFE_RC_INTERVAL_MS:
                    if failsafe_behavior == "rth":
                        send_failsafe_rth()
                    # "ignore" → không gửi gì, FC tự bay mission
                    last_failsafe_rc_time = now

            # ═══ QUAN TRỌNG: KHÔNG ĐÓNG SOCKET KHI FAILSAFE ═══
            #
            # Phiên bản cũ đóng socket ngay tại đây → GCS mất quyền
            # gửi Emergency DISARM/Safe Land → drone không dừng được!
            #
            # Phiên bản mới: GIỮ socket mở. Nếu GCS gửi data mới:
            # - recv() ở LUỒNG 1 sẽ nhận được
            # - last_msg_time được reset
            # - failsafe_triggered = False
            # - Lệnh Emergency (EM:) bypass failsafe hoàn toàn
            #
            # Socket chỉ bị đóng khi send() ở LUỒNG 2 lỗi liên tiếp
            # vượt ngưỡng CONN_ERROR_THRESHOLD.

    # ─── Yield CPU + Garbage Collection định kỳ ───
    time.sleep_ms(1)  # Tránh CPU-hogging, giảm latency jitter
    gc_counter += 1
    if gc_counter >= GC_INTERVAL:
        gc.collect()  # Dọn heap chống phân mảnh bộ nhớ MicroPython
        gc_counter = 0