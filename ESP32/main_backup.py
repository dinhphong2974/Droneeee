import machine
import network
import socket
import time
import struct

# ══════════════════════════════════════════════════════════════
# 1. CẤU HÌNH UART (Giao tiếp với mạch FC SpeedyBee F405)
# ══════════════════════════════════════════════════════════════
# Sử dụng UART1, chân TX=17, RX=16 (theo sơ đồ chân ESP32)
uart = machine.UART(1, baudrate=115200, tx=17, rx=16)

# ══════════════════════════════════════════════════════════════
# 2. CẤU HÌNH WIFI ACCESS POINT
# ══════════════════════════════════════════════════════════════
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="Drone_GCS_Wifi", password="password123")

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
FAILSAFE_TIMEOUT_MS = 1500

# ── Chu kỳ gửi lệnh RC khi failsafe đang hoạt động (ms) ──
# FC cần nhận tín hiệu RC liên tục, nếu ngừng gửi FC sẽ tự DISARM
FAILSAFE_RC_INTERVAL_MS = 100

# ── Giá trị kênh RC mặc định (PWM μs) ──
RC_CENTER = 1500     # Giá trị trung tâm (Roll, Pitch, Yaw)
RC_LOW = 1000        # Giá trị thấp (Throttle min, AUX tắt)
RC_HIGH = 2000       # Giá trị cao (AUX bật)

# ── Ánh xạ kênh AUX theo cấu hình INAV Modes tab ──
# Thứ tự: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]
# AUX1 (CH5): ARM/DISARM — 2000=ARM, 1000=DISARM
# AUX2 (CH6): Flight Mode — 1000=Acro, 1500=ANGLE, 2000=ALTHOLD+POSHOLD
# AUX3 (CH7): Safe Land — 2000=Kích hoạt hạ cánh, 1000=Bình thường
# AUX4 (CH8): RTH — 2000=Kích hoạt Return To Home, 1000=Bình thường

# ── MSP Protocol Constants ──
MSP_SET_RAW_RC = 200  # Mã lệnh gửi 8 kênh RC xuống FC

# ── Prefix lệnh cấu hình failsafe từ PC ──
# PC gửi "FS:rth" hoặc "FS:ignore" để cấu hình hành vi failsafe
FS_PREFIX = b'FS:'

# ── Biến trạng thái ──
conn = None                    # Kết nối TCP hiện tại
last_msg_time = time.ticks_ms()  # Thời điểm nhận tin nhắn cuối từ PC
failsafe_triggered = False     # Failsafe đã kích hoạt chưa
last_failsafe_rc_time = 0      # Thời điểm gửi RC failsafe lần cuối

# ── Cấu hình hành vi failsafe (có thể thay đổi từ PC) ──
# "rth" = Mất WiFi → gửi RTH (AUX4=2000), drone bay về Home
# "ignore" = Mất WiFi → không can thiệp, drone tiếp tục bay mission từ FC
failsafe_behavior = "rth"


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
                  Thứ tự: [Roll, Pitch, Yaw, Throttle, AUX1, AUX2, AUX3, AUX4]

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

    Lưu ý: INAV yêu cầu cần gạo phải ở mức DISARM trước khi gạt
    sang mức ARM. Khi failsafe, drone đã ARM sẵn nên chỉ cần
    duy trì AUX1=2000 liên tục.
    """
    channels = [
        RC_CENTER,   # Roll — giữ trung tâm
        RC_CENTER,   # Pitch — giữ trung tâm
        RC_CENTER,   # Yaw — giữ trung tâm
        RC_CENTER,   # Throttle — FC tự điều khiển khi RTH
        RC_HIGH,     # AUX1 (CH5) = 2000 → Duy trì ARM
        RC_CENTER,   # AUX2 (CH6) = 1500 → ANGLE mode (ổn định)
        RC_LOW,      # AUX3 (CH7) = 1000 → Safe Land OFF
        RC_HIGH,     # AUX4 (CH8) = 2000 → KÍCH HOẠT RTH
    ]
    frame = build_msp_set_raw_rc(channels)
    uart.write(frame)


def send_failsafe_land():
    """
    Gửi lệnh Safe Land (hạ cánh tại chỗ) qua UART tới FC.

    Kích hoạt AUX3=2000 (CH7) để bật mode Emergency Landing.
    FC sẽ tự động chiếm quyền điều khiển và hạ cánh tại vị trí hiện tại.

    Dùng khi:
    - Nhận lệnh Safe Land từ PC
    - Drone đang bay quá xa và cần hạ cánh khẩn cấp
    """
    channels = [
        RC_CENTER,   # Roll — giữ trung tâm
        RC_CENTER,   # Pitch — giữ trung tâm
        RC_CENTER,   # Yaw — giữ trung tâm
        RC_CENTER,   # Throttle — FC tự điều khiển khi Landing
        RC_HIGH,     # AUX1 (CH5) = 2000 → Duy trì ARM
        RC_CENTER,   # AUX2 (CH6) = 1500 → ANGLE mode
        RC_HIGH,     # AUX3 (CH7) = 2000 → KÍCH HOẠT SAFE LAND
        RC_LOW,      # AUX4 (CH8) = 1000 → RTH OFF
    ]
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


# ══════════════════════════════════════════════════════════════
# 6. VÒNG LẶP CHÍNH (Cầu nối + Failsafe chủ động)
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
        except OSError:
            pass

    else:
        # ─── LUỒNG 1: Đọc lệnh từ PC (WiFi) → Gửi xuống FC (UART) ───
        try:
            data_from_pc = conn.recv(1024)
            if data_from_pc == b'':
                # Nhận mảng byte rỗng = PC đã chủ động ngắt mạng
                raise OSError

            if data_from_pc:
                # Kiểm tra xem đây có phải lệnh cấu hình failsafe nội bộ không
                if not check_failsafe_config(data_from_pc):
                    # Không phải lệnh cấu hình → chuyển tiếp MSP xuống FC
                    uart.write(data_from_pc)

                # Có tín hiệu từ PC! Reset Watchdog
                last_msg_time = time.ticks_ms()

                # Nếu đang trong trạng thái failsafe mà PC kết nối lại → tắt failsafe
                if failsafe_triggered:
                    print("[+] PC đã kết nối lại — tắt Failsafe")
                    failsafe_triggered = False

        except OSError:
            pass  # Lỗi mạng tạm thời, chạy tiếp

        # ─── LUỒNG 2: Đọc phản hồi từ FC (UART) → Gửi lên PC (WiFi) ───
        if uart.any():
            data_from_fc = uart.read()
            try:
                conn.send(data_from_fc)
            except OSError:
                pass  # Đang gửi thì rớt mạng, bỏ qua

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

                # Ngắt socket cũ bị lỗi, dọn dẹp
                try:
                    conn.close()
                except:
                    pass
                conn = None

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