import machine
import network
import socket
import time
import struct
import gc

# ══════════════════════════════════════════════════════════════
# 0. CẤU HÌNH DEBUG — Tắt khi bay thật để giảm latency
# ══════════════════════════════════════════════════════════════
DEBUG = True  # ★ Đặt False khi bay thật

def _log(msg):
    """In log chỉ khi DEBUG=True. Tránh print() blocking trong bay."""
    if DEBUG:
        print(msg)

# ══════════════════════════════════════════════════════════════
# 1. CẤU HÌNH UART (Giao tiếp với mạch FC SpeedyBee F405)
# ══════════════════════════════════════════════════════════════
# Sử dụng UART1, chân TX=17, RX=16 (theo sơ đồ chân ESP32)
# txbuf/rxbuf lớn hơn mặc định để tránh tràn khi busy
uart = machine.UART(1, baudrate=115200, tx=17, rx=16,
                    txbuf=512, rxbuf=512)

# ══════════════════════════════════════════════════════════════
# 2. CẤU HÌNH WIFI ACCESS POINT
# ══════════════════════════════════════════════════════════════
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="Drone_GCS_Wifi", password="D0ne$2026!Secure",
          authmode=4)  # 4 = AUTH_WPA2_PSK

_log("Đang khởi tạo Wifi...")
while not ap.active():
    time.sleep_ms(100)
_log("Wifi AP đã sẵn sàng. IP: " + ap.ifconfig()[0])

# ══════════════════════════════════════════════════════════════
# 3. CẤU HÌNH TCP SOCKET SERVER
# ══════════════════════════════════════════════════════════════
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bật SO_REUSEADDR giúp cổng 8080 rảnh ngay lập tức nếu ESP32 bị reset
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 8080))
s.listen(1)
s.setblocking(False)  # Ép server không được khóa luồng

_log("Đang chờ PC kết nối tới cổng 8080...")

# ══════════════════════════════════════════════════════════════
# 4. WATCHDOG TIMER PHẦN CỨNG
# ══════════════════════════════════════════════════════════════
# ★ FIX MEDIUM-04: Nếu code crash/treo, ESP32 tự reset sau 5 giây
# LƯU Ý: WDT KHÔNG THỂ TẮT sau khi bật (hardware limitation)
try:
    from machine import WDT
    wdt = WDT(timeout=5000)
    _log("[WDT] Watchdog Timer đã bật — timeout 5s")
except Exception:
    wdt = None
    _log("[WDT] Không hỗ trợ WDT trên board này")

# ══════════════════════════════════════════════════════════════
# 5. HẰNG SỐ VÀ BIẾN FAILSAFE
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
# ★ FIX BUG-01: Sửa comment thứ tự kênh đúng AETR
# Thứ tự: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
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
# 6. HÀM XÂY DỰNG FRAME MSP — ZERO ALLOCATION
# ══════════════════════════════════════════════════════════════

# ★ FIX HIGH-01: Pre-allocate buffer cố định — tránh tạo rác mỗi lần gọi
# Frame MSP_SET_RAW_RC: Header(3) + Size(1) + Cmd(1) + Payload(16) + Checksum(1) = 22 bytes
_rc_frame_buf = bytearray(22)
_rc_frame_buf[0:3] = b'$M<'
_rc_frame_buf[3] = 16          # Size cố định (8 kênh x 2 bytes)
_rc_frame_buf[4] = MSP_SET_RAW_RC  # Cmd = 200

# ★ FIX MEDIUM-01: Dùng tuple (immutable) thay list cho hằng số failsafe
# MicroPython có thể lưu tuple trong flash ROM, không chiếm heap
# Thứ tự AETR: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
_FAILSAFE_RTH_CHANNELS = (
    RC_CENTER,   # Roll — giữ trung tâm
    RC_CENTER,   # Pitch — giữ trung tâm
    RC_CENTER,   # Throttle — FC tự điều khiển khi RTH
    RC_CENTER,   # Yaw — giữ trung tâm
    RC_HIGH,     # AUX1 (CH5) = 2000 → Duy trì ARM
    RC_CENTER,   # AUX2 (CH6) = 1500 → ANGLE mode (ổn định)
    RC_LOW,      # AUX3 (CH7) = 1000 → Safe Land OFF
    RC_HIGH,     # AUX4 (CH8) = 2000 → KÍCH HOẠT RTH
)

_FAILSAFE_LAND_CHANNELS = (
    RC_CENTER,   # Roll — giữ trung tâm
    RC_CENTER,   # Pitch — giữ trung tâm
    RC_CENTER,   # Throttle — FC tự điều khiển khi Landing
    RC_CENTER,   # Yaw — giữ trung tâm
    RC_HIGH,     # AUX1 (CH5) = 2000 → Duy trì ARM
    RC_CENTER,   # AUX2 (CH6) = 1500 → ANGLE mode
    RC_HIGH,     # AUX3 (CH7) = 2000 → KÍCH HOẠT SAFE LAND
    RC_LOW,      # AUX4 (CH8) = 1000 → RTH OFF
)


def build_msp_set_raw_rc(channels):
    """
    Đóng gói MSP_SET_RAW_RC — ZERO ALLOCATION.

    Ghi trực tiếp vào pre-allocated buffer, không tạo object rác.
    struct.pack_into() ghi vào buffer có sẵn thay vì tạo bytes mới.

    Args:
        channels: Tuple/list 8 giá trị kênh RC (1000-2000 μs)
                  Thứ tự AETR: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]

    Returns:
        memoryview của buffer cố định (KHÔNG tạo object mới)
    """
    # Ghi 8 kênh uint16 LE trực tiếp vào buffer offset 5
    struct.pack_into('<8H', _rc_frame_buf, 5, *channels)

    # Tính checksum trên buffer — hằng số size^cmd = 16^200
    checksum = 16 ^ MSP_SET_RAW_RC  # = 16 ^ 200 (hằng số, có thể precompute)
    for i in range(5, 21):
        checksum ^= _rc_frame_buf[i]
    _rc_frame_buf[21] = checksum & 0xFF

    return _rc_frame_buf  # Trả về chính buffer, không copy


def send_failsafe_rth():
    """
    Gửi lệnh RTH qua UART tới FC — zero allocation.

    Kích hoạt AUX4=2000 (CH8) để bật mode NAV RTH trên INAV.
    Dùng tuple hằng _FAILSAFE_RTH_CHANNELS → không tạo list rác.
    """
    uart.write(build_msp_set_raw_rc(_FAILSAFE_RTH_CHANNELS))


def send_failsafe_land():
    """
    Gửi lệnh Safe Land qua UART tới FC — zero allocation.

    Kích hoạt AUX3=2000 (CH7) để bật Emergency Landing.
    """
    uart.write(build_msp_set_raw_rc(_FAILSAFE_LAND_CHANNELS))


def check_failsafe_config(data):
    """
    Kiểm tra và xử lý lệnh cấu hình failsafe từ PC.

    PC gửi prefix "FS:" kèm hành vi mong muốn:
    - "FS:rth" → Mất WiFi sẽ kích hoạt RTH
    - "FS:ignore" → Mất WiFi không can thiệp

    Args:
        data: Bytes dữ liệu (có thể là 1 phần của chunk lớn hơn)

    Returns:
        bool: True nếu đây là lệnh cấu hình (đã xử lý)
    """
    global failsafe_behavior

    if data.startswith(FS_PREFIX):
        config = data[len(FS_PREFIX):].strip()
        if config == b'rth':
            failsafe_behavior = "rth"
            _log("[CONFIG] Failsafe: RTH khi mất WiFi")
        elif config == b'ignore':
            failsafe_behavior = "ignore"
            _log("[CONFIG] Failsafe: IGNORE khi mất WiFi")
        else:
            _log("[CONFIG] Không nhận dạng: " + str(config))
        return True

    return False


# ══════════════════════════════════════════════════════════════
# 7. VÒNG LẶP CHÍNH (Cầu nối + Failsafe chủ động)
# ══════════════════════════════════════════════════════════════

# Chạy GC 1 lần trước khi vào main loop để có heap sạch
gc.collect()
_log("[MEM] Free: " + str(gc.mem_free()) + " bytes")

while True:

    # ★ FIX MEDIUM-04: Feed watchdog mỗi vòng lặp
    if wdt:
        wdt.feed()

    # ─── LUỒNG 0: Quản lý kết nối TCP ───
    if conn is None:

        # ★ FIX CRITICAL-01: Khi failsafe đang hoạt động,
        #   TIẾP TỤC gửi RC song song với việc chờ kết nối mới
        if failsafe_triggered and failsafe_behavior == "rth":
            now = time.ticks_ms()
            if time.ticks_diff(now, last_failsafe_rc_time) > FAILSAFE_RC_INTERVAL_MS:
                send_failsafe_rth()
                last_failsafe_rc_time = now

        # ★ FIX HIGH-02: Xả UART buffer tránh tràn khi không có PC
        if uart.any():
            uart.read()  # Đọc và bỏ (không có PC để gửi)

        # Thử accept kết nối mới (non-blocking)
        try:
            conn, addr = s.accept()
            conn.setblocking(False)
            _log("\n[+] PC đã kết nối từ: " + str(addr))

            # Reset watchdog failsafe
            last_msg_time = time.ticks_ms()
            failsafe_triggered = False
        except OSError:
            pass

        # ★ FIX CRITICAL-02: Nhường CPU khi chờ kết nối
        time.sleep_ms(10)

    else:
        # ─── LUỒNG 1: Đọc lệnh từ PC (WiFi) → Gửi xuống FC (UART) ───
        try:
            data_from_pc = conn.recv(1024)

            if data_from_pc == b'':
                # ★ FIX HIGH-04: PC ngắt kết nối chủ động → dọn dẹp ngay
                _log("[!] PC đã ngắt kết nối")
                try:
                    conn.close()
                except:
                    pass
                conn = None
                continue  # Quay lại đầu vòng lặp, không chờ watchdog timeout

            if data_from_pc:
                # ★ FIX HIGH-03: Tách lệnh FS: khỏi luồng MSP
                # TCP là stream — có thể nhận MSP + FS: ghép trong 1 chunk
                fs_idx = data_from_pc.find(FS_PREFIX)
                if fs_idx >= 0:
                    # Phần MSP trước FS: → forward xuống FC
                    if fs_idx > 0:
                        uart.write(data_from_pc[:fs_idx])
                    # Phần FS: → xử lý nội bộ
                    check_failsafe_config(data_from_pc[fs_idx:])
                else:
                    # Toàn bộ là MSP → forward xuống FC
                    uart.write(data_from_pc)

                # Có tín hiệu từ PC! Reset watchdog failsafe
                last_msg_time = time.ticks_ms()

                # Nếu đang failsafe mà PC kết nối lại → tắt failsafe
                if failsafe_triggered:
                    _log("[+] PC đã kết nối lại — tắt Failsafe")
                    failsafe_triggered = False

        except OSError as e:
            # errno 11 (EAGAIN) = non-blocking, không có data → bình thường
            # Các errno khác = lỗi mạng nghiêm trọng → đóng kết nối
            err_no = getattr(e, 'errno', 0) if hasattr(e, 'errno') else 0
            if err_no != 11 and err_no != 0:
                _log("[!] Lỗi mạng: errno=" + str(err_no))
                try:
                    conn.close()
                except:
                    pass
                conn = None
                continue

        # ─── LUỒNG 2: Đọc phản hồi từ FC (UART) → Gửi lên PC (WiFi) ───
        if uart.any():
            data_from_fc = uart.read(256)  # ★ Giới hạn 256 bytes/lần
            if data_from_fc:
                try:
                    conn.send(data_from_fc)
                except OSError:
                    # Đang gửi thì rớt mạng → đóng kết nối
                    try:
                        conn.close()
                    except:
                        pass
                    conn = None
                    continue

        # ─── LUỒNG 3: Watchdog Failsafe (chủ động gửi lệnh an toàn) ───
        elapsed = time.ticks_diff(time.ticks_ms(), last_msg_time)

        if elapsed > FAILSAFE_TIMEOUT_MS:
            if not failsafe_triggered:
                # ═══ LẦN ĐẦU PHÁT HIỆN MẤT KẾT NỐI ═══
                failsafe_triggered = True
                _log("\n[!] MẤT KẾT NỐI VỚI PC!")

                if failsafe_behavior == "rth":
                    _log("[FAILSAFE] Kích hoạt RTH — Drone bay về Home")
                    send_failsafe_rth()
                elif failsafe_behavior == "ignore":
                    _log("[FAILSAFE] IGNORE — Drone tiếp tục bay mission")
                else:
                    _log("[FAILSAFE] Mặc định: Kích hoạt RTH")
                    send_failsafe_rth()

                last_failsafe_rc_time = time.ticks_ms()

                # Ngắt socket cũ, dọn dẹp
                try:
                    conn.close()
                except:
                    pass
                conn = None
                # ★ FIX CRITICAL-01: KHÔNG break — vòng lặp tiếp theo
                # sẽ rơi vào nhánh `if conn is None` và TIẾP TỤC gửi
                # RC failsafe nhờ block failsafe ở đầu nhánh đó

            else:
                # ═══ FAILSAFE ĐANG HOẠT ĐỘNG — GỬI RC LIÊN TỤC ═══
                now = time.ticks_ms()
                if time.ticks_diff(now, last_failsafe_rc_time) > FAILSAFE_RC_INTERVAL_MS:
                    if failsafe_behavior == "rth":
                        send_failsafe_rth()
                    last_failsafe_rc_time = now

        # ★ FIX CRITICAL-02: Nhường CPU 1ms mỗi vòng
        # Vẫn đạt ~1000Hz — đủ nhanh cho RC 10Hz
        time.sleep_ms(1)
