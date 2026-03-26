import machine
import network
import socket
import time

# 1. CẤU HÌNH UART (Giao tiếp với mạch FC)
# Sử dụng UART1, chân TX=17, RX=16 (Tùy thuộc sơ đồ chân con ESP32 của bạn)
uart = machine.UART(1, baudrate=115200, tx=17, rx=16)

# 2. CẤU HÌNH WIFI ACCESS POINT
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="Drone_GCS_Wifi", password="password123") # Tên và mật khẩu Wifi

print("Đang khởi tạo Wifi...")
while not ap.active():
    pass
print("Wifi AP đã sẵn sàng. IP:", ap.ifconfig()[0]) # Thường là 192.168.4.1

# 3. CẤU HÌNH TCP SOCKET SERVER
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bật SO_REUSEADDR giúp cổng 8080 rảnh ngay lập tức nếu ESP32 bị reset đột ngột
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
s.bind(('', 8080))
s.listen(1)
s.setblocking(False) # Rất quan trọng: Ép server không được khóa luồng

print("Đang chờ PC kết nối tới cổng 8080...")
conn = None
last_msg_time = time.ticks_ms()
failsafe_triggered = False

# 4. VÒNG LẶP CHÍNH (Cầu nối trong suốt & Failsafe)
while True:
    
    # --- LUỒNG 0: Quản lý kết nối ---
    if conn is None:
        try:
            # Chờ kết nối, nếu không ai kết nối thì văng OSError và bỏ qua (pass)
            conn, addr = s.accept()
            conn.setblocking(False)
            print("\n[+] PC đã kết nối từ:", addr)
            
            # Reset lại đồng hồ đếm giờ an toàn
            last_msg_time = time.ticks_ms()
            failsafe_triggered = False
        except OSError:
            pass 
            
    else:
        # --- LUỒNG 1: Đọc lệnh từ PC (Wifi) -> Gửi xuống FC (UART) ---
        try:
            data_from_pc = conn.recv(1024)
            if data_from_pc == b'': 
                # Nhận được mảng byte rỗng đồng nghĩa với việc PC đã chủ động ngắt mạng
                raise OSError 
            
            if data_from_pc:
                uart.write(data_from_pc)
                # Có tín hiệu điều khiển! Reset Watchdog
                last_msg_time = time.ticks_ms() 
                failsafe_triggered = False
        except OSError:
            pass # Lỗi vặt do mạng chập chờn, cứ chạy tiếp

        # --- LUỒNG 2: Đọc thông số từ FC (UART) -> Gửi lên PC (Wifi) ---
        if uart.any():
            data_from_fc = uart.read()
            try:
                conn.send(data_from_fc)
            except OSError:
                pass # Đang gửi thì rớt mạng, bỏ qua để lát xử lý ở Luồng 3

        # --- LUỒNG 3: Watchdog Failsafe (Kích hoạt khi mất mạng) ---
        # Nếu quá 1000 mili-giây (1 giây) không nhận được gì từ PC
        if time.ticks_diff(time.ticks_ms(), last_msg_time) > 1000:
            if not failsafe_triggered:
                print("\n[!] MẤT KẾT NỐI VỚI PC! Đang chờ kết nối lại...")
                failsafe_triggered = True
                
                # TẠI ĐÂY: Bạn có thể tự gửi 1 chuỗi byte MSP yêu cầu RTH (Return to Home)
                # uart.write(b'$M<...') 
                
                # Ngắt socket cũ bị lỗi, dọn dẹp biến conn về None
                # Vòng lặp sẽ tự động quay lại Luồng 0 để chực chờ PC kết nối lại
                try:
                    conn.close()
                except:
                    pass
                conn = None