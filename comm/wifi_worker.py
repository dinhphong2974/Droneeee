import socket
import time
import random
from PySide6.QtCore import QThread, Signal

class WifiWorker(QThread):
    # Định nghĩa các "đường dây tín hiệu" để gửi báo cáo về main.py
    connection_status = Signal(bool, str) # Trạng thái mạng: (Thành công/Thất bại, Lời nhắn)
    telemetry_data = Signal(dict)         # Dữ liệu giải mã gửi lên UI hiển thị

    def __init__(self, ip="192.168.4.1", port=8080, is_mock=False):
        super().__init__()
        self.ip = ip
        self.port = int(port)
        self.is_mock = is_mock
        
        self.is_running = True
        self.sock = None

    def run(self):
        """Hàm này tự động chạy trên một luồng (thread) riêng biệt khi gọi self.start()"""
        if self.is_mock:
            self._run_mock_mode()
        else:
            self._run_real_mode()

    def _run_real_mode(self):
        """Chế độ kết nối thật tới ESP32 (Cho phép FC chưa kết nối)"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.2) # Set timeout ngắn lại để luồng chạy mượt hơn
        
        try:
            self.sock.connect((self.ip, self.port))
            self.connection_status.emit(True, f"Đã kết nối Wifi với {self.ip}")
            
            # Lệnh giả để nuôi Watchdog ESP32
            msp_keep_alive_ping = b'$M<\x00\x6e\x6e'
            
            while self.is_running:
                try:
                    # 1. GỬI NHỊP TIM: Gửi liên tục để ESP32 không đá PC
                    self.sock.sendall(msp_keep_alive_ping)
                    
                    # 2. CHỜ NHẬN DỮ LIỆU TỪ FC
                    data = self.sock.recv(1024) 
                    
                    if not data:
                        raise ConnectionAbortedError("ESP32 chủ động đóng kết nối TCP.")
                    
                    # Nếu có FC cắm vào, nó sẽ in ra dòng này
                    print(f"Dữ liệu từ FC: {data}") 
                    
                except socket.timeout:
                    # NẾU CHƯA CẮM FC NÓ SẼ NHẢY VÀO ĐÂY
                    # Thay vì ngắt kết nối, ta chỉ bỏ qua (pass) để vòng lặp tiếp tục gửi nhịp tim
                    print("Kết nối ESP32 ổn định. Đang chờ cắm mạch FC...")
                    pass 
                    
                # Nghỉ 0.1s (10Hz)
                time.sleep(0.1)
                    
        except ConnectionRefusedError:
            self.connection_status.emit(False, "ESP32 từ chối kết nối (Sai IP/Port).")
        except Exception as e:
            self.connection_status.emit(False, f"Mất kết nối: {str(e)}")
        finally:
            self._close_socket()

    def _run_mock_mode(self):
        """Chế độ giả lập để test UI mượt mà không cần cắm phần cứng"""
        self.connection_status.emit(True, "Đang chạy chế độ Mock Test (Giả lập)")
        
        # Thông số gốc giả lập
        mock_volt = 25.2
        mock_pitch = 0.0
        
        while self.is_running:
            # Giả lập pin tụt dần
            mock_volt -= random.uniform(0.001, 0.01)
            if mock_volt < 19.5:
                mock_volt = 25.2
                
            # Giả lập dao động góc nghiêng
            mock_pitch += random.uniform(-1.0, 1.0)
            if mock_pitch > 30: mock_pitch = 30
            elif mock_pitch < -30: mock_pitch = -30

            # Giả lập vòng tua động cơ
            motor_base = random.randint(1100, 1500)

            # Đóng gói dữ liệu giả và gửi lên UI
            fake_data = {
                "voltage": mock_volt,
                "pitch": mock_pitch,
                "roll": random.uniform(-5.0, 5.0),
                "yaw": random.uniform(0, 360),
                "motor1": motor_base + random.randint(-20, 20),
                "motor2": motor_base + random.randint(-20, 20),
                "motor3": motor_base + random.randint(-20, 20),
                "motor4": motor_base + random.randint(-20, 20),
            }
            self.telemetry_data.emit(fake_data)
            
            # Tốc độ cập nhật UI (20Hz = 50ms/lần)
            time.sleep(0.05) 
            
        self.connection_status.emit(False, "Đã dừng Mock Test")

    def _close_socket(self):
        """Đóng kết nối socket an toàn"""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None

    def stop(self):
        """Hàm được gọi từ main.py khi người dùng bấm nút Ngắt Kết Nối"""
        self.is_running = False
        self._close_socket()