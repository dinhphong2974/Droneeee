import time
import socket
from PySide6.QtCore import QThread, Signal

class WifiWorker(QThread):
    # Tín hiệu gửi từ luồng phụ (mạng) lên luồng chính (giao diện)
    connection_status = Signal(bool, str) # True/False, Lời nhắn
    telemetry_data = Signal(dict)         # Gói dữ liệu MSP đã giải mã
    
    def __init__(self, ip="192.168.4.1", port=8080, is_mock=False):
        super().__init__()
        self.ip = ip
        self.port = int(port)
        self.is_mock = is_mock
        self.is_running = True
        self.sock = None

    def run(self):
        """Khởi chạy luồng ngầm"""
        if self.is_mock:
            self._run_mock_mode()
        else:
            self._run_wifi_tcp()

    def _run_wifi_tcp(self):
        """Kết nối TCP Socket tới module ESP32"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3.0) # Chờ tối đa 3s để kết nối ban đầu
            self.sock.connect((self.ip, self.port))
            
            # Nếu qua được dòng trên là đã kết nối thành công
            self.connection_status.emit(True, f"Đã kết nối ESP32 tại {self.ip}:{self.port}")
            self.sock.settimeout(0.1) # Giảm timeout xuống cực thấp để vòng lặp đọc không bị nghẽn
            
            while self.is_running:
                try:
                    # Lắng nghe dữ liệu MSP truyền từ FC -> ESP32 -> PC
                    raw_data = self.sock.recv(1024)
                    
                    if raw_data:
                        # TẠI ĐÂY SẼ LÀ HÀM GIẢI MÃ MSP (Sẽ viết ở bước sau)
                        # parsed_data = parse_msp(raw_data)
                        
                        # Tạm thời phát tín hiệu rỗng
                        parsed_data = {} 
                        if parsed_data:
                            self.telemetry_data.emit(parsed_data)
                            
                except socket.timeout:
                    # Chuyện bình thường khi mạch FC chưa kịp gửi dữ liệu mới
                    pass
                except Exception as e:
                    self.connection_status.emit(False, f"Mất tín hiệu mạng: {str(e)}")
                    break # Thoát vòng lặp nếu rớt mạng
                
                time.sleep(0.01) # Nhường CPU cho các tác vụ khác
                
        except Exception as e:
            self.connection_status.emit(False, f"Không thể kết nối Wifi: {str(e)}")
        finally:
            self.is_running = False
            if self.sock:
                self.sock.close()

    def _run_mock_mode(self):
        """Chế độ mô phỏng Test Giao Diện"""
        self.connection_status.emit(True, "Đang chạy chế độ Mock Test 🧪")
        voltage = 25.2 # Pin 6S đầy
        motor_base = 1270
        
        while self.is_running:
            voltage -= 0.05
            if voltage < 19.8: voltage = 19.8
                
            mock_data = {
                "voltage": voltage,
                "current": 10.5,
                "pitch": 5.0, "roll": -2.1, "yaw": 15.0,
                "motor1": motor_base + 1, "motor2": motor_base - 5,
                "motor3": motor_base - 8, "motor4": motor_base + 2,
                "gps_fix": "3D FIX", "sats": 12, "alt": 15.5
            }
            self.telemetry_data.emit(mock_data)
            time.sleep(0.1) # Giả lập tần số 10Hz

    def stop(self):
        """Dừng luồng an toàn khi đóng ứng dụng"""
        self.is_running = False
        self.wait()