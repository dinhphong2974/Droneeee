# Danh Sách Kỹ Năng / Skills Đề Xuất (DroneGCS)

> **Mục đích:** Hướng dẫn các AI Agent (như Claude, Gemini) gọi đúng các công cụ (Skills) thông qua việc tag `@skill-name` trong prompt hoặc kích hoạt hệ thống chuyên dụng. File này phân loại toàn bộ skills cần thiết để bảo trì, tối ưu hóa và mở rộng hệ thống DroneGCS.

---

## 1. 🎛️ Lập Trình Nhúng & Giao Tiếp Phần Cứng
*Dành cho code trên bo mạch ESP32 (MicroPython), giao thức MSP và giao tiếp UART/I2C.*

- **`@firmware-analyst`**
  - **Sử dụng khi:** Chỉnh sửa, bảo trì file `ESP32/main.py`.
  - **Mục đích:** Tối ưu hóa chu kì bộ nhớ (GC), tránh phân mảnh bộ nhớ RAM hẹp trên MCU, phân tích rủi ro buffer overflows khi đọc UART và quản lý TCP Socket non-blocking.
- **`@c-pro` / `@cpp-pro`**
  - **Sử dụng khi:** Đọc và đối chiếu mã nguồn của INAV/Betaflight firmware để hiểu chính xác định dạng byte của MSP protocols (MultiWii Serial Protocol), hoặc nếu muốn viết lại firmware ESP32 bằng C/C++ để tối ưu latency.
- **`@network-engineer`**
  - **Sử dụng khi:** Tối ưu hóa giao thức TCP qua Wifi nội bộ, giảm ping, kiểm soát QoS, hoặc viết lại transport layer sang UDP để gửi telemetry mượt mà hơn.

## 2. 🧠 Logic Điều Khiển & Core GCS
*Dành cho xử lý đa luồng, state machine, và tính toán động học.*

- **`@python-pro`**
  - **Sử dụng khi:** Code các module trong `core/` và `comm/`. 
  - **Mục đích:** Viết code Python sạch, đảm bảo Thread Safety khi giao tiếp giữa QThread (WifiWorker) và Main Thread (GUI).
- **`@backend-architect`**
  - **Sử dụng khi:** Hệ thống cần mở rộng để quản lý nhiều Drone cùng lúc (Swarm Logic) hoặc cấu trúc lại Event Bus cho dự án.
- **`@math-analyst`** *(nếu hỗ trợ)*
  - **Sử dụng khi:** Tính toán góc Pitch/Roll, ma trận xoay (Rotation Matrix / Quaternions) trong mô phỏng 3D Attitude, hoặc tính bù nhiệt, hiệu chỉnh PID.

## 3. 🎨 Giao Diện Người Dùng & Tích Hợp Web (UI/UX)
*Dành cho Desktop App PySide6 và WebView JS Integrations.*

- **`@ui-skills`** & **`@frontend-design`**
  - **Sử dụng khi:** Chỉnh sửa file trong `ui/`. 
  - **Mục đích:** Xây dựng giao diện bảng điều khiển mang phong cách Dark Theme/Mission Control chuyên nghiệp, tối ưu hóa Animation, và trải nghiệm thao tác người dùng (UX) khi khẩn cấp.
- **`@javascript-pro`**
  - **Sử dụng khi:** Tích hợp, fix lỗi OpenStreetMap/Leaflet trong `mission_tab.py` qua `QWebEngineView`. Giúp xử lý callback JS → Python, chống race condition trên DOM.
- **`@3d-web-experience`** / **`@threejs-skills`**
  - **Sử dụng khi:** Muốn nâng cấp Attitude 3D Widget hiện tại (từ OpenGL/Panda3D) sang một Engine 3D mượt mà hơn nhúng qua WebEngine.

## 4. 🐛 Truy Vết Lỗi & Đảm Bảo An Toàn (Debugging & QA)
*Bắt buộc phải áp dụng trong mọi module ảnh hưởng đến ARM/DISARM.*

- **`@systematic-debugging`**
  - **Sử dụng khi:** Xảy ra lỗi bất thường (VD: Lệnh gửi đi bị delay, bản đồ không hiển thị, hay reconnect thất bại). Agent cần sử dụng phương pháp loại trừ giả thuyết thay vì đoán mò.
- **`@debugger`** / **`@error-detective`**
  - **Sử dụng khi:** Đọc Traceback Python hoặc parse dump memory của ESP32.
- **`@test-driven-development`** (TDD)
  - **Sử dụng khi:** Viết logic mới trong parser (VD: thêm protocol MSP mới). AI cần viết dummy packet hex (test case) trước khi code thuật toán.
- **`@security-auditor`**
  - **Sử dụng khi:** Review rủi ro bị "cướp quyền điều khiển" qua sóng Wifi, rủi ro Injection attack khi gửi dữ liệu Map, Audit độ an toàn mật khẩu.

## 5. 🏛️ Kiến Trúc Tổng Thể & Workflow
*Để quản lý dự án lâu dài.*

- **`@senior-architect`**
  - **Sử dụng khi:** Cân nhắc thay thế thư viện lõi (VD: Chuyển từ PySide6 sang React Native Desktop, hoặc đổi từ MSP sang MAVLink) đánh giá ưu/nhược điểm và trade-off.
- **`@documentation`** / **`@api-documenter`**
  - **Sử dụng khi:** Sinh tài liệu README.md, Docstrings cho các class tĩnh hoặc update AGENT.md.
- **`@git-advanced-workflows`**
  - **Sử dụng khi:** Khi DroneGCS phát triển lớn (có nhiều branch cho nhiều loại frame drone khác nhau), cần xử lý CI/CD deployment phiên bản release (Build `.exe` cho PC).

---

**Mẹo cho người dùng (User):** Khi giao task mới cho hệ thống, hãy kèm theo các tag trên, ví dụ: 
*"Dùng `@ui-skills` và `@javascript-pro`, giúp tôi thêm tính năng vẽ vùng an toàn (Geo-fencing) trên Mission Tab bản đồ Leaflet."*
