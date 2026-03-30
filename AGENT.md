# Kiến Trúc Dự Án DroneGCS

## Sơ Đồ Module

```
main.py                     ← Khởi tạo + Điều phối Signal/Slot
├── comm/
│   ├── wifi_client.py      ← Kết nối TCP thô (socket)
│   ├── wifi_worker.py      ← QThread chạy ngầm (dùng wifi_client + msp_parser)
│   └── msp_parser.py       ← Đóng gói/giải mã giao thức MSP
├── core/
│   └── drone_state.py      ← Trạng thái drone (dữ liệu chia sẻ)
└── ui/
    ├── gcs_dashboard.py    ← UI chính (auto-generated từ Qt Designer)
    ├── config_tab.py       ← Tab cấu hình
    └── dashboard_tab.py    ← Tab dashboard
```

## Luồng Dữ Liệu

```
[FC] ←TCP→ [ESP32] ←TCP→ [WifiClient] → [WifiWorker/QThread] → Signal → [main.py/UI]
                          (socket thô)    (vòng lặp + MSPParser)          (hiển thị)
```

## Quy Tắc

1. **main.py** chỉ khởi tạo và kết nối Signal/Slot
2. **wifi_client.py** chỉ xử lý kết nối TCP thô (connect/send/receive/close)
3. **wifi_worker.py** chỉ chạy vòng lặp ngầm trên QThread, dùng wifi_client và msp_parser
4. **msp_parser.py** chỉ đóng gói/giải mã giao thức MSP, không biết về socket hay thread

## Thông Số Phần Cứng

- **Pin**: Lipo 6S (19.8V rỗng → 25.2V đầy)
- **Động cơ**: 1960kv (PWM range: 1000-2000μs)