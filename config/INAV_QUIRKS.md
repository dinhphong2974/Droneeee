# ⚠️ INAV Firmware Notes & Quirks

Tệp này ra đời sau sự kiện conflict BoxIDs do khác biệt firmware (giữa bản custom của hãng và INAV chuẩn), nhằm ghi lại các đặc tính dị biệt và kinh nghiệm gỡ lỗi INAV trong quá trình làm DroneGCS.

Mỗi khi phân tích CLI INAV, hãy đọc kỹ tệp này để đối chiếu.

## 1. Sự sai lệch Box IDs (Mode IDs)
Với các bản INAV cũ/chuẩn, BoxIDs thường cố định:
- `3`: NAV ALTHOLD
- `8`: NAV RTH
- `9`: NAV POSHOLD
- `18`: FAILSAFE

**TUY NHIÊN**, với các custom fork mới (Ví dụ: `INAV/SPEEDYBEEF405AIO 9.0.1`), hệ thống ID đã bị dịch chuyển đi rất nhiều, ví dụ cụ thể đo được từ GUI:
- **`3`** 👉 NAV ALTHOLD
- **`10`** 👉 NAV RTH (Thay vì 8)
- **`11`** 👉 NAV POSHOLD (Thay vì 9)
- **`27`** 👉 FAILSAFE (Thay vì 18)
- **`28`** 👉 NAV WP (Waypoint)

**QUY TẮC XỬ LÝ:** 
- Hạn chế khuyên người dùng cấu hình Mode qua CLI vì sự rủi ro của BoxIDs khác nhau. Ưu tiên hướng dẫn họ setup bằng giao diện Configurator (GUI).
- Nếu người dùng gửi ảnh GUI và CLI, **hãy tin tưởng hoàn toàn vào giao diện GUI** để map lại danh sách BoxID trong ngữ cảnh hiện tại.

## 2. Lưu ý về Cơ chế Cấu hình `aux` & Chồng chéo
Cú pháp lệnh: `aux <Index> <BoxID> <ChannelID> <Min> <Max>`
- Dán lệnh CLI với cùng một `<Index>` (Slot ô nhớ) sẽ ghi đè (Reset) ô đó thành thiết lập mới.
- Khai báo cùng thiết lập nhưng ở các `<Index>` khác nhau sẽ sinh ra "chồng chéo" (bật song song).
- **CHÚ Ý:** Mã nguồn `flight_controller.py` của DroneGCS chủ đích "chồng chéo" 3 chế độ (NAV ALTHOLD, NAV POSHOLD, NAV WP) vào chung Channel 6 ở mức 2000. Đây là logic đúng để cất cánh và tự hành. Khi đánh giá lỗi, tuyệt đối không được phân tích sự "chồng chéo" này là lỗi.
