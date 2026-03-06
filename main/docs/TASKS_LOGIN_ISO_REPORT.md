# Phân tích task – Tái cấu trúc màn Login theo báo cáo ISO

Tài liệu này liệt kê các task cần làm dựa trên **báo cáo phân tích và tái cấu trúc tọa độ màn hình Login theo tiêu chuẩn ISO** (ISO 9241-110/11/410/412, IEC 62624).

**Lưu ý:** Hiện tại BSP dùng resolution **240×320** (`board_config.h`). Báo cáo giả định 1024×768; mọi tọa độ/kích thước phải đưa vào **hằng số trong `ui_theme.h`** và có thể điều chỉnh theo resolution (hoặc scale) khi đổi màn hình.

---

## 1. Tổng quan theo nhóm

| Nhóm | Mô tả | Số task | Ưu tiên |
|------|--------|---------|---------|
| A | Cập nhật hằng số theme (margin, spacing, kích thước) | 1 | Cao |
| B | Bố trí lại tọa độ (Y/X) từ theme | 1 | Cao |
| C | Nút ngôn ngữ – tăng khoảng cách | 1 | Cao |
| D | Keyboard giảm 45% → 40%, không che Login | 1 | Cao |
| E | Status label multiline (tối đa 3 dòng) | 1 | Trung bình |
| F | Nút Shutdown (góc dưới trái) | 1 | Trung bình |
| G | Icon trạng thái HW (FPGA/STM32) góc trên phải | 1 | Trung bình |
| H | i18n / audit / tích hợp luồng | 2 | Thấp (đã có sẵn phần lớn) |

---

## 2. Chi tiết từng task

### A. Cập nhật hằng số trong `ui_theme.h`

**File:** `main/config/ui_theme.h`

**Việc cần làm:**

| Hằng số hiện tại | Giá trị đề xuất (báo cáo) | Ghi chú |
|------------------|---------------------------|---------|
| `UI_THEME_LOGIN_MARGIN_X` | 16 → **20** | ISO 9241-410: margin thoải mái |
| Khoảng cách giữa nút ngôn ngữ | 4 → **8–10** (ví dụ 10) | Thêm `UI_THEME_LOGIN_LANG_BTN_GAP` |
| `UI_THEME_LOGIN_INPUT_WIDTH` | 208 → **280** (hoặc giữ 208 nếu màn 240px) | Với 240×320 có thể giữ ~208 hoặc `lv_pct(85)` |
| `UI_THEME_LOGIN_INPUT_HEIGHT` | 34 → **40** | Nút chạm tối thiểu |
| `UI_THEME_LOGIN_BUTTON_HEIGHT` | 36 → **48** | Nút Login nổi bật hơn |
| Keyboard | `lv_pct(45)` cố định → **40%** hoặc pixel cố định | Thêm `UI_THEME_LOGIN_KEYBOARD_HEIGHT_PCT` = 40 hoặc `UI_THEME_LOGIN_KEYBOARD_HEIGHT_PX` |

**Hằng số mới cần thêm (nếu áp dụng đủ báo cáo):**

- `UI_THEME_LOGIN_LANG_BTN_GAP` = 10
- `UI_THEME_LOGIN_SPACING_V` = 12 (khoảng cách dọc giữa nhóm)
- `UI_THEME_LOGIN_KEYBOARD_HEIGHT_PCT` = 40 (hoặc dùng pixel từ content height)
- `UI_THEME_LOGIN_SHUTDOWN_BTN_WIDTH` = 80
- `UI_THEME_LOGIN_SHUTDOWN_BTN_HEIGHT` = 40
- `UI_THEME_LOGIN_SHUTDOWN_OFFSET_BOTTOM` = 100
- `UI_THEME_LOGIN_HW_STATUS_ICON_SIZE` = 40
- `UI_THEME_LOGIN_HW_STATUS_OFFSET_RIGHT` = 60
- `UI_THEME_LOGIN_STATUS_MAX_LINES` = 3 (hoặc chiều cao max cho label)

**Ràng buộc:** Toàn bộ tọa độ Y (title, row1, row2, button, status) nên tính từ các hằng trên (không magic number), có thể dựa trên `UI_THEME_LOGIN_LANG_ROW_H` và `UI_THEME_LOGIN_SPACING_V` như hiện tại.

---

### B. Bố trí lại tọa độ (Y) – nút Login “gần keyboard”

**File:** `main/config/ui_theme.h` (công thức Y), `main/ui/screens/screen_login.c` (dùng đúng hằng)

**Việc cần làm:**

- Đặt **nút Login** sao cho nằm **ngay phía trên vùng keyboard** (không bị keyboard che), ví dụ:  
  `LOGIN_BUTTON_Y = content_height - keyboard_height - button_height - margin_bottom`
- Hoặc dùng công thức cố định từ trên xuống (như hiện tại) nhưng đảm bảo khoảng trống giữa status và keyboard ≥ chiều cao nút Login + margin.
- Cập nhật tất cả `UI_THEME_LOGIN_*_Y` để thống nhất với A và với content height (có thể dùng `UI_THEME_CONTENT_HEIGHT` nếu cần).

**Tham chiếu:** Báo cáo đề xuất ví dụ Y = 330 cho nút Login (với content 768px); với 240×320 cần scale hoặc công thức từ content height.

---

### C. Nút ngôn ngữ – tăng khoảng cách (ISO 9241-410)

**File:** `main/ui/screens/screen_login.c`

**Hiện tại:**  
`btn_x += (int32_t)(UI_THEME_LOGIN_LANG_BTN_WIDTH + 4);`  → 4px gap.

**Cần làm:**

- Dùng hằng mới `UI_THEME_LOGIN_LANG_BTN_GAP` (ví dụ 10) thay cho 4.
- Đảm bảo `screen_login.c` chỉ dùng hằng từ theme, không hardcode 4.

---

### D. Keyboard 40% và không che nút Login

**File:** `main/ui/screens/screen_login.c`

**Hiện tại:**  
`lv_obj_set_size(view->keyboard, lv_pct(100), lv_pct(45));`

**Cần làm:**

- Đổi 45 → 40 (hoặc dùng `UI_THEME_LOGIN_KEYBOARD_HEIGHT_PCT`).
- Đảm bảo thứ tự tạo/layout: form + nút Login + status ở trên, keyboard căn BOTTOM_MID; với 40% chiều cao, vùng “trên keyboard” vẫn đủ cho Login + status.

---

### E. Status label multiline (tối đa 3 dòng)

**File:** `main/ui/screens/screen_login.c`

**Hiện tại:**  
`view->status_label` là label 1 dòng, có thể tràn khi lockout/error dài.

**Cần làm:**

- `lv_label_set_long_mode(view->status_label, LV_LABEL_LONG_WRAP)`.
- Set width tối đa (ví dụ bằng `UI_THEME_LOGIN_INPUT_WIDTH` hoặc hằng riêng).
- Có thể set max height tương đương 3 dòng (từ theme, ví dụ `UI_THEME_LOGIN_STATUS_MAX_HEIGHT`).

---

### F. Nút Shutdown (góc dưới trái, xa nút Login)

**File:** `main/config/ui_theme.h`, `main/ui/screens/screen_login.c`, `main/middleware/i18n/i18n_table.h` + `.c`

**Cần làm:**

1. **Theme:** Thêm hằng vị trí và kích thước (ví dụ `UI_THEME_LOGIN_SHUTDOWN_*` như trên).
2. **Login screen:** Tạo nút Shutdown, align `LV_ALIGN_BOTTOM_LEFT` với offset (X = margin, Y = -offset_bottom - height). Gắn event (ví dụ gọi shutdown/standby hoặc quay về màn an toàn).
3. **i18n:** Thêm key (ví dụ `I18N_KEY_SHUTDOWN` / "Shutdown" / "Tắt máy") và chuỗi 4 ngôn ngữ.
4. **Logic:** Quyết định hành vi (tắt máy thật, hay chỉ logout + về Login, hay màn “Confirm shutdown”) và gọi API/event tương ứng; có thể ghi audit nếu cần.

---

### G. Icon trạng thái kết nối HW (FPGA/STM32)

**File:** `main/ui/screens/screen_login.c`, `main/application/hmi_state.h` (hoặc nơi lưu comm status)

**Cần làm:**

1. **Theme:** Thêm hằng kích thước và vị trí (ví dụ `UI_THEME_LOGIN_HW_STATUS_ICON_SIZE`, offset góc trên phải).
2. **Login screen:** Tạo object (label/icon) ở góc trên phải content.
3. **Nguồn dữ liệu:** Lấy trạng thái FPGA/STM32 từ `hmi_state` (hoặc nơi đang cung cấp comm status).
4. **Logic hiển thị:** Ẩn khi “all OK”, hiện (và có thể đổi màu/icon) khi mất kết nối; cập nhật khi có event comm (có thể từ event bus hoặc khi mở màn Login).

---

### H. i18n / audit / luồng (đã có sẵn phần lớn)

**Đã có:**

- Session timeout, lockout, audit login/logout đã implement.
- Luồng Boot → Splash → Login → Home đã có.

**Có thể bổ sung (tùy chọn):**

- i18n cho mọi chuỗi mới (Shutdown, tooltip icon HW nếu có).
- Audit event khi user bấm Shutdown (nếu có hành vi shutdown thật).

---

## 3. Thứ tự thực hiện đề xuất

1. **A + B + C + D** – Theme và layout cơ bản (margin, spacing, form, keyboard, vị trí Login).
2. **E** – Status multiline.
3. **F** – Nút Shutdown (theme + UI + i18n + hành vi).
4. **G** – Icon trạng thái HW.

Sau mỗi bước nên build và kiểm tra trên màn 240×320 (và chỉnh lại hằng nếu cần).

---

## 4. Checklist trước khi đóng task

- [ ] Mọi số (px, %) đều từ `ui_theme.h`, không magic number trong `screen_login.c`.
- [ ] Khoảng cách nút ngôn ngữ ≥ 8px (hằng trong theme).
- [ ] Keyboard ≤ 40% content height, không che nút Login.
- [ ] Nút Login dễ với tay (phía trên keyboard), kích thước ≥ 40px chiều cao (theme).
- [ ] Status label wrap, tối đa 3 dòng (hoặc max height từ theme).
- [ ] Nút Shutdown (nếu có) ở góc dưới trái, xa nút Login, có i18n.
- [ ] Icon trạng thái HW (nếu có) góc trên phải, ẩn khi OK.
- [ ] Build thành công, không regression màn khác.

---

## 5. File cần sửa (tóm tắt)

| File | Task |
|------|------|
| `main/config/ui_theme.h` | A, B (hằng mới và công thức Y), F, G (hằng vị trí/kích thước) |
| `main/ui/screens/screen_login.c` | B, C, D, E, F (tạo nút + event), G (tạo icon + cập nhật) |
| `main/middleware/i18n/i18n_table.h` | F (key Shutdown; G nếu có tooltip) |
| `main/middleware/i18n/i18n_table.c` | F (chuỗi 4 ngôn ngữ) |
| (Tùy chọn) `main/application/` hoặc BSP | Hành vi Shutdown, nguồn comm status cho G |

---

*Tài liệu: TASKS_LOGIN_ISO_REPORT.md – Phân tích task từ báo cáo tái cấu trúc Login theo ISO.*
