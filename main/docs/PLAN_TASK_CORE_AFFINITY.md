# Plan: Gán core cho task (dual-core, MISRA)

Tài liệu mô tả thay đổi gán **core cố định** cho task LVGL và app task để tận dụng 2 core ESP32, tránh task watchdog (IDLE0 không chạy khi LVGL block lâu), **không hard-code** và tuân thủ **MISRA C:2012**.

---

## Mục tiêu

| Mục tiêu | Mô tả |
|----------|--------|
| **Tách core** | LVGL chạy trên một core, app tasks + IDLE0 trên core kia để IDLE0 luôn có thời gian reset task WDT. |
| **Không magic number** | Mọi core ID dùng hằng named trong config (Rule 2.5, 10.1). |
| **Single source of truth** | Core affinity định nghĩa tại `config/system_config.h`. |
| **Dễ bảo trì** | Đổi core chỉ sửa config, không sửa nhiều file. |

---

## Nguyên tắc MISRA áp dụng

| Rule / Nguyên tắc | Áp dụng |
|-------------------|--------|
| **Rule 2.5** (Macro/constant) | Core ID là hằng named (`ESP32TOUCH_TASK_CORE_PRO`, `ESP32TOUCH_TASK_CORE_APP`), không literal 0/1 tại call site. |
| **Rule 10.1** (Implicit conversion) | Cast rõ ràng sang `BaseType_t` khi gọi `xTaskCreatePinnedToCore` (API FreeRTOS dùng BaseType_t). |
| **Magic number** | Toàn bộ số core (0, 1) nằm trong enum tại `system_config.h`. |
| **Single source of truth** | `ESP32TOUCH_LVGL_TASK_CORE_ID`, `ESP32TOUCH_APP_TASK_CORE_ID` lấy từ config. |

---

## Thay đổi cấu hình

### File: `main/config/system_config.h`

- Thêm **enum core ID** (một lần định nghĩa 0/1):
  - `ESP32TOUCH_TASK_CORE_PRO = 0`
  - `ESP32TOUCH_TASK_CORE_APP = 1`
- Thêm hằng gán core cho từng nhóm task:
  - `ESP32TOUCH_LVGL_TASK_CORE_ID = ESP32TOUCH_TASK_CORE_APP`
  - `ESP32TOUCH_APP_TASK_CORE_ID = ESP32TOUCH_TASK_CORE_PRO`

Comment ngắn gọn: LVGL gán APP_CPU để IDLE0 (PRO_CPU) không bị chiếm; app task gán PRO_CPU.

---

## Thay đổi code

### File: `main/bsp/display/display_lvgl.c`

| Nội dung | MISRA |
|----------|--------|
| Đổi `xTaskCreate(...)` thành `xTaskCreatePinnedToCore(..., (BaseType_t)ESP32TOUCH_LVGL_TASK_CORE_ID)`. | Cast rõ ràng (Rule 10.1); core ID từ config. |
| Giữ nguyên stack, priority, name, entry, param. | Không thêm magic number. |

### File: `main/application/app_task.c`

| Nội dung | MISRA |
|----------|--------|
| Đổi mỗi `xTaskCreate(...)` thành `xTaskCreatePinnedToCore(..., (BaseType_t)ESP32TOUCH_APP_TASK_CORE_ID)` cho: system_monitor_task, hardware_poll_task, event_dispatch_task, logging_task. | Cast rõ ràng; core ID từ config. |

---

## Lưu ý

- **LVGL**: Mọi gọi lv_* vẫn từ **một task** (task LVGL); không gọi LVGL từ task khác (LVGL không thread-safe).
- **Task WDT**: ESP-IDF theo dõi idle task; khi LVGL chạy trên APP_CPU, IDLE0 (PRO_CPU) không bị block nên có thể reset WDT.
- **Tương lai**: Nếu đổi chip single-core hoặc muốn bỏ pinning, có thể dùng `tskNO_AFFINITY` (nếu được hỗ trợ) hoặc thêm macro điều kiện trong config.

---

## Checklist khi implement

- [ ] `system_config.h`: enum core + 2 hằng `*_CORE_ID`, có comment.
- [ ] `display_lvgl.c`: `xTaskCreatePinnedToCore` với `(BaseType_t)ESP32TOUCH_LVGL_TASK_CORE_ID`.
- [ ] `app_task.c`: bốn lần `xTaskCreatePinnedToCore` với `(BaseType_t)ESP32TOUCH_APP_TASK_CORE_ID`.
- [ ] Không còn literal `0` hoặc `1` dùng làm core ID tại call site.
- [ ] Build thành công; test dual-core (watchdog không còn do IDLE0 bị starve).
