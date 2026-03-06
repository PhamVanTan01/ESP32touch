# Kế hoạch nâng cấp luồng UI theo ISO và MISRA

Tài liệu này mô tả plan sửa/bổ sung toàn bộ các phần đã đánh giá (ISO 9241, 11064, 20282, 25119, 13849/IEC 62624), tuân thủ nguyên tắc **MISRA C:2012** (không magic number, kiểu rõ ràng, cấu hình tách biệt, single responsibility, defensive coding).

---

## Nguyên tắc MISRA áp dụng trong plan

| Nguyên tắc | Áp dụng |
|------------|--------|
| **Rule 2.1** (Dead code) | Không để code không dùng; mỗi hàm có một nhiệm vụ rõ ràng. |
| **Rule 10.x** (Type) | Dùng `uint32_t`/`int32_t` cho số, cast rõ ràng; tránh implicit conversion. |
| **Rule 14.x** (Control flow) | Mỗi hàm một điểm thoát khi có thể; `break`/`return` rõ ràng. |
| **Magic number** | Mọi hằng số đặt trong config/header (ví dụ `auth_demo_config.h`, `ui_theme.h`, schema). |
| **Single source of truth** | Timeout, số lần sai, màu, kích thước: lấy từ schema hoặc config, không hardcode trong UI. |
| **Defensive** | Kiểm tra `NULL`, kiểm tra index/bounds trước khi dùng. |
| **Tách biệt chức năng** | Session/timeout trong module riêng; alarm trong module alarm; audit chỉ ghi log. |

---

## Phase 1: Audit và session (ISO 25119, nền tảng)

### 1.1 Audit login/logout

**Mục tiêu:** Mọi đăng nhập/đăng xuất đều ghi audit log.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `middleware/audit/audit_log.h` | Thêm `AUDIT_EVENT_LOGOUT` vào enum. | Enum có giá trị rõ ràng. |
| `application/auth_manager.c` | Trong `auth_manager_login_password` (khi thành công): gọi `audit_log_record(AUDIT_EVENT_LOGIN, msg)` với message chứa user/role (format cố định, độ dài ≤ 63 ký tự). | Message build trong buffer cục bộ, kiểm tra length. |
| `application/auth_manager.c` | Trong `auth_manager_logout`: gọi `audit_log_record(AUDIT_EVENT_LOGOUT, msg)`. | Tương tự. |
| `ui/screens/screen_login.c` | Không thay đổi logic; audit do auth_manager đảm nhiệm. | Tách biệt: UI không gọi audit trực tiếp cho login/logout. |

**Config:** Không thêm; dùng `AUDIT_EVENT_*` hiện có.

---

### 1.2 Session timeout (tự động logout)

**Mục tiêu:** Sau `session_timeout_minutes` (theo role) không có tương tác → logout và chuyển về Login.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `config/auth_demo_config.h` hoặc schema | Đã có `session_timeout_minutes` theo role; dùng từ `hmi_schema` (role_definitions[].session_timeout_minutes). | Không thêm magic number. |
| `application/session_manager.h` (mới) | API: `session_manager_init(config)`, `session_manager_deinit()`, `session_manager_touch()` (gọi khi có input/touch), `session_manager_tick()` (gọi định kỳ 1s hoặc 60s). Nội bộ: lưu `last_activity_tick`, `timeout_minutes` lấy từ role hiện tại. | Kiểu: `uint32_t` tick, so sánh với timeout lấy từ config. |
| `application/session_manager.c` (mới) | Trong `session_manager_tick()`: nếu `(now - last_activity) >= timeout_minutes * 60` → gọi callback hoặc set flag. Callback do app_task hoặc ui_manager đăng ký: `auth_manager_logout()` + `ui_manager_show(UI_SCREEN_LOGIN)` + `audit_log_record(LOGOUT, "Session timeout")`. | Một điểm quyết định timeout; không magic number (60 → hằng `SECONDS_PER_MINUTE` trong config). |
| `application/app_task.c` | Định kỳ gọi `session_manager_tick()`. Khi có event touch/input (nếu có event từ LVGL/input): gọi `session_manager_touch()`. | Tách: app_task chỉ gọi API, không chứa logic timeout. |
| `ui_manager.c` hoặc BSP touch | Khi có touch/click hợp lệ: gọi `session_manager_touch()`. Có thể qua event_bus hoặc callback từ display_lvgl. | Cần một điểm duy nhất “user activity” để tránh trùng. |

**Config:** Hằng `SECONDS_PER_MINUTE = 60U` trong `config/system_config.h` hoặc `session_manager.c` (static const).

---

### 1.3 Login lockout (sau N lần sai)

**Mục tiêu:** Sau ≥ N lần đăng nhập sai → khóa T phút; hiển thị thông báo “Khóa đến HH:MM”.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `config/auth_demo_config.h` | Thêm: `AUTH_MAX_FAILED_ATTEMPTS = 5U`, `AUTH_LOCKOUT_DURATION_MINUTES = 5U`. | Hằng named. |
| `application/auth_manager.c` | Thêm state: `failed_attempts` (uint32_t), `lockout_until_tick` (uint32_t, 0 = không khóa). Trong `auth_manager_login_password`: nếu đang khóa (now < lockout_until_tick) → return ESP_ERR_INVALID_STATE; nếu sai user/pass: `failed_attempts++`, nếu `failed_attempts >= AUTH_MAX_FAILED_ATTEMPTS` thì set `lockout_until_tick = now + AUTH_LOCKOUT_DURATION_MINUTES*60`; nếu đúng: `failed_attempts = 0`. | So sánh với hằng config; kiểu uint32_t. |
| `application/auth_manager.h` | Thêm: `bool auth_manager_is_locked_out(uint32_t now_tick_seconds, uint32_t *unlock_at_tick_seconds)`. Trả về true nếu đang khóa; unlock_at để UI hiển thị. | Output parameter rõ ràng. |
| `ui/screens/screen_login.c` | Trước khi xử lý nút Login: gọi `auth_manager_is_locked_out()`, nếu true thì hiển thị i18n “Khóa đến HH:MM” (format từ unlock_at), không gọi login. | UI chỉ hiển thị; logic khóa trong auth. |
| `middleware/i18n/i18n_table.h` + `.c` | Thêm key: `I18N_KEY_LOGIN_LOCKED_UNTIL` (chuỗi có placeholder hoặc “Khóa đến %s”). | i18n cho mọi chuỗi user. |

---

## Phase 2: Đa ngôn ngữ và Help (ISO 9241-110, 20282)

### 2.1 Chọn ngôn ngữ trên UI

**Mục tiêu:** Trên Login và/hoặc status bar có thể chọn ngôn ngữ (VI, EN, ES, ZH).

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `config/hmi_schema.h` | Đã có `language_support.available_languages`, `available_language_count`. Dùng làm nguồn. | Không magic. |
| `ui/screens/screen_login.c` | Trên màn Login: thêm dropdown hoặc danh sách nút (4 ngôn ngữ). Label lấy từ i18n (có thể key mới `I18N_KEY_LANG_VI` … hoặc dùng “Tiếng Việt”/“English” trong bảng). Khi chọn: `hmi_state_set_language(lang)`; có thể tái tạo label/placeholder bằng i18n. | Vòng lặp theo `available_language_count`; không hardcode 4. |
| `ui/components/ui_shell.c` | Trên status bar: nút nhỏ (icon/flag) “Ngôn ngữ”. Bấm → mở panel nhỏ liệt kê ngôn ngữ; chọn → `hmi_state_set_language()`; đóng panel. | Cùng nguồn schema. |
| `middleware/i18n/i18n_table.h` | Thêm key cho tên ngôn ngữ (ví dụ `I18N_KEY_LANG_VI`, `I18N_KEY_LANG_EN`, …) hoặc dùng một key “Language” và tên cố định từ schema. | Chuỗi qua i18n. |

**Config:** Số ngôn ngữ và danh sách từ `hmi_loader_get_config()->language_support`.

---

### 2.2 Help / tooltip theo ngữ cảnh

**Mục tiêu:** Mỗi màn có nút “?” mở mô tả ngắn; có thể thêm tooltip cho nút.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `config/ui_theme.h` | Thêm: `UI_THEME_HELP_BUTTON_SIZE`, vị trí (góc phải title bar). | Hằng layout. |
| `middleware/i18n/i18n_table.h` | Thêm key: `I18N_KEY_HELP_DASHBOARD`, `I18N_KEY_HELP_RECIPE`, … (một key per screen). Hoặc key tổng `I18N_KEY_HELP` + tham số screen. Đơn giản: một key per screen. | Bảng chuỗi. |
| `ui/components/ui_shell.c` | Trong `ui_shell_create`: tạo nút “?” (label từ i18n “Help”), đặt góc phải title bar. Event: mở popup (lv_msgbox hoặc panel) với nội dung là chuỗi help theo `active_screen`. Cần map `ui_screen_id_t` → `i18n_key_t` help (bảng static trong .c). | Bảng ánh xạ screen_id → help_key; không switch chuỗi. |
| `ui/screens/screen_*.c` | Không bắt buộc: nếu màn có nút đặc thù, có thể set tooltip (LVGL long-press hoặc thuộc tính tooltip nếu có). | Tùy chọn. |

**Config:** Map screen → help key trong một file (ví dụ `ui/ui_help_map.c` + header), dùng enum cho key.

---

## Phase 3: Cảnh báo và E-Stop (ISO 11064, IEC 62624, 13849)

### 3.1 Cảnh báo theo ISA-18.2 (severity, banner, popup)

**Mục tiêu:** Phân cấp cảnh báo (critical/warning/info); banner trên status bar; critical → popup + âm thanh; màn Alarms có danh sách + acknowledge.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `config/hmi_schema.h` hoặc `application/alarm_config.h` (mới) | Định nghĩa enum `alarm_severity_t` (CRITICAL, WARNING, INFO). Hằng màu từ theme: `UI_THEME_ALARM_CRITICAL_COLOR`, `UI_THEME_ALARM_WARNING_COLOR`, `UI_THEME_ALARM_INFO_COLOR`. | Enum và màu trong config. |
| `application/alarm_manager.h` (mới) | API: `alarm_manager_init()`, `alarm_manager_add(id, severity, text_id)`, `alarm_manager_ack(id)`, `alarm_manager_get_active_count(severity)`, `alarm_manager_get_list()`. Lưu danh sách alarm đang active (số lượng giới hạn, ví dụ 32). | Cấu trúc dữ liệu có kích thước tối đa (MISRA 21.x). |
| `application/alarm_manager.c` (mới) | Logic thêm/xóa/ack; không gọi UI. Event bus: publish `APP_EVENT_ALARM_UPDATED` khi thay đổi. | Tách biệt logic và UI. |
| `ui/components/ui_shell.c` | Phần status bar: nếu `alarm_manager_get_active_count(CRITICAL) > 0` → vẽ banner nhỏ hoặc đổi màu status bar (theme critical); tương tự warning/info. Chuỗi “X alarm(s)” từ i18n. | Màu và chuỗi từ config/i18n. |
| `ui/ui_manager.c` hoặc layer riêng | Khi có alarm critical mới: hiện popup (không che nút E-Stop), nội dung từ alarm; có nút “Ack” gọi `alarm_manager_ack()`. Có thể tích hợp trong app_task khi nhận `APP_EVENT_ALARM_UPDATED`. | Popup tạo một lần, cập nhật nội dung. |
| `ui/screens/screen_alarms.c` | Thay placeholder bằng danh sách thật: gọi `alarm_manager_get_list()`, hiển thị từng dòng (id, severity, text, nút Ack). Dùng i18n cho label severity và nút. | Vòng lặp có giới hạn. |
| `middleware/i18n/i18n_table.h` | Thêm key: `I18N_KEY_ALARM_CRITICAL`, `I18N_KEY_ALARM_WARNING`, `I18N_KEY_ALARM_INFO`, `I18N_KEY_ACK`. | Chuỗi qua i18n. |

**Config:** Số alarm tối đa, màu severity trong theme; danh sách alarm text theo id trong bảng (i18n hoặc config).

---

### 3.2 E-Stop (ảo + tích hợp)

**Mục tiêu:** Nút E-Stop luôn hiện, ưu tiên cao (không bị popup che); bấm → xác nhận → lệnh dừng + audit.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `config/ui_theme.h` | Thêm: `UI_THEME_ESTOP_BUTTON_WIDTH`, `UI_THEME_ESTOP_BUTTON_HEIGHT`, `UI_THEME_COLOR_ESTOP` (đỏ). Vị trí: ví dụ góc phải trên, hoặc cố định dưới nav. | Hằng layout/màu. |
| `application/estop_manager.h` (mới) | API: `estop_manager_init()`, `estop_manager_trigger()` (gọi từ UI sau xác nhận), `estop_manager_reset()`. Trạng thái: ESTOP_ACTIVE/INACTIVE. Khi trigger: gửi command (event_bus hoặc protocol), `audit_log_record(AUDIT_EVENT_ESTOP, "E-Stop activated")`. | Một nguồn sự thật cho trạng thái. |
| `middleware/audit/audit_log.h` | Thêm `AUDIT_EVENT_ESTOP` (và nếu cần `AUDIT_EVENT_ESTOP_RESET`). | Enum mở rộng. |
| `ui/components/ui_shell.c` hoặc `ui_nav_bar.c` | Thêm nút E-Stop (màu đỏ, label từ i18n “E-Stop”). Đặt trong layer luôn hiển thị (ví dụ cùng shell root, z-order cao). Click → hiện msgbox xác nhận “Xác nhận dừng khẩn cấp?”; OK → `estop_manager_trigger()`, đóng popup, chuyển màn “an toàn” (ví dụ Home hoặc màn trống) nếu cần. | Không che nút E-Stop bởi popup (popup nằm dưới hoặc E-Stop luôn trên cùng). |
| `middleware/i18n/i18n_table.h` | Thêm: `I18N_KEY_ESTOP`, `I18N_KEY_ESTOP_CONFIRM`. | i18n. |

**Config:** Màu và kích thước từ theme; chuỗi từ i18n.

---

## Phase 4: Splash và trải nghiệm khởi động

### 4.1 Màn hình Splash

**Mục tiêu:** Sau khi BSP/LVGL sẵn sàng, hiển thị splash (logo, “Đang khởi động…”); khi init xong chuyển sang Login.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `ui/ui_manager.h` | Thêm `UI_SCREEN_SPLASH` vào enum. | Enum. |
| `ui/screens/screen_splash.h` + `.c` (mới) | `screen_splash_create()`, `screen_splash_destroy()`. Nội dung: logo (nếu có), label “Đang khởi động…” (i18n). Không nav bar. | Layout từ theme. |
| `ui/ui_manager.c` | Map `UI_SCREEN_SPLASH`: create/destroy, permission (luôn cho phép), `screen_to_name` “splash”. | Tách case. |
| `main/app_main.c` | Thay vì `ui_manager_show(UI_SCREEN_LOGIN)` ngay: gọi `ui_manager_show(UI_SCREEN_SPLASH)`. Sau khi init xong (sau `app_task_start()` hoặc sau một delay/timer), gọi `ui_manager_show(UI_SCREEN_LOGIN)`. Có thể dùng timer LVGL hoặc task delay. | Một luồng khởi động rõ ràng. |
| `config/ui_theme.h` | Thêm hằng cho splash (ví dụ thời gian hiển thị tối thiểu `UI_THEME_SPLASH_DISPLAY_MS`). | Không magic number. |

---

## Phase 5: Xác nhận thao tác nguy hiểm và nhất quán

### 5.1 Xác nhận mật khẩu cho thao tác nguy hiểm

**Mục tiêu:** Trước khi thực hiện thao tác nguy hiểm (xóa recipe, reset, calibrate,…): popup yêu cầu nhập lại mật khẩu (hoặc mật khẩu role cao hơn).

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `application/auth_manager.h` | Thêm: `esp_err_t auth_manager_verify_password(const char *password)`. So sánh với user hiện tại (trong demo: so sánh với AUTH_DEMO_PASSWORD). Trả về ESP_OK nếu đúng. | Chỉ xác thực; không đổi state. |
| `ui/components/ui_confirm_dialog.c` (mới) | Component: dialog có ô nhập mật khẩu + OK/Cancel. Gọi callback khi OK với chuỗi password (hoặc chỉ gọi khi verify thành công). | Tái sử dụng; không hardcode chuỗi. |
| Các màn có thao tác nguy hiểm (Recipe xóa, Maintenance reset, …) | Trước khi gọi logic nguy hiểm: mở confirm dialog; khi user nhập và OK → `auth_manager_verify_password()`; nếu OK thì thực hiện thao tác + `audit_log_record(...)`. | Logic xác nhận tách trong dialog. |
| `middleware/audit/audit_log.h` | Đảm bảo có event cho từng loại thao tác (RECIPE_CHANGE, CALIBRATION, …). | Đã có. |

---

### 5.2 Chuẩn hóa layout (nhất quán)

**Mục tiêu:** Mọi màn con dùng chung shell (status + title + content + nav); chỉ khác nội dung content; theme thống nhất.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `ui/screens/screen_recipe.c`, `screen_calibration.c`, … | Đã dùng `screen_placeholder_create` với shell; kiểm tra tất cả dùng chung `ui_shell_create`, không tạo layout riêng lệch (padding, font size từ theme). | Màu/kích thước từ `ui_theme.h`. |
| `config/ui_theme.h` | Bổ sung hằng chung cho padding content, font size (nếu chưa có). | Một nguồn. |
| `ui/components/ui_shell.c` | Đảm bảo status bar luôn hiển thị role và comm; vị trí nav cố định. | Không magic number. |

---

## Phase 6: Bổ sung audit và config

### 6.1 Audit đầy đủ

**Mục tiêu:** Ghi audit khi đổi recipe, calibrate, config, alarm ack, E-Stop.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `middleware/audit/audit_log.h` | Thêm `AUDIT_EVENT_LOGOUT`, `AUDIT_EVENT_ESTOP` (và RESET nếu cần). | Enum. |
| `application/auth_manager.c` | Như Phase 1.1. | |
| Các màn Recipe/Calibration/Maintenance | Khi thực hiện thao tác thay đổi: gọi `audit_log_record(AUDIT_EVENT_RECIPE_CHANGE, "Recipe X selected")` (ví dụ). Message format cố định, độ dài giới hạn. | Buffer cục bộ, snprintf với sizeof. |
| `application/alarm_manager.c` | Khi ack: `audit_log_record(AUDIT_EVENT_ALARM_ACK, "Alarm id=X")`. | Tương tự. |

---

### 6.2 Export audit log

**Mục tiêu:** Xuất log ra SD hoặc lưu flash; hỗ trợ đọc/export.

**Thay đổi:**

| File | Nội dung | MISRA |
|------|----------|--------|
| `middleware/audit/audit_log.c` | Thêm `audit_log_export_to_buffer()` hoặc `audit_log_iterate()` để đọc từng entry. Format text hoặc CSV; kích thước buffer giới hạn. | Không cấp phát động; giới hạn kích thước. |
| Màn Maintenance hoặc màn riêng “Logs” | Nút “Export” mở dialog chọn đích (nếu có SD) hoặc hiển thị nội dung; gọi audit export. | UI gọi API. |

---

## Thứ tự thực hiện đề xuất

| Bước | Phase | Lý do |
|------|--------|--------|
| 1 | 1.1 Audit login/logout | Nhanh, không phụ thuộc; đáp ứng ISO 25119. |
| 2 | 1.2 Session timeout | Dùng schema có sẵn; nền tảng bảo mật. |
| 3 | 1.3 Login lockout | Tăng bảo mật; config trong auth_demo_config. |
| 4 | 2.1 Chọn ngôn ngữ UI | Đáp ứng ISO 9241-110; dùng schema. |
| 5 | 3.2 E-Stop | An toàn ưu tiên; API nhỏ. |
| 6 | 3.1 Cảnh báo ISA-18.2 | Nhiều file; cần alarm_manager. |
| 7 | 2.2 Help | Dùng map screen → help key. |
| 8 | 4.1 Splash | Trải nghiệm khởi động. |
| 9 | 5.1 Xác nhận mật khẩu | Dùng auth_manager_verify_password. |
| 10 | 5.2 Nhất quán layout + 6.1–6.2 | Hoàn thiện audit và giao diện. |

---

## Checklist MISRA khi implement từng task

- [ ] Không magic number: mọi hằng trong config/theme/header.
- [ ] Kiểu rõ ràng: `uint32_t`/`int32_t` cho số; cast khi cần.
- [ ] Kiểm tra NULL và bounds trước khi dùng.
- [ ] Mỗi module một trách nhiệm (auth, session, alarm, audit tách nhau).
- [ ] Chuỗi user qua i18n; message audit có độ dài giới hạn.
- [ ] Enum có giá trị xác định; không default không rõ.
- [ ] Vòng lặp có giới hạn (số phần tử từ config hoặc hằng).

---

*Tài liệu: PLAN_UI_ISO_MISRA.md – Kế hoạch nâng cấp UI theo ISO và MISRA.*
