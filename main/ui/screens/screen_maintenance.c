/*
 * Maintenance screen: placeholder + Factory reset with password confirm (Phase 5.1).
 */

#include "ui/screens/screen_maintenance.h"

#include <string.h>

#include "application/auth_manager.h"
#include "application/hmi_state.h"
#include "config/ui_theme.h"
#include "middleware/audit/audit_log.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_confirm_dialog.h"
#include "ui/components/ui_shell.h"
#include "ui/ui_manager.h"

#define MAINT_RESET_BTN_Y   80
#define MAINT_RESET_BTN_W   160
#define MAINT_RESET_BTN_H   36
#define MAINT_EXPORT_BTN_Y  124
#define MAINT_EXPORT_BUF_SZ 512

static void maintenance_export_btn_cb(lv_event_t *e)
{
    char buf[MAINT_EXPORT_BUF_SZ];
    size_t len = 0U;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    lv_obj_t *mbox;
    const char *display_text;

    (void)e;
    if (audit_log_export_to_buffer(buf, sizeof(buf), &len) != ESP_OK) {
        display_text = "(export failed)";
    } else if (len == 0U) {
        display_text = "(empty)";
    } else {
        buf[len < sizeof(buf) ? len : sizeof(buf) - 1U] = '\0';
        display_text = buf;
    }
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }
    mbox = lv_msgbox_create(NULL);
    if (mbox != NULL) {
        lv_msgbox_add_title(mbox, i18n_table_get(lang, I18N_KEY_AUDIT_LOG));
        lv_msgbox_add_text(mbox, display_text);
        lv_msgbox_add_close_button(mbox);
    }
}

static void maintenance_reset_confirm_cb(const char *password, void *user_data)
{
    (void)user_data;
    if (auth_manager_verify_password(password) == ESP_OK) {
        (void)audit_log_record(AUDIT_EVENT_CONFIG_CHANGE, "Maintenance reset confirmed");
    }
}

static void maintenance_reset_btn_cb(lv_event_t *e)
{
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    const char *title;
    const char *msg;

    (void)e;
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }
    title = i18n_table_get(lang, I18N_KEY_CONFIRM_PASSWORD_TITLE);
    msg = i18n_table_get(lang, I18N_KEY_CONFIRM_PASSWORD_MSG);
    (void)ui_confirm_dialog_show(NULL, title, msg, maintenance_reset_confirm_cb, NULL);
}

esp_err_t screen_maintenance_create(lv_display_t *display, screen_maintenance_view_t *view)
{
    lv_obj_t *content;
    lv_obj_t *label;
    lv_obj_t *btn;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    const char *title;

    if ((display == NULL) || (view == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }
    title = i18n_table_get(lang, I18N_KEY_MAINTENANCE);

    (void)memset(view, 0, sizeof(*view));
    if (ui_shell_create_with_back(display, UI_SCREEN_MAINTENANCE, title, true, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }
    lv_obj_set_style_pad_all(content, (int32_t)UI_THEME_CONTENT_PADDING, 0);

    label = lv_label_create(content);
    lv_label_set_text(label, "TODO: Hardware test, service counters and safety diagnostics");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_width(label, 200);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 24);

    btn = lv_button_create(content);
    lv_obj_set_size(btn, MAINT_RESET_BTN_W, (lv_coord_t)UI_THEME_BUTTON_MIN_HEIGHT);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, MAINT_RESET_BTN_Y);
    label = lv_label_create(btn);
    lv_label_set_text(label, i18n_table_get(lang, I18N_KEY_FACTORY_RESET));
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, maintenance_reset_btn_cb, LV_EVENT_CLICKED, NULL);

    btn = lv_button_create(content);
    lv_obj_set_size(btn, MAINT_RESET_BTN_W, (lv_coord_t)UI_THEME_BUTTON_MIN_HEIGHT);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, MAINT_EXPORT_BTN_Y);
    label = lv_label_create(btn);
    lv_label_set_text(label, i18n_table_get(lang, I18N_KEY_EXPORT_LOG));
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, maintenance_export_btn_cb, LV_EVENT_CLICKED, NULL);

    view->is_created = true;
    return ESP_OK;
}

void screen_maintenance_destroy(screen_maintenance_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
