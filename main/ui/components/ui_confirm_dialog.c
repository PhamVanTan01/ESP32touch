/*
 * Password confirmation dialog for dangerous actions (Phase 5.1).
 */

#include "ui/components/ui_confirm_dialog.h"

#include <string.h>

#include "application/hmi_state.h"
#include "config/ui_theme.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_shell.h"

typedef struct
{
    ui_confirm_password_cb_t on_confirm;
    void *user_data;
    lv_obj_t *textarea;
} confirm_dialog_ud_t;

static void confirm_ok_cb(lv_event_t *e)
{
    lv_obj_t *dialog = (lv_obj_t *)lv_event_get_user_data(e);
    confirm_dialog_ud_t *ud;
    const char *password;

    if (dialog == NULL) {
        return;
    }
    ud = (confirm_dialog_ud_t *)lv_obj_get_user_data(dialog);
    if (ud == NULL) {
        lv_obj_delete(dialog);
        return;
    }
    password = (ud->textarea != NULL) ? lv_textarea_get_text(ud->textarea) : NULL;
    if (password == NULL) {
        password = "";
    }
    if (ud->on_confirm != NULL) {
        ud->on_confirm(password, ud->user_data);
    }
    lv_obj_delete(dialog);
}

static void confirm_cancel_cb(lv_event_t *e)
{
    lv_obj_t *dialog = (lv_obj_t *)lv_event_get_user_data(e);
    if (dialog != NULL) {
        lv_obj_delete(dialog);
    }
}

esp_err_t ui_confirm_dialog_show(lv_obj_t *parent,
                                 const char *title,
                                 const char *message,
                                 ui_confirm_password_cb_t on_confirm,
                                 void *user_data)
{
    lv_obj_t *dialog;
    lv_obj_t *label;
    lv_obj_t *ta;
    lv_obj_t *ok_btn;
    lv_obj_t *cancel_btn;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    static confirm_dialog_ud_t s_ud;

    if (title == NULL || message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }

    dialog = lv_obj_create(parent != NULL ? parent : lv_screen_active());
    if (dialog == NULL) {
        return ESP_ERR_NO_MEM;
    }
    lv_obj_set_size(dialog, UI_THEME_DIALOG_WIDTH, UI_THEME_DIALOG_HEIGHT);
    lv_obj_align(dialog, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(dialog, lv_color_hex(UI_THEME_COLOR_NAV_MORE_PANEL), 0);
    lv_obj_set_style_pad_all(dialog, 12, 0);
    lv_obj_clear_flag(dialog, LV_OBJ_FLAG_SCROLLABLE);

    s_ud.on_confirm = on_confirm;
    s_ud.user_data = user_data;
    s_ud.textarea = NULL;
    lv_obj_set_user_data(dialog, &s_ud);

    label = lv_label_create(dialog);
    lv_label_set_text(label, title);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 8, 4);

    label = lv_label_create(dialog);
    lv_label_set_text(label, message);
    lv_obj_set_width(label, (lv_coord_t)UI_THEME_DIALOG_MESSAGE_WIDTH);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 8, 28);

    ta = lv_textarea_create(dialog);
    s_ud.textarea = ta;
    lv_obj_set_size(ta, (lv_coord_t)UI_THEME_DIALOG_INPUT_WIDTH, (lv_coord_t)UI_THEME_DIALOG_INPUT_HEIGHT);
    lv_obj_align(ta, LV_ALIGN_TOP_LEFT, 10, 72);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_password_mode(ta, true);
    lv_textarea_set_placeholder_text(ta, "");

    ok_btn = lv_button_create(dialog);
    lv_obj_set_size(ok_btn, (lv_coord_t)UI_THEME_DIALOG_BTN_WIDTH, (lv_coord_t)UI_THEME_DIALOG_BTN_HEIGHT);
    lv_obj_align(ok_btn, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    label = lv_label_create(ok_btn);
    lv_label_set_text(label, i18n_table_get(lang, I18N_KEY_OK));
    lv_obj_center(label);
    lv_obj_add_event_cb(ok_btn, confirm_ok_cb, LV_EVENT_CLICKED, dialog);

    cancel_btn = lv_button_create(dialog);
    lv_obj_set_size(cancel_btn, (lv_coord_t)UI_THEME_DIALOG_BTN_WIDTH, (lv_coord_t)UI_THEME_DIALOG_BTN_HEIGHT);
    lv_obj_align(cancel_btn, LV_ALIGN_BOTTOM_RIGHT, -((int32_t)UI_THEME_DIALOG_CANCEL_OFFSET_X), -8);
    label = lv_label_create(cancel_btn);
    lv_label_set_text(label, i18n_table_get(lang, I18N_KEY_CANCEL));
    lv_obj_center(label);
    lv_obj_add_event_cb(cancel_btn, confirm_cancel_cb, LV_EVENT_CLICKED, dialog);

    return ESP_OK;
}
