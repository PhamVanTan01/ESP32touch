/*
 * Alarms screen: list from alarm_manager, severity label, Ack button (Phase 3.1).
 */

#include "ui/screens/screen_alarms.h"

#include <string.h>

#include "application/alarm_config.h"
#include "application/alarm_manager.h"
#include "application/hmi_state.h"
#include "config/ui_theme.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_shell.h"
#include "ui/ui_manager.h"

typedef struct
{
    uint32_t id;
} alarm_ack_ud_t;

static const i18n_key_t s_severity_keys[] = {
    I18N_KEY_ALARM_INFO,
    I18N_KEY_ALARM_WARNING,
    I18N_KEY_ALARM_CRITICAL
};

static void alarm_ack_btn_cb(lv_event_t *e)
{
    alarm_ack_ud_t *ud = (alarm_ack_ud_t *)lv_event_get_user_data(e);
    if (ud != NULL) {
        (void)alarm_manager_ack(ud->id);
    }
}

esp_err_t screen_alarms_create(lv_display_t *display, screen_alarms_view_t *view)
{
    lv_obj_t *content;
    alarm_entry_t entries[ALARM_MAX_ACTIVE];
    size_t count = 0U;
    size_t i;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    const char *title;

    if ((display == NULL) || (view == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }
    title = i18n_table_get(lang, I18N_KEY_ALARMS);

    (void)memset(view, 0, sizeof(*view));
    if (ui_shell_create_with_back(display, UI_SCREEN_ALARMS, title, true, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }
    lv_obj_add_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(content, (int32_t)UI_THEME_CONTENT_PADDING, 0);

    if (alarm_manager_get_list(entries, ALARM_MAX_ACTIVE, &count) != ESP_OK) {
        count = 0U;
    }

    static alarm_ack_ud_t s_ack_ud[ALARM_MAX_ACTIVE];
    for (i = 0U; i < count; i++) {
        lv_obj_t *row = lv_obj_create(content);
        lv_obj_set_size(row, lv_pct(100) - (UI_THEME_ALARM_ROW_MARGIN * 2), (lv_coord_t)UI_THEME_ALARM_ROW_HEIGHT);
        lv_obj_align(row, LV_ALIGN_TOP_LEFT, (int32_t)UI_THEME_ALARM_ROW_MARGIN,
                     (int32_t)(i * (uint32_t)(UI_THEME_ALARM_ROW_HEIGHT + UI_THEME_ALARM_ROW_MARGIN)) + (int32_t)UI_THEME_ALARM_ROW_MARGIN);
        lv_obj_set_style_bg_color(row, lv_color_hex(UI_THEME_COLOR_CARD_BG), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *sev_label = lv_label_create(row);
        lv_label_set_text(sev_label,
                          i18n_table_get(lang, s_severity_keys[(uint32_t)entries[i].severity]));
        lv_obj_align(sev_label, LV_ALIGN_LEFT_MID, UI_THEME_HORIZONTAL_PADDING, 0);

        lv_obj_t *msg_label = lv_label_create(row);
        lv_label_set_text(msg_label, entries[i].message);
        lv_obj_set_width(msg_label, lv_pct(100) - (UI_THEME_HORIZONTAL_PADDING * 2) - UI_THEME_ALARM_ACK_BTN_WIDTH - 8);
        lv_label_set_long_mode(msg_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_align(msg_label, LV_ALIGN_LEFT_MID, 80, 0);

        if (!entries[i].acked) {
            lv_obj_t *ack_btn = lv_button_create(row);
            lv_obj_set_size(ack_btn, (lv_coord_t)UI_THEME_ALARM_ACK_BTN_WIDTH, (lv_coord_t)UI_THEME_ALARM_ACK_BTN_HEIGHT);
            lv_obj_align(ack_btn, LV_ALIGN_RIGHT_MID, -UI_THEME_HORIZONTAL_PADDING, 0);
            lv_obj_t *ack_lbl = lv_label_create(ack_btn);
            lv_label_set_text(ack_lbl, i18n_table_get(lang, I18N_KEY_ACK));
            lv_obj_center(ack_lbl);
            if (i < ALARM_MAX_ACTIVE) {
                s_ack_ud[i].id = entries[i].id;
                lv_obj_add_event_cb(ack_btn, alarm_ack_btn_cb, LV_EVENT_CLICKED, &s_ack_ud[i]);
            }
        }
    }

    if (count == 0U) {
        lv_obj_t *empty = lv_label_create(content);
        lv_label_set_text(empty, i18n_table_get(lang, I18N_KEY_NO_ALARMS));
        lv_obj_align(empty, LV_ALIGN_CENTER, 0, 0);
    }

    view->is_created = true;
    return ESP_OK;
}

void screen_alarms_destroy(screen_alarms_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
