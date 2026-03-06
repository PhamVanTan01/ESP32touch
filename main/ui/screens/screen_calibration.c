/*
 * Calibration screen: placeholder + Run calibration with audit (Phase 6.1).
 */

#include "ui/screens/screen_calibration.h"

#include <stdio.h>
#include <string.h>

#include "application/hmi_state.h"
#include "middleware/audit/audit_log.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_shell.h"
#include "ui/ui_manager.h"
#include "config/ui_theme.h"

#define CALIB_BTN_Y  80
#define CALIB_BTN_W  160
#define CALIB_MSG_SZ 48

static void calibration_run_btn_cb(lv_event_t *e)
{
    char msg[CALIB_MSG_SZ];

    (void)e;
    (void)snprintf(msg, sizeof(msg), "Calibration started");
    (void)audit_log_record(AUDIT_EVENT_CALIBRATION, msg);
}

esp_err_t screen_calibration_create(lv_display_t *display, screen_calibration_view_t *view)
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
    title = i18n_table_get(lang, I18N_KEY_CALIBRATION);

    (void)memset(view, 0, sizeof(*view));
    if (ui_shell_create_with_back(display, UI_SCREEN_CALIBRATION, title, true, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }
    lv_obj_set_style_pad_all(content, (int32_t)UI_THEME_CONTENT_PADDING, 0);

    label = lv_label_create(content);
    lv_label_set_text(label, "TODO: Sensor and camera calibration flow");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_width(label, 200);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 24);

    btn = lv_button_create(content);
    lv_obj_set_size(btn, CALIB_BTN_W, (lv_coord_t)UI_THEME_BUTTON_MIN_HEIGHT);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, CALIB_BTN_Y);
    label = lv_label_create(btn);
    lv_label_set_text(label, "Run calibration");
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, calibration_run_btn_cb, LV_EVENT_CLICKED, NULL);

    view->is_created = true;
    return ESP_OK;
}

void screen_calibration_destroy(screen_calibration_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
