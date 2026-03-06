/*
 * Recipe screen: placeholder + Select recipe with audit (Phase 6.1).
 */

#include "ui/screens/screen_recipe.h"

#include <stdio.h>
#include <string.h>

#include "application/hmi_state.h"
#include "middleware/audit/audit_log.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_shell.h"
#include "ui/ui_manager.h"
#include "config/ui_theme.h"

#define RECIPE_BTN_Y  80
#define RECIPE_BTN_W  160
#define RECIPE_MSG_SZ 48

static void recipe_select_btn_cb(lv_event_t *e)
{
    char msg[RECIPE_MSG_SZ];

    (void)e;
    (void)snprintf(msg, sizeof(msg), "Recipe 1 selected");
    (void)audit_log_record(AUDIT_EVENT_RECIPE_CHANGE, msg);
}

esp_err_t screen_recipe_create(lv_display_t *display, screen_recipe_view_t *view)
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
    title = i18n_table_get(lang, I18N_KEY_RECIPE);

    (void)memset(view, 0, sizeof(*view));
    if (ui_shell_create_with_back(display, UI_SCREEN_RECIPE, title, true, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }
    lv_obj_set_style_pad_all(content, (int32_t)UI_THEME_CONTENT_PADDING, 0);

    label = lv_label_create(content);
    lv_label_set_text(label, "TODO: Dynamic recipe manager from JSON profile");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_width(label, 200);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 24);

    btn = lv_button_create(content);
    lv_obj_set_size(btn, RECIPE_BTN_W, (lv_coord_t)UI_THEME_BUTTON_MIN_HEIGHT);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, RECIPE_BTN_Y);
    label = lv_label_create(btn);
    lv_label_set_text(label, "Select recipe 1");
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, recipe_select_btn_cb, LV_EVENT_CLICKED, NULL);

    view->is_created = true;
    return ESP_OK;
}

void screen_recipe_destroy(screen_recipe_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
