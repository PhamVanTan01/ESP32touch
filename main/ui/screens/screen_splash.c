/*
 * Splash screen: "Starting..." (i18n), no nav bar (Phase 4.1).
 */

#include "ui/screens/screen_splash.h"

#include <string.h>

#include "application/hmi_state.h"
#include "config/ui_theme.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_shell.h"
#include "ui/ui_manager.h"

esp_err_t screen_splash_create(lv_display_t *display, screen_splash_view_t *view)
{
    lv_obj_t *content;
    lv_obj_t *label;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    const char *text;

    if ((display == NULL) || (view == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }
    text = i18n_table_get(lang, I18N_KEY_SPLASH_TEXT);

    (void)memset(view, 0, sizeof(*view));
    if (ui_shell_create_with_back(display, UI_SCREEN_SPLASH, "", false, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }

    if (view->shell.title_bar != NULL) {
        lv_obj_add_flag(view->shell.title_bar, LV_OBJ_FLAG_HIDDEN);
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }

    label = lv_label_create(content);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    view->is_created = true;
    return ESP_OK;
}

void screen_splash_destroy(screen_splash_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
