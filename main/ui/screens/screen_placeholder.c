#include "ui/screens/screen_placeholder.h"

#include <string.h>

#include "config/ui_theme.h"
#include "ui/components/ui_shell.h"
#include "ui/ui_manager.h"

esp_err_t screen_placeholder_create(lv_display_t *display,
                                    screen_placeholder_view_t *view,
                                    ui_screen_id_t active_screen,
                                    const char *title,
                                    const char *subtitle)
{
    lv_obj_t *content;
    lv_obj_t *label;
    lv_obj_t *sub_label;

    if ((display == NULL) || (view == NULL) || (title == NULL) || (subtitle == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    bool show_back;

    (void)memset(view, 0, sizeof(*view));
    show_back = (active_screen != UI_SCREEN_HOME) &&
                (active_screen != UI_SCREEN_LOGIN) &&
                (active_screen != UI_SCREEN_SPLASH);
    if (ui_shell_create_with_back(display, active_screen, title, show_back, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }

    lv_obj_set_style_pad_all(content, UI_THEME_CONTENT_PADDING, 0);

    label = lv_label_create(content);
    lv_label_set_text(label, title);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 24);

    sub_label = lv_label_create(content);
    lv_label_set_text(sub_label, subtitle);
    lv_obj_set_width(sub_label, 200);
    lv_label_set_long_mode(sub_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(sub_label, LV_ALIGN_CENTER, 0, 10);

    view->is_created = true;
    return ESP_OK;
}

void screen_placeholder_destroy(screen_placeholder_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
