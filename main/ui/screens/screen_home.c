/*
 * Home menu screen: centered list of navigable UI entries.
 * Table-driven, no hardcoded strings; theme and i18n from config.
 * MISRA-oriented: explicit types, single responsibility, config data separate.
 */

#include "ui/screens/screen_home.h"

#include <stdint.h>
#include <string.h>

#include "application/auth_manager.h"
#include "application/hmi_state.h"
#include "config/ui_theme.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/ui_manager.h"

#include "ui/components/ui_shell.h"

/* Single source of truth: home menu entries (screen_id -> i18n label key). */
typedef struct
{
    ui_screen_id_t screen_id;
    i18n_key_t i18n_key;
} home_menu_entry_t;

static const home_menu_entry_t s_home_entries[] = {
    { UI_SCREEN_MAIN,          I18N_KEY_DASHBOARD },
    { UI_SCREEN_RECIPE,        I18N_KEY_RECIPE },
    { UI_SCREEN_VISION_TUNING, I18N_KEY_VISION_TUNING },
    { UI_SCREEN_STATISTICS,    I18N_KEY_STATISTICS },
    { UI_SCREEN_ALARMS,        I18N_KEY_ALARMS },
    { UI_SCREEN_CALIBRATION,   I18N_KEY_CALIBRATION },
    { UI_SCREEN_MAINTENANCE,   I18N_KEY_MAINTENANCE },
    { UI_SCREEN_LOGIN,         I18N_KEY_LOGIN }
};

#define HOME_ENTRY_COUNT  (sizeof(s_home_entries) / sizeof(s_home_entries[0]))

static void home_menu_button_event_cb(lv_event_t *e)
{
    ui_screen_id_t screen_id;
    esp_err_t result;

    screen_id = (ui_screen_id_t)(uintptr_t)lv_event_get_user_data(e);
    if ((uint32_t)screen_id >= (uint32_t)UI_SCREEN_HOME) {
        return;
    }

    if (screen_id == UI_SCREEN_LOGIN) {
        auth_manager_logout();
    }

    result = ui_manager_show(screen_id);
    (void)result;
}

static esp_err_t create_menu_buttons(lv_obj_t *container,
                                    hmi_language_t lang,
                                    uint32_t entry_count)
{
    uint32_t index;
    lv_obj_t *btn;
    lv_obj_t *label;
    const char *text;
    home_menu_entry_t entry;

    if (container == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (index = 0U; index < entry_count; index++) {
        entry = s_home_entries[index];
        if (!ui_manager_can_access_screen(entry.screen_id)) {
            continue;
        }

        text = i18n_table_get(lang, entry.i18n_key);
        if (text == NULL) {
            text = "";
        }

        btn = lv_button_create(container);
        if (btn == NULL) {
            return ESP_ERR_NO_MEM;
        }
        lv_obj_set_size(btn, UI_THEME_HOME_BUTTON_WIDTH, UI_THEME_HOME_BUTTON_HEIGHT);
        lv_obj_set_style_radius(btn, (int32_t)UI_THEME_CARD_RADIUS, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(UI_THEME_COLOR_NAV_BUTTON_BG), 0);
        lv_obj_add_event_cb(btn, home_menu_button_event_cb, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)entry.screen_id);

        label = lv_label_create(btn);
        if (label == NULL) {
            lv_obj_delete(btn);
            return ESP_ERR_NO_MEM;
        }
        lv_label_set_text(label, text);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_center(label);
    }

    return ESP_OK;
}

esp_err_t screen_home_create(lv_display_t *display, screen_home_view_t *view)
{
    lv_obj_t *content;
    lv_obj_t *container;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    const char *title;
    esp_err_t result;

    if ((display == NULL) || (view == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(view, 0, sizeof(*view));

    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }

    title = i18n_table_get(lang, I18N_KEY_HOME);
    if (title == NULL) {
        title = "";
    }

    result = ui_shell_create(display, UI_SCREEN_HOME, title, &view->shell);
    if (result != ESP_OK) {
        return result;
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }
    lv_obj_add_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(content, (int32_t)UI_THEME_CONTENT_PADDING, 0);

    container = lv_obj_create(content);
    if (container == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_ERR_NO_MEM;
    }

    lv_obj_set_size(container, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_pad_all(container, 0, 0);
    lv_obj_set_layout(container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_column(container, (int32_t)UI_THEME_HOME_BUTTON_GAP, 0);
    lv_obj_set_style_pad_row(container, (int32_t)UI_THEME_HOME_BUTTON_GAP, 0);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    result = create_menu_buttons(container, lang, (uint32_t)HOME_ENTRY_COUNT);
    if (result != ESP_OK) {
        lv_obj_delete(container);
        ui_shell_destroy(&view->shell);
        return result;
    }

    view->is_created = true;
    return ESP_OK;
}

void screen_home_destroy(screen_home_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
