#include "ui/screens/screen_login.h"

#include <stdio.h>
#include <string.h>

#include "application/auth_manager.h"
#include "application/hmi_state.h"
#include "config/auth_demo_config.h"
#include "config/ui_theme.h"
#include "esp_log.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/ui_manager.h"

static const char *TAG = "screen_login";

static void set_status(screen_login_view_t *view, const char *text, lv_color_t color);

/* Show keyboard and bind to the focused textarea; hide when defocused. */
static void textarea_focus_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target(e);
    screen_login_view_t *view = (screen_login_view_t *)lv_event_get_user_data(e);

    if (view == NULL || view->keyboard == NULL) {
        return;
    }

    if (code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(view->keyboard, ta);
        lv_obj_remove_flag(view->keyboard, LV_OBJ_FLAG_HIDDEN);
    }

    if (code == LV_EVENT_DEFOCUSED) {
        lv_keyboard_set_textarea(view->keyboard, NULL);
        lv_obj_add_flag(view->keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void set_status(screen_login_view_t *view, const char *text, lv_color_t color)
{
    if ((view == NULL) || (view->status_label == NULL) || (text == NULL)) {
        return;
    }

    lv_obj_set_style_text_color(view->status_label, color, 0);
    lv_label_set_text(view->status_label, text);
}

static void login_button_event_cb(lv_event_t *event)
{
    const char *username;
    const char *password;
    esp_err_t result;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    screen_login_view_t *view = (screen_login_view_t *)lv_event_get_user_data(event);
    uint32_t now_seconds;
    uint32_t unlock_at_seconds;
    uint32_t remaining_min;

    if ((view == NULL) || (view->username_input == NULL) || (view->password_input == NULL)) {
        return;
    }

    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }

    now_seconds = auth_manager_get_current_seconds();
    if (auth_manager_is_locked_out(now_seconds, &unlock_at_seconds)) {
        remaining_min = (unlock_at_seconds - now_seconds + 59U) / 60U;
        if (remaining_min == 0U) {
            remaining_min = 1U;
        }
        {
            char buf[UI_THEME_LOGIN_STATUS_BUF_SIZE];
            (void)snprintf(buf, sizeof(buf), i18n_table_get(lang, I18N_KEY_LOGIN_LOCKED_UNTIL), (unsigned)remaining_min);
            set_status(view, buf, lv_palette_main(LV_PALETTE_RED));
        }
        return;
    }

    username = lv_textarea_get_text(view->username_input);
    password = lv_textarea_get_text(view->password_input);

    if ((username == NULL) || (password == NULL) ||
        (strcmp(username, AUTH_DEMO_USERNAME) != 0) ||
        (strcmp(password, AUTH_DEMO_PASSWORD) != 0)) {
        set_status(view, i18n_table_get(lang, I18N_KEY_ERR_WRONG_CREDENTIALS), lv_palette_main(LV_PALETTE_RED));
        ESP_LOGW(TAG, "Rejected login attempt for username='%s'", (username != NULL) ? username : "(null)");
        return;
    }

    result = auth_manager_login_password(username, password, AUTH_DEMO_ROLE);
    if (result != ESP_OK) {
        set_status(view, i18n_table_get(lang, I18N_KEY_ERR_LOGIN_FAILED), lv_palette_main(LV_PALETTE_RED));
        ESP_LOGW(TAG, "auth_manager_login_password failed: %s", esp_err_to_name(result));
        return;
    }

    (void)hmi_state_set_role(AUTH_DEMO_ROLE);
    set_status(view, i18n_table_get(lang, I18N_KEY_LOGIN_SUCCESS), lv_palette_main(LV_PALETTE_GREEN));

    result = ui_manager_show(UI_SCREEN_HOME);
    if (result != ESP_OK) {
        set_status(view, i18n_table_get(lang, I18N_KEY_ERR_DASHBOARD_FAILED), lv_palette_main(LV_PALETTE_RED));
        ESP_LOGW(TAG, "ui_manager_show(UI_SCREEN_HOME) failed: %s", esp_err_to_name(result));
    }
}

esp_err_t screen_login_create(lv_display_t *display, screen_login_view_t *view)
{
    lv_obj_t *content;
    lv_obj_t *title;
    lv_obj_t *label;
    lv_obj_t *login_button;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;

    if ((display == NULL) || (view == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }

    (void)memset(view, 0, sizeof(*view));

    if (ui_shell_create_with_back(display, UI_SCREEN_LOGIN, i18n_table_get(lang, I18N_KEY_LOGIN), false, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        screen_login_destroy(view);
        return ESP_FAIL;
    }
    lv_obj_add_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(content, (int32_t)UI_THEME_CONTENT_PADDING, 0);

    title = lv_label_create(content);
    lv_label_set_text(title, i18n_table_get(lang, I18N_KEY_LOGIN_TITLE));
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, UI_THEME_LOGIN_TITLE_Y);

    label = lv_label_create(content);
    lv_label_set_text(label, i18n_table_get(lang, I18N_KEY_USERNAME));
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, UI_THEME_LOGIN_MARGIN_X, UI_THEME_LOGIN_ROW1_Y);

    view->username_input = lv_textarea_create(content);
    lv_obj_set_size(view->username_input, UI_THEME_LOGIN_INPUT_WIDTH, UI_THEME_LOGIN_INPUT_HEIGHT);
    lv_obj_align(view->username_input, LV_ALIGN_TOP_LEFT, UI_THEME_LOGIN_MARGIN_X, UI_THEME_LOGIN_ROW1_INPUT_Y);
    lv_textarea_set_one_line(view->username_input, true);
    lv_textarea_set_placeholder_text(view->username_input, AUTH_DEMO_USERNAME);
    lv_obj_add_event_cb(view->username_input, textarea_focus_event_cb, LV_EVENT_FOCUSED, view);
    lv_obj_add_event_cb(view->username_input, textarea_focus_event_cb, LV_EVENT_DEFOCUSED, view);

    label = lv_label_create(content);
    lv_label_set_text(label, i18n_table_get(lang, I18N_KEY_PASSWORD));
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, UI_THEME_LOGIN_MARGIN_X, UI_THEME_LOGIN_ROW2_Y);

    view->password_input = lv_textarea_create(content);
    lv_obj_set_size(view->password_input, UI_THEME_LOGIN_INPUT_WIDTH, UI_THEME_LOGIN_INPUT_HEIGHT);
    lv_obj_align(view->password_input, LV_ALIGN_TOP_LEFT, UI_THEME_LOGIN_MARGIN_X, UI_THEME_LOGIN_ROW2_INPUT_Y);
    lv_textarea_set_one_line(view->password_input, true);
    lv_textarea_set_password_mode(view->password_input, true);
    lv_textarea_set_placeholder_text(view->password_input, "");
    lv_obj_add_event_cb(view->password_input, textarea_focus_event_cb, LV_EVENT_FOCUSED, view);
    lv_obj_add_event_cb(view->password_input, textarea_focus_event_cb, LV_EVENT_DEFOCUSED, view);

    login_button = lv_button_create(content);
    lv_obj_set_size(login_button, UI_THEME_LOGIN_BUTTON_WIDTH, UI_THEME_LOGIN_BUTTON_HEIGHT);
    lv_obj_align(login_button, LV_ALIGN_TOP_LEFT, UI_THEME_LOGIN_MARGIN_X, UI_THEME_LOGIN_BUTTON_Y);
    lv_obj_add_event_cb(login_button, login_button_event_cb, LV_EVENT_CLICKED, view);

    label = lv_label_create(login_button);
    lv_label_set_text(label, i18n_table_get(lang, I18N_KEY_LOGIN_BUTTON));
    lv_obj_center(label);

    view->status_label = lv_label_create(content);
    lv_obj_set_width(view->status_label, (lv_coord_t)UI_THEME_LOGIN_STATUS_LABEL_WIDTH);
    lv_obj_set_style_max_height(view->status_label, UI_THEME_LOGIN_STATUS_MAX_HEIGHT, 0);
    lv_label_set_long_mode(view->status_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(view->status_label, LV_ALIGN_TOP_LEFT, UI_THEME_LOGIN_MARGIN_X, UI_THEME_LOGIN_STATUS_Y);
    set_status(view, i18n_table_get(lang, I18N_KEY_LOGIN_HINT), lv_color_hex(0x888888U));

    view->keyboard = lv_keyboard_create(content);
    if (view->keyboard != NULL) {
        lv_obj_set_size(view->keyboard, lv_pct(100), (lv_coord_t)UI_THEME_LOGIN_KEYBOARD_HEIGHT_PX);
        lv_obj_align(view->keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_keyboard_set_mode(view->keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
        lv_keyboard_set_textarea(view->keyboard, NULL);
        lv_obj_add_flag(view->keyboard, LV_OBJ_FLAG_HIDDEN);
    }

    view->is_created = true;
    return ESP_OK;
}

void screen_login_destroy(screen_login_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}
