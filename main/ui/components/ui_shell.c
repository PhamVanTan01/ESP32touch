#include "ui/components/ui_shell.h"

#include <string.h>

#include "application/auth_manager.h"
#include "application/hmi_state.h"
#include "config/hmi_loader.h"
#include "config/hmi_schema.h"
#include "config/ui_theme.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/ui_manager.h"

static const i18n_key_t s_lang_name_keys[] = {
    I18N_KEY_LANG_VI, I18N_KEY_LANG_EN, I18N_KEY_LANG_ES, I18N_KEY_LANG_ZH
};

typedef struct
{
    ui_shell_t *shell;
    hmi_language_t lang;
} shell_lang_ud_t;

static void shell_lang_select_cb(lv_event_t *e)
{
    shell_lang_ud_t *ud = (shell_lang_ud_t *)lv_event_get_user_data(e);
    char status_text[64];
    const char *fmt;
    const char *comm_status = "INIT";

    if (ud == NULL || ud->shell == NULL) {
        return;
    }
    (void)hmi_state_set_language(ud->lang);
    fmt = i18n_table_get(ud->lang, I18N_KEY_STATUS_ROLE_COMM);
    (void)lv_snprintf(status_text, sizeof(status_text), fmt,
                      auth_manager_get_role_name(), comm_status);
    lv_label_set_text(ud->shell->status_label, status_text);
    if (ud->shell->lang_panel != NULL) {
        lv_obj_add_flag(ud->shell->lang_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

static void shell_lang_btn_cb(lv_event_t *e)
{
    ui_shell_t *shell = (ui_shell_t *)lv_event_get_user_data(e);
    if (shell != NULL && shell->lang_panel != NULL) {
        if (lv_obj_has_flag(shell->lang_panel, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_clear_flag(shell->lang_panel, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(shell->lang_panel, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* Map ui_screen_id_t to help i18n key (order matches ui_manager.h enum). */
static const i18n_key_t s_help_keys[] = {
    I18N_KEY_HELP_SPLASH,
    I18N_KEY_HELP_DASHBOARD,
    I18N_KEY_HELP_RECIPE,
    I18N_KEY_HELP_CALIBRATION,
    I18N_KEY_HELP_VISION_TUNING,
    I18N_KEY_HELP_STATISTICS,
    I18N_KEY_HELP_ALARMS,
    I18N_KEY_HELP_MAINTENANCE,
    I18N_KEY_HELP_LOGIN,
    I18N_KEY_HELP_HOME
};

#define UI_SCREEN_COUNT  (sizeof(s_help_keys) / sizeof(s_help_keys[0]))

static void shell_help_btn_cb(lv_event_t *e)
{
    ui_shell_t *shell = (ui_shell_t *)lv_event_get_user_data(e);
    ui_screen_id_t screen_id;
    uint32_t index;
    i18n_key_t help_key;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    const char *help_text;

    if (shell == NULL) {
        return;
    }
    screen_id = ui_manager_get_active_screen();
    index = (uint32_t)screen_id;
    if (index >= UI_SCREEN_COUNT) {
        help_key = I18N_KEY_HELP;
    } else {
        help_key = s_help_keys[index];
    }
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }
    help_text = i18n_table_get(lang, help_key);
    {
        lv_obj_t *mbox = lv_msgbox_create(NULL);
        if (mbox != NULL) {
            lv_msgbox_add_title(mbox, i18n_table_get(lang, I18N_KEY_HELP));
            lv_msgbox_add_text(mbox, help_text);
            lv_msgbox_add_close_button(mbox);
        }
    }
}

esp_err_t ui_shell_create_with_back(lv_display_t *display,
                                    ui_screen_id_t active_screen,
                                    const char *title,
                                    bool show_back_button,
                                    ui_screen_id_t back_to_screen,
                                    ui_shell_t *shell)
{
    char status_text[64];
    hmi_runtime_state_t state;
    const char *fmt;
    const char *comm_status = "INIT";
    uint32_t content_height;

    if ((display == NULL) || (title == NULL) || (shell == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (((uint32_t)back_to_screen >= UI_SCREEN_COUNT) ||
        ((uint32_t)active_screen >= UI_SCREEN_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(shell, 0, sizeof(*shell));
    shell->display = display;
    shell->root = lv_obj_create(NULL);
    if (shell->root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    lv_obj_set_style_bg_color(shell->root, lv_color_hex(UI_THEME_COLOR_ROOT_BG), 0);
    lv_obj_set_style_border_width(shell->root, 0, 0);
    lv_obj_set_style_pad_all(shell->root, 0, 0);
    lv_obj_clear_flag(shell->root, LV_OBJ_FLAG_SCROLLABLE);

    shell->status_bar = lv_obj_create(shell->root);
    lv_obj_set_size(shell->status_bar, lv_pct(100), UI_THEME_STATUS_BAR_HEIGHT);
    lv_obj_align(shell->status_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_radius(shell->status_bar, 0, 0);
    lv_obj_set_style_border_width(shell->status_bar, 0, 0);
    lv_obj_set_style_bg_color(shell->status_bar, lv_color_hex(UI_THEME_COLOR_STATUS_BAR_BG), 0);
    lv_obj_set_style_pad_all(shell->status_bar, 2, 0);
    lv_obj_clear_flag(shell->status_bar, LV_OBJ_FLAG_SCROLLABLE);

    shell->status_label = lv_label_create(shell->status_bar);
    lv_obj_set_style_text_color(shell->status_label, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(shell->status_label, LV_ALIGN_LEFT_MID, UI_THEME_HORIZONTAL_PADDING, 0);
    if (hmi_state_get(&state) == ESP_OK) {
        fmt = i18n_table_get(state.current_language, I18N_KEY_STATUS_ROLE_COMM);
        (void)lv_snprintf(status_text, sizeof(status_text), fmt,
                          auth_manager_get_role_name(), comm_status);
    } else {
        (void)lv_snprintf(status_text, sizeof(status_text), "Role: %s  Comm: %s",
                          auth_manager_get_role_name(), comm_status);
    }
    lv_label_set_text(shell->status_label, status_text);

    shell->alarm_count_label = lv_label_create(shell->status_bar);
    lv_obj_set_style_text_color(shell->alarm_count_label, lv_color_white(), 0);
    lv_obj_align(shell->alarm_count_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(shell->alarm_count_label, LV_OBJ_FLAG_HIDDEN);

    {
        const hmi_system_config_t *config = hmi_loader_get_config();
        if (config != NULL && config->language_support.available_language_count > 0U) {
            lv_obj_t *lang_btn = lv_button_create(shell->status_bar);
            lv_obj_set_size(lang_btn, UI_THEME_LANG_BTN_WIDTH, UI_THEME_LANG_BTN_HEIGHT);
            lv_obj_align(lang_btn, LV_ALIGN_RIGHT_MID, -UI_THEME_HORIZONTAL_PADDING, 0);
            lv_obj_t *lbl = lv_label_create(lang_btn);
            lv_label_set_text(lbl, i18n_table_get(state.current_language, I18N_KEY_LANGUAGE));
            lv_obj_center(lbl);
            lv_obj_add_event_cb(lang_btn, shell_lang_btn_cb, LV_EVENT_CLICKED, shell);

            shell->lang_panel = lv_obj_create(shell->root);
            lv_obj_set_size(shell->lang_panel, UI_THEME_LANG_PANEL_WIDTH,
                            (int32_t)(config->language_support.available_language_count) * UI_THEME_LANG_PANEL_ITEM_H + 8);
            lv_obj_align(shell->lang_panel, LV_ALIGN_TOP_RIGHT, -UI_THEME_HORIZONTAL_PADDING, UI_THEME_STATUS_BAR_HEIGHT);
            lv_obj_set_style_bg_color(shell->lang_panel, lv_color_hex(UI_THEME_COLOR_NAV_MORE_PANEL), 0);
            lv_obj_set_style_pad_all(shell->lang_panel, 4, 0);
            lv_obj_add_flag(shell->lang_panel, LV_OBJ_FLAG_HIDDEN);

            static shell_lang_ud_t s_shell_lang_ud[HMI_MAX_LANGUAGE_COUNT];
            for (uint32_t i = 0U; i < (uint32_t)config->language_support.available_language_count && i < HMI_MAX_LANGUAGE_COUNT; i++) {
                hmi_language_t l = config->language_support.available_languages[i];
                lv_obj_t *item = lv_button_create(shell->lang_panel);
                lv_obj_set_size(item, lv_pct(100), UI_THEME_LANG_PANEL_ITEM_H - 4);
                lv_obj_align(item, LV_ALIGN_TOP_LEFT, 0, (int32_t)(i * (uint32_t)UI_THEME_LANG_PANEL_ITEM_H) + 2);
                lbl = lv_label_create(item);
                lv_label_set_text(lbl, i18n_table_get(state.current_language, s_lang_name_keys[(uint32_t)l]));
                lv_obj_center(lbl);
                s_shell_lang_ud[i].shell = shell;
                s_shell_lang_ud[i].lang = l;
                lv_obj_add_event_cb(item, shell_lang_select_cb, LV_EVENT_CLICKED, &s_shell_lang_ud[i]);
            }
        } else {
            shell->lang_panel = NULL;
        }
    }

    shell->title_bar = lv_obj_create(shell->root);
    lv_obj_set_size(shell->title_bar, lv_pct(100), UI_THEME_TITLE_BAR_HEIGHT);
    lv_obj_align(shell->title_bar, LV_ALIGN_TOP_MID, 0, UI_THEME_STATUS_BAR_HEIGHT);
    lv_obj_set_style_radius(shell->title_bar, 0, 0);
    lv_obj_set_style_border_width(shell->title_bar, 0, 0);
    lv_obj_set_style_bg_color(shell->title_bar, lv_color_hex(UI_THEME_COLOR_TITLE_BAR_BG), 0);
    lv_obj_set_style_pad_all(shell->title_bar, 4, 0);
    lv_obj_clear_flag(shell->title_bar, LV_OBJ_FLAG_SCROLLABLE);

    shell->title_label = lv_label_create(shell->title_bar);
    lv_label_set_text(shell->title_label, title);
    lv_obj_set_style_text_color(shell->title_label, lv_color_white(), 0);
    lv_obj_align(shell->title_label, LV_ALIGN_LEFT_MID, UI_THEME_HORIZONTAL_PADDING, 0);

    {
        lv_obj_t *help_btn = lv_button_create(shell->title_bar);
        lv_obj_set_size(help_btn, UI_THEME_HELP_BUTTON_SIZE, UI_THEME_HELP_BUTTON_SIZE);
        lv_obj_align(help_btn, LV_ALIGN_RIGHT_MID, -UI_THEME_HORIZONTAL_PADDING, 0);
        lv_obj_t *help_lbl = lv_label_create(help_btn);
        lv_label_set_text(help_lbl, "?");
        lv_obj_center(help_lbl);
        lv_obj_add_event_cb(help_btn, shell_help_btn_cb, LV_EVENT_CLICKED, shell);
    }

    shell->content = lv_obj_create(shell->root);
    content_height = (uint32_t)UI_DISPLAY_HEIGHT - (uint32_t)UI_THEME_CONTENT_TOP - (uint32_t)UI_THEME_HORIZONTAL_PADDING;
    lv_obj_set_size(shell->content, lv_pct(100), (int32_t)content_height);
    lv_obj_align(shell->content, LV_ALIGN_TOP_MID, 0, UI_THEME_CONTENT_TOP);
    lv_obj_set_style_bg_opa(shell->content, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(shell->content, 0, 0);
    lv_obj_set_style_pad_all(shell->content, 0, 0);
    lv_obj_clear_flag(shell->content, LV_OBJ_FLAG_SCROLLABLE);

    shell->back_button = NULL;
    if (show_back_button) {
        lv_obj_t *back_parent = (shell->title_bar != NULL) ? shell->title_bar : shell->root;
        shell->back_button = ui_back_button_create(back_parent, back_to_screen);
    }

    return ESP_OK;
}

esp_err_t ui_shell_create(lv_display_t *display,
                          ui_screen_id_t active_screen,
                          const char *title,
                          ui_shell_t *shell)
{
    return ui_shell_create_with_back(display, active_screen, title, false, UI_SCREEN_HOME, shell);
}

void ui_shell_destroy(ui_shell_t *shell)
{
    if (shell == NULL) {
        return;
    }

    shell->back_button = NULL;

    if (shell->root != NULL) {
        lv_obj_delete(shell->root);
        shell->root = NULL;
    }
}

lv_obj_t *ui_shell_get_content(ui_shell_t *shell)
{
    return (shell != NULL) ? shell->content : NULL;
}

void ui_shell_set_status_text(ui_shell_t *shell, const char *text)
{
    if ((shell != NULL) && (shell->status_label != NULL) && (text != NULL)) {
        lv_label_set_text(shell->status_label, text);
    }
}

void ui_shell_set_alarm_banner(ui_shell_t *shell, uint32_t critical_count,
                               uint32_t warning_count, uint32_t info_count)
{
    uint32_t total;
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    char buf[32];
    const char *fmt;

    if (shell == NULL || shell->status_bar == NULL) {
        return;
    }
    total = critical_count + warning_count + info_count;
    if (total > 0U) {
        if (critical_count > 0U) {
            lv_obj_set_style_bg_color(shell->status_bar, lv_color_hex(UI_THEME_ALARM_CRITICAL_COLOR), 0);
        } else if (warning_count > 0U) {
            lv_obj_set_style_bg_color(shell->status_bar, lv_color_hex(UI_THEME_ALARM_WARNING_COLOR), 0);
        } else {
            lv_obj_set_style_bg_color(shell->status_bar, lv_color_hex(UI_THEME_ALARM_INFO_COLOR), 0);
        }
        if (shell->alarm_count_label != NULL) {
            if (hmi_state_get(&state) == ESP_OK) {
                lang = state.current_language;
            }
            fmt = i18n_table_get(lang, I18N_KEY_ALARMS_COUNT);
            (void)lv_snprintf(buf, sizeof(buf), fmt, (unsigned)total);
            lv_label_set_text(shell->alarm_count_label, buf);
            lv_obj_clear_flag(shell->alarm_count_label, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        lv_obj_set_style_bg_color(shell->status_bar, lv_color_hex(UI_THEME_COLOR_STATUS_BAR_BG), 0);
        if (shell->alarm_count_label != NULL) {
            lv_obj_add_flag(shell->alarm_count_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
}
