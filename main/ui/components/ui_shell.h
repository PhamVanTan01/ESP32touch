#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"

#include "ui/components/ui_back_button.h"
#include "ui/ui_manager.h"

typedef struct
{
    lv_display_t *display;
    lv_obj_t *root;
    lv_obj_t *status_bar;
    lv_obj_t *status_label;
    lv_obj_t *alarm_count_label;
    lv_obj_t *lang_panel;
    lv_obj_t *title_bar;
    lv_obj_t *title_label;
    lv_obj_t *content;
    lv_obj_t *back_button;
} ui_shell_t;

esp_err_t ui_shell_create_with_back(lv_display_t *display,
                                    ui_screen_id_t active_screen,
                                    const char *title,
                                    bool show_back_button,
                                    ui_screen_id_t back_to_screen,
                                    ui_shell_t *shell);

esp_err_t ui_shell_create(lv_display_t *display,
                          ui_screen_id_t active_screen,
                          const char *title,
                          ui_shell_t *shell);
void ui_shell_destroy(ui_shell_t *shell);
lv_obj_t *ui_shell_get_content(ui_shell_t *shell);
void ui_shell_set_status_text(ui_shell_t *shell, const char *text);
void ui_shell_set_alarm_banner(ui_shell_t *shell, uint32_t critical_count,
                               uint32_t warning_count, uint32_t info_count);
