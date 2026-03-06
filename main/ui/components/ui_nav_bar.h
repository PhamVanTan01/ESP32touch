#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "lvgl.h"

#include "ui/ui_manager.h"

typedef struct
{
    lv_obj_t *root;
    lv_obj_t *button_home;
    lv_obj_t *button_dashboard;
    lv_obj_t *button_recipe;
    lv_obj_t *button_vision;
    lv_obj_t *button_alarms;
    lv_obj_t *button_more;
    lv_obj_t *more_panel;
} ui_nav_bar_t;

esp_err_t ui_nav_bar_create(lv_obj_t *parent,
                            ui_screen_id_t active_screen,
                            ui_nav_bar_t *nav_bar);
void ui_nav_bar_destroy(ui_nav_bar_t *nav_bar);
void ui_nav_bar_set_active(ui_nav_bar_t *nav_bar, ui_screen_id_t active_screen);
