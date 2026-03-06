#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "lvgl.h"

#include "application/data_manager.h"
#include "ui/components/ui_shell.h"

typedef struct
{
    ui_shell_t shell;
    lv_obj_t *chip_value_label;
    lv_obj_t *memory_value_label;
    lv_obj_t *system_value_label;
    lv_obj_t *network_value_label;
    lv_display_rotation_t rotation;
    bool is_created;
} screen_main_view_t;

esp_err_t screen_main_create(lv_display_t *display, screen_main_view_t *view);
void screen_main_destroy(screen_main_view_t *view);
esp_err_t screen_main_update_status(screen_main_view_t *view, const app_data_snapshot_t *snapshot);
