#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "lvgl.h"

#include "ui/components/ui_shell.h"

typedef struct
{
    ui_shell_t shell;
    bool is_created;
} screen_home_view_t;

esp_err_t screen_home_create(lv_display_t *display, screen_home_view_t *view);
void screen_home_destroy(screen_home_view_t *view);
