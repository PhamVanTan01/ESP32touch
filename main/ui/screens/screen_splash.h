#pragma once

#include "esp_err.h"
#include "lvgl.h"
#include "ui/components/ui_shell.h"

typedef struct
{
    ui_shell_t shell;
    bool is_created;
} screen_splash_view_t;

esp_err_t screen_splash_create(lv_display_t *display, screen_splash_view_t *view);
void screen_splash_destroy(screen_splash_view_t *view);
