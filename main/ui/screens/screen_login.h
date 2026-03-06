#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "lvgl.h"

#include "ui/components/ui_shell.h"

typedef struct
{
    ui_shell_t shell;
    lv_obj_t *username_input;
    lv_obj_t *password_input;
    lv_obj_t *status_label;
    lv_obj_t *keyboard;
    bool is_created;
} screen_login_view_t;

esp_err_t screen_login_create(lv_display_t *display, screen_login_view_t *view);
void screen_login_destroy(screen_login_view_t *view);
