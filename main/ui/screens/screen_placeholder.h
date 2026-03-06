#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "lvgl.h"
#include "ui/components/ui_shell.h"
#include "ui/ui_manager.h"

typedef struct
{
    ui_shell_t shell;
    bool is_created;
} screen_placeholder_view_t;

esp_err_t screen_placeholder_create(lv_display_t *display,
                                    screen_placeholder_view_t *view,
                                    ui_screen_id_t active_screen,
                                    const char *title,
                                    const char *subtitle);
void screen_placeholder_destroy(screen_placeholder_view_t *view);
