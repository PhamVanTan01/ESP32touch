#pragma once

#include "ui/screens/screen_placeholder.h"

typedef screen_placeholder_view_t screen_alarms_view_t;

esp_err_t screen_alarms_create(lv_display_t *display, screen_alarms_view_t *view);
void screen_alarms_destroy(screen_alarms_view_t *view);
