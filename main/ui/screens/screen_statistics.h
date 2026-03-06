#pragma once

#include "ui/screens/screen_placeholder.h"

typedef screen_placeholder_view_t screen_statistics_view_t;

esp_err_t screen_statistics_create(lv_display_t *display, screen_statistics_view_t *view);
void screen_statistics_destroy(screen_statistics_view_t *view);
