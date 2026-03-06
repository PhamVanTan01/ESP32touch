#pragma once

#include "ui/screens/screen_placeholder.h"

typedef screen_placeholder_view_t screen_maintenance_view_t;

esp_err_t screen_maintenance_create(lv_display_t *display, screen_maintenance_view_t *view);
void screen_maintenance_destroy(screen_maintenance_view_t *view);
