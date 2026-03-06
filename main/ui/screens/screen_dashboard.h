#pragma once

#include "ui/screens/screen_main.h"

typedef screen_main_view_t screen_dashboard_view_t;

esp_err_t screen_dashboard_create(lv_display_t *display, screen_dashboard_view_t *view);
void screen_dashboard_destroy(screen_dashboard_view_t *view);
esp_err_t screen_dashboard_update(screen_dashboard_view_t *view, const app_data_snapshot_t *snapshot);
