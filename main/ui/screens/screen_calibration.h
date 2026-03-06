#pragma once

#include "ui/screens/screen_placeholder.h"

typedef screen_placeholder_view_t screen_calibration_view_t;

esp_err_t screen_calibration_create(lv_display_t *display, screen_calibration_view_t *view);
void screen_calibration_destroy(screen_calibration_view_t *view);
