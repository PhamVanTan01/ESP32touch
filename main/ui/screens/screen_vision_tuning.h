#pragma once

#include "ui/screens/screen_placeholder.h"

typedef screen_placeholder_view_t screen_vision_tuning_view_t;

esp_err_t screen_vision_tuning_create(lv_display_t *display, screen_vision_tuning_view_t *view);
void screen_vision_tuning_destroy(screen_vision_tuning_view_t *view);
