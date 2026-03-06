#pragma once

#include "ui/screens/screen_placeholder.h"

typedef screen_placeholder_view_t screen_recipe_view_t;

esp_err_t screen_recipe_create(lv_display_t *display, screen_recipe_view_t *view);
void screen_recipe_destroy(screen_recipe_view_t *view);
