#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

typedef struct
{
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    bool is_initialized;
} bsp_display_t;

esp_err_t bsp_display_init(bsp_display_t *display);
esp_err_t bsp_display_deinit(bsp_display_t *display);
esp_err_t bsp_display_backlight_set(bool enable);
