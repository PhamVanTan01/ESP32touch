#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"

typedef struct
{
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_touch_handle_t touch_handle;
    bool is_initialized;
} bsp_touch_t;

esp_err_t bsp_touch_init(bsp_touch_t *touch);
esp_err_t bsp_touch_deinit(bsp_touch_t *touch);
