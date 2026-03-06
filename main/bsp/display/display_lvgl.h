#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "lvgl.h"

#include "bsp/display/display_driver.h"
#include "bsp/touch/touch_driver.h"

typedef struct
{
    lv_display_t *display;
    lv_indev_t *touch_input_device;
    esp_timer_handle_t tick_timer;
    TaskHandle_t task_handle;
    void *draw_buffer_primary;
    void *draw_buffer_secondary;
    size_t draw_buffer_size_bytes;
    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_touch_handle_t touch_handle;
    lv_display_rotation_t applied_rotation;
    bool is_initialized;
} display_lvgl_t;

esp_err_t display_lvgl_init(const bsp_display_t *bsp_display,
                            const bsp_touch_t *bsp_touch,
                            display_lvgl_t *lvgl_port);
esp_err_t display_lvgl_deinit(display_lvgl_t *lvgl_port);
