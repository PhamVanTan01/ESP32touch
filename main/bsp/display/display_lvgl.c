#include "bsp/display/display_lvgl.h"

#include <stdint.h>
#include <stdlib.h>

#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/board_config.h"
#include "config/system_config.h"
#include "ui/ui_manager.h"
#include "utils/lvgl_lock.h"

static const char *TAG = "display_lvgl";

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *event_data,
                                    void *user_ctx)
{
    lv_display_t *display = (lv_display_t *)user_ctx;

    LV_UNUSED(panel_io);
    LV_UNUSED(event_data);

    if (display != NULL) {
        lv_display_flush_ready(display);
    }

    return false;
}

static esp_err_t apply_display_rotation(display_lvgl_t *context, lv_display_t *display)
{
    esp_err_t result = ESP_OK;
    lv_display_rotation_t rotation;

    if ((context == NULL) || (display == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    rotation = lv_display_get_rotation(display);
    if (rotation == context->applied_rotation) {
        return ESP_OK;
    }

    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        result = esp_lcd_panel_swap_xy(context->panel_handle, false);
        if (result == ESP_OK) {
            result = esp_lcd_panel_mirror(context->panel_handle, true, false);
        }
        break;
    case LV_DISPLAY_ROTATION_90:
        result = esp_lcd_panel_swap_xy(context->panel_handle, true);
        if (result == ESP_OK) {
            result = esp_lcd_panel_mirror(context->panel_handle, true, true);
        }
        break;
    case LV_DISPLAY_ROTATION_180:
        result = esp_lcd_panel_swap_xy(context->panel_handle, false);
        if (result == ESP_OK) {
            result = esp_lcd_panel_mirror(context->panel_handle, false, true);
        }
        break;
    case LV_DISPLAY_ROTATION_270:
        result = esp_lcd_panel_swap_xy(context->panel_handle, true);
        if (result == ESP_OK) {
            result = esp_lcd_panel_mirror(context->panel_handle, false, false);
        }
        break;
    default:
        result = ESP_ERR_INVALID_STATE;
        break;
    }

    if (result == ESP_OK) {
        context->applied_rotation = rotation;
    }

    return result;
}

static void display_lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *pixel_map)
{
    display_lvgl_t *context = (display_lvgl_t *)lv_display_get_user_data(display);
    const int32_t offset_x1 = area->x1;
    const int32_t offset_x2 = area->x2;
    const int32_t offset_y1 = area->y1;
    const int32_t offset_y2 = area->y2;

    if ((context == NULL) || (pixel_map == NULL)) {
        return;
    }

    (void)apply_display_rotation(context, display);
    lv_draw_sw_rgb565_swap(pixel_map,
                           (uint32_t)(offset_x2 + 1 - offset_x1) *
                           (uint32_t)(offset_y2 + 1 - offset_y1));
    (void)esp_lcd_panel_draw_bitmap(context->panel_handle,
                                    offset_x1,
                                    offset_y1,
                                    offset_x2 + 1,
                                    offset_y2 + 1,
                                    pixel_map);
}

static void display_lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    display_lvgl_t *context = (display_lvgl_t *)lv_indev_get_user_data(indev);
    uint16_t touch_x[1] = {0U};
    uint16_t touch_y[1] = {0U};
    uint8_t touch_count = 0U;
    bool is_pressed;

    if ((context == NULL) || (context->touch_handle == NULL) || (data == NULL)) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    (void)esp_lcd_touch_read_data(context->touch_handle);
    is_pressed = esp_lcd_touch_get_coordinates(context->touch_handle,
                                               touch_x,
                                               touch_y,
                                               NULL,
                                               &touch_count,
                                               1U);

    if (is_pressed && (touch_count > 0U)) {
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void increase_lvgl_tick(void *arg)
{
    LV_UNUSED(arg);
    lv_tick_inc(ESP32TOUCH_LVGL_TICK_PERIOD_MS);
}

static void display_lvgl_task(void *arg)
{
    display_lvgl_t *context = (display_lvgl_t *)arg;
    const uint32_t scheduler_period_ms = 1000U / CONFIG_FREERTOS_HZ;

    ESP_LOGI(TAG, "Starting LVGL task");

    for (;;) {
        uint32_t wait_ms = ESP32TOUCH_LVGL_TASK_MAX_DELAY_MS;

        if ((context != NULL) &&
            (context->display != NULL) &&
            (lvgl_lock_acquire(ESP32TOUCH_LVGL_LOCK_TIMEOUT_MS) == ESP_OK)) {
            wait_ms = lv_timer_handler();
            ui_manager_try_load_pending_screen();
            (void)lvgl_lock_release();
        }

        if (wait_ms < scheduler_period_ms) {
            wait_ms = scheduler_period_ms;
        }
        if (wait_ms < ESP32TOUCH_LVGL_TASK_MIN_DELAY_MS) {
            wait_ms = ESP32TOUCH_LVGL_TASK_MIN_DELAY_MS;
        }
        if (wait_ms > ESP32TOUCH_LVGL_TASK_MAX_DELAY_MS) {
            wait_ms = ESP32TOUCH_LVGL_TASK_MAX_DELAY_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

esp_err_t display_lvgl_deinit(display_lvgl_t *lvgl_port)
{
    if (lvgl_port == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (lvgl_port->task_handle != NULL) {
        vTaskDelete(lvgl_port->task_handle);
        lvgl_port->task_handle = NULL;
    }

    if (lvgl_port->tick_timer != NULL) {
        (void)esp_timer_stop(lvgl_port->tick_timer);
        (void)esp_timer_delete(lvgl_port->tick_timer);
        lvgl_port->tick_timer = NULL;
    }

    if (lvgl_port->touch_input_device != NULL) {
        lv_indev_delete(lvgl_port->touch_input_device);
        lvgl_port->touch_input_device = NULL;
    }

    if (lvgl_port->display != NULL) {
        lv_display_delete(lvgl_port->display);
        lvgl_port->display = NULL;
    }

    if (lvgl_port->draw_buffer_primary != NULL) {
        free(lvgl_port->draw_buffer_primary);
        lvgl_port->draw_buffer_primary = NULL;
    }

    if (lvgl_port->draw_buffer_secondary != NULL) {
        free(lvgl_port->draw_buffer_secondary);
        lvgl_port->draw_buffer_secondary = NULL;
    }

    lvgl_port->draw_buffer_size_bytes = 0U;
    lvgl_port->panel_handle = NULL;
    lvgl_port->io_handle = NULL;
    lvgl_port->touch_handle = NULL;
    lvgl_port->is_initialized = false;
    return ESP_OK;
}

esp_err_t display_lvgl_init(const bsp_display_t *bsp_display,
                            const bsp_touch_t *bsp_touch,
                            display_lvgl_t *lvgl_port)
{
    esp_err_t result;
    BaseType_t task_result;
    const esp_timer_create_args_t tick_timer_args = {
        .callback = increase_lvgl_tick,
        .name = "lvgl_tick",
    };
    const esp_lcd_panel_io_callbacks_t panel_callbacks = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };

    if ((bsp_display == NULL) || (lvgl_port == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    *lvgl_port = (display_lvgl_t) {0};
    lvgl_port->panel_handle = bsp_display->panel_handle;
    lvgl_port->io_handle = bsp_display->io_handle;
    lvgl_port->touch_handle = ((bsp_touch != NULL) ? bsp_touch->touch_handle : NULL);
    lvgl_port->applied_rotation = LV_DISPLAY_ROTATION_180;

    result = lvgl_lock_init();
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();

    lvgl_port->display = lv_display_create(ESP32TOUCH_LCD_HORIZONTAL_RESOLUTION,
                                           ESP32TOUCH_LCD_VERTICAL_RESOLUTION);
    if (lvgl_port->display == NULL) {
        return ESP_ERR_NO_MEM;
    }

    lvgl_port->draw_buffer_size_bytes = (size_t)ESP32TOUCH_LCD_HORIZONTAL_RESOLUTION *
                                        (size_t)ESP32TOUCH_LVGL_DRAW_BUFFER_LINE_COUNT *
                                        sizeof(lv_color16_t);
    lvgl_port->draw_buffer_primary = spi_bus_dma_memory_alloc(ESP32TOUCH_LCD_SPI_HOST,
                                                              lvgl_port->draw_buffer_size_bytes,
                                                              0U);
    lvgl_port->draw_buffer_secondary = spi_bus_dma_memory_alloc(ESP32TOUCH_LCD_SPI_HOST,
                                                                lvgl_port->draw_buffer_size_bytes,
                                                                0U);
    /* MISRA boundary note: buffer allocation is constrained to the LVGL/driver integration layer. */
    if ((lvgl_port->draw_buffer_primary == NULL) || (lvgl_port->draw_buffer_secondary == NULL)) {
        (void)display_lvgl_deinit(lvgl_port);
        return ESP_ERR_NO_MEM;
    }

    lv_display_set_buffers(lvgl_port->display,
                           lvgl_port->draw_buffer_primary,
                           lvgl_port->draw_buffer_secondary,
                           lvgl_port->draw_buffer_size_bytes,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_color_format(lvgl_port->display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_user_data(lvgl_port->display, lvgl_port);
    lv_display_set_flush_cb(lvgl_port->display, display_lvgl_flush_cb);

    result = apply_display_rotation(lvgl_port, lvgl_port->display);
    if (result != ESP_OK) {
        (void)display_lvgl_deinit(lvgl_port);
        return result;
    }

    result = esp_timer_create(&tick_timer_args, &lvgl_port->tick_timer);
    if (result != ESP_OK) {
        (void)display_lvgl_deinit(lvgl_port);
        return result;
    }

    result = esp_timer_start_periodic(lvgl_port->tick_timer, ESP32TOUCH_LVGL_TICK_PERIOD_MS * 1000U);
    if (result != ESP_OK) {
        (void)display_lvgl_deinit(lvgl_port);
        return result;
    }

    result = esp_lcd_panel_io_register_event_callbacks(lvgl_port->io_handle,
                                                       &panel_callbacks,
                                                       lvgl_port->display);
    if (result != ESP_OK) {
        (void)display_lvgl_deinit(lvgl_port);
        return result;
    }

    if (lvgl_port->touch_handle != NULL) {
        lvgl_port->touch_input_device = lv_indev_create();
        if (lvgl_port->touch_input_device == NULL) {
            (void)display_lvgl_deinit(lvgl_port);
            return ESP_ERR_NO_MEM;
        }

        lv_indev_set_type(lvgl_port->touch_input_device, LV_INDEV_TYPE_POINTER);
        lv_indev_set_display(lvgl_port->touch_input_device, lvgl_port->display);
        lv_indev_set_user_data(lvgl_port->touch_input_device, lvgl_port);
        lv_indev_set_read_cb(lvgl_port->touch_input_device, display_lvgl_touch_cb);
    }

    task_result = xTaskCreatePinnedToCore(display_lvgl_task,
                                          "lvgl",
                                          ESP32TOUCH_LVGL_TASK_STACK_SIZE_BYTES,
                                          lvgl_port,
                                          ESP32TOUCH_LVGL_TASK_PRIORITY,
                                          &lvgl_port->task_handle,
                                          (BaseType_t)ESP32TOUCH_LVGL_TASK_CORE_ID);
    if (task_result != pdPASS) {
        (void)display_lvgl_deinit(lvgl_port);
        return ESP_FAIL;
    }

    lvgl_port->is_initialized = true;
    return ESP_OK;
}
