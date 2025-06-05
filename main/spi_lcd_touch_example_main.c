
/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

 #include <stdio.h>
 #include <unistd.h>
 #include <sys/lock.h>
 #include <sys/param.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_timer.h"
 #include "esp_lcd_panel_io.h"
 #include "esp_lcd_panel_vendor.h"
 #include "esp_lcd_panel_ops.h"
 #include "driver/gpio.h"
 #include "driver/spi_master.h"
 #include "esp_err.h"
 #include "esp_log.h"
 #include "lvgl.h"
 

 #include "esp_lcd_ili9341.h"

 #include "esp_lcd_touch_xpt2046.h"

 
 static const char *TAG = "example";
 
 // Using SPI2 in the example
 #define LCD_HOST  SPI2_HOST
 #define TOUCH_HOST SPI3_HOST // Sử dụng SPI3 cho touch controller
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 #define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
 #define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
 #define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
 #define EXAMPLE_PIN_NUM_SCLK           18
 #define EXAMPLE_PIN_NUM_MOSI           19
 #define EXAMPLE_PIN_NUM_MISO           21
 #define EXAMPLE_PIN_NUM_LCD_DC         5
 #define EXAMPLE_PIN_NUM_LCD_RST        3
 #define EXAMPLE_PIN_NUM_LCD_CS         4
 #define EXAMPLE_PIN_NUM_BK_LIGHT       2
 #define EXAMPLE_PIN_NUM_TOUCH_CS       15
 #define EXAMPLE_PIN_NUM_TOUCH_SCLK  37 // GPIO cho SCLK của touch
 #define EXAMPLE_PIN_NUM_TOUCH_MOSI  38 // GPIO cho MOSI của touch
 #define EXAMPLE_PIN_NUM_TOUCH_MISO  39 // GPIO cho MISO của touch
 #define EXAMPLE_PIN_NUM_TOUCH_IRQ   47 // GPIO cho IRQ của touch (nếu có)
 // The pixel number in horizontal and vertical

 #define EXAMPLE_LCD_H_RES              240
 #define EXAMPLE_LCD_V_RES              320

 // Bit number used to represent command and parameter
 #define EXAMPLE_LCD_CMD_BITS           8
 #define EXAMPLE_LCD_PARAM_BITS         8
 
 #define EXAMPLE_LVGL_DRAW_BUF_LINES    20 // number of display lines in each draw buffer
 #define EXAMPLE_LVGL_TICK_PERIOD_MS    2
 #define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
 #define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
 #define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
 #define EXAMPLE_LVGL_TASK_PRIORITY     2
 
 // LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
 static _lock_t lvgl_api_lock;
 
 extern void example_lvgl_demo_ui(lv_disp_t *disp);
 
 static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
 {
     lv_display_t *disp = (lv_display_t *)user_ctx;
     lv_display_flush_ready(disp);
     return false;
 }
 
 /* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
 static void example_lvgl_port_update_callback(lv_display_t *disp)
 {
     esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
     lv_display_rotation_t rotation = lv_display_get_rotation(disp);
 
     switch (rotation) {
     case LV_DISPLAY_ROTATION_0:
         // Rotate LCD display
         esp_lcd_panel_swap_xy(panel_handle, false);
         esp_lcd_panel_mirror(panel_handle, true, false);
         break;
     case LV_DISPLAY_ROTATION_90:
         // Rotate LCD display
         esp_lcd_panel_swap_xy(panel_handle, true);
         esp_lcd_panel_mirror(panel_handle, true, true);
         break;
     case LV_DISPLAY_ROTATION_180:
         // Rotate LCD display
         esp_lcd_panel_swap_xy(panel_handle, false);
         esp_lcd_panel_mirror(panel_handle, false, true);
         break;
     case LV_DISPLAY_ROTATION_270:
         // Rotate LCD display
         esp_lcd_panel_swap_xy(panel_handle, true);
         esp_lcd_panel_mirror(panel_handle, false, false);
         break;
     }
 }
 
 static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
 {
     example_lvgl_port_update_callback(disp);
     esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
     int offsetx1 = area->x1;
     int offsetx2 = area->x2;
     int offsety1 = area->y1;
     int offsety2 = area->y2;
     // because SPI LCD is big-endian, we need to swap the RGB bytes order
     lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
     // copy a buffer's content to a specific area of the display
     esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
 }
 

 static void example_lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
 {
     uint16_t touchpad_x[1] = {0};
     uint16_t touchpad_y[1] = {0};
     uint8_t touchpad_cnt = 0;
 
     esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
     esp_lcd_touch_read_data(touch_pad);
     /* Get coordinates */
     bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
 
     if (touchpad_pressed && touchpad_cnt > 0) {
         data->point.x = touchpad_x[0];
         data->point.y = touchpad_y[0];
         data->state = LV_INDEV_STATE_PRESSED;
     } else {
         data->state = LV_INDEV_STATE_RELEASED;
     }
 }

 
 static void example_increase_lvgl_tick(void *arg)
 {
     /* Tell LVGL how many milliseconds has elapsed */
     lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
 }
 
 static void example_lvgl_port_task(void *arg)
 {
     ESP_LOGI(TAG, "Starting LVGL task");
     uint32_t time_till_next_ms = 0;
     uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
     while (1) {
         _lock_acquire(&lvgl_api_lock);
         time_till_next_ms = lv_timer_handler();
         _lock_release(&lvgl_api_lock);
         // in case of triggering a task watch dog time out
         time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
         usleep(1000 * time_till_next_ms);
     }
 }
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static lv_display_t *display = NULL;

static void init_backlight(void)
{
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);
}

static void init_spi_lcd_bus(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus for LCD");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}
static void init_lcd_panel(void)
{
    ESP_LOGI(TAG, "Install LCD panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install ILI9341 driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    // 🔥🔥 Required init/reset sequence 🔥🔥
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // 💡 Now turn on backlight after panel is ready
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
}

static void init_lvgl_display(void)
{
    ESP_LOGI(TAG, "Initialize LVGL and create display");

    lv_init();
    display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    size_t draw_buf_size = EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buf_size, 0);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buf_size, 0);
    assert(buf1 && buf2);

    lv_display_set_buffers(display, buf1, buf2, draw_buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(display, panel_handle);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));
}

static void start_lvgl_tick_timer(void)
{
    ESP_LOGI(TAG, "Start LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));
}


static void init_touch_controller(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus for touch");
    spi_bus_config_t touch_buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_TOUCH_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_TOUCH_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_TOUCH_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_HOST, &touch_buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config =

        ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(EXAMPLE_PIN_NUM_TOUCH_CS);

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = CONFIG_EXAMPLE_LCD_MIRROR_Y,
        },
    };
    esp_lcd_touch_handle_t tp = NULL;


    ESP_LOGI(TAG, "Init XPT2046");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));


    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, tp);
    lv_indev_set_read_cb(indev, example_lvgl_touch_cb);
}


static void start_lvgl_task(void)
{
    ESP_LOGI(TAG, "Start LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
}

static void start_demo_ui(void)
{
    ESP_LOGI(TAG, "Show demo UI");
    _lock_acquire(&lvgl_api_lock);
    example_lvgl_demo_ui(display);
    _lock_release(&lvgl_api_lock);
}
void app_main(void)
{
    init_backlight();
    init_spi_lcd_bus();
    init_lcd_panel();

    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    init_lvgl_display();
    start_lvgl_tick_timer();


    init_touch_controller();


    start_lvgl_task();
    start_demo_ui();
}
