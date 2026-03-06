#include "bsp/display/display_driver.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_ili9341.h"
#include "esp_log.h"

#include "bsp/board_config.h"

static const char *TAG = "display_driver";

static esp_err_t backlight_gpio_init(void)
{
    const gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << (uint32_t)ESP32TOUCH_PIN_LCD_BK_LIGHT),
    };

    return gpio_config(&bk_gpio_config);
}

esp_err_t bsp_display_backlight_set(bool enable)
{
    return gpio_set_level(ESP32TOUCH_PIN_LCD_BK_LIGHT,
                          enable ? (uint32_t)ESP32TOUCH_LCD_BACKLIGHT_ON_LEVEL
                                 : (uint32_t)ESP32TOUCH_LCD_BACKLIGHT_OFF_LEVEL);
}

esp_err_t bsp_display_deinit(bsp_display_t *display)
{
    if (display == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (display->panel_handle != NULL) {
        (void)esp_lcd_panel_disp_on_off(display->panel_handle, false);
        (void)esp_lcd_panel_del(display->panel_handle);
        display->panel_handle = NULL;
    }

    if (display->io_handle != NULL) {
        (void)esp_lcd_panel_io_del(display->io_handle);
        display->io_handle = NULL;
    }

    (void)spi_bus_free(ESP32TOUCH_LCD_SPI_HOST);
    display->is_initialized = false;
    return bsp_display_backlight_set(false);
}

esp_err_t bsp_display_init(bsp_display_t *display)
{
    esp_err_t result;

    if (display == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *display = (bsp_display_t) {0};

    ESP_LOGI(TAG, "Initialize display backlight");
    result = backlight_gpio_init();
    if (result != ESP_OK) {
        return result;
    }

    result = bsp_display_backlight_set(false);
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Initialize LCD SPI bus");
    const spi_bus_config_t bus_config = {
        .sclk_io_num = ESP32TOUCH_PIN_LCD_SCLK,
        .mosi_io_num = ESP32TOUCH_PIN_LCD_MOSI,
        .miso_io_num = ESP32TOUCH_PIN_LCD_MISO,
        .quadwp_io_num = ESP32TOUCH_GPIO_UNUSED,
        .quadhd_io_num = ESP32TOUCH_GPIO_UNUSED,
        .max_transfer_sz = ESP32TOUCH_LCD_HORIZONTAL_RESOLUTION *
                           ESP32TOUCH_LCD_TRANSFER_LINE_COUNT *
                           (int)sizeof(uint16_t),
    };
    result = spi_bus_initialize(ESP32TOUCH_LCD_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = ESP32TOUCH_PIN_LCD_DC,
        .cs_gpio_num = ESP32TOUCH_PIN_LCD_CS,
        .pclk_hz = ESP32TOUCH_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = ESP32TOUCH_LCD_COMMAND_BITS,
        .lcd_param_bits = ESP32TOUCH_LCD_PARAMETER_BITS,
        .spi_mode = 0,
        .trans_queue_depth = ESP32TOUCH_LCD_TRANSACTION_QUEUE_DEPTH,
    };
    result = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ESP32TOUCH_LCD_SPI_HOST,
                                      &io_config,
                                      &display->io_handle);
    if (result != ESP_OK) {
        (void)spi_bus_free(ESP32TOUCH_LCD_SPI_HOST);
        return result;
    }

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = ESP32TOUCH_PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = ESP32TOUCH_LCD_BITS_PER_PIXEL,
    };
    result = esp_lcd_new_panel_ili9341(display->io_handle, &panel_config, &display->panel_handle);
    if (result != ESP_OK) {
        (void)bsp_display_deinit(display);
        return result;
    }

    result = esp_lcd_panel_reset(display->panel_handle);
    if (result != ESP_OK) {
        (void)bsp_display_deinit(display);
        return result;
    }

    result = esp_lcd_panel_init(display->panel_handle);
    if (result != ESP_OK) {
        (void)bsp_display_deinit(display);
        return result;
    }

    result = esp_lcd_panel_mirror(display->panel_handle, true, false);
    if (result != ESP_OK) {
        (void)bsp_display_deinit(display);
        return result;
    }

    result = esp_lcd_panel_disp_on_off(display->panel_handle, true);
    if (result != ESP_OK) {
        (void)bsp_display_deinit(display);
        return result;
    }

    result = bsp_display_backlight_set(true);
    if (result != ESP_OK) {
        (void)bsp_display_deinit(display);
        return result;
    }

    display->is_initialized = true;
    return ESP_OK;
}
