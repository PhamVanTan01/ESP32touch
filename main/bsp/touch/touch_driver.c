#include "bsp/touch/touch_driver.h"

#include "driver/spi_master.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_log.h"

#include "bsp/board_config.h"

static const char *TAG = "touch_driver";

esp_err_t bsp_touch_deinit(bsp_touch_t *touch)
{
    if (touch == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (touch->touch_handle != NULL) {
        (void)esp_lcd_touch_del(touch->touch_handle);
        touch->touch_handle = NULL;
    }

    if (touch->io_handle != NULL) {
        (void)esp_lcd_panel_io_del(touch->io_handle);
        touch->io_handle = NULL;
    }

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED && CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_XPT2046
    (void)spi_bus_free(ESP32TOUCH_TOUCH_SPI_HOST);
#endif
    touch->is_initialized = false;
    return ESP_OK;
}

esp_err_t bsp_touch_init(bsp_touch_t *touch)
{
    esp_err_t result;

    if (touch == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *touch = (bsp_touch_t) {0};

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED && CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_XPT2046
    ESP_LOGI(TAG, "Initialize touch SPI bus");
    const spi_bus_config_t touch_bus_config = {
        .sclk_io_num = ESP32TOUCH_PIN_TOUCH_SCLK,
        .mosi_io_num = ESP32TOUCH_PIN_TOUCH_MOSI,
        .miso_io_num = ESP32TOUCH_PIN_TOUCH_MISO,
        .quadwp_io_num = ESP32TOUCH_GPIO_UNUSED,
        .quadhd_io_num = ESP32TOUCH_GPIO_UNUSED,
        .max_transfer_sz = ESP32TOUCH_TOUCH_TRANSFER_SIZE_BYTES,
    };
    result = spi_bus_initialize(ESP32TOUCH_TOUCH_SPI_HOST, &touch_bus_config, SPI_DMA_CH_AUTO);
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Install touch panel IO");
    const esp_lcd_panel_io_spi_config_t touch_io_config =
        ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(ESP32TOUCH_PIN_TOUCH_CS);
    result = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ESP32TOUCH_TOUCH_SPI_HOST,
                                      &touch_io_config,
                                      &touch->io_handle);
    if (result != ESP_OK) {
        (void)bsp_touch_deinit(touch);
        return result;
    }

    const esp_lcd_touch_config_t touch_config = {
        .x_max = ESP32TOUCH_LCD_HORIZONTAL_RESOLUTION,
        .y_max = ESP32TOUCH_LCD_VERTICAL_RESOLUTION,
        .rst_gpio_num = ESP32TOUCH_GPIO_UNUSED,
        .int_gpio_num = ESP32TOUCH_GPIO_UNUSED,
        .flags = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = CONFIG_EXAMPLE_LCD_MIRROR_Y,
        },
    };

    ESP_LOGI(TAG, "Initialize XPT2046 touch controller");
    result = esp_lcd_touch_new_spi_xpt2046(touch->io_handle, &touch_config, &touch->touch_handle);
    if (result != ESP_OK) {
        (void)bsp_touch_deinit(touch);
        return result;
    }

    touch->is_initialized = true;
#else
    ESP_LOGI(TAG, "Touch disabled by configuration");
#endif

    return ESP_OK;
}
