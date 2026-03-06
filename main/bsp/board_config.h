#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_common.h"

enum
{
    ESP32TOUCH_LCD_PIXEL_CLOCK_HZ = 20000000,
    ESP32TOUCH_LCD_HORIZONTAL_RESOLUTION = 240,
    ESP32TOUCH_LCD_VERTICAL_RESOLUTION = 320,
    ESP32TOUCH_LCD_COMMAND_BITS = 8,
    ESP32TOUCH_LCD_PARAMETER_BITS = 8,
    ESP32TOUCH_LCD_BITS_PER_PIXEL = 16,
    ESP32TOUCH_LCD_TRANSFER_LINE_COUNT = 80,
    ESP32TOUCH_LCD_TRANSACTION_QUEUE_DEPTH = 10,
    ESP32TOUCH_TOUCH_TRANSFER_SIZE_BYTES = 0,
    ESP32TOUCH_GPIO_UNUSED = -1
};

static const spi_host_device_t ESP32TOUCH_LCD_SPI_HOST = SPI2_HOST;
static const spi_host_device_t ESP32TOUCH_TOUCH_SPI_HOST = SPI3_HOST;

static const uint32_t ESP32TOUCH_LCD_BACKLIGHT_ON_LEVEL = 1U;
static const uint32_t ESP32TOUCH_LCD_BACKLIGHT_OFF_LEVEL = 0U;

static const gpio_num_t ESP32TOUCH_PIN_LCD_SCLK = GPIO_NUM_18;
static const gpio_num_t ESP32TOUCH_PIN_LCD_MOSI = GPIO_NUM_19;
static const gpio_num_t ESP32TOUCH_PIN_LCD_MISO = GPIO_NUM_21;
static const gpio_num_t ESP32TOUCH_PIN_LCD_DC = GPIO_NUM_5;
static const gpio_num_t ESP32TOUCH_PIN_LCD_RST = GPIO_NUM_3;
static const gpio_num_t ESP32TOUCH_PIN_LCD_CS = GPIO_NUM_4;
static const gpio_num_t ESP32TOUCH_PIN_LCD_BK_LIGHT = GPIO_NUM_2;

static const gpio_num_t ESP32TOUCH_PIN_TOUCH_CS = GPIO_NUM_15;
static const gpio_num_t ESP32TOUCH_PIN_TOUCH_SCLK = GPIO_NUM_37;
static const gpio_num_t ESP32TOUCH_PIN_TOUCH_MOSI = GPIO_NUM_38;
static const gpio_num_t ESP32TOUCH_PIN_TOUCH_MISO = GPIO_NUM_39;
static const gpio_num_t ESP32TOUCH_PIN_TOUCH_IRQ = GPIO_NUM_47;
