#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t lvgl_lock_init(void);
esp_err_t lvgl_lock_acquire(uint32_t timeout_ms);
esp_err_t lvgl_lock_release(void);
