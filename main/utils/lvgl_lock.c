#include "utils/lvgl_lock.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lvgl_mutex;
static StaticSemaphore_t s_lvgl_mutex_buffer;

esp_err_t lvgl_lock_init(void)
{
    if (s_lvgl_mutex == NULL) {
        // LVGL input callbacks can synchronously call code paths that re-enter UI APIs.
        // A recursive mutex avoids self-deadlock when the same task acquires the lock again.
        s_lvgl_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_lvgl_mutex_buffer);
    }

    return (s_lvgl_mutex != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t lvgl_lock_acquire(uint32_t timeout_ms)
{
    if (s_lvgl_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return (xSemaphoreTakeRecursive(s_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t lvgl_lock_release(void)
{
    if (s_lvgl_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return (xSemaphoreGiveRecursive(s_lvgl_mutex) == pdTRUE) ? ESP_OK : ESP_FAIL;
}
