#include "drivers/stm32/stm32_link.h"

#include <string.h>

static stm32_status_snapshot_t s_stm32_status;
static bool s_stm32_initialized;

esp_err_t stm32_link_init(void)
{
    (void)memset(&s_stm32_status, 0, sizeof(s_stm32_status));
    s_stm32_status.baud_rate = 115200UL;
    s_stm32_initialized = true;
    return ESP_OK;
}

void stm32_link_deinit(void)
{
    (void)memset(&s_stm32_status, 0, sizeof(s_stm32_status));
    s_stm32_initialized = false;
}

esp_err_t stm32_link_poll(void)
{
    if (!s_stm32_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_stm32_status.online = true;
    return ESP_OK;
}

esp_err_t stm32_link_get_status(stm32_status_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_stm32_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *snapshot = s_stm32_status;
    return ESP_OK;
}
