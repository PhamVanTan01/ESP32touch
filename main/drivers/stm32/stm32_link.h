#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct
{
    bool online;
    uint32_t baud_rate;
    uint16_t active_alarm_count;
} stm32_status_snapshot_t;

esp_err_t stm32_link_init(void);
void stm32_link_deinit(void);
esp_err_t stm32_link_poll(void);
esp_err_t stm32_link_get_status(stm32_status_snapshot_t *snapshot);
