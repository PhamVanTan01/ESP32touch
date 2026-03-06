#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct
{
    uint32_t free_heap_bytes;
    uint32_t minimum_heap_bytes;
    uint8_t cpu_load_percent;
    bool heap_warning;
} resource_monitor_snapshot_t;

esp_err_t resource_monitor_init(void);
void resource_monitor_deinit(void);
esp_err_t resource_monitor_refresh(void);
esp_err_t resource_monitor_get_snapshot(resource_monitor_snapshot_t *snapshot);
