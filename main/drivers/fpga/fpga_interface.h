#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct
{
    bool online;
    uint8_t node_count;
    uint16_t rejected_count;
    uint16_t accepted_count;
} fpga_status_snapshot_t;

esp_err_t fpga_interface_init(void);
void fpga_interface_deinit(void);
esp_err_t fpga_interface_poll(void);
esp_err_t fpga_interface_get_status(fpga_status_snapshot_t *snapshot);
