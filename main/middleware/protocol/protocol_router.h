#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct
{
    bool fpga_online;
    bool stm32_online;
} protocol_status_t;

esp_err_t protocol_router_init(void);
void protocol_router_deinit(void);
esp_err_t protocol_router_poll_once(void);
esp_err_t protocol_router_get_status(protocol_status_t *status);
