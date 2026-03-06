#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "config/hmi_schema.h"

typedef struct
{
    hmi_language_t current_language;
    hmi_role_t current_role;
    hmi_ui_block_t current_screen;
    bool fpga_online;
    bool stm32_online;
    uint32_t active_alarm_count;
} hmi_runtime_state_t;

esp_err_t hmi_state_init(const hmi_system_config_t *config);
void hmi_state_deinit(void);
esp_err_t hmi_state_get(hmi_runtime_state_t *state);
esp_err_t hmi_state_set_language(hmi_language_t language);
esp_err_t hmi_state_set_role(hmi_role_t role);
esp_err_t hmi_state_set_screen(hmi_ui_block_t screen);
esp_err_t hmi_state_set_comm_status(bool fpga_online, bool stm32_online);
