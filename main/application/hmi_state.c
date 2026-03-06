#include "application/hmi_state.h"

#include <stdbool.h>
#include <string.h>

typedef struct
{
    hmi_runtime_state_t runtime_state;
    bool is_initialized;
} hmi_state_context_t;

static hmi_state_context_t s_hmi_state;

esp_err_t hmi_state_init(const hmi_system_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(&s_hmi_state, 0, sizeof(s_hmi_state));
    s_hmi_state.runtime_state.current_language = config->language_support.default_language;
    s_hmi_state.runtime_state.current_role = HMI_ROLE_OPERATOR;
    s_hmi_state.runtime_state.current_screen = HMI_UI_BLOCK_DASHBOARD;
    s_hmi_state.is_initialized = true;
    return ESP_OK;
}

void hmi_state_deinit(void)
{
    (void)memset(&s_hmi_state, 0, sizeof(s_hmi_state));
}

esp_err_t hmi_state_get(hmi_runtime_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_hmi_state.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *state = s_hmi_state.runtime_state;
    return ESP_OK;
}

esp_err_t hmi_state_set_language(hmi_language_t language)
{
    if (!s_hmi_state.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    s_hmi_state.runtime_state.current_language = language;
    return ESP_OK;
}

esp_err_t hmi_state_set_role(hmi_role_t role)
{
    if (!s_hmi_state.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    s_hmi_state.runtime_state.current_role = role;
    return ESP_OK;
}

esp_err_t hmi_state_set_screen(hmi_ui_block_t screen)
{
    if (!s_hmi_state.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    s_hmi_state.runtime_state.current_screen = screen;
    return ESP_OK;
}

esp_err_t hmi_state_set_comm_status(bool fpga_online, bool stm32_online)
{
    if (!s_hmi_state.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    s_hmi_state.runtime_state.fpga_online = fpga_online;
    s_hmi_state.runtime_state.stm32_online = stm32_online;
    return ESP_OK;
}
