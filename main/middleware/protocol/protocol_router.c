#include "middleware/protocol/protocol_router.h"

#include "drivers/fpga/fpga_interface.h"
#include "drivers/stm32/stm32_link.h"

static protocol_status_t s_protocol_status;
static bool s_protocol_initialized;

esp_err_t protocol_router_init(void)
{
    esp_err_t result;

    result = fpga_interface_init();
    if (result != ESP_OK) {
        return result;
    }

    result = stm32_link_init();
    if (result != ESP_OK) {
        fpga_interface_deinit();
        return result;
    }

    s_protocol_status = (protocol_status_t){0};
    s_protocol_initialized = true;
    return ESP_OK;
}

void protocol_router_deinit(void)
{
    stm32_link_deinit();
    fpga_interface_deinit();
    s_protocol_status = (protocol_status_t){0};
    s_protocol_initialized = false;
}

esp_err_t protocol_router_poll_once(void)
{
    fpga_status_snapshot_t fpga_status;
    stm32_status_snapshot_t stm32_status;
    esp_err_t result;

    if (!s_protocol_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    result = fpga_interface_poll();
    if (result != ESP_OK) {
        return result;
    }

    result = stm32_link_poll();
    if (result != ESP_OK) {
        return result;
    }

    result = fpga_interface_get_status(&fpga_status);
    if (result != ESP_OK) {
        return result;
    }

    result = stm32_link_get_status(&stm32_status);
    if (result != ESP_OK) {
        return result;
    }

    s_protocol_status.fpga_online = fpga_status.online;
    s_protocol_status.stm32_online = stm32_status.online;
    return ESP_OK;
}

esp_err_t protocol_router_get_status(protocol_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_protocol_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *status = s_protocol_status;
    return ESP_OK;
}
