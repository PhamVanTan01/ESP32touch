#include "drivers/fpga/fpga_interface.h"

#include <string.h>

static fpga_status_snapshot_t s_fpga_status;
static bool s_fpga_initialized;

esp_err_t fpga_interface_init(void)
{
    (void)memset(&s_fpga_status, 0, sizeof(s_fpga_status));
    s_fpga_status.online = false;
    s_fpga_status.node_count = 4U;
    s_fpga_initialized = true;
    return ESP_OK;
}

void fpga_interface_deinit(void)
{
    (void)memset(&s_fpga_status, 0, sizeof(s_fpga_status));
    s_fpga_initialized = false;
}

esp_err_t fpga_interface_poll(void)
{
    if (!s_fpga_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_fpga_status.online = true;
    s_fpga_status.accepted_count++;
    return ESP_OK;
}

esp_err_t fpga_interface_get_status(fpga_status_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_fpga_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *snapshot = s_fpga_status;
    return ESP_OK;
}
