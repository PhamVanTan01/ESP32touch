#include "config/hmi_loader.h"

#include <stdbool.h>
#include <stddef.h>

extern const char g_hmi_embedded_schema_json[];
extern const size_t g_hmi_embedded_schema_json_length;

static bool s_hmi_loader_initialized;

esp_err_t hmi_loader_init(void)
{
    /* MISRA note: keep schema in flash as const data to avoid runtime file I/O on target. */
    s_hmi_loader_initialized = true;
    return ESP_OK;
}

void hmi_loader_deinit(void)
{
    s_hmi_loader_initialized = false;
}

const hmi_system_config_t *hmi_loader_get_config(void)
{
    return s_hmi_loader_initialized ? hmi_schema_get_default() : NULL;
}

const char *hmi_loader_get_embedded_schema(size_t *length)
{
    if (length != NULL) {
        *length = g_hmi_embedded_schema_json_length;
    }

    return s_hmi_loader_initialized ? g_hmi_embedded_schema_json : NULL;
}
