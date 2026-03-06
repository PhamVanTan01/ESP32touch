#pragma once

#include <stddef.h>

#include "esp_err.h"

#include "config/hmi_schema.h"

esp_err_t hmi_loader_init(void);
void hmi_loader_deinit(void);
const hmi_system_config_t *hmi_loader_get_config(void);
const char *hmi_loader_get_embedded_schema(size_t *length);
