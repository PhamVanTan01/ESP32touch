#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    STORAGE_RECORD_AUDIT = 0,
    STORAGE_RECORD_RECIPE,
    STORAGE_RECORD_SNAPSHOT_META
} storage_record_type_t;

esp_err_t storage_backend_init(void);
void storage_backend_deinit(void);
esp_err_t storage_backend_write(storage_record_type_t record_type, const void *data, size_t data_size);
esp_err_t storage_backend_read(storage_record_type_t record_type, void *data, size_t buffer_size, size_t *data_size);
