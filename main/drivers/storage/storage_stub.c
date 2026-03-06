#include "drivers/storage/storage_backend.h"

#include <stdbool.h>
#include <string.h>

enum
{
    STORAGE_STUB_BUFFER_SIZE = 512
};

typedef struct
{
    uint8_t data[STORAGE_STUB_BUFFER_SIZE];
    size_t size;
} storage_stub_slot_t;

static storage_stub_slot_t s_storage_slots[3];
static bool s_storage_initialized;

esp_err_t storage_backend_init(void)
{
    (void)memset(s_storage_slots, 0, sizeof(s_storage_slots));
    s_storage_initialized = true;
    return ESP_OK;
}

void storage_backend_deinit(void)
{
    (void)memset(s_storage_slots, 0, sizeof(s_storage_slots));
    s_storage_initialized = false;
}

esp_err_t storage_backend_write(storage_record_type_t record_type, const void *data, size_t data_size)
{
    if ((record_type > STORAGE_RECORD_SNAPSHOT_META) || (data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_storage_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data_size > STORAGE_STUB_BUFFER_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    (void)memcpy(s_storage_slots[(uint32_t)record_type].data, data, data_size);
    s_storage_slots[(uint32_t)record_type].size = data_size;
    return ESP_OK;
}

esp_err_t storage_backend_read(storage_record_type_t record_type, void *data, size_t buffer_size, size_t *data_size)
{
    size_t slot_size;

    if ((record_type > STORAGE_RECORD_SNAPSHOT_META) || (data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_storage_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    slot_size = s_storage_slots[(uint32_t)record_type].size;
    if (buffer_size < slot_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    (void)memcpy(data, s_storage_slots[(uint32_t)record_type].data, slot_size);
    if (data_size != NULL) {
        *data_size = slot_size;
    }
    return ESP_OK;
}
