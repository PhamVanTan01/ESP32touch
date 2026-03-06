#include "middleware/audit/audit_log.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "drivers/storage/storage_backend.h"

enum
{
    AUDIT_LOG_ENTRY_COUNT = 16
};

static audit_log_entry_t s_audit_entries[AUDIT_LOG_ENTRY_COUNT];
static uint8_t s_audit_write_index;
static bool s_audit_initialized;

static void copy_message(char *destination, size_t destination_size, const char *source)
{
    if ((destination != NULL) && (destination_size > 0U) && (source != NULL)) {
        (void)snprintf(destination, destination_size, "%s", source);
    }
}

esp_err_t audit_log_init(void)
{
    esp_err_t result = storage_backend_init();

    if (result != ESP_OK) {
        return result;
    }

    (void)memset(s_audit_entries, 0, sizeof(s_audit_entries));
    s_audit_write_index = 0U;
    s_audit_initialized = true;
    return ESP_OK;
}

void audit_log_deinit(void)
{
    storage_backend_deinit();
    (void)memset(s_audit_entries, 0, sizeof(s_audit_entries));
    s_audit_write_index = 0U;
    s_audit_initialized = false;
}

esp_err_t audit_log_record(audit_event_type_t event_type, const char *message)
{
    if (!s_audit_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_audit_entries[s_audit_write_index].event_type = event_type;
    s_audit_entries[s_audit_write_index].timestamp_seconds += 1UL;
    copy_message(s_audit_entries[s_audit_write_index].message,
                 sizeof(s_audit_entries[s_audit_write_index].message),
                 message);
    s_audit_write_index = (uint8_t)((s_audit_write_index + 1U) % AUDIT_LOG_ENTRY_COUNT);
    return ESP_OK;
}

esp_err_t audit_log_process_pending(void)
{
    if (!s_audit_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return storage_backend_write(STORAGE_RECORD_AUDIT, s_audit_entries, sizeof(s_audit_entries));
}

esp_err_t audit_log_export_to_buffer(char *buf, size_t buf_size, size_t *out_len)
{
    size_t written = 0U;
    uint8_t i;

    if ((buf == NULL) || (out_len == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0U;
    if (!s_audit_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (buf_size == 0U) {
        return ESP_OK;
    }
    buf[0] = '\0';

    for (i = 0U; i < AUDIT_LOG_ENTRY_COUNT; i++) {
        const uint8_t idx = (uint8_t)((s_audit_write_index + (uint8_t)i) % AUDIT_LOG_ENTRY_COUNT);
        const audit_log_entry_t *e = &s_audit_entries[idx];
        int n;

        if (e->message[0] == '\0' && e->event_type == 0U && e->timestamp_seconds == 0U) {
            continue;
        }
        n = snprintf(buf + written, buf_size - written, "%lu,%u,%s\n",
                     (unsigned long)e->timestamp_seconds,
                     (unsigned)e->event_type,
                     e->message);
        if (n < 0) {
            return ESP_ERR_INVALID_STATE;
        }
        if ((size_t)n >= buf_size - written) {
            written = buf_size - 1U;
            buf[written] = '\0';
            break;
        }
        written += (size_t)n;
    }
    *out_len = written;
    return ESP_OK;
}
