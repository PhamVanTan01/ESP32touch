#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    AUDIT_EVENT_LOGIN = 0,
    AUDIT_EVENT_LOGOUT,
    AUDIT_EVENT_RECIPE_CHANGE,
    AUDIT_EVENT_CONFIG_CHANGE,
    AUDIT_EVENT_ALARM_ACK,
    AUDIT_EVENT_CALIBRATION,
    AUDIT_EVENT_SHUTDOWN
} audit_event_type_t;

typedef struct
{
    audit_event_type_t event_type;
    uint32_t timestamp_seconds;
    char message[64];
} audit_log_entry_t;

esp_err_t audit_log_init(void);
void audit_log_deinit(void);
esp_err_t audit_log_record(audit_event_type_t event_type, const char *message);
esp_err_t audit_log_process_pending(void);

/** Export entries as text lines to buffer (Phase 6.2). Format: "timestamp,event_id,message\n". */
esp_err_t audit_log_export_to_buffer(char *buf, size_t buf_size, size_t *out_len);
