/*
 * Alarm manager: ISA-18.2 severity, list, ack, event bus (Phase 3.1).
 */

#include "application/alarm_manager.h"

#include <stdio.h>
#include <string.h>

#include "application/event_bus.h"
#include "esp_log.h"
#include "middleware/audit/audit_log.h"

static const char *TAG = "alarm_mgr";

typedef struct
{
    alarm_entry_t entries[ALARM_MAX_ACTIVE];
    size_t count;
    alarm_critical_callback_t on_critical;
    bool initialized;
} alarm_manager_context_t;

static alarm_manager_context_t s_alarm;

esp_err_t alarm_manager_init(alarm_critical_callback_t on_critical)
{
    (void)memset(&s_alarm, 0, sizeof(s_alarm));
    s_alarm.on_critical = on_critical;
    s_alarm.initialized = true;
    ESP_LOGI(TAG, "Initialized, max=%u", (unsigned)ALARM_MAX_ACTIVE);
    return ESP_OK;
}

void alarm_manager_deinit(void)
{
    (void)memset(&s_alarm, 0, sizeof(s_alarm));
}

static alarm_entry_t *find_alarm(uint32_t id)
{
    size_t i;

    for (i = 0U; i < s_alarm.count; i++) {
        if (s_alarm.entries[i].id == id) {
            return &s_alarm.entries[i];
        }
    }
    return NULL;
}

esp_err_t alarm_manager_add(uint32_t id, alarm_severity_t severity, const char *message)
{
    app_event_t event;
    alarm_entry_t *slot;
    size_t copy_len;
    bool is_new_critical = false;

    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_alarm.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    slot = find_alarm(id);
    if (slot != NULL) {
        /* Update existing: severity/message, keep acked state */
        slot->severity = severity;
        copy_len = sizeof(slot->message) - 1U;
        (void)strncpy(slot->message, message, copy_len);
        slot->message[copy_len] = '\0';
    } else {
        if (s_alarm.count >= ALARM_MAX_ACTIVE) {
            ESP_LOGW(TAG, "Alarm list full, drop id=%lu", (unsigned long)id);
            return ESP_ERR_NO_MEM;
        }
        slot = &s_alarm.entries[s_alarm.count];
        slot->id = id;
        slot->severity = severity;
        copy_len = sizeof(slot->message) - 1U;
        (void)strncpy(slot->message, message, copy_len);
        slot->message[copy_len] = '\0';
        slot->acked = false;
        s_alarm.count++;
        if (severity == ALARM_SEVERITY_CRITICAL) {
            is_new_critical = true;
        }
    }

    event.event_type = APP_EVENT_ALARM_UPDATED;
    event.value = id;
    (void)event_bus_publish(&event);

    if (is_new_critical && (s_alarm.on_critical != NULL)) {
        s_alarm.on_critical(id, slot->message);
    }
    return ESP_OK;
}

esp_err_t alarm_manager_ack(uint32_t id)
{
    alarm_entry_t *slot;
    char msg[64];

    if (!s_alarm.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    slot = find_alarm(id);
    if (slot == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    slot->acked = true;
    (void)snprintf(msg, sizeof(msg), "alarm id=%lu acked", (unsigned long)id);
    (void)audit_log_record(AUDIT_EVENT_ALARM_ACK, msg);

    {
        app_event_t event;
        event.event_type = APP_EVENT_ALARM_UPDATED;
        event.value = id;
        (void)event_bus_publish(&event);
    }
    return ESP_OK;
}

uint32_t alarm_manager_get_active_count(alarm_severity_t severity)
{
    uint32_t n = 0U;
    size_t i;

    for (i = 0U; i < s_alarm.count; i++) {
        if (!s_alarm.entries[i].acked && s_alarm.entries[i].severity == severity) {
            n++;
        }
    }
    return n;
}

esp_err_t alarm_manager_get_list(alarm_entry_t *out, size_t max_count, size_t *out_count)
{
    size_t i;
    size_t n = 0U;

    if (out == NULL || out_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_count = 0U;
    for (i = 0U; i < s_alarm.count && n < max_count; i++) {
        out[n] = s_alarm.entries[i];
        n++;
    }
    *out_count = n;
    return ESP_OK;
}
