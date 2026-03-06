#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "application/alarm_config.h"

/** Single alarm entry for list. */
typedef struct
{
    uint32_t id;
    alarm_severity_t severity;
    char message[48];
    bool acked;
} alarm_entry_t;

/** Callback when a critical alarm is added (for popup). */
typedef void (*alarm_critical_callback_t)(uint32_t id, const char *message);

esp_err_t alarm_manager_init(alarm_critical_callback_t on_critical);
void alarm_manager_deinit(void);
esp_err_t alarm_manager_add(uint32_t id, alarm_severity_t severity, const char *message);
esp_err_t alarm_manager_ack(uint32_t id);
uint32_t alarm_manager_get_active_count(alarm_severity_t severity);
esp_err_t alarm_manager_get_list(alarm_entry_t *out, size_t max_count, size_t *out_count);
