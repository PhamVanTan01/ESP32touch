#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    APP_EVENT_NONE = 0,
    APP_EVENT_COMM_STATUS_UPDATED,
    APP_EVENT_ALARM_UPDATED,
    APP_EVENT_LOGIN_COMPLETED,
    APP_EVENT_LOG_FLUSH_REQUEST
} app_event_type_t;

typedef struct
{
    app_event_type_t event_type;
    uint32_t value;
} app_event_t;

esp_err_t event_bus_init(void);
void event_bus_deinit(void);
esp_err_t event_bus_publish(const app_event_t *event);
esp_err_t event_bus_receive(app_event_t *event, uint32_t timeout_ms);
