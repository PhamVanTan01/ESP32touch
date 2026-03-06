/*
 * Session manager: inactivity timeout per role (ISO 25119).
 * Timeout and constants from config; no magic numbers (MISRA).
 */

#include "application/session_manager.h"

#include <string.h>

#include "application/auth_manager.h"
#include "config/system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct
{
    const hmi_system_config_t *config;
    session_timeout_callback_t on_timeout;
    uint32_t last_activity_seconds;
    bool initialized;
} session_manager_context_t;

static session_manager_context_t s_session;

static uint32_t get_current_seconds(void)
{
    const uint32_t ticks = (uint32_t)xTaskGetTickCount();
    const uint32_t ms = ticks * (uint32_t)portTICK_PERIOD_MS;
    return ms / 1000U;
}

static uint16_t get_timeout_minutes_for_current_role(void)
{
    uint32_t index;
    hmi_role_t role;
    const hmi_system_config_t *config = s_session.config;

    if (config == NULL) {
        return 0U;
    }

    role = auth_manager_get_role();
    for (index = 0U; index < config->role_count; index++) {
        if (config->role_definitions[index].role_id == role) {
            return config->role_definitions[index].session_timeout_minutes;
        }
    }

    return 0U;
}

esp_err_t session_manager_init(const hmi_system_config_t *config,
                               session_timeout_callback_t on_timeout)
{
    if ((config == NULL) || (on_timeout == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(&s_session, 0, sizeof(s_session));
    s_session.config = config;
    s_session.on_timeout = on_timeout;
    s_session.last_activity_seconds = get_current_seconds();
    s_session.initialized = true;
    return ESP_OK;
}

void session_manager_deinit(void)
{
    (void)memset(&s_session, 0, sizeof(s_session));
}

void session_manager_touch(void)
{
    if (s_session.initialized) {
        s_session.last_activity_seconds = get_current_seconds();
    }
}

void session_manager_tick(void)
{
    uint32_t now_seconds;
    uint32_t elapsed_seconds;
    uint32_t timeout_seconds;
    uint16_t timeout_minutes;

    if (!s_session.initialized || (s_session.on_timeout == NULL)) {
        return;
    }

    if (!auth_manager_is_logged_in()) {
        return;
    }

    timeout_minutes = get_timeout_minutes_for_current_role();
    if (timeout_minutes == 0U) {
        return;
    }

    now_seconds = get_current_seconds();
    elapsed_seconds = now_seconds - s_session.last_activity_seconds;
    timeout_seconds = (uint32_t)timeout_minutes * (uint32_t)SECONDS_PER_MINUTE;

    if (elapsed_seconds >= timeout_seconds) {
        s_session.on_timeout();
    }
}
