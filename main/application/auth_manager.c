#include "application/auth_manager.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "config/auth_demo_config.h"
#include "config/system_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "middleware/audit/audit_log.h"

static const char *TAG = "auth_manager";
static const char *FALLBACK_ROLE_NAME = "?";

typedef struct
{
    const hmi_system_config_t *config;
    hmi_role_t current_role;
    bool logged_in;
    uint32_t failed_attempts;
    uint32_t lockout_until_seconds;
} auth_manager_context_t;

static auth_manager_context_t s_auth_manager;

static uint32_t get_current_seconds(void)
{
    const uint32_t ticks = (uint32_t)xTaskGetTickCount();
    const uint32_t ms = ticks * (uint32_t)portTICK_PERIOD_MS;
    return ms / 1000U;
}

esp_err_t auth_manager_init(const hmi_system_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(&s_auth_manager, 0, sizeof(s_auth_manager));
    s_auth_manager.config = config;
    s_auth_manager.current_role = AUTH_DEMO_DEFAULT_ROLE;
    s_auth_manager.logged_in = false;
    ESP_LOGI(TAG, "Initialized with default role=%d", (int)s_auth_manager.current_role);
    return ESP_OK;
}

void auth_manager_deinit(void)
{
    (void)memset(&s_auth_manager, 0, sizeof(s_auth_manager));
}

esp_err_t auth_manager_login_password(const char *user_name, const char *password, hmi_role_t requested_role)
{
    const uint32_t now_seconds = get_current_seconds();

    if ((user_name == NULL) || (password == NULL) || (s_auth_manager.config == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Clear expired lockout */
    if ((s_auth_manager.lockout_until_seconds != 0U) && (now_seconds >= s_auth_manager.lockout_until_seconds)) {
        s_auth_manager.lockout_until_seconds = 0U;
        s_auth_manager.failed_attempts = 0U;
    }

    if ((s_auth_manager.lockout_until_seconds != 0U) && (now_seconds < s_auth_manager.lockout_until_seconds)) {
        return ESP_ERR_INVALID_STATE;
    }

    if ((strcmp(user_name, AUTH_DEMO_USERNAME) != 0) || (strcmp(password, AUTH_DEMO_PASSWORD) != 0)) {
        s_auth_manager.failed_attempts++;
        if (s_auth_manager.failed_attempts >= AUTH_MAX_FAILED_ATTEMPTS) {
            s_auth_manager.lockout_until_seconds = now_seconds
                + ((uint32_t)AUTH_LOCKOUT_DURATION_MINUTES * (uint32_t)SECONDS_PER_MINUTE);
            ESP_LOGW(TAG, "Login locked out for %u min after %u failed attempts",
                     (unsigned)AUTH_LOCKOUT_DURATION_MINUTES, (unsigned)s_auth_manager.failed_attempts);
        }
        return ESP_ERR_INVALID_STATE;
    }

    (void)requested_role;
    s_auth_manager.failed_attempts = 0U;
    s_auth_manager.lockout_until_seconds = 0U;
    s_auth_manager.logged_in = true;
    s_auth_manager.current_role = AUTH_DEMO_ROLE;
    {
        char msg[64];
        (void)snprintf(msg, sizeof(msg), "user=%s role=%s", user_name, auth_manager_get_role_name());
        (void)audit_log_record(AUDIT_EVENT_LOGIN, msg);
    }
    return ESP_OK;
}

esp_err_t auth_manager_verify_password(const char *password)
{
    if (password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_auth_manager.logged_in) {
        return ESP_ERR_INVALID_STATE;
    }
    if (strcmp(password, AUTH_DEMO_PASSWORD) != 0) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

void auth_manager_logout(void)
{
    (void)audit_log_record(AUDIT_EVENT_LOGOUT, "Session ended");
    s_auth_manager.logged_in = false;
    s_auth_manager.current_role = AUTH_DEMO_DEFAULT_ROLE;
}

bool auth_manager_is_logged_in(void)
{
    return s_auth_manager.logged_in;
}

bool auth_manager_is_locked_out(uint32_t now_tick_seconds, uint32_t *unlock_at_tick_seconds)
{
    if (s_auth_manager.lockout_until_seconds == 0U) {
        return false;
    }
    if (now_tick_seconds >= s_auth_manager.lockout_until_seconds) {
        s_auth_manager.lockout_until_seconds = 0U;
        s_auth_manager.failed_attempts = 0U;
        return false;
    }
    if (unlock_at_tick_seconds != NULL) {
        *unlock_at_tick_seconds = s_auth_manager.lockout_until_seconds;
    }
    return true;
}

uint32_t auth_manager_get_current_seconds(void)
{
    return get_current_seconds();
}

hmi_role_t auth_manager_get_role(void)
{
    return s_auth_manager.current_role;
}

const char *auth_manager_get_role_name(void)
{
    uint32_t index;

    if (s_auth_manager.config == NULL) {
        return FALLBACK_ROLE_NAME;
    }

    for (index = 0U; index < s_auth_manager.config->role_count; index++) {
        if (s_auth_manager.config->role_definitions[index].role_id == s_auth_manager.current_role) {
            return s_auth_manager.config->role_definitions[index].role_name;
        }
    }

    return FALLBACK_ROLE_NAME;
}

bool auth_manager_has_permission(uint32_t permission_mask)
{
    uint32_t index;

    if (s_auth_manager.config == NULL) {
        return false;
    }

    if (!s_auth_manager.logged_in) {
        return false;
    }

    for (index = 0U; index < s_auth_manager.config->role_count; index++) {
        if (s_auth_manager.config->role_definitions[index].role_id != s_auth_manager.current_role) {
            continue;
        }
        if (s_auth_manager.config->role_definitions[index].level >= AUTH_ROLE_LEVEL_SUPERUSER) {
            return true;
        }
        return ((s_auth_manager.config->role_definitions[index].permissions & permission_mask) != 0U);
    }

    return false;
}
