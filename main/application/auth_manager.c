#include "application/auth_manager.h"

#include <stdbool.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "auth_manager";

typedef struct
{
    const hmi_system_config_t *config;
    hmi_role_t current_role;
    bool logged_in;
} auth_manager_context_t;

static auth_manager_context_t s_auth_manager;

esp_err_t auth_manager_init(const hmi_system_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(&s_auth_manager, 0, sizeof(s_auth_manager));
    s_auth_manager.config = config;
    s_auth_manager.current_role = HMI_ROLE_OPERATOR;
    ESP_LOGI(TAG, "Initialized with default role=%d", (int)s_auth_manager.current_role);
    return ESP_OK;
}

void auth_manager_deinit(void)
{
    (void)memset(&s_auth_manager, 0, sizeof(s_auth_manager));
}

esp_err_t auth_manager_login_password(const char *user_name, const char *password, hmi_role_t requested_role)
{
    if ((user_name == NULL) || (password == NULL) || (s_auth_manager.config == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_auth_manager.logged_in = true;
    s_auth_manager.current_role = requested_role;
    return ESP_OK;
}

void auth_manager_logout(void)
{
    s_auth_manager.logged_in = false;
    s_auth_manager.current_role = HMI_ROLE_OPERATOR;
}

hmi_role_t auth_manager_get_role(void)
{
    return s_auth_manager.current_role;
}

bool auth_manager_has_permission(uint32_t permission_mask)
{
    uint32_t index;

    if (s_auth_manager.config == NULL) {
        return false;
    }

    for (index = 0U; index < s_auth_manager.config->role_count; index++) {
        if (s_auth_manager.config->role_definitions[index].role_id == s_auth_manager.current_role) {
            return ((s_auth_manager.config->role_definitions[index].permissions & permission_mask) != 0U);
        }
    }

    return false;
}
