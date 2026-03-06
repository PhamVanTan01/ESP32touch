#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "config/hmi_schema.h"

esp_err_t auth_manager_init(const hmi_system_config_t *config);
void auth_manager_deinit(void);
esp_err_t auth_manager_login_password(const char *user_name, const char *password, hmi_role_t requested_role);
/** Verify password for current user (e.g. before dangerous action). Returns ESP_OK if correct. */
esp_err_t auth_manager_verify_password(const char *password);
void auth_manager_logout(void);
bool auth_manager_is_logged_in(void);
/** Returns true if currently locked out; if so, *unlock_at_tick_seconds is set (seconds since boot). */
bool auth_manager_is_locked_out(uint32_t now_tick_seconds, uint32_t *unlock_at_tick_seconds);
/** Seconds since boot (for UI to pass into auth_manager_is_locked_out and format lock message). */
uint32_t auth_manager_get_current_seconds(void);
hmi_role_t auth_manager_get_role(void);
const char *auth_manager_get_role_name(void);
bool auth_manager_has_permission(uint32_t permission_mask);
