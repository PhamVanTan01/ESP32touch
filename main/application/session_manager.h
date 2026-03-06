#pragma once

#include "esp_err.h"

#include "config/hmi_schema.h"

/** Callback invoked when session has timed out (no activity for timeout_minutes). */
typedef void (*session_timeout_callback_t)(void);

/**
 * Initialize session manager.
 * \param config HMI config (role definitions with session_timeout_minutes).
 * \param on_timeout Callback to run on timeout (e.g. logout and show login).
 * \return ESP_OK on success.
 */
esp_err_t session_manager_init(const hmi_system_config_t *config,
                               session_timeout_callback_t on_timeout);

void session_manager_deinit(void);

/** Record user activity (call on touch/navigation). */
void session_manager_touch(void);

/** Call periodically (e.g. every 1 s); checks timeout and invokes callback if expired. */
void session_manager_tick(void);

/** Recommended interval in ms for calling session_manager_tick(). */
#define SESSION_TICK_PERIOD_MS  1000
