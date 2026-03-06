#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "application/alarm_manager.h"
#include "application/app_task.h"
#include "application/auth_manager.h"
#include "application/data_manager.h"
#include "application/event_bus.h"
#include "application/hmi_state.h"
#include "application/resource_monitor.h"
#include "application/session_manager.h"
#include "bsp/display/display_driver.h"
#include "bsp/display/display_lvgl.h"
#include "bsp/touch/touch_driver.h"
#include "config/hmi_loader.h"
#include "config/ui_theme.h"
#include "middleware/audit/audit_log.h"
#include "middleware/protocol/protocol_router.h"
#include "ui/ui_manager.h"

static const char *TAG = "app_main";
static bsp_display_t s_display;
static bsp_touch_t s_touch;
static display_lvgl_t s_lvgl_port;

static void on_session_timeout(void)
{
    auth_manager_logout();
    (void)ui_manager_show(UI_SCREEN_LOGIN);
}

static void on_critical_alarm(uint32_t id, const char *message)
{
    ui_manager_show_critical_alarm_popup(id, message);
}

static void app_cleanup(void)
{
    app_task_stop();
    session_manager_deinit();
    alarm_manager_deinit();
    ui_manager_deinit();
    protocol_router_deinit();
    audit_log_deinit();
    event_bus_deinit();
    resource_monitor_deinit();
    auth_manager_deinit();
    hmi_state_deinit();
    hmi_loader_deinit();
    (void)display_lvgl_deinit(&s_lvgl_port);
    (void)bsp_touch_deinit(&s_touch);
    (void)bsp_display_deinit(&s_display);
    data_manager_deinit();
}

static esp_err_t app_start(void)
{
    app_data_snapshot_t snapshot;
    esp_err_t result;
    const hmi_system_config_t *hmi_config;

    ESP_LOGI(TAG, "Initialize data layer");
    result = data_manager_init();
    if (result != ESP_OK) {
        return result;
    }

    result = hmi_loader_init();
    if (result != ESP_OK) {
        return result;
    }

    hmi_config = hmi_loader_get_config();
    if (hmi_config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    result = hmi_state_init(hmi_config);
    if (result != ESP_OK) {
        return result;
    }

    result = auth_manager_init(hmi_config);
    if (result != ESP_OK) {
        return result;
    }

    result = event_bus_init();
    if (result != ESP_OK) {
        return result;
    }

    result = audit_log_init();
    if (result != ESP_OK) {
        return result;
    }

    result = alarm_manager_init(on_critical_alarm);
    if (result != ESP_OK) {
        return result;
    }

    result = protocol_router_init();
    if (result != ESP_OK) {
        return result;
    }

    result = resource_monitor_init();
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Initialize board support package");
    result = bsp_display_init(&s_display);
    if (result != ESP_OK) {
        return result;
    }

    result = bsp_touch_init(&s_touch);
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Initialize LVGL port");
    result = display_lvgl_init(&s_display, &s_touch, &s_lvgl_port);
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Initialize UI");
    result = ui_manager_init(s_lvgl_port.display);
    if (result != ESP_OK) {
        return result;
    }

    result = session_manager_init(hmi_config, on_session_timeout);
    if (result != ESP_OK) {
        return result;
    }

    result = ui_manager_show(UI_SCREEN_SPLASH);
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Start application tasks");
    result = app_task_start();
    if (result != ESP_OK) {
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(UI_THEME_SPLASH_DISPLAY_MS));

    result = ui_manager_show(UI_SCREEN_LOGIN);
    if (result != ESP_OK) {
        return result;
    }

    result = data_manager_get_snapshot(&snapshot);
    if (result == ESP_OK) {
        result = ui_manager_update_system_status(&snapshot);
    }
    if (result != ESP_OK) {
        return result;
    }

    return ESP_OK;
}

void app_main(void)
{
    const esp_err_t result = app_start();

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Application startup failed: %s", esp_err_to_name(result));
        app_cleanup();
    }
}
