#include "application/app_task.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "application/data_manager.h"
#include "application/event_bus.h"
#include "application/hmi_state.h"
#include "application/resource_monitor.h"
#include "application/session_manager.h"
#include "config/system_config.h"
#include "middleware/audit/audit_log.h"
#include "middleware/protocol/protocol_router.h"
#include "ui/ui_manager.h"

static const char *TAG = "app_task";
static TaskHandle_t s_monitor_task_handle;
static TaskHandle_t s_hardware_task_handle;
static TaskHandle_t s_event_task_handle;
static TaskHandle_t s_logging_task_handle;

enum
{
    APP_HARDWARE_TASK_STACK_SIZE_BYTES = (4 * 1024),
    APP_EVENT_TASK_STACK_SIZE_BYTES = (4 * 1024),
    APP_LOGGING_TASK_STACK_SIZE_BYTES = (4 * 1024),
    APP_HARDWARE_POLL_PERIOD_MS = 100,
    APP_EVENT_POLL_PERIOD_MS = 100,
    APP_LOGGING_PERIOD_MS = 1000
};

static bool snapshots_equal(const app_data_snapshot_t *lhs, const app_data_snapshot_t *rhs)
{
    if ((lhs == NULL) || (rhs == NULL)) {
        return false;
    }

    if ((strcmp(lhs->chip_info.model_name, rhs->chip_info.model_name) != 0) ||
        (strcmp(lhs->chip_info.idf_version, rhs->chip_info.idf_version) != 0) ||
        (strcmp(lhs->chip_info.mac_address, rhs->chip_info.mac_address) != 0)) {
        return false;
    }

    if ((lhs->chip_info.revision != rhs->chip_info.revision) ||
        (lhs->chip_info.cpu_core_count != rhs->chip_info.cpu_core_count) ||
        (lhs->chip_info.features.has_embedded_flash != rhs->chip_info.features.has_embedded_flash) ||
        (lhs->chip_info.features.has_wifi != rhs->chip_info.features.has_wifi) ||
        (lhs->chip_info.features.has_ble != rhs->chip_info.features.has_ble) ||
        (lhs->chip_info.features.has_classic_bluetooth !=
         rhs->chip_info.features.has_classic_bluetooth) ||
        (lhs->chip_info.features.has_ieee802154 != rhs->chip_info.features.has_ieee802154) ||
        (lhs->chip_info.features.has_embedded_psram != rhs->chip_info.features.has_embedded_psram)) {
        return false;
    }

    if ((lhs->runtime_info.uptime_seconds != rhs->runtime_info.uptime_seconds) ||
        (lhs->runtime_info.free_heap_bytes != rhs->runtime_info.free_heap_bytes) ||
        (lhs->runtime_info.flash_size_bytes != rhs->runtime_info.flash_size_bytes) ||
        (lhs->runtime_info.psram_size_bytes != rhs->runtime_info.psram_size_bytes) ||
        (lhs->runtime_info.cpu_frequency_mhz != rhs->runtime_info.cpu_frequency_mhz) ||
        (lhs->runtime_info.temperature_deci_celsius != rhs->runtime_info.temperature_deci_celsius) ||
        (lhs->runtime_info.temperature_valid != rhs->runtime_info.temperature_valid)) {
        return false;
    }

    return true;
}

static void system_monitor_task(void *arg)
{
    app_data_snapshot_t snapshot;
    app_data_snapshot_t previous_snapshot;
    bool has_previous_snapshot = false;

    (void)arg;
    (void)memset(&previous_snapshot, 0, sizeof(previous_snapshot));

    ESP_LOGI(TAG, "Starting application task");

    /* MISRA deviation note: this is a long-lived RTOS service task by design. */
    for (;;) {
        if (data_manager_refresh() == ESP_OK) {
            if (data_manager_get_snapshot(&snapshot) == ESP_OK) {
                if ((!has_previous_snapshot) || (!snapshots_equal(&snapshot, &previous_snapshot))) {
                    (void)ui_manager_update_system_status(&snapshot);
                    previous_snapshot = snapshot;
                    has_previous_snapshot = true;
                }
            }
        }

        (void)resource_monitor_refresh();

        vTaskDelay(pdMS_TO_TICKS(ESP32TOUCH_APP_UPDATE_PERIOD_MS));
    }
}

static void hardware_poll_task(void *arg)
{
    app_event_t event;
    protocol_status_t protocol_status;

    (void)arg;

    /* MISRA deviation note: hardware polling task is intentionally persistent. */
    for (;;) {
        if (protocol_router_poll_once() == ESP_OK) {
            if (protocol_router_get_status(&protocol_status) == ESP_OK) {
                (void)hmi_state_set_comm_status(protocol_status.fpga_online, protocol_status.stm32_online);
                event.event_type = APP_EVENT_COMM_STATUS_UPDATED;
                event.value = ((protocol_status.fpga_online ? 1UL : 0UL) |
                              (protocol_status.stm32_online ? 2UL : 0UL));
                (void)event_bus_publish(&event);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(APP_HARDWARE_POLL_PERIOD_MS));
    }
}

static void event_dispatch_task(void *arg)
{
    app_event_t event;
    uint32_t session_tick_acc_ms = 0U;

    (void)arg;

    /* MISRA deviation note: event task stays alive for the application lifetime. */
    for (;;) {
        session_tick_acc_ms += APP_EVENT_POLL_PERIOD_MS;
        if (session_tick_acc_ms >= SESSION_TICK_PERIOD_MS) {
            session_manager_tick();
            session_tick_acc_ms = 0U;
        }

        if (event_bus_receive(&event, APP_EVENT_POLL_PERIOD_MS) == ESP_OK) {
            switch (event.event_type) {
            case APP_EVENT_COMM_STATUS_UPDATED:
                (void)audit_log_record(AUDIT_EVENT_CONFIG_CHANGE, "Communication state updated");
                break;
            case APP_EVENT_LOGIN_COMPLETED:
                /* Login audit is recorded in auth_manager_login_password(). */
                break;
            case APP_EVENT_LOG_FLUSH_REQUEST:
                (void)audit_log_process_pending();
                break;
            case APP_EVENT_ALARM_UPDATED:
                ui_manager_update_alarm_banner();
                break;
            case APP_EVENT_NONE:
            default:
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(APP_EVENT_POLL_PERIOD_MS));
    }
}

static void logging_task(void *arg)
{
    (void)arg;

    /* MISRA deviation note: periodic logging runs as a dedicated background task. */
    for (;;) {
        (void)audit_log_process_pending();
        vTaskDelay(pdMS_TO_TICKS(APP_LOGGING_PERIOD_MS));
    }
}

esp_err_t app_task_start(void)
{
    if (xTaskCreatePinnedToCore(system_monitor_task,
                                "app_mon",
                                ESP32TOUCH_APP_TASK_STACK_SIZE_BYTES,
                                NULL,
                                ESP32TOUCH_APP_TASK_PRIORITY,
                                &s_monitor_task_handle,
                                (BaseType_t)ESP32TOUCH_APP_TASK_CORE_ID) != pdPASS) {
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(hardware_poll_task,
                                "app_hw",
                                APP_HARDWARE_TASK_STACK_SIZE_BYTES,
                                NULL,
                                ESP32TOUCH_APP_TASK_PRIORITY,
                                &s_hardware_task_handle,
                                (BaseType_t)ESP32TOUCH_APP_TASK_CORE_ID) != pdPASS) {
        app_task_stop();
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(event_dispatch_task,
                                "app_evt",
                                APP_EVENT_TASK_STACK_SIZE_BYTES,
                                NULL,
                                ESP32TOUCH_APP_TASK_PRIORITY,
                                &s_event_task_handle,
                                (BaseType_t)ESP32TOUCH_APP_TASK_CORE_ID) != pdPASS) {
        app_task_stop();
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(logging_task,
                                "app_log",
                                APP_LOGGING_TASK_STACK_SIZE_BYTES,
                                NULL,
                                ESP32TOUCH_APP_TASK_PRIORITY,
                                &s_logging_task_handle,
                                (BaseType_t)ESP32TOUCH_APP_TASK_CORE_ID) != pdPASS) {
        app_task_stop();
        return ESP_FAIL;
    }

    return ESP_OK;
}

void app_task_stop(void)
{
    if (s_monitor_task_handle != NULL) {
        vTaskDelete(s_monitor_task_handle);
        s_monitor_task_handle = NULL;
    }
    if (s_hardware_task_handle != NULL) {
        vTaskDelete(s_hardware_task_handle);
        s_hardware_task_handle = NULL;
    }
    if (s_event_task_handle != NULL) {
        vTaskDelete(s_event_task_handle);
        s_event_task_handle = NULL;
    }
    if (s_logging_task_handle != NULL) {
        vTaskDelete(s_logging_task_handle);
        s_logging_task_handle = NULL;
    }
}
