#include "application/resource_monitor.h"

#include <string.h>

#include "esp_system.h"

static resource_monitor_snapshot_t s_resource_snapshot;
static bool s_resource_initialized;

enum
{
    RESOURCE_MONITOR_CPU_LOAD_UNAVAILABLE_PERCENT = 0U,
    RESOURCE_MONITOR_HEAP_WARNING_THRESHOLD_BYTES = (128UL * 1024UL)
};

esp_err_t resource_monitor_init(void)
{
    (void)memset(&s_resource_snapshot, 0, sizeof(s_resource_snapshot));
    s_resource_initialized = true;
    return ESP_OK;
}

void resource_monitor_deinit(void)
{
    (void)memset(&s_resource_snapshot, 0, sizeof(s_resource_snapshot));
    s_resource_initialized = false;
}

esp_err_t resource_monitor_refresh(void)
{
    if (!s_resource_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_resource_snapshot.free_heap_bytes = esp_get_free_heap_size();
    s_resource_snapshot.minimum_heap_bytes = esp_get_minimum_free_heap_size();

    /* Runtime stats are disabled in sdkconfig; report as unavailable instead of hardcoded pseudo-value. */
    s_resource_snapshot.cpu_load_percent = RESOURCE_MONITOR_CPU_LOAD_UNAVAILABLE_PERCENT;
    s_resource_snapshot.heap_warning =
        (s_resource_snapshot.free_heap_bytes < RESOURCE_MONITOR_HEAP_WARNING_THRESHOLD_BYTES);
    return ESP_OK;
}

esp_err_t resource_monitor_get_snapshot(resource_monitor_snapshot_t *snapshot)
{
    if ((snapshot == NULL) || !s_resource_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *snapshot = s_resource_snapshot;
    return ESP_OK;
}
