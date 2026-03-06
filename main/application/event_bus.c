#include "application/event_bus.h"

#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum
{
    APP_EVENT_QUEUE_LENGTH = 16
};

static StaticQueue_t s_event_queue_buffer;
static uint8_t s_event_storage[APP_EVENT_QUEUE_LENGTH * sizeof(app_event_t)];
static QueueHandle_t s_event_queue;
static bool s_event_bus_initialized;

esp_err_t event_bus_init(void)
{
    s_event_queue = xQueueCreateStatic(APP_EVENT_QUEUE_LENGTH,
                                       sizeof(app_event_t),
                                       s_event_storage,
                                       &s_event_queue_buffer);
    s_event_bus_initialized = (s_event_queue != NULL);
    return s_event_bus_initialized ? ESP_OK : ESP_FAIL;
}

void event_bus_deinit(void)
{
    s_event_queue = NULL;
    s_event_bus_initialized = false;
}

esp_err_t event_bus_publish(const app_event_t *event)
{
    if ((event == NULL) || !s_event_bus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return (xQueueSend(s_event_queue, event, 0U) == pdPASS) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t event_bus_receive(app_event_t *event, uint32_t timeout_ms)
{
    if ((event == NULL) || !s_event_bus_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return (xQueueReceive(s_event_queue, event, pdMS_TO_TICKS(timeout_ms)) == pdPASS) ? ESP_OK : ESP_ERR_TIMEOUT;
}
