#include "application/data_manager.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "driver/temperature_sensor.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

typedef struct
{
    app_data_snapshot_t snapshot;
    temperature_sensor_handle_t temperature_sensor;
    bool is_initialized;
} data_manager_context_t;

static data_manager_context_t s_data_manager;

static void copy_text(char *destination, size_t destination_size, const char *source)
{
    if ((destination != NULL) && (destination_size > 0U) && (source != NULL)) {
        (void)snprintf(destination, destination_size, "%s", source);
    }
}

static const char *chip_model_to_string(esp_chip_model_t model)
{
    const char *model_name = "Unknown";

    switch (model) {
    case CHIP_ESP32:
        model_name = "ESP32";
        break;
    case CHIP_ESP32S2:
        model_name = "ESP32-S2";
        break;
    case CHIP_ESP32S3:
        model_name = "ESP32-S3";
        break;
    case CHIP_ESP32C3:
        model_name = "ESP32-C3";
        break;
    case CHIP_ESP32C2:
        model_name = "ESP32-C2";
        break;
    case CHIP_ESP32C6:
        model_name = "ESP32-C6";
        break;
    case CHIP_ESP32H2:
        model_name = "ESP32-H2";
        break;
    case CHIP_ESP32P4:
        model_name = "ESP32-P4";
        break;
    case CHIP_ESP32C61:
        model_name = "ESP32-C61";
        break;
    case CHIP_ESP32C5:
        model_name = "ESP32-C5";
        break;
    case CHIP_ESP32H21:
        model_name = "ESP32-H21";
        break;
    case CHIP_ESP32H4:
        model_name = "ESP32-H4";
        break;
    case CHIP_POSIX_LINUX:
        model_name = "POSIX";
        break;
    default:
        break;
    }

    return model_name;
}

static void load_chip_features(app_chip_features_t *features, uint32_t raw_features)
{
    if (features != NULL) {
        features->has_embedded_flash = ((raw_features & CHIP_FEATURE_EMB_FLASH) != 0U);
        features->has_wifi = ((raw_features & CHIP_FEATURE_WIFI_BGN) != 0U);
        features->has_ble = ((raw_features & CHIP_FEATURE_BLE) != 0U);
        features->has_classic_bluetooth = ((raw_features & CHIP_FEATURE_BT) != 0U);
        features->has_ieee802154 = ((raw_features & CHIP_FEATURE_IEEE802154) != 0U);
        features->has_embedded_psram = ((raw_features & CHIP_FEATURE_EMB_PSRAM) != 0U);
    }
}

static void load_static_snapshot(app_data_snapshot_t *snapshot)
{
    esp_chip_info_t chip_info;
    uint8_t mac_address[6];
    uint32_t flash_size_bytes = 0U;

    if (snapshot == NULL) {
        return;
    }

    (void)memset(&chip_info, 0, sizeof(chip_info));
    (void)memset(mac_address, 0, sizeof(mac_address));

    esp_chip_info(&chip_info);

    copy_text(snapshot->chip_info.model_name,
              sizeof(snapshot->chip_info.model_name),
              chip_model_to_string(chip_info.model));
    copy_text(snapshot->chip_info.idf_version,
              sizeof(snapshot->chip_info.idf_version),
              esp_get_idf_version());

    snapshot->chip_info.revision = chip_info.revision;
    snapshot->chip_info.cpu_core_count = chip_info.cores;
    load_chip_features(&snapshot->chip_info.features, chip_info.features);

    if (esp_read_mac(mac_address, ESP_MAC_WIFI_STA) == ESP_OK) {
        (void)snprintf(snapshot->chip_info.mac_address,
                       sizeof(snapshot->chip_info.mac_address),
                       "%02X:%02X:%02X:%02X:%02X:%02X",
                       mac_address[0], mac_address[1], mac_address[2],
                       mac_address[3], mac_address[4], mac_address[5]);
    } else {
        copy_text(snapshot->chip_info.mac_address,
                  sizeof(snapshot->chip_info.mac_address),
                  "Unavailable");
    }

    if (esp_flash_get_size(NULL, &flash_size_bytes) == ESP_OK) {
        snapshot->runtime_info.flash_size_bytes = flash_size_bytes;
    } else {
        snapshot->runtime_info.flash_size_bytes = 0U;
    }

    snapshot->runtime_info.cpu_frequency_mhz = esp_rom_get_cpu_ticks_per_us();
}

static int16_t convert_temperature_to_deci_celsius(float temperature_celsius)
{
    float rounded_value = temperature_celsius * 10.0F;
    int32_t scaled_value;

    if (rounded_value >= 0.0F) {
        rounded_value += 0.5F;
    } else {
        rounded_value -= 0.5F;
    }

    scaled_value = (int32_t)rounded_value;

    if (scaled_value > INT16_MAX) {
        scaled_value = INT16_MAX;
    } else if (scaled_value < INT16_MIN) {
        scaled_value = INT16_MIN;
    } else {
        /* value already in range */
    }

    return (int16_t)scaled_value;
}

static void load_runtime_snapshot(app_data_snapshot_t *snapshot, temperature_sensor_handle_t temperature_sensor)
{
    float temperature_celsius = 0.0F;
    uint64_t uptime_microseconds;

    if (snapshot == NULL) {
        return;
    }

    uptime_microseconds = (uint64_t)esp_timer_get_time();
    snapshot->runtime_info.uptime_seconds = (uint32_t)(uptime_microseconds / 1000000ULL);
    snapshot->runtime_info.free_heap_bytes = esp_get_free_heap_size();

#if CONFIG_SPIRAM
    if (esp_psram_is_initialized()) {
        snapshot->runtime_info.psram_size_bytes = (uint32_t)esp_psram_get_size();
    } else {
        snapshot->runtime_info.psram_size_bytes = 0U;
    }
#else
    snapshot->runtime_info.psram_size_bytes = 0U;
#endif

    if ((temperature_sensor != NULL) &&
        (temperature_sensor_get_celsius(temperature_sensor, &temperature_celsius) == ESP_OK)) {
        snapshot->runtime_info.temperature_valid = true;
        snapshot->runtime_info.temperature_deci_celsius = convert_temperature_to_deci_celsius(temperature_celsius);
    } else {
        snapshot->runtime_info.temperature_valid = false;
        snapshot->runtime_info.temperature_deci_celsius = 0;
    }
}

static esp_err_t initialize_temperature_sensor(data_manager_context_t *context)
{
    esp_err_t result;
    const temperature_sensor_config_t temperature_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    if (context == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    result = temperature_sensor_install(&temperature_config, &context->temperature_sensor);
    if (result != ESP_OK) {
        context->temperature_sensor = NULL;
        return result;
    }

    result = temperature_sensor_enable(context->temperature_sensor);
    if (result != ESP_OK) {
        (void)temperature_sensor_uninstall(context->temperature_sensor);
        context->temperature_sensor = NULL;
    }

    return result;
}

esp_err_t data_manager_init(void)
{
    (void)memset(&s_data_manager, 0, sizeof(s_data_manager));

    load_static_snapshot(&s_data_manager.snapshot);
    (void)initialize_temperature_sensor(&s_data_manager);
    load_runtime_snapshot(&s_data_manager.snapshot, s_data_manager.temperature_sensor);
    s_data_manager.is_initialized = true;

    return ESP_OK;
}

void data_manager_deinit(void)
{
    if (s_data_manager.temperature_sensor != NULL) {
        (void)temperature_sensor_disable(s_data_manager.temperature_sensor);
        (void)temperature_sensor_uninstall(s_data_manager.temperature_sensor);
        s_data_manager.temperature_sensor = NULL;
    }

    s_data_manager.is_initialized = false;
}

esp_err_t data_manager_refresh(void)
{
    if (!s_data_manager.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    load_runtime_snapshot(&s_data_manager.snapshot, s_data_manager.temperature_sensor);
    return ESP_OK;
}

esp_err_t data_manager_get_snapshot(app_data_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_data_manager.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *snapshot = s_data_manager.snapshot;
    return ESP_OK;
}
