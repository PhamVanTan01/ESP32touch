#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

enum
{
    APP_DATA_MODEL_NAME_LENGTH = 16,
    APP_DATA_IDF_VERSION_LENGTH = 32,
    APP_DATA_MAC_ADDRESS_LENGTH = 18
};

typedef struct
{
    bool has_embedded_flash;
    bool has_wifi;
    bool has_ble;
    bool has_classic_bluetooth;
    bool has_ieee802154;
    bool has_embedded_psram;
} app_chip_features_t;

typedef struct
{
    char model_name[APP_DATA_MODEL_NAME_LENGTH];
    char idf_version[APP_DATA_IDF_VERSION_LENGTH];
    char mac_address[APP_DATA_MAC_ADDRESS_LENGTH];
    app_chip_features_t features;
    uint16_t revision;
    uint8_t cpu_core_count;
} app_chip_info_t;

typedef struct
{
    uint32_t uptime_seconds;
    uint32_t free_heap_bytes;
    uint32_t flash_size_bytes;
    uint32_t psram_size_bytes;
    uint32_t cpu_frequency_mhz;
    int16_t temperature_deci_celsius;
    bool temperature_valid;
} app_runtime_info_t;

typedef struct
{
    app_chip_info_t chip_info;
    app_runtime_info_t runtime_info;
} app_data_snapshot_t;

esp_err_t data_manager_init(void);
void data_manager_deinit(void);
esp_err_t data_manager_refresh(void);
esp_err_t data_manager_get_snapshot(app_data_snapshot_t *snapshot);
