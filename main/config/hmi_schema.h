#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum
{
    HMI_MAX_LANGUAGE_COUNT = 4,
    HMI_MAX_ROLE_COUNT = 5,
    HMI_MAX_ROLE_NAME_LENGTH = 24,
    HMI_MAX_PROFILE_NAME_LENGTH = 32,
    HMI_MAX_UI_BLOCK_COUNT = 9
};

typedef enum
{
    HMI_LANGUAGE_VI = 0,
    HMI_LANGUAGE_EN,
    HMI_LANGUAGE_ES,
    HMI_LANGUAGE_ZH,
    HMI_LANGUAGE_COUNT
} hmi_language_t;

typedef enum
{
    HMI_ROLE_AUDITOR = 0,
    HMI_ROLE_OPERATOR,
    HMI_ROLE_SENIOR_OPERATOR,
    HMI_ROLE_QC_ENGINEER,
    HMI_ROLE_MAINTENANCE_ADMIN,
    HMI_ROLE_COUNT
} hmi_role_t;

typedef enum
{
    HMI_UI_BLOCK_DASHBOARD = 0,
    HMI_UI_BLOCK_RECIPE_MANAGER,
    HMI_UI_BLOCK_CALIBRATION,
    HMI_UI_BLOCK_VISION_TUNING,
    HMI_UI_BLOCK_STATISTICS,
    HMI_UI_BLOCK_ALARMS,
    HMI_UI_BLOCK_MAINTENANCE,
    HMI_UI_BLOCK_LOGIN,
    HMI_UI_BLOCK_HOME
} hmi_ui_block_t;

typedef enum
{
    HMI_PERMISSION_VIEW_DASHBOARD = (1UL << 0),
    HMI_PERMISSION_VIEW_ALARMS = (1UL << 1),
    HMI_PERMISSION_START_STOP = (1UL << 2),
    HMI_PERMISSION_SELECT_RECIPE = (1UL << 3),
    HMI_PERMISSION_ADJUST_SENSITIVITY = (1UL << 4),
    HMI_PERMISSION_ACKNOWLEDGE_ALARMS = (1UL << 5),
    HMI_PERMISSION_VIEW_BASIC_STATS = (1UL << 6),
    HMI_PERMISSION_EDIT_RECIPE = (1UL << 7),
    HMI_PERMISSION_CALIBRATE = (1UL << 8),
    HMI_PERMISSION_VIEW_DEFECT_ANALYSIS = (1UL << 9),
    HMI_PERMISSION_EXPORT_DATA = (1UL << 10),
    HMI_PERMISSION_OTA_UPDATE = (1UL << 11),
    HMI_PERMISSION_HARDWARE_TEST = (1UL << 12),
    HMI_PERMISSION_FACTORY_RESET = (1UL << 13),
    HMI_PERMISSION_LICENSE_MANAGE = (1UL << 14),
    HMI_PERMISSION_VIEW_LOGS = (1UL << 15),
    HMI_PERMISSION_VIEW_STATS = (1UL << 16),
    HMI_PERMISSION_EXPORT_REPORTS = (1UL << 17)
} hmi_permission_bits_t;

typedef struct
{
    hmi_language_t default_language;
    hmi_language_t fallback_language;
    hmi_language_t available_languages[HMI_MAX_LANGUAGE_COUNT];
    uint8_t available_language_count;
} hmi_language_support_t;

typedef struct
{
    hmi_role_t role_id;
    char role_name[HMI_MAX_ROLE_NAME_LENGTH];
    uint8_t level;
    uint32_t permissions;
    uint16_t session_timeout_minutes;
} hmi_role_definition_t;

typedef struct
{
    bool enabled;
    uint16_t retention_days;
    uint8_t export_format_count;
} hmi_audit_config_t;

typedef struct
{
    uint16_t min_touch_size_px;
    uint16_t debounce_ms;
    uint16_t double_tap_interval_ms;
    uint16_t long_press_duration_ms;
    bool glove_mode;
    bool wet_hand_mode;
    bool haptic_feedback;
} hmi_touch_config_t;

typedef struct
{
    bool auto_switch;
    bool high_contrast_themes;
    bool screen_reader_support;
    float font_scale_default;
    float font_scale_min;
    float font_scale_max;
} hmi_display_config_t;

typedef struct
{
    uint16_t max_ram_usage_mb;
    uint8_t cpu_throttle_percent;
    uint8_t storage_warning_percent;
    uint8_t storage_critical_percent;
    uint16_t max_snapshot_images;
    uint16_t max_log_size_mb;
} hmi_resource_limits_t;

typedef struct
{
    uint8_t node_count;
    uint16_t critical_poll_ms;
    uint16_t normal_poll_ms;
    uint16_t slow_poll_ms;
    uint16_t timeout_ms;
    uint8_t retry_count;
} hmi_fpga_comm_config_t;

typedef struct
{
    uint32_t baud_rate;
    uint16_t polling_rate_ms;
} hmi_stm32_comm_config_t;

typedef struct
{
    hmi_fpga_comm_config_t fpga_config;
    hmi_stm32_comm_config_t stm32_config;
} hmi_hardware_comm_config_t;

typedef struct
{
    char startup_profile[HMI_MAX_PROFILE_NAME_LENGTH];
    uint8_t ui_block_count;
    hmi_ui_block_t ui_blocks[HMI_MAX_UI_BLOCK_COUNT];
} hmi_profile_t;

typedef struct
{
    char version[8];
    hmi_language_support_t language_support;
    hmi_role_definition_t role_definitions[HMI_MAX_ROLE_COUNT];
    uint8_t role_count;
    hmi_audit_config_t audit_config;
    hmi_touch_config_t touch_config;
    hmi_display_config_t display_config;
    hmi_resource_limits_t resource_limits;
    hmi_hardware_comm_config_t hardware_comm;
    hmi_profile_t startup_profile;
} hmi_system_config_t;

const hmi_system_config_t *hmi_schema_get_default(void);
