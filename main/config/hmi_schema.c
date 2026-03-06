#include "config/hmi_schema.h"

#include <string.h>

static const hmi_system_config_t g_default_config = {
    .version = "2.0.0",
    .language_support = {
        .default_language = HMI_LANGUAGE_VI,
        .fallback_language = HMI_LANGUAGE_EN,
        .available_languages = {HMI_LANGUAGE_VI, HMI_LANGUAGE_EN, HMI_LANGUAGE_ES, HMI_LANGUAGE_ZH},
        .available_language_count = 4U
    },
    .role_definitions = {
        {.role_id = HMI_ROLE_OPERATOR, .role_name = "operator", .level = 10U, .permissions = HMI_PERMISSION_VIEW_DASHBOARD | HMI_PERMISSION_VIEW_ALARMS | HMI_PERMISSION_START_STOP | HMI_PERMISSION_SELECT_RECIPE, .session_timeout_minutes = 30U},
        {.role_id = HMI_ROLE_SENIOR_OPERATOR, .role_name = "senior_operator", .level = 20U, .permissions = HMI_PERMISSION_VIEW_DASHBOARD | HMI_PERMISSION_VIEW_ALARMS | HMI_PERMISSION_START_STOP | HMI_PERMISSION_SELECT_RECIPE | HMI_PERMISSION_ADJUST_SENSITIVITY | HMI_PERMISSION_ACKNOWLEDGE_ALARMS | HMI_PERMISSION_VIEW_BASIC_STATS, .session_timeout_minutes = 30U},
        {.role_id = HMI_ROLE_QC_ENGINEER, .role_name = "qc_engineer", .level = 30U, .permissions = HMI_PERMISSION_EDIT_RECIPE | HMI_PERMISSION_CALIBRATE | HMI_PERMISSION_VIEW_DEFECT_ANALYSIS | HMI_PERMISSION_EXPORT_DATA | HMI_PERMISSION_VIEW_STATS, .session_timeout_minutes = 45U},
        {.role_id = HMI_ROLE_MAINTENANCE_ADMIN, .role_name = "maintenance_admin", .level = 40U, .permissions = HMI_PERMISSION_OTA_UPDATE | HMI_PERMISSION_HARDWARE_TEST | HMI_PERMISSION_FACTORY_RESET | HMI_PERMISSION_LICENSE_MANAGE | HMI_PERMISSION_VIEW_LOGS, .session_timeout_minutes = 60U},
        {.role_id = HMI_ROLE_AUDITOR, .role_name = "auditor", .level = 5U, .permissions = HMI_PERMISSION_VIEW_LOGS | HMI_PERMISSION_VIEW_STATS | HMI_PERMISSION_EXPORT_REPORTS, .session_timeout_minutes = 30U}
    },
    .role_count = 5U,
    .audit_config = {
        .enabled = true,
        .retention_days = 90U,
        .export_format_count = 2U
    },
    .touch_config = {
        .min_touch_size_px = 60U,
        .debounce_ms = 50U,
        .double_tap_interval_ms = 300U,
        .long_press_duration_ms = 800U,
        .glove_mode = true,
        .wet_hand_mode = true,
        .haptic_feedback = true
    },
    .display_config = {
        .auto_switch = true,
        .high_contrast_themes = true,
        .screen_reader_support = true,
        .font_scale_default = 1.2F,
        .font_scale_min = 1.0F,
        .font_scale_max = 2.0F
    },
    .resource_limits = {
        .max_ram_usage_mb = 400U,
        .cpu_throttle_percent = 80U,
        .storage_warning_percent = 85U,
        .storage_critical_percent = 95U,
        .max_snapshot_images = 100U,
        .max_log_size_mb = 50U
    },
    .hardware_comm = {
        .fpga_config = {
            .node_count = 4U,
            .critical_poll_ms = 50U,
            .normal_poll_ms = 200U,
            .slow_poll_ms = 1000U,
            .timeout_ms = 500U,
            .retry_count = 3U
        },
        .stm32_config = {
            .baud_rate = 115200UL,
            .polling_rate_ms = 100U
        }
    },
    .startup_profile = {
        .startup_profile = "operator_basic_ui",
        .ui_block_count = 8U,
        .ui_blocks = {
            HMI_UI_BLOCK_DASHBOARD,
            HMI_UI_BLOCK_RECIPE_MANAGER,
            HMI_UI_BLOCK_CALIBRATION,
            HMI_UI_BLOCK_VISION_TUNING,
            HMI_UI_BLOCK_STATISTICS,
            HMI_UI_BLOCK_ALARMS,
            HMI_UI_BLOCK_MAINTENANCE,
            HMI_UI_BLOCK_LOGIN
        }
    }
};

const hmi_system_config_t *hmi_schema_get_default(void)
{
    return &g_default_config;
}
