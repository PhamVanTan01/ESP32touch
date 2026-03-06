#include "ui/ui_manager.h"

#include <string.h>

#include "esp_log.h"

#include "application/auth_manager.h"
#include "application/hmi_state.h"
#include "config/system_config.h"
#include "ui/screens/screen_alarms.h"
#include "ui/screens/screen_calibration.h"
#include "ui/screens/screen_dashboard.h"
#include "ui/screens/screen_login.h"
#include "ui/screens/screen_maintenance.h"
#include "ui/screens/screen_recipe.h"
#include "ui/screens/screen_main.h"
#include "ui/screens/screen_statistics.h"
#include "ui/screens/screen_vision_tuning.h"
#include "utils/lvgl_lock.h"

static const char *TAG = "ui_manager";

typedef struct
{
    lv_display_t *display;
    screen_dashboard_view_t dashboard_screen;
    screen_recipe_view_t recipe_screen;
    screen_calibration_view_t calibration_screen;
    screen_vision_tuning_view_t vision_tuning_screen;
    screen_statistics_view_t statistics_screen;
    screen_alarms_view_t alarms_screen;
    screen_maintenance_view_t maintenance_screen;
    screen_login_view_t login_screen;
    ui_screen_id_t active_screen;
    bool is_initialized;
} ui_manager_context_t;

static ui_manager_context_t s_ui_manager;

static const char *screen_to_name(ui_screen_id_t screen_id)
{
    switch (screen_id) {
    case UI_SCREEN_MAIN:
        return "main";
    case UI_SCREEN_RECIPE:
        return "recipe";
    case UI_SCREEN_CALIBRATION:
        return "calibration";
    case UI_SCREEN_VISION_TUNING:
        return "vision_tuning";
    case UI_SCREEN_STATISTICS:
        return "statistics";
    case UI_SCREEN_ALARMS:
        return "alarms";
    case UI_SCREEN_MAINTENANCE:
        return "maintenance";
    case UI_SCREEN_LOGIN:
        return "login";
    default:
        return "unknown";
    }
}

static uint32_t screen_to_permission(ui_screen_id_t screen_id)
{
    uint32_t permission_mask;

    switch (screen_id) {
    case UI_SCREEN_MAIN:
        permission_mask = HMI_PERMISSION_VIEW_DASHBOARD;
        break;
    case UI_SCREEN_RECIPE:
        permission_mask = HMI_PERMISSION_SELECT_RECIPE;
        break;
    case UI_SCREEN_CALIBRATION:
        permission_mask = HMI_PERMISSION_CALIBRATE;
        break;
    case UI_SCREEN_VISION_TUNING:
        permission_mask = HMI_PERMISSION_VIEW_DEFECT_ANALYSIS;
        break;
    case UI_SCREEN_STATISTICS:
        permission_mask = HMI_PERMISSION_VIEW_STATS;
        break;
    case UI_SCREEN_ALARMS:
        permission_mask = HMI_PERMISSION_VIEW_ALARMS;
        break;
    case UI_SCREEN_MAINTENANCE:
        permission_mask = HMI_PERMISSION_HARDWARE_TEST;
        break;
    case UI_SCREEN_LOGIN:
    default:
        permission_mask = 0U;
        break;
    }

    return permission_mask;
}

esp_err_t ui_manager_init(lv_display_t *display)
{
    if (display == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(&s_ui_manager, 0, sizeof(s_ui_manager));
    s_ui_manager.display = display;
    s_ui_manager.active_screen = UI_SCREEN_MAIN;
    s_ui_manager.is_initialized = true;

    return ESP_OK;
}

void ui_manager_deinit(void)
{
    if (lvgl_lock_acquire(ESP32TOUCH_LVGL_LOCK_TIMEOUT_MS) == ESP_OK) {
        screen_dashboard_destroy(&s_ui_manager.dashboard_screen);
        screen_recipe_destroy(&s_ui_manager.recipe_screen);
        screen_calibration_destroy(&s_ui_manager.calibration_screen);
        screen_vision_tuning_destroy(&s_ui_manager.vision_tuning_screen);
        screen_statistics_destroy(&s_ui_manager.statistics_screen);
        screen_alarms_destroy(&s_ui_manager.alarms_screen);
        screen_maintenance_destroy(&s_ui_manager.maintenance_screen);
        screen_login_destroy(&s_ui_manager.login_screen);
        (void)lvgl_lock_release();
    }

    (void)memset(&s_ui_manager, 0, sizeof(s_ui_manager));
}

ui_screen_id_t ui_manager_get_active_screen(void)
{
    return s_ui_manager.active_screen;
}

bool ui_manager_can_access_screen(ui_screen_id_t screen_id)
{
    const uint32_t permission_mask = screen_to_permission(screen_id);

    if (screen_id == UI_SCREEN_LOGIN) {
        return true;
    }

    return auth_manager_has_permission(permission_mask);
}

static esp_err_t create_screen_if_needed(ui_screen_id_t screen_id, lv_obj_t **root_object)
{
    esp_err_t result = ESP_OK;

    if (root_object == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (screen_id) {
    case UI_SCREEN_MAIN:
        if (!s_ui_manager.dashboard_screen.is_created) {
            result = screen_dashboard_create(s_ui_manager.display, &s_ui_manager.dashboard_screen);
        }
        *root_object = s_ui_manager.dashboard_screen.shell.root;
        break;
    case UI_SCREEN_RECIPE:
        if (!s_ui_manager.recipe_screen.is_created) {
            result = screen_recipe_create(s_ui_manager.display, &s_ui_manager.recipe_screen);
        }
        *root_object = s_ui_manager.recipe_screen.shell.root;
        break;
    case UI_SCREEN_CALIBRATION:
        if (!s_ui_manager.calibration_screen.is_created) {
            result = screen_calibration_create(s_ui_manager.display, &s_ui_manager.calibration_screen);
        }
        *root_object = s_ui_manager.calibration_screen.shell.root;
        break;
    case UI_SCREEN_VISION_TUNING:
        if (!s_ui_manager.vision_tuning_screen.is_created) {
            result = screen_vision_tuning_create(s_ui_manager.display, &s_ui_manager.vision_tuning_screen);
        }
        *root_object = s_ui_manager.vision_tuning_screen.shell.root;
        break;
    case UI_SCREEN_STATISTICS:
        if (!s_ui_manager.statistics_screen.is_created) {
            result = screen_statistics_create(s_ui_manager.display, &s_ui_manager.statistics_screen);
        }
        *root_object = s_ui_manager.statistics_screen.shell.root;
        break;
    case UI_SCREEN_ALARMS:
        if (!s_ui_manager.alarms_screen.is_created) {
            result = screen_alarms_create(s_ui_manager.display, &s_ui_manager.alarms_screen);
        }
        *root_object = s_ui_manager.alarms_screen.shell.root;
        break;
    case UI_SCREEN_MAINTENANCE:
        if (!s_ui_manager.maintenance_screen.is_created) {
            result = screen_maintenance_create(s_ui_manager.display, &s_ui_manager.maintenance_screen);
        }
        *root_object = s_ui_manager.maintenance_screen.shell.root;
        break;
    case UI_SCREEN_LOGIN:
        if (!s_ui_manager.login_screen.is_created) {
            result = screen_login_create(s_ui_manager.display, &s_ui_manager.login_screen);
        }
        *root_object = s_ui_manager.login_screen.shell.root;
        break;
    default:
        result = ESP_ERR_NOT_SUPPORTED;
        *root_object = NULL;
        break;
    }

    return result;
}

esp_err_t ui_manager_show(ui_screen_id_t screen_id)
{
    esp_err_t result;
    lv_obj_t *root_object = NULL;

    if (!s_ui_manager.is_initialized || (s_ui_manager.display == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!ui_manager_can_access_screen(screen_id)) {
        ESP_LOGW(TAG,
                 "Access denied for screen_id=%d (%s), role=%d",
                 (int)screen_id,
                 screen_to_name(screen_id),
                 (int)auth_manager_get_role());
        return ESP_ERR_INVALID_STATE;
    }

    result = lvgl_lock_acquire(ESP32TOUCH_LVGL_LOCK_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }

    result = create_screen_if_needed(screen_id, &root_object);
    if ((result != ESP_OK) || (root_object == NULL)) {
        (void)lvgl_lock_release();
        ESP_LOGW(TAG,
                 "Failed to prepare screen_id=%d (%s), err=%s, root=%p",
                 (int)screen_id,
                 screen_to_name(screen_id),
                 esp_err_to_name(result),
                 (void *)root_object);
        return result;
    }

    lv_screen_load(root_object);
    s_ui_manager.active_screen = screen_id;
    (void)hmi_state_set_screen((hmi_ui_block_t)screen_id);
    (void)lvgl_lock_release();
    ESP_LOGI(TAG,
             "Loaded screen_id=%d (%s), root=%p",
             (int)screen_id,
             screen_to_name(screen_id),
             (void *)root_object);
    return ESP_OK;
}

esp_err_t ui_manager_update_system_status(const app_data_snapshot_t *snapshot)
{
    esp_err_t result;

    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_ui_manager.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    result = lvgl_lock_acquire(ESP32TOUCH_LVGL_LOCK_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }

    if (s_ui_manager.active_screen == UI_SCREEN_MAIN) {
        result = screen_dashboard_update(&s_ui_manager.dashboard_screen, snapshot);
    } else {
        result = ESP_OK;
    }

    (void)lvgl_lock_release();
    return result;
}
