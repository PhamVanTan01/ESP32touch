#include "ui/ui_manager.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "application/alarm_manager.h"
#include "application/auth_manager.h"
#include "application/hmi_state.h"
#include "application/session_manager.h"
#include "config/system_config.h"
#include "config/ui_theme.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_shell.h"
#include "ui/screens/screen_alarms.h"
#include "ui/screens/screen_calibration.h"
#include "ui/screens/screen_dashboard.h"
#include "ui/screens/screen_home.h"
#include "ui/screens/screen_login.h"
#include "ui/screens/screen_maintenance.h"
#include "ui/screens/screen_recipe.h"
#include "ui/screens/screen_main.h"
#include "ui/screens/screen_splash.h"
#include "ui/screens/screen_statistics.h"
#include "ui/screens/screen_vision_tuning.h"
#include "utils/lvgl_lock.h"

static const char *TAG = "ui_manager";

#define UI_LOADER_QUEUE_SIZE 2U
#define UI_LOADER_TASK_STACK_SIZE 4096U
#define UI_LOADER_TASK_PRIORITY 1U
#define UI_SCREEN_INVALID ((ui_screen_id_t)0xFFU)

typedef struct
{
    lv_display_t *display;
    screen_splash_view_t splash_screen;
    screen_dashboard_view_t dashboard_screen;
    screen_recipe_view_t recipe_screen;
    screen_calibration_view_t calibration_screen;
    screen_vision_tuning_view_t vision_tuning_screen;
    screen_statistics_view_t statistics_screen;
    screen_alarms_view_t alarms_screen;
    screen_maintenance_view_t maintenance_screen;
    screen_login_view_t login_screen;
    screen_home_view_t home_screen;
    ui_screen_id_t active_screen;
    bool is_initialized;
} ui_manager_context_t;

static ui_manager_context_t s_ui_manager;
static QueueHandle_t s_loader_queue = NULL;
static TaskHandle_t s_loader_task_handle = NULL;
static volatile lv_obj_t *s_pending_load_root = NULL;
static volatile ui_screen_id_t s_pending_screen_id = UI_SCREEN_INVALID;

static void screen_loader_task(void *arg);

static const char *screen_to_name(ui_screen_id_t screen_id)
{
    switch (screen_id) {
    case UI_SCREEN_SPLASH:
        return "splash";
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
    case UI_SCREEN_HOME:
        return "home";
    default:
        return "unknown";
    }
}

static uint32_t screen_to_permission(ui_screen_id_t screen_id)
{
    uint32_t permission_mask;

    switch (screen_id) {
    case UI_SCREEN_SPLASH:
        permission_mask = 0U;
        break;
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
        permission_mask = 0U;
        break;
    case UI_SCREEN_HOME:
        permission_mask = HMI_PERMISSION_VIEW_DASHBOARD;
        break;
    default:
        permission_mask = 0U;
        break;
    }

    return permission_mask;
}

esp_err_t ui_manager_init(lv_display_t *display)
{
    BaseType_t task_result;

    if (display == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(&s_ui_manager, 0, sizeof(s_ui_manager));
    s_ui_manager.display = display;
    s_ui_manager.active_screen = UI_SCREEN_LOGIN;
    s_ui_manager.is_initialized = true;

    s_pending_load_root = NULL;
    s_pending_screen_id = UI_SCREEN_INVALID;

    s_loader_queue = xQueueCreate(UI_LOADER_QUEUE_SIZE, sizeof(ui_screen_id_t));
    if (s_loader_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create loader queue");
        return ESP_ERR_NO_MEM;
    }

    task_result = xTaskCreatePinnedToCore(
        screen_loader_task,
        "screen_loader",
        UI_LOADER_TASK_STACK_SIZE,
        NULL,
        UI_LOADER_TASK_PRIORITY,
        &s_loader_task_handle,
        ESP32TOUCH_APP_TASK_CORE_ID);

    if (task_result != pdPASS) {
        vQueueDelete(s_loader_queue);
        s_loader_queue = NULL;
        ESP_LOGE(TAG, "Failed to create loader task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "UI manager initialized with async screen loader on core %d", ESP32TOUCH_APP_TASK_CORE_ID);
    return ESP_OK;
}

void ui_manager_deinit(void)
{
    if (s_loader_task_handle != NULL) {
        vTaskDelete(s_loader_task_handle);
        s_loader_task_handle = NULL;
    }

    if (s_loader_queue != NULL) {
        vQueueDelete(s_loader_queue);
        s_loader_queue = NULL;
    }

    if (lvgl_lock_acquire(ESP32TOUCH_LVGL_LOCK_TIMEOUT_MS) == ESP_OK) {
        screen_splash_destroy(&s_ui_manager.splash_screen);
        screen_dashboard_destroy(&s_ui_manager.dashboard_screen);
        screen_recipe_destroy(&s_ui_manager.recipe_screen);
        screen_calibration_destroy(&s_ui_manager.calibration_screen);
        screen_vision_tuning_destroy(&s_ui_manager.vision_tuning_screen);
        screen_statistics_destroy(&s_ui_manager.statistics_screen);
        screen_alarms_destroy(&s_ui_manager.alarms_screen);
        screen_maintenance_destroy(&s_ui_manager.maintenance_screen);
        screen_login_destroy(&s_ui_manager.login_screen);
        screen_home_destroy(&s_ui_manager.home_screen);
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

    if ((screen_id == UI_SCREEN_LOGIN) || (screen_id == UI_SCREEN_SPLASH)) {
        return true;
    }

    return auth_manager_has_permission(permission_mask);
}

static bool is_screen_created(ui_screen_id_t screen_id)
{
    switch (screen_id) {
    case UI_SCREEN_SPLASH:
        return s_ui_manager.splash_screen.is_created;
    case UI_SCREEN_MAIN:
        return s_ui_manager.dashboard_screen.is_created;
    case UI_SCREEN_RECIPE:
        return s_ui_manager.recipe_screen.is_created;
    case UI_SCREEN_CALIBRATION:
        return s_ui_manager.calibration_screen.is_created;
    case UI_SCREEN_VISION_TUNING:
        return s_ui_manager.vision_tuning_screen.is_created;
    case UI_SCREEN_STATISTICS:
        return s_ui_manager.statistics_screen.is_created;
    case UI_SCREEN_ALARMS:
        return s_ui_manager.alarms_screen.is_created;
    case UI_SCREEN_MAINTENANCE:
        return s_ui_manager.maintenance_screen.is_created;
    case UI_SCREEN_LOGIN:
        return s_ui_manager.login_screen.is_created;
    case UI_SCREEN_HOME:
        return s_ui_manager.home_screen.is_created;
    default:
        return false;
    }
}

static lv_obj_t *get_screen_root(ui_screen_id_t screen_id)
{
    switch (screen_id) {
    case UI_SCREEN_SPLASH:
        return s_ui_manager.splash_screen.shell.root;
    case UI_SCREEN_MAIN:
        return s_ui_manager.dashboard_screen.shell.root;
    case UI_SCREEN_RECIPE:
        return s_ui_manager.recipe_screen.shell.root;
    case UI_SCREEN_CALIBRATION:
        return s_ui_manager.calibration_screen.shell.root;
    case UI_SCREEN_VISION_TUNING:
        return s_ui_manager.vision_tuning_screen.shell.root;
    case UI_SCREEN_STATISTICS:
        return s_ui_manager.statistics_screen.shell.root;
    case UI_SCREEN_ALARMS:
        return s_ui_manager.alarms_screen.shell.root;
    case UI_SCREEN_MAINTENANCE:
        return s_ui_manager.maintenance_screen.shell.root;
    case UI_SCREEN_LOGIN:
        return s_ui_manager.login_screen.shell.root;
    case UI_SCREEN_HOME:
        return s_ui_manager.home_screen.shell.root;
    default:
        return NULL;
    }
}

static esp_err_t create_screen_if_needed(ui_screen_id_t screen_id, lv_obj_t **root_object);

static void screen_loader_task(void *arg)
{
    ui_screen_id_t screen_id;
    lv_obj_t *root_object = NULL;
    esp_err_t result;

    (void)arg;
    ESP_LOGI(TAG, "Screen loader task started on core %d", xPortGetCoreID());

    for (;;) {
        if (xQueueReceive(s_loader_queue, &screen_id, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Loader: creating screen_id=%d (%s)", (int)screen_id, screen_to_name(screen_id));

            result = lvgl_lock_acquire(ESP32TOUCH_LVGL_LOCK_TIMEOUT_MS);
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "Loader: failed to acquire lock for screen_id=%d", (int)screen_id);
                continue;
            }

            root_object = NULL;
            result = create_screen_if_needed(screen_id, &root_object);

            if ((result == ESP_OK) && (root_object != NULL)) {
                s_pending_load_root = root_object;
                s_pending_screen_id = screen_id;
                ESP_LOGI(TAG, "Loader: screen_id=%d ready, root=%p", (int)screen_id, (void *)root_object);
            } else {
                ESP_LOGE(TAG, "Loader: failed to create screen_id=%d, err=%s", (int)screen_id, esp_err_to_name(result));
            }

            (void)lvgl_lock_release();
        }
    }
}

void ui_manager_try_load_pending_screen(void)
{
    lv_obj_t *root_object;
    ui_screen_id_t screen_id;

    if (s_pending_load_root == NULL) {
        return;
    }

    root_object = (lv_obj_t *)s_pending_load_root;
    screen_id = s_pending_screen_id;

    s_pending_load_root = NULL;
    s_pending_screen_id = UI_SCREEN_INVALID;

    lv_screen_load(root_object);
    s_ui_manager.active_screen = screen_id;
    (void)hmi_state_set_screen((hmi_ui_block_t)screen_id);

    ESP_LOGI(TAG, "LVGL: loaded screen_id=%d (%s), root=%p", (int)screen_id, screen_to_name(screen_id), (void *)root_object);
}

static esp_err_t create_screen_if_needed(ui_screen_id_t screen_id, lv_obj_t **root_object);

static esp_err_t create_screen_if_needed(ui_screen_id_t screen_id, lv_obj_t **root_object)
{
    esp_err_t result = ESP_OK;

    if (root_object == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (screen_id) {
    case UI_SCREEN_SPLASH:
        if (!s_ui_manager.splash_screen.is_created) {
            result = screen_splash_create(s_ui_manager.display, &s_ui_manager.splash_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.splash_screen.shell.root;
        break;
    case UI_SCREEN_MAIN:
        if (!s_ui_manager.dashboard_screen.is_created) {
            result = screen_dashboard_create(s_ui_manager.display, &s_ui_manager.dashboard_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.dashboard_screen.shell.root;
        break;
    case UI_SCREEN_RECIPE:
        if (!s_ui_manager.recipe_screen.is_created) {
            result = screen_recipe_create(s_ui_manager.display, &s_ui_manager.recipe_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.recipe_screen.shell.root;
        break;
    case UI_SCREEN_CALIBRATION:
        if (!s_ui_manager.calibration_screen.is_created) {
            result = screen_calibration_create(s_ui_manager.display, &s_ui_manager.calibration_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.calibration_screen.shell.root;
        break;
    case UI_SCREEN_VISION_TUNING:
        if (!s_ui_manager.vision_tuning_screen.is_created) {
            result = screen_vision_tuning_create(s_ui_manager.display, &s_ui_manager.vision_tuning_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.vision_tuning_screen.shell.root;
        break;
    case UI_SCREEN_STATISTICS:
        if (!s_ui_manager.statistics_screen.is_created) {
            result = screen_statistics_create(s_ui_manager.display, &s_ui_manager.statistics_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.statistics_screen.shell.root;
        break;
    case UI_SCREEN_ALARMS:
        if (!s_ui_manager.alarms_screen.is_created) {
            result = screen_alarms_create(s_ui_manager.display, &s_ui_manager.alarms_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.alarms_screen.shell.root;
        break;
    case UI_SCREEN_MAINTENANCE:
        if (!s_ui_manager.maintenance_screen.is_created) {
            result = screen_maintenance_create(s_ui_manager.display, &s_ui_manager.maintenance_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.maintenance_screen.shell.root;
        break;
    case UI_SCREEN_LOGIN:
        if (!s_ui_manager.login_screen.is_created) {
            result = screen_login_create(s_ui_manager.display, &s_ui_manager.login_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.login_screen.shell.root;
        break;
    case UI_SCREEN_HOME:
        if (!s_ui_manager.home_screen.is_created) {
            result = screen_home_create(s_ui_manager.display, &s_ui_manager.home_screen);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        *root_object = s_ui_manager.home_screen.shell.root;
        break;
    default:
        result = ESP_ERR_NOT_SUPPORTED;
        *root_object = NULL;
        break;
    }

    return result;
}

static ui_shell_t *get_current_shell(void)
{
    switch (s_ui_manager.active_screen) {
    case UI_SCREEN_SPLASH:
        return &s_ui_manager.splash_screen.shell;
    case UI_SCREEN_MAIN:
        return &s_ui_manager.dashboard_screen.shell;
    case UI_SCREEN_RECIPE:
        return &s_ui_manager.recipe_screen.shell;
    case UI_SCREEN_CALIBRATION:
        return &s_ui_manager.calibration_screen.shell;
    case UI_SCREEN_VISION_TUNING:
        return &s_ui_manager.vision_tuning_screen.shell;
    case UI_SCREEN_STATISTICS:
        return &s_ui_manager.statistics_screen.shell;
    case UI_SCREEN_ALARMS:
        return &s_ui_manager.alarms_screen.shell;
    case UI_SCREEN_MAINTENANCE:
        return &s_ui_manager.maintenance_screen.shell;
    case UI_SCREEN_LOGIN:
        return &s_ui_manager.login_screen.shell;
    case UI_SCREEN_HOME:
        return &s_ui_manager.home_screen.shell;
    default:
        return NULL;
    }
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

    session_manager_touch();

    /* Check if screen is already created */
    if (is_screen_created(screen_id)) {
        /* Screen exists: load it directly (synchronous path) */
        result = lvgl_lock_acquire(ESP32TOUCH_LVGL_LOCK_TIMEOUT_MS);
        if (result != ESP_OK) {
            return result;
        }

        root_object = get_screen_root(screen_id);
        if (root_object == NULL) {
            (void)lvgl_lock_release();
            ESP_LOGE(TAG, "Screen_id=%d created but root is NULL", (int)screen_id);
            return ESP_ERR_INVALID_STATE;
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
    } else {
        /* Screen not created: post to loader queue (asynchronous path) */
        if (s_loader_queue == NULL) {
            ESP_LOGE(TAG, "Loader queue not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        if (xQueueSend(s_loader_queue, &screen_id, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Loader queue full, screen_id=%d not queued", (int)screen_id);
            return ESP_ERR_NO_MEM;
        }

        ESP_LOGI(TAG, "Screen_id=%d (%s) queued for async creation", (int)screen_id, screen_to_name(screen_id));
    }

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

void ui_manager_update_alarm_banner(void)
{
    ui_shell_t *shell;
    uint32_t crit;
    uint32_t warn;
    uint32_t info;

    if (!s_ui_manager.is_initialized) {
        return;
    }
    shell = get_current_shell();
    if (shell == NULL) {
        return;
    }
    crit = alarm_manager_get_active_count(ALARM_SEVERITY_CRITICAL);
    warn = alarm_manager_get_active_count(ALARM_SEVERITY_WARNING);
    info = alarm_manager_get_active_count(ALARM_SEVERITY_INFO);
    ui_shell_set_alarm_banner(shell, crit, warn, info);
}

typedef struct
{
    lv_obj_t *mbox;
    uint32_t id;
} critical_popup_ud_t;

static void critical_alarm_ack_cb(lv_event_t *e)
{
    critical_popup_ud_t *ud = (critical_popup_ud_t *)lv_event_get_user_data(e);
    if (ud != NULL) {
        (void)alarm_manager_ack(ud->id);
        if (ud->mbox != NULL) {
            lv_msgbox_close(ud->mbox);
        }
    }
}

void ui_manager_show_critical_alarm_popup(uint32_t id, const char *message)
{
    hmi_runtime_state_t state;
    hmi_language_t lang = HMI_LANGUAGE_EN;
    lv_obj_t *mbox;
    lv_obj_t *ack_btn;
    char title_buf[32];
    static critical_popup_ud_t s_popup_ud;

    if (message == NULL) {
        return;
    }
    if (!s_ui_manager.is_initialized) {
        return;
    }
    if (hmi_state_get(&state) == ESP_OK) {
        lang = state.current_language;
    }
    (void)lv_snprintf(title_buf, sizeof(title_buf), "%s", i18n_table_get(lang, I18N_KEY_ALARM_CRITICAL));
    mbox = lv_msgbox_create(NULL);
    if (mbox != NULL) {
        lv_msgbox_add_title(mbox, title_buf);
        lv_msgbox_add_text(mbox, message);
        ack_btn = lv_msgbox_add_footer_button(mbox, i18n_table_get(lang, I18N_KEY_ACK));
        s_popup_ud.mbox = mbox;
        s_popup_ud.id = id;
        if (ack_btn != NULL) {
            lv_obj_add_event_cb(ack_btn, critical_alarm_ack_cb, LV_EVENT_CLICKED, &s_popup_ud);
        }
        lv_msgbox_add_close_button(mbox);
    }
}
