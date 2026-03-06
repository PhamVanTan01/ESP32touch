#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "lvgl.h"

#include "application/data_manager.h"

typedef enum
{
    UI_SCREEN_SPLASH = 0,
    UI_SCREEN_MAIN,
    UI_SCREEN_RECIPE,
    UI_SCREEN_CALIBRATION,
    UI_SCREEN_VISION_TUNING,
    UI_SCREEN_STATISTICS,
    UI_SCREEN_ALARMS,
    UI_SCREEN_MAINTENANCE,
    UI_SCREEN_LOGIN,
    UI_SCREEN_HOME
} ui_screen_id_t;

esp_err_t ui_manager_init(lv_display_t *display);
esp_err_t ui_manager_show(ui_screen_id_t screen_id);
void ui_manager_deinit(void);
ui_screen_id_t ui_manager_get_active_screen(void);
bool ui_manager_can_access_screen(ui_screen_id_t screen_id);
esp_err_t ui_manager_update_system_status(const app_data_snapshot_t *snapshot);
void ui_manager_update_alarm_banner(void);
void ui_manager_show_critical_alarm_popup(uint32_t id, const char *message);
void ui_manager_try_load_pending_screen(void);
