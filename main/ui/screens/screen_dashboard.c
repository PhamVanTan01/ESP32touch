#include "ui/screens/screen_dashboard.h"

esp_err_t screen_dashboard_create(lv_display_t *display, screen_dashboard_view_t *view)
{
    return screen_main_create(display, view);
}

void screen_dashboard_destroy(screen_dashboard_view_t *view)
{
    screen_main_destroy(view);
}

esp_err_t screen_dashboard_update(screen_dashboard_view_t *view, const app_data_snapshot_t *snapshot)
{
    return screen_main_update_status(view, snapshot);
}
