#include "ui/screens/screen_statistics.h"

esp_err_t screen_statistics_create(lv_display_t *display, screen_statistics_view_t *view)
{
    return screen_placeholder_create(display,
                                     view,
                                     UI_SCREEN_STATISTICS,
                                     "Statistics",
                                     "TODO: Throughput, yield, defect and trend charts");
}

void screen_statistics_destroy(screen_statistics_view_t *view)
{
    screen_placeholder_destroy(view);
}
