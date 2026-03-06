#include "ui/screens/screen_vision_tuning.h"

esp_err_t screen_vision_tuning_create(lv_display_t *display, screen_vision_tuning_view_t *view)
{
    return screen_placeholder_create(display,
                                     view,
                                     UI_SCREEN_VISION_TUNING,
                                     "Vision tuning",
                                     "TODO: Threshold, ROI, color class and sensitivity tuning");
}

void screen_vision_tuning_destroy(screen_vision_tuning_view_t *view)
{
    screen_placeholder_destroy(view);
}
