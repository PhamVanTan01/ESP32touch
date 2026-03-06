#include "ui/components/ui_nav_bar.h"

#include <string.h>

#include "esp_log.h"

#include "lvgl.h"

static const char *TAG = "ui_nav_bar";

enum
{
    UI_NAV_BAR_HEIGHT = 42,
    UI_NAV_MORE_PANEL_WIDTH = 140,
    UI_NAV_MORE_PANEL_HEIGHT = 124
};

typedef struct
{
    ui_nav_bar_t *nav_bar;
    ui_screen_id_t screen_id;
} nav_button_context_t;

static nav_button_context_t s_dashboard_context;
static nav_button_context_t s_recipe_context;
static nav_button_context_t s_vision_context;
static nav_button_context_t s_alarms_context;
static nav_button_context_t s_statistics_context;
static nav_button_context_t s_calibration_context;
static nav_button_context_t s_maintenance_context;
static nav_button_context_t s_login_context;
static ui_nav_bar_t *s_open_more_nav_bar;

static bool is_more_screen(ui_screen_id_t screen_id)
{
    return (screen_id == UI_SCREEN_STATISTICS) ||
           (screen_id == UI_SCREEN_CALIBRATION) ||
           (screen_id == UI_SCREEN_MAINTENANCE) ||
           (screen_id == UI_SCREEN_LOGIN);
}

static void set_button_active_style(lv_obj_t *button, bool is_active)
{
    if (button == NULL) {
        return;
    }

    if (is_active) {
        lv_obj_set_style_bg_color(button, lv_palette_main(LV_PALETTE_INDIGO), 0);
        lv_obj_set_style_text_color(button, lv_color_white(), 0);
    } else {
        lv_obj_set_style_bg_color(button, lv_color_hex(0x1A2438), 0);
        lv_obj_set_style_text_color(button, lv_palette_main(LV_PALETTE_GREY), 0);
    }
}

static void hide_more_panel(ui_nav_bar_t *nav_bar)
{
    if ((nav_bar != NULL) && (nav_bar->more_panel != NULL)) {
        lv_obj_add_flag(nav_bar->more_panel, LV_OBJ_FLAG_HIDDEN);
        if (s_open_more_nav_bar == nav_bar) {
            s_open_more_nav_bar = NULL;
        }
    }
}

static void show_more_panel(ui_nav_bar_t *nav_bar)
{
    if ((nav_bar != NULL) && (nav_bar->more_panel != NULL)) {
        lv_obj_remove_flag(nav_bar->more_panel, LV_OBJ_FLAG_HIDDEN);
        s_open_more_nav_bar = nav_bar;
    }
}

static void nav_button_event_cb(lv_event_t *event)
{
    esp_err_t result;
    nav_button_context_t *context = (nav_button_context_t *)lv_event_get_user_data(event);

    if (context == NULL) {
        return;
    }

    hide_more_panel(context->nav_bar);
    result = ui_manager_show(context->screen_id);
    if (result != ESP_OK) {
        ESP_LOGW(TAG,
                 "Navigation denied/failed for screen_id=%d (err=%s)",
                 (int)context->screen_id,
                 esp_err_to_name(result));
    }
}

static void nav_more_button_event_cb(lv_event_t *event)
{
    ui_nav_bar_t *nav_bar = (ui_nav_bar_t *)lv_event_get_user_data(event);

    if ((nav_bar == NULL) || (nav_bar->more_panel == NULL)) {
        return;
    }

    if (lv_obj_has_flag(nav_bar->more_panel, LV_OBJ_FLAG_HIDDEN)) {
        show_more_panel(nav_bar);
    } else {
        hide_more_panel(nav_bar);
    }
}

static lv_obj_t *create_nav_button(lv_obj_t *parent,
                                   const char *label_text,
                                   nav_button_context_t *context,
                                   lv_event_cb_t event_cb,
                                   void *user_data)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_t *label = lv_label_create(button);

    lv_obj_set_style_radius(button, 10, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x1A2438), 0);
    lv_obj_set_style_border_width(button, 0, 0);
    lv_obj_set_style_pad_hor(button, 10, 0);
    lv_obj_set_style_pad_ver(button, 6, 0);
    lv_obj_add_flag(button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);

    lv_label_set_text(label, label_text);
    lv_obj_center(label);

    if (event_cb != NULL) {
        lv_obj_add_event_cb(button, event_cb, LV_EVENT_CLICKED, (user_data != NULL) ? user_data : context);
    }

    return button;
}

static void init_context(nav_button_context_t *context, ui_nav_bar_t *nav_bar, ui_screen_id_t screen_id)
{
    if (context != NULL) {
        context->nav_bar = nav_bar;
        context->screen_id = screen_id;
    }
}

void ui_nav_bar_set_active(ui_nav_bar_t *nav_bar, ui_screen_id_t active_screen)
{
    if (nav_bar == NULL) {
        return;
    }

    set_button_active_style(nav_bar->button_dashboard, active_screen == UI_SCREEN_MAIN);
    set_button_active_style(nav_bar->button_recipe, active_screen == UI_SCREEN_RECIPE);
    set_button_active_style(nav_bar->button_vision, active_screen == UI_SCREEN_VISION_TUNING);
    set_button_active_style(nav_bar->button_alarms, active_screen == UI_SCREEN_ALARMS);
    set_button_active_style(nav_bar->button_more, is_more_screen(active_screen));
}

esp_err_t ui_nav_bar_create(lv_obj_t *parent,
                            ui_screen_id_t active_screen,
                            ui_nav_bar_t *nav_bar)
{
    lv_obj_t *button;

    if ((parent == NULL) || (nav_bar == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(nav_bar, 0, sizeof(*nav_bar));

    nav_bar->root = lv_obj_create(parent);
    if (nav_bar->root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    lv_obj_set_size(nav_bar->root, lv_pct(100), UI_NAV_BAR_HEIGHT);
    lv_obj_set_style_radius(nav_bar->root, 0, 0);
    lv_obj_set_style_border_width(nav_bar->root, 0, 0);
    lv_obj_set_style_bg_color(nav_bar->root, lv_color_hex(0x0E1526), 0);
    lv_obj_set_style_pad_all(nav_bar->root, 4, 0);
    lv_obj_set_layout(nav_bar->root, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(nav_bar->root, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(nav_bar->root, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(nav_bar->root, LV_OBJ_FLAG_SCROLLABLE);

    init_context(&s_dashboard_context, nav_bar, UI_SCREEN_MAIN);
    init_context(&s_recipe_context, nav_bar, UI_SCREEN_RECIPE);
    init_context(&s_vision_context, nav_bar, UI_SCREEN_VISION_TUNING);
    init_context(&s_alarms_context, nav_bar, UI_SCREEN_ALARMS);
    init_context(&s_statistics_context, nav_bar, UI_SCREEN_STATISTICS);
    init_context(&s_calibration_context, nav_bar, UI_SCREEN_CALIBRATION);
    init_context(&s_maintenance_context, nav_bar, UI_SCREEN_MAINTENANCE);
    init_context(&s_login_context, nav_bar, UI_SCREEN_LOGIN);

    nav_bar->button_dashboard = create_nav_button(nav_bar->root, "Dash", &s_dashboard_context, nav_button_event_cb, NULL);
    nav_bar->button_recipe = create_nav_button(nav_bar->root, "Recipe", &s_recipe_context, nav_button_event_cb, NULL);
    nav_bar->button_vision = create_nav_button(nav_bar->root, "Vision", &s_vision_context, nav_button_event_cb, NULL);
    nav_bar->button_alarms = create_nav_button(nav_bar->root, "Alarms", &s_alarms_context, nav_button_event_cb, NULL);
    nav_bar->button_more = create_nav_button(nav_bar->root, "More", NULL, nav_more_button_event_cb, nav_bar);

    if (!ui_manager_can_access_screen(UI_SCREEN_RECIPE)) {
        lv_obj_add_flag(nav_bar->button_recipe, LV_OBJ_FLAG_HIDDEN);
    }
    if (!ui_manager_can_access_screen(UI_SCREEN_VISION_TUNING)) {
        lv_obj_add_flag(nav_bar->button_vision, LV_OBJ_FLAG_HIDDEN);
    }
    if (!ui_manager_can_access_screen(UI_SCREEN_ALARMS)) {
        lv_obj_add_flag(nav_bar->button_alarms, LV_OBJ_FLAG_HIDDEN);
    }

    nav_bar->more_panel = lv_obj_create(parent);
    if (nav_bar->more_panel == NULL) {
        ui_nav_bar_destroy(nav_bar);
        return ESP_ERR_NO_MEM;
    }

    lv_obj_set_size(nav_bar->more_panel, UI_NAV_MORE_PANEL_WIDTH, UI_NAV_MORE_PANEL_HEIGHT);
    lv_obj_align(nav_bar->more_panel, LV_ALIGN_BOTTOM_RIGHT, -8, -(UI_NAV_BAR_HEIGHT + 8));
    lv_obj_set_style_radius(nav_bar->more_panel, 12, 0);
    lv_obj_set_style_bg_color(nav_bar->more_panel, lv_color_hex(0x162033), 0);
    lv_obj_set_style_border_width(nav_bar->more_panel, 1, 0);
    lv_obj_set_style_border_color(nav_bar->more_panel, lv_palette_main(LV_PALETTE_BLUE_GREY), 0);
    lv_obj_set_style_pad_all(nav_bar->more_panel, 6, 0);
    lv_obj_set_layout(nav_bar->more_panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(nav_bar->more_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(nav_bar->more_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(nav_bar->more_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(nav_bar->more_panel, LV_OBJ_FLAG_HIDDEN);

    button = create_nav_button(nav_bar->more_panel, "Statistics", &s_statistics_context, nav_button_event_cb, NULL);
    if (!ui_manager_can_access_screen(UI_SCREEN_STATISTICS)) {
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
    }

    button = create_nav_button(nav_bar->more_panel, "Calibration", &s_calibration_context, nav_button_event_cb, NULL);
    if (!ui_manager_can_access_screen(UI_SCREEN_CALIBRATION)) {
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
    }

    button = create_nav_button(nav_bar->more_panel, "Maintenance", &s_maintenance_context, nav_button_event_cb, NULL);
    if (!ui_manager_can_access_screen(UI_SCREEN_MAINTENANCE)) {
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
    }

    button = create_nav_button(nav_bar->more_panel, "Login", &s_login_context, nav_button_event_cb, NULL);
    if (!ui_manager_can_access_screen(UI_SCREEN_LOGIN)) {
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
    }

    ui_nav_bar_set_active(nav_bar, active_screen);
    return ESP_OK;
}

void ui_nav_bar_destroy(ui_nav_bar_t *nav_bar)
{
    if (nav_bar == NULL) {
        return;
    }

    if (nav_bar->more_panel != NULL) {
        lv_obj_delete(nav_bar->more_panel);
        nav_bar->more_panel = NULL;
    }

    if (nav_bar->root != NULL) {
        lv_obj_delete(nav_bar->root);
        nav_bar->root = NULL;
    }
}
