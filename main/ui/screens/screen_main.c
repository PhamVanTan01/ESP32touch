#include "ui/screens/screen_main.h"

#include <stdbool.h>
#include <string.h>

#include "application/hmi_state.h"
#include "config/ui_theme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "middleware/i18n/i18n_table.h"
#include "ui/components/ui_status_card.h"

static lv_display_rotation_t get_next_rotation(lv_display_rotation_t rotation)
{
    lv_display_rotation_t next_rotation;

    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        next_rotation = LV_DISPLAY_ROTATION_90;
        break;
    case LV_DISPLAY_ROTATION_90:
        next_rotation = LV_DISPLAY_ROTATION_180;
        break;
    case LV_DISPLAY_ROTATION_180:
        next_rotation = LV_DISPLAY_ROTATION_270;
        break;
    case LV_DISPLAY_ROTATION_270:
    default:
        next_rotation = LV_DISPLAY_ROTATION_0;
        break;
    }

    return next_rotation;
}

static void rotate_button_event_cb(lv_event_t *event)
{
    screen_main_view_t *view = (screen_main_view_t *)lv_event_get_user_data(event);

    if ((view != NULL) && (view->shell.display != NULL)) {
        view->rotation = get_next_rotation(view->rotation);
        lv_display_set_rotation(view->shell.display, view->rotation);
    }
}

static void set_value_label(lv_obj_t *label, const char *text)
{
    if ((label != NULL) && (text != NULL) && (strcmp(lv_label_get_text(label), text) != 0)) {
        ui_status_card_set_value(label, text);
    }
}

static void append_feature_text(char *buffer, size_t buffer_size, const char *feature_name)
{
    size_t used_length;

    if ((buffer == NULL) || (buffer_size == 0U) || (feature_name == NULL)) {
        return;
    }

    used_length = strlen(buffer);
    if (used_length >= buffer_size) {
        return;
    }

    (void)lv_snprintf(buffer + used_length,
                      buffer_size - used_length,
                      "%s%s",
                      (used_length > 0U) ? ", " : "",
                      feature_name);
}

static void build_feature_text(const app_chip_features_t *features, char *buffer, size_t buffer_size)
{
    if ((features == NULL) || (buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    buffer[0] = '\0';

    if (features->has_wifi) {
        append_feature_text(buffer, buffer_size, "WiFi");
    }
    if (features->has_ble) {
        append_feature_text(buffer, buffer_size, "BLE");
    }
    if (features->has_classic_bluetooth) {
        append_feature_text(buffer, buffer_size, "BT");
    }
    if (features->has_ieee802154) {
        append_feature_text(buffer, buffer_size, "802.15.4");
    }
    if (features->has_embedded_flash) {
        append_feature_text(buffer, buffer_size, "EmbFlash");
    }
    if (features->has_embedded_psram) {
        append_feature_text(buffer, buffer_size, "EmbPSRAM");
    }

    if (buffer[0] == '\0') {
        (void)lv_snprintf(buffer, buffer_size, "N/A");
    }
}

esp_err_t screen_main_create(lv_display_t *display, screen_main_view_t *view)
{
    static lv_coord_t column_descriptors[] = {108, 108, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_descriptors[] = {UI_THEME_MAIN_GRID_ROW_H, UI_THEME_MAIN_GRID_ROW_H, LV_GRID_TEMPLATE_LAST};
    lv_obj_t *grid;
    lv_obj_t *rotate_button;
    lv_obj_t *button_label;
    lv_obj_t *content;
    lv_obj_t *button_row;
    char status_text[64];
    hmi_runtime_state_t state;
    const char *title = "Dashboard";

    if ((display == NULL) || (view == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)memset(view, 0, sizeof(*view));
    view->rotation = LV_DISPLAY_ROTATION_0;

    if (hmi_state_get(&state) == ESP_OK) {
        title = i18n_table_get(state.current_language, I18N_KEY_DASHBOARD);
    }
    if (ui_shell_create_with_back(display, UI_SCREEN_MAIN, title, true, UI_SCREEN_HOME, &view->shell) != ESP_OK) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    (void)lv_snprintf(status_text, sizeof(status_text), "Dashboard  Chip metrics  Role active");
    ui_shell_set_status_text(&view->shell, status_text);

    content = ui_shell_get_content(&view->shell);
    if (content == NULL) {
        ui_shell_destroy(&view->shell);
        return ESP_FAIL;
    }

    grid = lv_obj_create(content);
    lv_obj_set_size(grid, UI_THEME_MAIN_GRID_WIDTH, UI_THEME_MAIN_GRID_HEIGHT);
    lv_obj_align(grid, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_dsc_array(grid, column_descriptors, row_descriptors);
    vTaskDelay(pdMS_TO_TICKS(1));

    view->chip_value_label = ui_status_card_create(grid, "Chip", UI_THEME_MAIN_CARD_WIDTH, UI_THEME_MAIN_CARD_HEIGHT);
    lv_obj_set_grid_cell(lv_obj_get_parent(view->chip_value_label),
                         LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    view->memory_value_label = ui_status_card_create(grid, "Memory", UI_THEME_MAIN_CARD_WIDTH, UI_THEME_MAIN_CARD_HEIGHT);
    lv_obj_set_grid_cell(lv_obj_get_parent(view->memory_value_label),
                         LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    view->system_value_label = ui_status_card_create(grid, "System", UI_THEME_MAIN_CARD_WIDTH, UI_THEME_MAIN_CARD_HEIGHT);
    lv_obj_set_grid_cell(lv_obj_get_parent(view->system_value_label),
                         LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    view->network_value_label = ui_status_card_create(grid, "Network", UI_THEME_MAIN_CARD_WIDTH, UI_THEME_MAIN_CARD_HEIGHT);
    lv_obj_set_grid_cell(lv_obj_get_parent(view->network_value_label),
                         LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    set_value_label(view->chip_value_label, "Loading...");
    set_value_label(view->memory_value_label, "Loading...");
    set_value_label(view->system_value_label, "Loading...");
    set_value_label(view->network_value_label, "Loading...");

    button_row = lv_obj_create(content);
    lv_obj_set_size(button_row, lv_pct(100), 30);
    lv_obj_align(button_row, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(button_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(button_row, 0, 0);
    lv_obj_set_style_pad_all(button_row, 0, 0);
    lv_obj_clear_flag(button_row, LV_OBJ_FLAG_SCROLLABLE);

    rotate_button = lv_button_create(button_row);
    lv_obj_set_size(rotate_button, UI_THEME_MAIN_BUTTON_WIDTH, UI_THEME_MAIN_BUTTON_HEIGHT);
    lv_obj_align(rotate_button, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_radius(rotate_button, 12, 0);
    lv_obj_set_style_bg_color(rotate_button, lv_palette_main(LV_PALETTE_INDIGO), 0);
    lv_obj_add_event_cb(rotate_button, rotate_button_event_cb, LV_EVENT_CLICKED, view);

    button_label = lv_label_create(rotate_button);
    lv_label_set_text(button_label, LV_SYMBOL_REFRESH " Rotate");
    lv_obj_center(button_label);

    view->is_created = true;
    return ESP_OK;
}

void screen_main_destroy(screen_main_view_t *view)
{
    if (view != NULL) {
        ui_shell_destroy(&view->shell);
        (void)memset(view, 0, sizeof(*view));
    }
}

esp_err_t screen_main_update_status(screen_main_view_t *view, const app_data_snapshot_t *snapshot)
{
    char memory_buffer[128];
    char system_buffer[128];
    char network_buffer[96];
    char chip_buffer[160];
    char temperature_buffer[16];
    char feature_buffer[64];
    int16_t whole_temperature;
    int16_t fractional_temperature;
    int16_t absolute_temperature;

    if ((view == NULL) || (snapshot == NULL) || !view->is_created) {
        return ESP_ERR_INVALID_ARG;
    }

    build_feature_text(&snapshot->chip_info.features, feature_buffer, sizeof(feature_buffer));

    if (snapshot->runtime_info.temperature_valid) {
        whole_temperature = (int16_t)(snapshot->runtime_info.temperature_deci_celsius / 10);
        absolute_temperature = (snapshot->runtime_info.temperature_deci_celsius >= 0)
                               ? snapshot->runtime_info.temperature_deci_celsius
                               : (int16_t)(-snapshot->runtime_info.temperature_deci_celsius);
        fractional_temperature = (int16_t)(absolute_temperature % 10);
        (void)lv_snprintf(temperature_buffer,
                          sizeof(temperature_buffer),
                          "%d.%d C",
                          whole_temperature,
                          fractional_temperature);
    } else {
        (void)lv_snprintf(temperature_buffer, sizeof(temperature_buffer), "N/A");
    }

    (void)lv_snprintf(chip_buffer,
                      sizeof(chip_buffer),
                      "%s rev %u\nCores: %u\n%s",
                      snapshot->chip_info.model_name,
                      (unsigned int)snapshot->chip_info.revision,
                      (unsigned int)snapshot->chip_info.cpu_core_count,
                      feature_buffer);

    (void)lv_snprintf(memory_buffer,
                      sizeof(memory_buffer),
                      "Flash: %lu MB\nPSRAM: %lu MB\nHeap: %lu KB",
                      (unsigned long)(snapshot->runtime_info.flash_size_bytes / (1024UL * 1024UL)),
                      (unsigned long)(snapshot->runtime_info.psram_size_bytes / (1024UL * 1024UL)),
                      (unsigned long)(snapshot->runtime_info.free_heap_bytes / 1024UL));

    (void)lv_snprintf(system_buffer,
                      sizeof(system_buffer),
                      "CPU: %lu MHz\nTemp: %s\nUptime: %lus\nIDF: %s",
                      (unsigned long)snapshot->runtime_info.cpu_frequency_mhz,
                      temperature_buffer,
                      (unsigned long)snapshot->runtime_info.uptime_seconds,
                      snapshot->chip_info.idf_version);

    (void)lv_snprintf(network_buffer,
                      sizeof(network_buffer),
                      "MAC:\n%s",
                      snapshot->chip_info.mac_address);

    set_value_label(view->chip_value_label, chip_buffer);
    set_value_label(view->memory_value_label, memory_buffer);
    set_value_label(view->system_value_label, system_buffer);
    set_value_label(view->network_value_label, network_buffer);

    return ESP_OK;
}
