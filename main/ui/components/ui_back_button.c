/**
 * @file ui_back_button.c
 * @brief Back button component implementation
 * 
 * MISRA-C compliant implementation of simple back navigation button.
 * Replaces complex navigation bar that caused watchdog timeouts.
 */

#include "ui/components/ui_back_button.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "application/hmi_state.h"
#include "esp_log.h"
#include "middleware/i18n/i18n_table.h"

static const char *TAG = "ui_back_button";

static bool is_valid_screen_id(ui_screen_id_t screen_id)
{
    switch (screen_id) {
    case UI_SCREEN_SPLASH:
    case UI_SCREEN_MAIN:
    case UI_SCREEN_RECIPE:
    case UI_SCREEN_CALIBRATION:
    case UI_SCREEN_VISION_TUNING:
    case UI_SCREEN_STATISTICS:
    case UI_SCREEN_ALARMS:
    case UI_SCREEN_MAINTENANCE:
    case UI_SCREEN_LOGIN:
    case UI_SCREEN_HOME:
        return true;
    default:
        return false;
    }
}

/* Internal context structure (MISRA Rule 8.7: Internal linkage) */
struct ui_back_button_context_s {
    ui_screen_id_t target_screen;
    uint32_t reserved;  /* For future use, ensure alignment */
};

/* Forward declarations */
static void back_button_event_cb(lv_event_t *e);
static void back_button_cleanup_cb(lv_event_t *e);

/**
 * @brief Event callback for back button click
 * 
 * @param e LVGL event (must not be NULL, guaranteed by LVGL)
 * 
 * MISRA Rule 8.2: Function type shall be in scope.
 * MISRA Rule 17.7: Return value shall be used (void return allowed for callbacks).
 */
static void back_button_event_cb(lv_event_t *e)
{
    ui_back_button_context_t *context;
    ui_screen_id_t target;
    esp_err_t result;

    if (e == NULL) {
        /* Defensive: LVGL should never pass NULL, but check anyway (MISRA Rule 17.3) */
        ESP_LOGE(TAG, "Event is NULL");
        return;
    }

    context = (ui_back_button_context_t *)lv_event_get_user_data(e);
    if (context == NULL) {
        ESP_LOGE(TAG, "Context is NULL");
        return;
    }

    target = context->target_screen;

    /* Validate target screen ID (MISRA Rule 14.3: Check before use) */
    if (!is_valid_screen_id(target)) {
        ESP_LOGE(TAG, "Invalid target screen: %d", (int)target);
        return;
    }

    ESP_LOGI(TAG, "Back button clicked, target=%d", (int)target);

    result = ui_manager_show(target);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Failed to show screen %d: %s", (int)target, esp_err_to_name(result));
    }
}

/**
 * @brief Cleanup callback when button destroyed
 * 
 * @param e LVGL event
 * 
 * Frees context memory (MISRA Rule 21.3: Match malloc with free).
 */
static void back_button_cleanup_cb(lv_event_t *e)
{
    ui_back_button_context_t *context;

    if (e == NULL) {
        return;
    }

    context = (ui_back_button_context_t *)lv_event_get_user_data(e);
    if (context != NULL) {
        free(context);
        ESP_LOGD(TAG, "Freed back button context");
    }
}

lv_obj_t* ui_back_button_create(lv_obj_t *parent, ui_screen_id_t back_to_screen)
{
    lv_obj_t *button = NULL;
    lv_obj_t *label = NULL;
    ui_back_button_context_t *context = NULL;
    hmi_language_t lang;
    const char *text;

    /* Input validation (MISRA Rule 17.3) */
    if (parent == NULL) {
        ESP_LOGE(TAG, "Parent is NULL");
        return NULL;
    }

    if (!is_valid_screen_id(back_to_screen)) {
        ESP_LOGE(TAG, "Invalid screen ID: %d", (int)back_to_screen);
        return NULL;
    }

    /* Allocate context (MISRA Rule 21.3: Use heap carefully) */
    context = (ui_back_button_context_t *)malloc(sizeof(ui_back_button_context_t));
    if (context == NULL) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return NULL;
    }

    (void)memset(context, 0, sizeof(*context));
    context->target_screen = back_to_screen;

    /* Create button object */
    button = lv_btn_create(parent);
    if (button == NULL) {
        ESP_LOGE(TAG, "Failed to create button");
        free(context);
        return NULL;
    }

    /* Set button properties (MISRA Rule 2.3: No magic numbers) */
    lv_obj_set_size(button, (int32_t)UI_BACK_BUTTON_WIDTH, (int32_t)UI_BACK_BUTTON_HEIGHT);
    lv_obj_align(button, LV_ALIGN_TOP_LEFT, (int32_t)UI_BACK_BUTTON_MARGIN_LEFT, (int32_t)UI_BACK_BUTTON_MARGIN_TOP);
    
    /* Style configuration */
    lv_obj_set_style_radius(button, (int32_t)UI_BACK_BUTTON_RADIUS, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(UI_BACK_BUTTON_BG_COLOR), 0);
    lv_obj_set_style_border_width(button, (int32_t)UI_BACK_BUTTON_BORDER_WIDTH, 0);
    lv_obj_set_style_pad_hor(button, (int32_t)UI_BACK_BUTTON_PAD_HOR, 0);
    lv_obj_set_style_pad_ver(button, (int32_t)UI_BACK_BUTTON_PAD_VER, 0);

    /* Pressed state style */
    lv_obj_set_style_bg_color(button, lv_color_hex(UI_BACK_BUTTON_PRESSED_COLOR), LV_STATE_PRESSED);

    /* Create label inside button */
    label = lv_label_create(button);
    if (label == NULL) {
        ESP_LOGE(TAG, "Failed to create label");
        lv_obj_del(button);
        free(context);
        return NULL;
    }

    /* Set label text (i18n support) */
    {
        hmi_runtime_state_t runtime_state;
        if (hmi_state_get(&runtime_state) == ESP_OK) {
            lang = runtime_state.current_language;
        } else {
            lang = HMI_LANGUAGE_EN;
        }
    }
    text = i18n_table_get(lang, I18N_KEY_BACK);
    if (text == NULL) {
        text = "< Back";  /* Fallback (MISRA Rule 17.3: Defensive programming) */
        ESP_LOGW(TAG, "I18N_KEY_BACK not found, using fallback");
    }
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(UI_BACK_BUTTON_TEXT_COLOR), 0);
    lv_obj_center(label);

    /* Register event callbacks */
    lv_obj_add_event_cb(button, back_button_event_cb, LV_EVENT_CLICKED, context);
    lv_obj_add_event_cb(button, back_button_cleanup_cb, LV_EVENT_DELETE, context);

    ESP_LOGI(TAG, "Created back button, target=%d", (int)back_to_screen);

    return button;
}

esp_err_t ui_back_button_set_visible(lv_obj_t *button, bool visible)
{
    /* Input validation (MISRA Rule 17.3) */
    if (button == NULL) {
        ESP_LOGE(TAG, "Button is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (visible) {
        lv_obj_clear_flag(button, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
    }

    return ESP_OK;
}

esp_err_t ui_back_button_update_target(lv_obj_t *button, ui_screen_id_t new_target)
{
    ui_back_button_context_t *context = NULL;
    uint32_t event_count;
    uint32_t i;

    /* Input validation */
    if (button == NULL) {
        ESP_LOGE(TAG, "Button is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_valid_screen_id(new_target)) {
        ESP_LOGE(TAG, "Invalid screen ID: %d", (int)new_target);
        return ESP_ERR_INVALID_ARG;
    }

    /* Find context from event callbacks (MISRA compliant iteration) */
    event_count = lv_obj_get_event_count(button);
    for (i = 0U; i < event_count; i++) {
        lv_event_dsc_t *dsc = lv_obj_get_event_dsc(button, i);
        if (dsc != NULL) {
            void *user_data = lv_event_dsc_get_user_data(dsc);
            if (user_data != NULL) {
                context = (ui_back_button_context_t *)user_data;
                context->target_screen = new_target;
                ESP_LOGI(TAG, "Updated target to %d", (int)new_target);
                return ESP_OK;
            }
        }
    }

    ESP_LOGW(TAG, "Context not found");
    return ESP_ERR_NOT_FOUND;
}
