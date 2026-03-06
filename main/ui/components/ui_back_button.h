/**
 * @file ui_back_button.h
 * @brief Back button component for screen navigation
 * 
 * Replaces navigation bar with simple back-to-home button.
 * MISRA-C compliant: Explicit types, no magic numbers, input validation.
 * 
 * @note Created as part of UI refactoring to remove navigation bar bottleneck
 * @date 2026-03-07
 */

#ifndef UI_BACK_BUTTON_H
#define UI_BACK_BUTTON_H

#include "esp_err.h"
#include "lvgl.h"
#include "ui/ui_manager.h"
#include "config/ui_theme.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque back button context structure
 * 
 * Stores target screen ID for navigation.
 * MISRA Rule 8.7: Object with internal linkage should be defined in one file only.
 */
typedef struct ui_back_button_context_s ui_back_button_context_t;

/**
 * @brief Create back button component
 * 
 * @param parent Parent LVGL object (must not be NULL)
 * @param back_to_screen Target screen ID to navigate when clicked
 * @return lv_obj_t* Back button object, or NULL on failure
 * 
 * @note Caller must validate parent before calling (MISRA Rule 17.3)
 * @note Button automatically destroyed when parent destroyed
 */
lv_obj_t* ui_back_button_create(lv_obj_t *parent, ui_screen_id_t back_to_screen);

/**
 * @brief Set back button visibility
 * 
 * @param button Back button object (must not be NULL)
 * @param visible true to show, false to hide
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if button is NULL
 * 
 * MISRA Rule 17.3: All function parameters must be validated.
 */
esp_err_t ui_back_button_set_visible(lv_obj_t *button, bool visible);

/**
 * @brief Update back button target screen
 * 
 * @param button Back button object (must not be NULL)
 * @param new_target New target screen ID
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if button is NULL
 * 
 * Use case: Dynamic navigation (e.g., Recipe → Calibration → back to Recipe, not HOME)
 */
esp_err_t ui_back_button_update_target(lv_obj_t *button, ui_screen_id_t new_target);

#ifdef __cplusplus
}
#endif

#endif /* UI_BACK_BUTTON_H */
