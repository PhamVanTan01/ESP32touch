#pragma once

#include "esp_err.h"
#include "lvgl.h"

/** Callback when user confirms: (password, user_data). Caller verifies and performs action. */
typedef void (*ui_confirm_password_cb_t)(const char *password, void *user_data);

/** Show modal dialog: title, message, password input, OK/Cancel. OK invokes callback with password. */
esp_err_t ui_confirm_dialog_show(lv_obj_t *parent,
                                 const char *title,
                                 const char *message,
                                 ui_confirm_password_cb_t on_confirm,
                                 void *user_data);
