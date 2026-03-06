#pragma once

#include "lvgl.h"

lv_obj_t *ui_status_card_create(lv_obj_t *parent, const char *title_text, lv_coord_t width, lv_coord_t height);
void ui_status_card_set_value(lv_obj_t *label, const char *text);
