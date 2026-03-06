#include "ui/components/ui_status_card.h"

#include <string.h>

#include "config/ui_theme.h"

lv_obj_t *ui_status_card_create(lv_obj_t *parent, const char *title_text, lv_coord_t width, lv_coord_t height)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_t *title = lv_label_create(card);
    lv_obj_t *value = lv_label_create(card);

    lv_obj_set_size(card, width, height);
    lv_obj_set_style_radius(card, UI_THEME_CARD_RADIUS, 0);
    lv_obj_set_style_border_width(card, UI_THEME_CARD_BORDER_WIDTH, 0);
    lv_obj_set_style_border_color(card, lv_palette_main(LV_PALETTE_BLUE_GREY), 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(UI_THEME_COLOR_CARD_BG), 0);
    lv_obj_set_style_pad_all(card, UI_THEME_CARD_PAD, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_label_set_text(title, title_text);
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_set_width(value, (lv_coord_t)(width - UI_THEME_CARD_VALUE_MARGIN));
    lv_label_set_long_mode(value, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(value, lv_color_white(), 0);
    lv_obj_align(value, LV_ALIGN_TOP_LEFT, 0, UI_THEME_CARD_VALUE_OFFSET);

    return value;
}

void ui_status_card_set_value(lv_obj_t *label, const char *text)
{
    if ((label != NULL) && (text != NULL) && (strcmp(lv_label_get_text(label), text) != 0)) {
        lv_label_set_text(label, text);
    }
}
