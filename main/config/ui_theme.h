#pragma once

#include <stdint.h>

#include "bsp/board_config.h"

/* Display dimensions from BSP (single source of truth). */
#define UI_DISPLAY_WIDTH   ESP32TOUCH_LCD_HORIZONTAL_RESOLUTION
#define UI_DISPLAY_HEIGHT  ESP32TOUCH_LCD_VERTICAL_RESOLUTION

/* Shell layout (pixels). */
#define UI_THEME_STATUS_BAR_HEIGHT      22
#define UI_THEME_LANG_BTN_WIDTH         40
#define UI_THEME_LANG_BTN_HEIGHT        18
#define UI_THEME_LANG_PANEL_WIDTH       120
#define UI_THEME_LANG_PANEL_ITEM_H      28
#define UI_THEME_TITLE_BAR_HEIGHT       30
#define UI_THEME_HELP_BUTTON_SIZE       36
#define UI_THEME_NAV_BAR_HEIGHT        42
#define UI_THEME_HORIZONTAL_PADDING    8
#define UI_THEME_CONTENT_TOP           56
#define UI_THEME_CONTENT_BOTTOM_MARGIN 48

#define UI_THEME_CONTENT_HEIGHT \
    (UI_DISPLAY_HEIGHT - (UI_THEME_CONTENT_TOP) - (UI_THEME_CONTENT_BOTTOM_MARGIN))

/* Content area padding (Phase 5.2 – consistent layout). */
#define UI_THEME_CONTENT_PADDING  8

/* Touch target and button minimum (ISO audit, JSON spec). */
#define UI_THEME_TOUCH_TARGET_MIN      36
#define UI_THEME_BUTTON_MIN_WIDTH      36
#define UI_THEME_BUTTON_MIN_HEIGHT     36

/* Nav bar (JSON spec: 40×40, gap 4, more_panel Y 50). */
#define UI_THEME_NAV_MORE_PANEL_WIDTH  140
#define UI_THEME_NAV_MORE_PANEL_HEIGHT 124
#define UI_THEME_NAV_PANEL_OFFSET      8
#define UI_THEME_NAV_PANEL_OFFSET_Y    50
#define UI_THEME_NAV_BUTTON_WIDTH      40
#define UI_THEME_NAV_BUTTON_HEIGHT     40
#define UI_THEME_NAV_BUTTON_GAP        4
#define UI_THEME_NAV_BUTTON_RADIUS     10
#define UI_THEME_NAV_BUTTON_PAD_HOR    10
#define UI_THEME_NAV_BUTTON_PAD_VER    6
#define UI_THEME_NAV_MORE_PANEL_PAD    6
/* Back button (NEW - replaces nav bar, eliminates watchdog timeout) */
#define UI_THEME_BACK_BUTTON_ENABLED     (1U)
#define UI_BACK_BUTTON_WIDTH             (70U)
#define UI_BACK_BUTTON_HEIGHT            (24U)
#define UI_BACK_BUTTON_MARGIN_LEFT       (8U)
#define UI_BACK_BUTTON_MARGIN_TOP        (3U)
#define UI_BACK_BUTTON_RADIUS            (4U)
#define UI_BACK_BUTTON_BG_COLOR          (0x3E3E4EU)
#define UI_BACK_BUTTON_PRESSED_COLOR     (0x5A5A6EU)
#define UI_BACK_BUTTON_TEXT_COLOR        (0xFFFFFFU)
#define UI_BACK_BUTTON_BORDER_WIDTH      (0U)
#define UI_BACK_BUTTON_PAD_HOR           (8U)
#define UI_BACK_BUTTON_PAD_VER           (4U)

/* Colors (hex, LVGL format – JSON theme audit). */
#define UI_THEME_COLOR_ROOT_BG         0x1A1A23U
#define UI_THEME_COLOR_STATUS_BAR_BG   0x2A2A35U
#define UI_THEME_COLOR_TITLE_BAR_BG    0x3A3A45U
#define UI_THEME_COLOR_NAV_BAR_BG      0x2A2A35U
#define UI_THEME_COLOR_NAV_BUTTON_BG   0x3E3E4EU
#define UI_THEME_COLOR_NAV_MORE_PANEL  0x162033U
#define UI_THEME_COLOR_CARD_BG         0x1E1E2EU

/* Alarm severity banner colors (ISA-18.2). */
#define UI_THEME_ALARM_CRITICAL_COLOR  0xB91C1CU
#define UI_THEME_ALARM_WARNING_COLOR   0xB45309U
#define UI_THEME_ALARM_INFO_COLOR      0x0369A1U

/* Confirm dialog (JSON spec). */
#define UI_THEME_DIALOG_WIDTH           220
#define UI_THEME_DIALOG_HEIGHT          180
#define UI_THEME_DIALOG_MESSAGE_WIDTH   204
#define UI_THEME_DIALOG_INPUT_WIDTH     200
#define UI_THEME_DIALOG_INPUT_HEIGHT    32
#define UI_THEME_DIALOG_BTN_WIDTH       72
#define UI_THEME_DIALOG_BTN_HEIGHT      36
#define UI_THEME_DIALOG_CANCEL_OFFSET_X 88

/* Status card. */
#define UI_THEME_CARD_RADIUS           12
#define UI_THEME_CARD_BORDER_WIDTH     1
#define UI_THEME_CARD_PAD              10
#define UI_THEME_CARD_VALUE_OFFSET     24
#define UI_THEME_CARD_VALUE_MARGIN     20

/* Login form layout (ISO 9241-410, IEC 62624, JSON spec). */
#define UI_THEME_LOGIN_MARGIN_X            8
#define UI_THEME_LOGIN_SPACING_V           12
#define UI_THEME_LOGIN_LANG_BTN_GAP        10
#define UI_THEME_LOGIN_KEYBOARD_HEIGHT_PX  75
#define UI_THEME_LOGIN_INPUT_WIDTH         200
#define UI_THEME_LOGIN_INPUT_HEIGHT        36
#define UI_THEME_LOGIN_BUTTON_WIDTH        200
#define UI_THEME_LOGIN_BUTTON_HEIGHT       44
#define UI_THEME_LOGIN_LANG_LABEL_Y        0
#define UI_THEME_LOGIN_LANG_BTN_Y          20
#define UI_THEME_LOGIN_LANG_ROW_H          40
#define UI_THEME_LOGIN_TITLE_Y             10
#define UI_THEME_LOGIN_ROW1_Y              40
#define UI_THEME_LOGIN_ROW1_INPUT_Y        70
#define UI_THEME_LOGIN_ROW2_Y              116
#define UI_THEME_LOGIN_ROW2_INPUT_Y        146
#define UI_THEME_LOGIN_BUTTON_Y            192
#define UI_THEME_LOGIN_STATUS_Y            246
#define UI_THEME_LOGIN_LANG_BTN_WIDTH      48
#define UI_THEME_LOGIN_LANG_BTN_HEIGHT     26
#define UI_THEME_LOGIN_STATUS_MAX_HEIGHT   48
#define UI_THEME_LOGIN_STATUS_LABEL_WIDTH  200
/* Shutdown button (IEC 62624: away from main action). */
#define UI_THEME_LOGIN_SHUTDOWN_BTN_WIDTH   80
#define UI_THEME_LOGIN_SHUTDOWN_BTN_HEIGHT  40
#define UI_THEME_LOGIN_SHUTDOWN_OFFSET_BOTTOM 20
/* HW status icon (FPGA/STM32). */
#define UI_THEME_LOGIN_HW_STATUS_ICON_SIZE   24
#define UI_THEME_LOGIN_HW_STATUS_OFFSET_X    8
#define UI_THEME_LOGIN_HW_STATUS_OFFSET_Y    8
#define UI_THEME_LOGIN_STATUS_BUF_SIZE       64

/* Dashboard (screen_main) layout (JSON spec). */
#define UI_THEME_MAIN_GRID_WIDTH      224
#define UI_THEME_MAIN_GRID_HEIGHT     180
#define UI_THEME_MAIN_GRID_ROW_H       86
#define UI_THEME_MAIN_CARD_WIDTH      104
#define UI_THEME_MAIN_CARD_HEIGHT     98
#define UI_THEME_MAIN_BUTTON_WIDTH    84
#define UI_THEME_MAIN_BUTTON_HEIGHT   28

/* Home menu (JSON: grid 2 columns). */
#define UI_THEME_HOME_BUTTON_WIDTH    108
#define UI_THEME_HOME_BUTTON_HEIGHT   44
#define UI_THEME_HOME_BUTTON_GAP      8
#define UI_THEME_HOME_GRID_COLUMNS    2

/* Alarms list (JSON spec). */
#define UI_THEME_ALARM_ROW_HEIGHT     36
#define UI_THEME_ALARM_ROW_MARGIN     4
#define UI_THEME_ALARM_ACK_BTN_WIDTH  48
#define UI_THEME_ALARM_ACK_BTN_HEIGHT 28
#define UI_THEME_ALARM_LIST_WIDTH     224

/* Splash (Phase 4.1). */
#define UI_THEME_SPLASH_DISPLAY_MS    2000
