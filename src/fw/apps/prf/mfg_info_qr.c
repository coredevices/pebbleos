/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/app_watch_info.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/qr_code.h"
#include "applib/ui/window.h"
#include "applib/ui/text_layer.h"
#include "kernel/pbl_malloc.h"
#include "mfg/mfg_info.h"
#include "mfg/mfg_serials.h"
#include "process_state/app_state/app_state.h"
#include "process_management/pebble_process_md.h"
#include "pbl/services/battery/battery_state.h"
#include "pbl/services/bluetooth/local_id.h"
#include "pbl/util/size.h"

#include "git_version.auto.h"

#include <bluetooth/bluetooth_types.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
  WatchInfoColor color;
  const char *short_name;
} ColorTable;

static const ColorTable s_color_table[] = {
#ifdef CONFIG_BOARD_ASTERIX
  { .color = WATCH_INFO_COLOR_COREDEVICES_P2D_BLACK, .short_name = "BK" },
  { .color = WATCH_INFO_COLOR_COREDEVICES_P2D_WHITE, .short_name = "WH" },
#elif defined(CONFIG_BOARD_OBELIX)
  { .color = WATCH_INFO_COLOR_COREDEVICES_PT2_BLACK_GREY, .short_name = "BG" },
  { .color = WATCH_INFO_COLOR_COREDEVICES_PT2_BLACK_RED, .short_name = "BR" },
  { .color = WATCH_INFO_COLOR_COREDEVICES_PT2_SILVER_BLUE, .short_name = "SB" },
  { .color = WATCH_INFO_COLOR_COREDEVICES_PT2_SILVER_GREY, .short_name = "SG" },
#elif defined(CONFIG_BOARD_GETAFIX)
  { .color = WATCH_INFO_COLOR_COREDEVICES_PR2_BLACK_20, .short_name = "BK20" },
  { .color = WATCH_INFO_COLOR_COREDEVICES_PR2_SILVER_14, .short_name = "SV14" },
  { .color = WATCH_INFO_COLOR_COREDEVICES_PR2_SILVER_20, .short_name = "SV20" },
  { .color = WATCH_INFO_COLOR_COREDEVICES_PR2_GOLD_14, .short_name = "GD14" },
#endif
};

static const char* prv_get_color_short_name(WatchInfoColor color) {
  // Pointer-based iteration so an empty table (boards with no color list)
  // compiles without a `< 0` unsigned comparison warning.
  for (const ColorTable *entry = s_color_table;
       entry < s_color_table + ARRAY_LENGTH(s_color_table); entry++) {
    if (entry->color == color) {
      return entry->short_name;
    }
  }
  return "??";
}

typedef struct {
  Window window;

  QRCode qr_code;
  TextLayer serial;

  char serial_buffer[MFG_SERIAL_NUMBER_SIZE + 1];
  char bt_mac_buffer[BT_DEVICE_ADDRESS_FMT_BUFFER_SIZE];
  char qr_buffer[128];
} AppData;

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

  // Get all the required information
  const char *serial_number = mfg_get_serial_number();
  strncpy(data->serial_buffer, serial_number, MFG_SERIAL_NUMBER_SIZE);
  data->serial_buffer[MFG_SERIAL_NUMBER_SIZE] = '\0';

  bt_local_id_copy_address_mac_string(data->bt_mac_buffer);

  BatteryChargeState battery_state = battery_get_charge_state();
  uint16_t battery_mv = battery_state_get_voltage();
  uint8_t battery_pct = battery_state.charge_percent;

  WatchInfoColor watch_color = mfg_info_get_watch_color();
  const char *color_short_name = prv_get_color_short_name(watch_color);

  // Build QR code string
  snprintf(data->qr_buffer, sizeof(data->qr_buffer),
           "%s;%s;%s;%u;%u;%s",
           serial_number, data->bt_mac_buffer, GIT_TAG, battery_mv, battery_pct, color_short_name);

  QRCode* qr_code = &data->qr_code;
  qr_code_init_with_parameters(qr_code,
#if PBL_ROUND
#define QR_CODE_SIZE ((window->layer.bounds.size.w * 10) / 14)
                               &GRect((window->layer.bounds.size.w - QR_CODE_SIZE) / 2,
                                      (window->layer.bounds.size.h - QR_CODE_SIZE) / 2,
                                      QR_CODE_SIZE, QR_CODE_SIZE),
#else
                               &GRect(10, 10, window->layer.bounds.size.w - 20,
                                      window->layer.bounds.size.h - 30),
#endif
                               data->qr_buffer, strlen(data->qr_buffer), QRCodeECCMedium,
                               GColorBlack, GColorWhite);
  layer_add_child(&window->layer, &qr_code->layer);

  TextLayer* serial = &data->serial;
  text_layer_init_with_parameters(serial,
                                  &GRect(0, window->layer.bounds.size.h - PBL_IF_RECT_ELSE(20, 40),
                                         window->layer.bounds.size.w, 20),
                                         data->serial_buffer, fonts_get_system_font(FONT_KEY_GOTHIC_14),
                                  GColorBlack, GColorWhite, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &serial->layer);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd* mfg_info_qr_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: 4f8a2d3e-1c5b-4a9f-8e7d-6c3b2a1f0e9d
    .common.uuid = { 0x4f, 0x8a, 0x2d, 0x3e, 0x1c, 0x5b, 0x4a, 0x9f,
                     0x8e, 0x7d, 0x6c, 0x3b, 0x2a, 0x1f, 0x0e, 0x9d },
    .name = "MfgInfoQR",
  };
  return (const PebbleProcessMd*) &s_app_info;
}
