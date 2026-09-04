/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/qr_code.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "kernel/pbl_malloc.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "services/bluetooth/bluetooth_ctl.h"
#include "services/bluetooth/local_id.h"

#include <string.h>

typedef struct {
  Window window;
  QRCode qr_code;
  TextLayer mac_label;
  char mac_buffer[BT_DEVICE_ADDRESS_FMT_BUFFER_SIZE];
} AppData;

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "BLE Adv");
  window_set_fullscreen(window, true);

  bt_local_id_copy_address_mac_string(data->mac_buffer);

  QRCode *qr_code = &data->qr_code;
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
                               data->mac_buffer, strlen(data->mac_buffer), QRCodeECCMedium,
                               GColorBlack, GColorWhite);
  layer_add_child(&window->layer, &qr_code->layer);

  TextLayer *mac_label = &data->mac_label;
  text_layer_init_with_parameters(mac_label,
                                  &GRect(0, window->layer.bounds.size.h - PBL_IF_RECT_ELSE(20, 40),
                                         window->layer.bounds.size.w, 20),
                                  data->mac_buffer, fonts_get_system_font(FONT_KEY_GOTHIC_14),
                                  GColorBlack, GColorWhite, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &mac_label->layer);

  app_window_stack_push(window, true);
}

static void s_main(void) {
  // Restart BLE so it begins advertising
  bt_ctl_set_enabled(false);
  bt_ctl_set_enabled(true);

  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd *mfg_adv_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
      .common.main_func = &s_main,
      // UUID: 4c8e2a1f-7d3b-4f9e-b5a2-6e1c8d3f7a9b
      .common.uuid = {0x4c, 0x8e, 0x2a, 0x1f, 0x7d, 0x3b, 0x4f, 0x9e,
                      0xb5, 0xa2, 0x6e, 0x1c, 0x8d, 0x3f, 0x7a, 0x9b},
      .name = "MfgAdv",
  };
  return (const PebbleProcessMd *)&s_app_info;
}
