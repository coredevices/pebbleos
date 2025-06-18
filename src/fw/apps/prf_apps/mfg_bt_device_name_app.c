/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/qr_code.h"
#include "applib/ui/window.h"
#include "applib/ui/window_private.h"
#include "applib/ui/text_layer.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "process_management/pebble_process_md.h"
#include "services/common/bluetooth/local_id.h"
#include "util/bitset.h"
#include "util/size.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
  Window window;

  QRCode qr_code;
  TextLayer name;

  char name_buffer[BT_DEVICE_NAME_BUFFER_SIZE];
} AppData;

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

  bt_local_id_copy_device_name(data->name_buffer, false);

  QRCode* qr_code = &data->qr_code;
  qr_code_init_with_parameters(qr_code,
                               &GRect(10, 10, window->layer.bounds.size.w - 20,
                                      window->layer.bounds.size.h - 30),
                               data->name_buffer, strlen(data->name_buffer), QRCodeECCMedium,
                               GColorBlack, GColorWhite);
  layer_add_child(&window->layer, &qr_code->layer);

  TextLayer* name = &data->name;
  text_layer_init_with_parameters(name,
                                  &GRect(0, window->layer.bounds.size.h - 20,
                                         window->layer.bounds.size.w, 20),
                                  data->name_buffer, fonts_get_system_font(FONT_KEY_GOTHIC_14),
                                  GColorBlack, GColorWhite, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &name->layer);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();
}

const PebbleProcessMd* mfg_bt_device_name_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: 31b5a232-d638-4ccb-b89a-910202d85a1f
    .common.uuid = { 0x31, 0xb5, 0xa2, 0x32, 0xd6, 0x38, 0x4c, 0xcb,
                     0xb8, 0x9a, 0x91, 0x02, 0x02, 0xd8, 0x5a, 0x1f },
    .name = "MfgBTDeviceName",
  };
  return (const PebbleProcessMd*) &s_app_info;
}

