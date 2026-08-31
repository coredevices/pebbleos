/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/fonts/fonts.h"
#include "applib/graphics/graphics.h"
#include "applib/graphics/text.h"
#include "applib/ui/window_private.h"
#include "applib/ui/app_window_stack.h"
#include "board/board.h"
#include "kernel/event_loop.h"
#include "kernel/panic.h"
#include "kernel/pbl_malloc.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/runlevel.h"
#include "system/reset.h"

#include <stdio.h>

static const uint8_t sad_watch[] = {
  0x04, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x20, 0x00, 0xff, 0xff, 0xff, 0xff, /* bytes 0 - 16 */
  0xff, 0x0f, 0xf8, 0xff, 0xff, 0x57, 0xf5, 0xff, 0xff, 0xa7, 0xf2, 0xff, 0xff, 0x57, 0xf5, 0xff, /* bytes 16 - 32 */
  0xff, 0xa9, 0xca, 0xff, 0xff, 0x06, 0xb0, 0xff, 0xff, 0xfe, 0xbf, 0xff, 0x7f, 0x06, 0x30, 0xff, /* bytes 32 - 48 */
  0x7f, 0xfa, 0x2f, 0xff, 0x7f, 0xfa, 0x2f, 0xff, 0x7f, 0xaa, 0x2a, 0xff, 0xff, 0xda, 0xad, 0xff, /* bytes 48 - 64 */
  0xff, 0xaa, 0x2a, 0xff, 0xff, 0xfa, 0x2f, 0xff, 0xff, 0xfa, 0x2f, 0xff, 0xff, 0x1a, 0x2c, 0xff, /* bytes 64 - 80 */
  0xff, 0xea, 0xab, 0xff, 0xff, 0xfa, 0x2f, 0xff, 0xff, 0xfa, 0x2f, 0xff, 0xff, 0xfa, 0x2f, 0xff, /* bytes 80 - 96 */
  0xff, 0x06, 0x20, 0xff, 0xff, 0xfe, 0xbf, 0xff, 0xff, 0xfe, 0xbf, 0xff, 0xff, 0x06, 0xb0, 0xff, /* bytes 96 - 112 */
  0xff, 0xa9, 0xca, 0xff, 0xff, 0x57, 0xf5, 0xff, 0xff, 0xa7, 0xf2, 0xff, 0xff, 0x57, 0xf5, 0xff, /* bytes 112 - 128 */
  0xff, 0x0f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

typedef struct PanicWindowAppData {
  Window window;
  Layer layer;
} PanicWindowAppData;

static void prv_update_proc(Layer* layer, GContext* ctx) {
  graphics_context_set_compositing_mode(ctx, GCompOpAssignInverted);

  GBitmap sad_watch_bitmap;
  gbitmap_init_with_data(&sad_watch_bitmap, sad_watch);

  const GRect bitmap_dest_rect = GRect(56, 68, 32, 32);
  graphics_draw_bitmap_in_rect(ctx, &sad_watch_bitmap, &bitmap_dest_rect);

  GFont error_code_face = fonts_get_system_font(FONT_KEY_GOTHIC_14);
  const GRect text_dest_rect = GRect(38, 108, 70, 30);

  char text_buffer[11];
  snprintf(text_buffer, sizeof(text_buffer), "0x%"PRIx32, launcher_panic_get_current_error());

  graphics_draw_text(ctx, text_buffer, error_code_face,
      text_dest_rect, GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
}

static void prv_panic_reset_callback(void* data) {
  RebootReason reason = {
    .code = RebootReasonCode_LauncherPanic,
    .extra = { .value = launcher_panic_get_current_error() }
  };
  reboot_reason_set(&reason);

  system_reset();
}

static void prv_panic_button_click_handler(ClickRecognizerRef recognizer, void *context) {
  launcher_task_add_callback(prv_panic_reset_callback, NULL);
}

static void prv_panic_click_config_provider(void* context) {
  for (ButtonId button_id = BUTTON_ID_BACK; button_id < NUM_BUTTONS; ++button_id) {
    window_single_click_subscribe(button_id, prv_panic_button_click_handler);
  }
}

static void prv_handle_init(void) {
  PanicWindowAppData* data = app_malloc_check(sizeof(PanicWindowAppData));

  app_state_set_user_data(data);
  services_set_runlevel(RunLevel_BareMinimum);

  Window *window = &data->window;

  window_init(window, WINDOW_NAME("Panic"));
  window_set_overrides_back_button(window, true);
  window_set_background_color(window, GColorBlack);
  window_set_click_config_provider(window, prv_panic_click_config_provider);

  layer_init(&data->layer, &window_get_root_layer(&data->window)->frame);
  layer_set_update_proc(&data->layer, prv_update_proc);
  layer_add_child(window_get_root_layer(&data->window), &data->layer);

  const bool animated = false;
  app_window_stack_push(window, animated);
}

static void s_main(void) {
  prv_handle_init();
  app_event_loop();
}

const PebbleProcessMd* panic_app_get_app_info() {
  static const PebbleProcessMdSystem s_app_md = {
    .common = {
      .main_func = s_main,
      .visibility = ProcessVisibilityHidden,
      // UUID: 130fb6d7-da9e-485a-87ca-a5ca4bf21912
      .uuid = {0x13, 0x0f, 0xb6, 0xd7, 0xda, 0x9e, 0x48, 0x5a, 0x87, 0xca, 0xa5, 0xca, 0x4b, 0xf2, 0x19, 0x12},
    },
    .name = "Panic App",
    .run_level = ProcessAppRunLevelCritical,
  };
  return (const PebbleProcessMd*) &s_app_md;
}

