/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/ui/ui.h"
#include "applib/app.h"
#include "applib/app_timer.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/ui/layer.h"
#include "applib/ui/window_private.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource_ids.auto.h"

#define LOW_POWER_APP_STATE_UPDATE_TIME_MS 2000

typedef struct {
  Window window;
  KinoLayer kino_layer;
  GRect charging_kino_area;
  GRect discharging_kino_area;
  BatteryChargeState saved_state;
  AppTimer *timer;
} LowPowerAppData;

////////////////////////////////////////////////////////////
// Update Logic

static void prv_refresh_state(void *data_in) {
  LowPowerAppData *data = (LowPowerAppData*) data_in;
  BatteryChargeState current_state = battery_get_charge_state();
  uint32_t res_id;
  GRect kino_area;

  if (current_state.is_charging && !data->saved_state.is_charging) {
    kino_area = data->charging_kino_area;
    res_id = RESOURCE_ID_RECOVERY_LOW_POWER_CHARGING;
  } else if (!current_state.is_charging && data->saved_state.is_charging) {
    kino_area = data->discharging_kino_area;
    res_id = RESOURCE_ID_RECOVERY_LOW_POWER_DISCHARGING;
  } else {
    goto reschedule;
  }

  layer_set_frame((Layer *) &data->kino_layer, &kino_area);
  kino_layer_set_reel_with_resource(&data->kino_layer, res_id);
  layer_mark_dirty(&data->kino_layer.layer);

  reschedule:
    data->saved_state = current_state;
    data->timer = app_timer_register(LOW_POWER_APP_STATE_UPDATE_TIME_MS, prv_refresh_state, data);
}

////////////////////////////////////////////////////////////
// Window loading, unloading, initializing

static void prv_window_unload_handler(Window* window) {
  LowPowerAppData *data = window_get_user_data(window);
  if (!data) {
    // Sanity check
    return;
  }

  kino_layer_deinit(&data->kino_layer);
  app_timer_cancel(data->timer);
  app_free(data);
}

static void prv_window_load_handler(Window* window) {
  LowPowerAppData *data = window_get_user_data(window);

  data->discharging_kino_area = GRect(PBL_IF_RECT_ELSE(4, 5),
                                      PBL_IF_RECT_ELSE(2, 4),
                                      data->window.layer.bounds.size.w,
                                      data->window.layer.bounds.size.h);
  data->charging_kino_area = GRect(0, 0, data->window.layer.bounds.size.w,
                                   data->window.layer.bounds.size.h);

  kino_layer_init(&data->kino_layer, &data->discharging_kino_area);
  kino_layer_set_reel_with_resource(&data->kino_layer, RESOURCE_ID_RECOVERY_LOW_POWER_DISCHARGING);
  layer_add_child(&window->layer, &data->kino_layer.layer);

  data->timer = app_timer_register(LOW_POWER_APP_STATE_UPDATE_TIME_MS, prv_refresh_state, data);
}

static void prv_prf_low_power_app_window_push(void) {
  LowPowerAppData *data = app_malloc_check(sizeof(LowPowerAppData));

  *data = (LowPowerAppData){};

  Window* window = &data->window;
  window_init(window, WINDOW_NAME("Low Power App"));
  window_set_user_data(window, data);
  window_set_overrides_back_button(window, true);
  window_set_fullscreen(window, true);
  window_set_background_color(window, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  window_set_window_handlers(window, &(WindowHandlers){
    .load = prv_window_load_handler,
    .unload = prv_window_unload_handler,
  });
  app_window_stack_push(window, false);
}

static void s_main(void) {
  launcher_block_popups(true);

  prv_prf_low_power_app_window_push();

  app_event_loop();

  launcher_block_popups(false);
}

////////////////////////////////////////////////////////////
// Public functions

const PebbleProcessMd* prf_low_power_app_get_info() {
  static const PebbleProcessMdSystem s_app_info = {
    .common = {
      .main_func = &s_main,
      .visibility = ProcessVisibilityHidden,
      // UUID: f29f18ac-bbec-452b-9262-49b5f6e5c920
      .uuid = {0xf2, 0x9f, 0x18, 0xac, 0xbb, 0xec, 0x45, 0x2b,
               0x92, 0x62, 0x49, 0xb5, 0xf6, 0xe5, 0xc9, 0x20},
    },
    .name = "Low Power App",
    .run_level = ProcessAppRunLevelSystem,
  };
  return (const PebbleProcessMd*) &s_app_info;
}
