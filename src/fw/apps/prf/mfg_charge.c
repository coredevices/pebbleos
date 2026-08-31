/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mfg_charge.h"

#include "applib/app.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/ui.h"
#include "apps/prf/mfg_test_result.h"
#include <pbl/drivers/battery.h>
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include <pbl/logging/logging.h>

#include <stdio.h>

typedef enum {
  ChargeStateWaitPlug = 0,
  ChargeStateWaitCharge,
  ChargeStatePass,
  ChargeStateFail,
} ChargeTestState;

static const char* status_text[] = {
  [ChargeStateWaitPlug] =    "Plug Charger",
  [ChargeStateWaitCharge] =  "Wait Charge...",
  [ChargeStatePass] =        "PASS - Unplug",
  [ChargeStateFail] =        "FAIL - Unplug",
};

#if defined(CONFIG_BOARD_ASTERIX) || defined(CONFIG_BOARD_OBELIX) || defined(CONFIG_BOARD_GETAFIX)
static const int TEMP_MIN_MC = 15000; // 15.0C
static const int TEMP_MAX_MC = 35000; // 35.0C
#else
static const int TEMP_MIN_MC = 0;
static const int TEMP_MAX_MC = 0;
#endif

static const int WAIT_CHARGE_TIMEOUT_S = 5;

typedef struct {
  Window window;

  TextLayer status;
  char status_string[20];

  TextLayer details;
  char details_string[64];

  ChargeTestState test_state;
  uint32_t seconds_remaining;
} AppData;

static void prv_handle_second_tick(struct tm *tick_time, TimeUnits units_changed) {
  AppData *data = app_state_get_user_data();

  ChargeTestState next_state = data->test_state;

  BatteryConstants battery_const;
  int ret = battery_get_constants(&battery_const);
  if (ret < 0) {
    PBL_LOG_ERR("Skipping bad constants reading");
    return;
  }

  const int charge_mv = battery_const.v_mv;
  const int32_t temp_mc = battery_const.t_mc;
  const BatteryChargeState charge_state = battery_get_charge_state();

  switch (data->test_state) {
    case ChargeStateWaitPlug:
      if (charge_state.is_plugged) {
        next_state = ChargeStateWaitCharge;
        data->seconds_remaining = WAIT_CHARGE_TIMEOUT_S;
      }
      break;
    case ChargeStateWaitCharge:
      if (!charge_state.is_plugged) {
        next_state = ChargeStateWaitPlug;
        break;
      }
      if (charge_state.is_charging) {
        if (temp_mc >= TEMP_MIN_MC && temp_mc <= TEMP_MAX_MC) {
          next_state = ChargeStatePass;
        } else {
          PBL_LOG_ERR("Temperature out of range: %" PRId32 "mC", temp_mc);
          next_state = ChargeStateFail;
        }
      } else {
        --data->seconds_remaining;
        if (data->seconds_remaining == 0) {
          PBL_LOG_ERR("Timed out waiting for charge to start");
          next_state = ChargeStateFail;
        }
      }
      break;
    case ChargeStatePass:
    case ChargeStateFail:
      if (!charge_state.is_plugged) {
        bool passed = (data->test_state == ChargeStatePass);
        mfg_test_result_report(MfgTestId_Charge, passed, 0);
        app_window_stack_pop(true);
        return;
      }
      break;
    default:
      break;
  }

  data->test_state = next_state;

  sniprintf(data->status_string, sizeof(data->status_string),
            "CHARGE\n%s", status_text[data->test_state]);
  text_layer_set_text(&data->status, data->status_string);

  int8_t temp_c = (int8_t)(temp_mc / 1000);
  uint8_t temp_c_frac = ((temp_mc > 0 ? temp_mc : -temp_mc) % 1000) / 10;
  sniprintf(data->details_string, sizeof(data->details_string),
            "%umV %" PRId8 ".%02" PRIu8 "C\r\nUSB: %s\r\nCharging: %s",
            charge_mv,
            temp_c, temp_c_frac,
            charge_state.is_plugged ? "yes" : "no",
            charge_state.is_charging ? "yes" : "no");
  text_layer_set_text(&data->details, data->details_string);
}

static void app_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  *data = (AppData) {
    .test_state = ChargeStateWaitPlug,
    .seconds_remaining = WAIT_CHARGE_TIMEOUT_S,
  };

  Window *window = &data->window;
  window_init(window, "Charge Test");

  TextLayer *status = &data->status;
  text_layer_init(status, &window->layer.bounds);
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_24));
  text_layer_set_text_alignment(status, GTextAlignmentCenter);
  text_layer_set_text(status, status_text[data->test_state]);
  layer_add_child(&window->layer, &status->layer);

  TextLayer *details = &data->details;
  text_layer_init(details,
                  &GRect(0, 65, window->layer.bounds.size.w, window->layer.bounds.size.h - 65));
  text_layer_set_font(details, fonts_get_system_font(FONT_KEY_GOTHIC_24));
  text_layer_set_text_alignment(details, GTextAlignmentCenter);
  layer_add_child(&window->layer, &details->layer);

  window_set_fullscreen(window, true);

  tick_timer_service_subscribe(SECOND_UNIT, prv_handle_second_tick);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  app_init();

  app_event_loop();
}

const PebbleProcessMd* mfg_charge_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: fbb6d0e6-2d7d-40bc-8b01-f2f8beb9c394
    .common.uuid = { 0xfb, 0xb6, 0xd0, 0xe6, 0x2d, 0x7d, 0x40, 0xbc,
                     0x8b, 0x01, 0xf2, 0xf8, 0xbe, 0xb9, 0xc3, 0x94 },
    .name = "Charge App",
  };

  return (PebbleProcessMd*) &s_app_info;
}
