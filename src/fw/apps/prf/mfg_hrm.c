/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mfg_hrm.h"

#include "applib/app.h"
#include "applib/tick_timer_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include <pbl/drivers/hrm.h>
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "process_management/pebble_process_md.h"
#include "pbl/services/hrm/hrm_manager.h"

#include <stdbool.h>
#include <stdio.h>

#ifdef CONFIG_HRM

#define STATUS_STRING_LEN 32
#define BPM_STRING_LEN 32
#define SPO2_STRING_LEN 32

typedef struct {
  Window window;
  EventServiceInfo hrm_event_info;

  TextLayer title_text_layer;
  TextLayer status_text_layer;
  TextLayer bpm_text_layer;
  TextLayer spo2_text_layer;
  char status_string[STATUS_STRING_LEN];
  char bpm_string[BPM_STRING_LEN];
  char spo2_string[SPO2_STRING_LEN];
  HRMSessionRef hrm_session;
} AppData;

static void prv_handle_hrm_data(PebbleEvent *e, void *context) {
  AppData *app_data = app_state_get_user_data();

  if (e->type == PEBBLE_HRM_EVENT) {
    snprintf(app_data->status_string, STATUS_STRING_LEN, "Sampling...");

    if (e->hrm.event_type == HRMEvent_BPM) {
      snprintf(app_data->bpm_string, BPM_STRING_LEN,
              "HR:%d (quality:%d)", e->hrm.bpm.bpm, e->hrm.bpm.quality);
    } else if (e->hrm.event_type == HRMEvent_SpO2) {
      snprintf(app_data->spo2_string, SPO2_STRING_LEN,
              "SpO2:%d (quality:%d)", e->hrm.spo2.percent, e->hrm.spo2.quality);
    }

    layer_mark_dirty(&app_data->window.layer);
  }
}

static void prv_handle_init(void) {
  AppData *data = task_zalloc(sizeof(*data));
  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

  TextLayer *title = &data->title_text_layer;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "HRM TEST");
  layer_add_child(&window->layer, &title->layer);

  sniprintf(data->status_string, STATUS_STRING_LEN, "Starting...");

  snprintf(data->bpm_string, BPM_STRING_LEN, "HR:--");
  snprintf(data->spo2_string, SPO2_STRING_LEN, "SpO2:--");

  TextLayer *status = &data->status_text_layer;
  text_layer_init(status,
                  &GRect(5, 40, window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 40));
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(status, GTextAlignmentCenter);
  text_layer_set_text(status, data->status_string);
  layer_add_child(&window->layer, &status->layer);

  TextLayer *bpm = &data->bpm_text_layer;
  text_layer_init(bpm,
                  &GRect(5, 80, window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 80));
  text_layer_set_font(bpm, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(bpm, GTextAlignmentCenter);
  text_layer_set_text(bpm, data->bpm_string);
  layer_add_child(&window->layer, &bpm->layer);

  TextLayer *spo2 = &data->spo2_text_layer;
  text_layer_init(spo2,
                  &GRect(5, 120, window->layer.bounds.size.w - 5, window->layer.bounds.size.h - 120));
  text_layer_set_font(spo2, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(spo2, GTextAlignmentCenter);
  text_layer_set_text(spo2, data->spo2_string);
  layer_add_child(&window->layer, &spo2->layer);

  data->hrm_event_info = (EventServiceInfo){
    .type = PEBBLE_HRM_EVENT,
    .handler = prv_handle_hrm_data,
  };
  event_service_client_subscribe(&data->hrm_event_info);

  // Use app data as session ref
  AppInstallId  app_id = 1;
  data->hrm_session = sys_hrm_manager_app_subscribe(app_id, 1, SECONDS_PER_HOUR,
                                                    HRMFeature_BPM | HRMFeature_SpO2);

  app_window_stack_push(window, true);
}

static void prv_handle_deinit(void) {
  AppData *data = app_state_get_user_data();
  event_service_client_unsubscribe(&data->hrm_event_info);
  sys_hrm_manager_unsubscribe(data->hrm_session);

  text_layer_deinit(&data->title_text_layer);
  text_layer_deinit(&data->status_text_layer);
  window_deinit(&data->window);
  app_free(data);
}

static void prv_main(void) {
  prv_handle_init();
  app_event_loop();
  prv_handle_deinit();
}

const PebbleProcessMd* mfg_hrm_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &prv_main,
    .name = "MfgHRM",
  };
  return (const PebbleProcessMd*) &s_app_info;
}

#endif // CONFIG_HRM
