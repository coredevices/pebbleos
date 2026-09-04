/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "recovery_first_use.h"

#include "getting_started_button_combo.h"

#include "apps/core/spinner_ui_window.h"
#include "applib/fonts/fonts.h"
#include "comm/ble/gap_le_connection.h"
#include "comm/ble/gap_le_device_name.h"
#include "comm/ble/gap_le_connect.h"
#include "mfg/mfg_info.h"
#include "mfg/mfg_serials.h"

#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"

#include "applib/app.h"
#include "applib/event_service_client.h"
#include "applib/graphics/gtypes.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/layer.h"
#include "applib/ui/qr_code.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "applib/ui/window_private.h"
#include "process_state/app_state/app_state.h"

#include "pbl/services/bluetooth/local_id.h"
#include "pbl/services/bluetooth/pairability.h"
#include "pbl/services/comm_session/session.h"

#include "git_version.auto.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define QR_URL_BUFFER_SIZE 72
#define NAME_BUFFER_SIZE (BT_DEVICE_NAME_BUFFER_SIZE + 2)

typedef struct RecoveryFUAppData {
  Window launch_app_window;

  QRCode qr_code;
  char qr_url_buffer[QR_URL_BUFFER_SIZE];
  bool is_showing_version;
  TextLayer name_text_layer;
  char name_text_buffer[NAME_BUFFER_SIZE];

  AppTimer *spinner_close_timer;

  //! Is the mobile app currently connected (comm session is up?)
  bool is_pebble_mobile_app_connected;
  //! Has the mobile app ever connected during this boot? Used to avoid flickering the layout
  //! for brief disconnects.
  bool has_pebble_mobile_app_connected;
  bool is_pairing_allowed;
  bool spinner_is_visible;
  bool spinner_should_close;

  EventServiceInfo pebble_mobile_app_event_info;
  EventServiceInfo bt_connection_event_info;
  EventServiceInfo pebble_gather_logs_event_info;
  EventServiceInfo ble_device_name_updated_event_info;

  GettingStartedButtonComboState button_combo_state;
} RecoveryFUAppData;

static const char *s_qr_url_fmt = "https://qr.repebble.com/?sn=%s&model=%s";

// Unfortunately, the event_service_client_subscribe doesn't take a void *context...
static RecoveryFUAppData *s_fu_app_data;

static void prv_update_name_text(RecoveryFUAppData *data);

////////////////////////////////////////////////////////////
// Spinner Logic

static void prv_pop_spinner(void *not_used) {
  if (s_fu_app_data && s_fu_app_data->spinner_should_close) {
    app_window_stack_pop(false /* animated */);
    s_fu_app_data->spinner_is_visible = false;
    s_fu_app_data->spinner_should_close = false;
  }
}

static void prv_show_spinner(RecoveryFUAppData *data) {
  if (!data->spinner_is_visible) {
    Window *spinner_window = spinner_ui_window_get(PBL_IF_COLOR_ELSE(GColorRed, GColorDarkGray));
    app_window_stack_push(spinner_window, false /* animated */);
  }
  data->spinner_is_visible = true;
  data->spinner_should_close = false;
}

static void prv_hide_spinner(RecoveryFUAppData *data) {
  data->spinner_should_close = true;
  data->spinner_close_timer = app_timer_register(3000, prv_pop_spinner, data);
}

////////////////////////////////////////////////////////////
// Button Handlers
static void prv_select_combo_callback(void *cb_data) {
  // When the user holds select for a long period of time, toggle between showing the help URL
  // and the version of the firmware.

  RecoveryFUAppData *data = app_state_get_user_data();
  data->is_showing_version = !data->is_showing_version;

  prv_update_name_text(data);
}

static void prv_raw_down_handler(ClickRecognizerRef recognizer, void *context) {
  RecoveryFUAppData *data = app_state_get_user_data();

  getting_started_button_combo_button_pressed(&data->button_combo_state,
                                              click_recognizer_get_button_id(recognizer));
}

static void prv_raw_up_handler(ClickRecognizerRef recognizer, void *context) {
  RecoveryFUAppData *data = app_state_get_user_data();

  getting_started_button_combo_button_released(&data->button_combo_state,
                                               click_recognizer_get_button_id(recognizer));
}

static void prv_click_configure(void* context) {
  window_raw_click_subscribe(BUTTON_ID_UP, prv_raw_down_handler, prv_raw_up_handler, NULL);
  window_raw_click_subscribe(BUTTON_ID_SELECT, prv_raw_down_handler, prv_raw_up_handler, NULL);
  window_raw_click_subscribe(BUTTON_ID_DOWN, prv_raw_down_handler, prv_raw_up_handler, NULL);
}

////////////////////////////////////////////////////////////
// Windows

static void prv_update_name_text(RecoveryFUAppData *data) {
  const GAPLEConnection *gap_conn = gap_le_connection_any();

  // Set the name text
  if (data->is_showing_version) {
    size_t len = MIN(strlen(GIT_TAG), sizeof(data->name_text_buffer) - 1);
    memcpy(data->name_text_buffer, GIT_TAG, len);
    data->name_text_buffer[len] = '\0';
  } else if ((comm_session_get_system_session() != NULL) && (gap_conn != NULL)) {
    // If we have connected to a device and we have a connection to the mobile app, show the device
    // name (we are required to have a connection to mobile app to get the name).
    gap_le_connection_copy_device_name(gap_conn, data->name_text_buffer,
                                       BT_DEVICE_NAME_BUFFER_SIZE);
  } else {
    // If we aren't connected and/or don't have a session, display the name of the device
    // so it's easier for a user to figure out what they should be trying to connect to
    bt_local_id_copy_device_name(data->name_text_buffer, false);

    // For debugging purposes, we are going to add -'s to the beginning and end of the name
    // if we are connected to a BLE device but don't have a session
    if (gap_le_connect_is_connected_as_slave()) {
      memmove(&data->name_text_buffer[1], &data->name_text_buffer[0], BT_DEVICE_NAME_BUFFER_SIZE);
      data->name_text_buffer[0] = '-';
      strcat(data->name_text_buffer, "-");
    }
  }
  text_layer_set_text(&data->name_text_layer, data->name_text_buffer);
}

static void prv_window_load(Window* window) {
  struct RecoveryFUAppData *data = (struct RecoveryFUAppData*) window_get_user_data(window);

  char serial_number[MFG_SERIAL_NUMBER_SIZE + 1];
  char model_name[MFG_INFO_MODEL_STRING_LENGTH];

  mfg_info_get_serialnumber(serial_number, sizeof(serial_number));
  mfg_info_get_model(model_name);

  snprintf(data->qr_url_buffer, QR_URL_BUFFER_SIZE, s_qr_url_fmt, serial_number, model_name);

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
                               data->qr_url_buffer, strlen(data->qr_url_buffer), QRCodeECCMedium,
                               GColorBlack, GColorWhite);
  layer_add_child(&window->layer, &qr_code->layer);

#if defined(CONFIG_BOARD_OBELIX) || defined(CONFIG_BOARD_GETAFIX)
  const uint16_t name_height = 30;
#else
  const uint16_t name_height = 20;
#endif

  TextLayer* name_text_layer = &data->name_text_layer;
  text_layer_init_with_parameters(name_text_layer,
                                  &GRect(0, window->layer.bounds.size.h -
                                         PBL_IF_RECT_ELSE(name_height, name_height + 10),
                                         window->layer.bounds.size.w, name_height),
                                  NULL,
#if defined(CONFIG_BOARD_OBELIX) || defined(CONFIG_BOARD_GETAFIX)
                                  fonts_get_system_font(FONT_KEY_GOTHIC_24),
#else
                                  fonts_get_system_font(FONT_KEY_GOTHIC_14),
#endif
                                  GColorBlack, GColorWhite, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &name_text_layer->layer);
  data->is_showing_version = false;

  prv_update_name_text(data);
}

static void prv_push_window(struct RecoveryFUAppData* data) {
  Window* window = &data->launch_app_window;

  window_init(window, WINDOW_NAME("First Use / Recovery"));
  window_set_user_data(window, data);
  window_set_window_handlers(window, &(WindowHandlers){
    .load = prv_window_load,
  });
  window_set_click_config_provider_with_context(window, prv_click_configure, window);

  window_set_fullscreen(window, true);
  window_set_overrides_back_button(window, true);

  const bool animated = false;
  app_window_stack_push(window, animated);
}

////////////////////
// App Event Handler + Loop

static void prv_allow_pairing(RecoveryFUAppData* data, bool allow) {
  if (data->is_pairing_allowed == allow) {
    return;
  }
  data->is_pairing_allowed = allow;
  if (allow) {
    bt_pairability_use();
  } else {
    bt_pairability_release();
  }
}

static void prv_pebble_mobile_app_event_handler(PebbleEvent *event, void *context) {
  if (!s_fu_app_data) {
    return;
  }

  if (!event->bluetooth.comm_session_event.is_system) {
    return;
  }

  const bool is_connected = event->bluetooth.comm_session_event.is_open;

  s_fu_app_data->is_pebble_mobile_app_connected = event->bluetooth.comm_session_event.is_open;
  if (is_connected) {
    s_fu_app_data->has_pebble_mobile_app_connected = true;
    gap_le_device_name_request_all();
  }
  prv_update_name_text(s_fu_app_data);
}

static void prv_bt_event_handler(PebbleEvent *event, void *context) {
  if (!s_fu_app_data) {
    return;
  }
  prv_update_name_text(s_fu_app_data);
}

static void prv_gather_debug_info_event_handler(PebbleEvent *event, void *context) {
  if (!s_fu_app_data) {
    return;
  }
  if (event->debug_info.state == DebugInfoStateStarted) {
    prv_show_spinner(s_fu_app_data);
  } else {
    prv_hide_spinner(s_fu_app_data);
  }
}

////////////////////
// App boilerplate

static void handle_init(void) {
  launcher_block_popups(true);

  RecoveryFUAppData *data = app_malloc_check(sizeof(RecoveryFUAppData));
  s_fu_app_data = data;

  *data = (RecoveryFUAppData){};
  app_state_set_user_data(data);

  const bool is_connected = (comm_session_get_system_session() != NULL);
  data->is_pebble_mobile_app_connected = is_connected;
  prv_allow_pairing(data, !is_connected);

  data->pebble_mobile_app_event_info = (EventServiceInfo) {
    .type = PEBBLE_COMM_SESSION_EVENT,
    .handler = prv_pebble_mobile_app_event_handler,
  };
  event_service_client_subscribe(&data->pebble_mobile_app_event_info);

  data->pebble_gather_logs_event_info = (EventServiceInfo) {
    .type = PEBBLE_GATHER_DEBUG_INFO_EVENT,
    .handler = prv_gather_debug_info_event_handler,
  };
  event_service_client_subscribe(&data->pebble_gather_logs_event_info);

  data->bt_connection_event_info = (EventServiceInfo) {
    .type = PEBBLE_BT_CONNECTION_EVENT,
    .handler = prv_bt_event_handler,
  };
  event_service_client_subscribe(&data->bt_connection_event_info);

  data->ble_device_name_updated_event_info = (EventServiceInfo) {
    .type = PEBBLE_BLE_DEVICE_NAME_UPDATED_EVENT,
    .handler = prv_bt_event_handler,
  };
  event_service_client_subscribe(&data->ble_device_name_updated_event_info);

  getting_started_button_combo_init(&data->button_combo_state, prv_select_combo_callback);

  prv_push_window(data);
}

static void handle_deinit(void) {
  RecoveryFUAppData *data = app_state_get_user_data();

  getting_started_button_combo_deinit(&data->button_combo_state);

  event_service_client_unsubscribe(&data->pebble_mobile_app_event_info);
  event_service_client_unsubscribe(&data->bt_connection_event_info);
  event_service_client_unsubscribe(&data->pebble_gather_logs_event_info);
  event_service_client_unsubscribe(&data->ble_device_name_updated_event_info);

  app_window_stack_pop_all(false);

  prv_allow_pairing(data, false);

  app_free(data);
  s_fu_app_data = NULL;

  launcher_block_popups(false);
}

static void prv_main(void) {
  handle_init();

  app_event_loop();

  handle_deinit();
}

const PebbleProcessMd* recovery_first_use_app_get_app_info(void) {
  static const PebbleProcessMdSystem s_recovery_first_use_app = {
    .common = {
      .main_func = prv_main,
      .visibility = ProcessVisibilityHidden,
      // UUID: 85b80081-d78f-41aa-96fa-a821c79f3f0f
      .uuid = {
        0x85, 0xb8, 0x00, 0x81, 0xd7, 0x8f, 0x41, 0xaa,
        0x96, 0xfa, 0xa8, 0x21, 0xc7, 0x9f, 0x3f, 0x0f
      },
    },
    .name = "Getting Started",
    .run_level = ProcessAppRunLevelSystem,
  };
  return (const PebbleProcessMd*) &s_recovery_first_use_app;
}
