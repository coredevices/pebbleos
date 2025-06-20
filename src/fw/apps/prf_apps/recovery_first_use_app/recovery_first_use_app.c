/*
 * Copyright 2024 Google LLC
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

#include "recovery_first_use_app.h"

#include "getting_started_button_combo.h"

#include "apps/core_apps/spinner_ui_window.h"
#include "applib/fonts/fonts.h"
#include "comm/ble/gap_le_connection.h"
#include "comm/ble/gap_le_device_name.h"
#include "comm/ble/gap_le_connect.h"
#include "drivers/backlight.h"
#include "mfg/mfg_info.h"
#include "mfg/mfg_serials.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"

#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"

#include "resource/resource_ids.auto.h"
#include "system/logging.h"
#include "system/passert.h"

#include "applib/app.h"
#include "applib/event_service_client.h"
#include "applib/graphics/gtypes.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/bitmap_layer.h"
#include "applib/ui/kino/kino_layer.h"
#include "applib/ui/kino/kino_reel.h"
#include "applib/ui/layer.h"
#include "applib/ui/qr_code.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "applib/ui/window_private.h"
#include "applib/ui/window_stack.h"
#include "process_state/app_state/app_state.h"

#include "apps/system_apps/settings/settings_time.h"

#include "services/common/bluetooth/local_id.h"
#include "services/common/bluetooth/pairability.h"
#include "services/common/comm_session/session.h"
#include "services/common/shared_prf_storage/shared_prf_storage.h"

#include "git_version.auto.h"

#include <bluetooth/classic_connect.h>

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define QR_URL_BUFFER_SIZE 72
#define URL_BUFFER_SIZE 32
#define NAME_BUFFER_SIZE (BT_DEVICE_NAME_BUFFER_SIZE + 2)

typedef struct RecoveryFUAppData {
  Window launch_app_window;

#if PLATFORM_ASTERIX
  QRCode qr_code;
  char qr_url_buffer[QR_URL_BUFFER_SIZE];
#else
  KinoLayer kino_layer;
  TextLayer url_text_layer;
  char url_text_buffer[URL_BUFFER_SIZE];
#endif
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

#if PLATFORM_ASTERIX
static const char *s_qr_url_fmt = "https://app.repebble.com/qr?sn=%s&model=%s";
#endif

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

#if !PLATFORM_ASTERIX
static void prv_update_background_image_and_url_text(RecoveryFUAppData *data) {
  uint32_t icon_res_id;
  const char *url_string;
  GColor background;
  GRect kino_area;
  int16_t icon_x_offset;
  int16_t icon_y_offset;
  int16_t text_y_offset;

  // Have we gone through first use before? If not, show first use UI. Otherwise show recovery UI.
  const bool first_use_is_complete = shared_prf_storage_get_getting_started_complete();

  // Pick the right layout for the screen
  if (first_use_is_complete || data->has_pebble_mobile_app_connected) {
    // If first use was completed, it means we're in recovery mode.
    icon_res_id = RESOURCE_ID_LAUNCH_APP;
#if PLATFORM_ROBERT || PLATFORM_CALCULUS
    icon_x_offset = 41;
    icon_y_offset = -21;
    text_y_offset = 140;
#else
    icon_x_offset = PBL_IF_RECT_ELSE(49, 67);
    icon_y_offset = 28;
    text_y_offset = 124;
#endif
  } else {
    icon_res_id = RESOURCE_ID_MOBILE_APP_ICON;
#if PLATFORM_ROBERT || PLATFORM_CALCULUS
    icon_x_offset = 74;
    icon_y_offset = 56;
    text_y_offset = 121;
#else
    icon_x_offset = PBL_IF_RECT_ELSE(49, 67);
    icon_y_offset = 38;
    text_y_offset = 90;
#endif
  }

  if (first_use_is_complete) {
#if PBL_BW && !PLATFORM_TINTIN
    // Override the icon to use the fullscreen version
    icon_res_id = RESOURCE_ID_LAUNCH_APP;
    icon_x_offset = 0;
    icon_y_offset = 0;

    url_string = ""; // URL is baked into the background image
    background = GColorWhite;
#else
    url_string = "pebble.com/sos";
    background = PBL_IF_COLOR_ELSE(GColorRed, GColorWhite);;
#endif
  } else {
    url_string = "pebble.com/app";
    background = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
  }

  // Create the icon
  KinoReel *icon_reel = kino_reel_create_with_resource(icon_res_id);
  if (!icon_reel) {
    PBL_CROAK("Couldn't create kino reel");
  }

  // Position the icon
  kino_area = GRect(icon_x_offset, icon_y_offset, data->launch_app_window.layer.bounds.size.w,
                    data->launch_app_window.layer.bounds.size.h);
  layer_set_frame((Layer *) &data->kino_layer, &kino_area);
  kino_layer_set_alignment(&data->kino_layer, GAlignTopLeft);
  window_set_background_color(&data->launch_app_window, background);

  kino_layer_set_reel(&data->kino_layer, icon_reel, /* take_ownership */ true);

  // Configure the url text layer
  data->url_text_layer.layer.frame.origin.y = text_y_offset;
  text_layer_set_text(&data->url_text_layer, url_string);
}
#endif

static void prv_update_name_text(RecoveryFUAppData *data) {
  const GAPLEConnection *gap_conn = gap_le_connection_any();

  // Set the name text
  if (data->is_showing_version) {
    size_t len = MIN(strlen(GIT_TAG), sizeof(data->name_text_buffer) - 1);
    memcpy(data->name_text_buffer, GIT_TAG, len);
    data->name_text_buffer[len] = '\0';
  } else if (bt_driver_classic_is_connected()) {
    // If BT Classic connected, show the name of the connected device
    bt_driver_classic_copy_connected_device_name(data->name_text_buffer);
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

#if !PLATFORM_ASTERIX
  // Set the name font
#if !PLATFORM_ROBERT && !PLATFORM_CALCULUS
  const bool first_use_is_complete = shared_prf_storage_get_getting_started_complete();
  const char *name_font_key;
  if (first_use_is_complete || data->has_pebble_mobile_app_connected || data->is_showing_version) {
    name_font_key = FONT_KEY_GOTHIC_14;
  } else {
    name_font_key = FONT_KEY_GOTHIC_24;
  }
  text_layer_set_font(&data->name_text_layer, fonts_get_system_font(name_font_key));
#endif

  // Update the size of the name text layer based on the new content.

  // First set the text layer to be the width of the entire window and only a single line of text
  // high.
  layer_set_frame(&data->name_text_layer.layer,
                  &(GRect) { { 0, 0 }, { data->launch_app_window.layer.frame.size.w, 26 } });

  // Ask the text layer for a content size based on the frame we just set. If there's no text,
  // hide the layer by setting the size to zero.
  GSize content_size = { 0, 0 };
  if (strlen(data->name_text_layer.text)) {
    content_size = text_layer_get_content_size(app_state_get_graphics_context(),
                                               &data->name_text_layer);
    content_size.w += 4;
    content_size.h += 4;
  }

  // Actually set the frame centered on the screen and just below the url_text_layer.
  const int16_t window_width = data->launch_app_window.layer.frame.size.w;
  const int16_t text_x_offset = (window_width - content_size.w) / 2;
#if PLATFORM_ROBERT || PLATFORM_CALCULUS
  const int16_t text_y_offset = 33;
#else
  const int16_t text_y_offset = 22;
#endif
  const GRect frame = { { text_x_offset, text_y_offset }, content_size };
  layer_set_frame(&data->name_text_layer.layer, &frame);
#endif
}

static void prv_window_load(Window* window) {
  struct RecoveryFUAppData *data = (struct RecoveryFUAppData*) window_get_user_data(window);

#if PLATFORM_ASTERIX
  char serial_number[MFG_SERIAL_NUMBER_SIZE + 1];
  char model_name[MFG_INFO_MODEL_STRING_LENGTH];

  mfg_info_get_serialnumber(serial_number, sizeof(serial_number));
  mfg_info_get_model(model_name);

  snprintf(data->qr_url_buffer, QR_URL_BUFFER_SIZE, s_qr_url_fmt, serial_number, model_name);

  QRCode* qr_code = &data->qr_code;
  qr_code_init_with_parameters(qr_code,
                               &GRect(10, 10, window->layer.bounds.size.w - 20,
                                      window->layer.bounds.size.h - 30),
                               data->qr_url_buffer, strlen(data->qr_url_buffer), QRCodeECCMedium,
                               GColorBlack, GColorWhite);
  layer_add_child(&window->layer, &qr_code->layer);

  TextLayer* name_text_layer = &data->name_text_layer;
  text_layer_init_with_parameters(name_text_layer,
                                  &GRect(0, window->layer.bounds.size.h - 20,
                                         window->layer.bounds.size.w, 20),
                                  NULL, fonts_get_system_font(FONT_KEY_GOTHIC_14),
                                  GColorBlack, GColorWhite, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &name_text_layer->layer);
  data->is_showing_version = false;

  prv_update_name_text(data);
#else
  KinoLayer *kino_layer = &data->kino_layer;
  kino_layer_init(kino_layer, &window->layer.bounds);
  layer_add_child(&window->layer, &kino_layer->layer);

#if PLATFORM_ROBERT || PLATFORM_CALCULUS
  const char *url_font_key = FONT_KEY_GOTHIC_28_BOLD;
  const GColor name_bg_color = GColorClear;
  const char *name_font_key = FONT_KEY_GOTHIC_24;
#else
  const char *url_font_key = FONT_KEY_GOTHIC_18_BOLD;
  const GColor name_bg_color = GColorWhite;
  const char *name_font_key = FONT_KEY_GOTHIC_14;
#endif

  TextLayer* url_text_layer = &data->url_text_layer;
  text_layer_init_with_parameters(url_text_layer,
                                  &GRect(0, 124, window->layer.bounds.size.w, 64),
                                  NULL, fonts_get_system_font(url_font_key),
                                  GColorBlack, GColorClear, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&window->layer, &url_text_layer->layer);

  TextLayer* name_text_layer = &data->name_text_layer;
  text_layer_init_with_parameters(name_text_layer,
                                  &data->launch_app_window.layer.frame,
                                  NULL, fonts_get_system_font(name_font_key),
                                  GColorBlack, name_bg_color, GTextAlignmentCenter,
                                  GTextOverflowModeTrailingEllipsis);
  layer_add_child(&url_text_layer->layer, &name_text_layer->layer);
  data->is_showing_version = false;

  prv_update_background_image_and_url_text(data);
  prv_update_name_text(data);
#endif
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
#if !PLATFORM_ASTERIX
  prv_update_background_image_and_url_text(s_fu_app_data);
#endif
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

#if !PLATFORM_ASTERIX
  kino_layer_deinit(&data->kino_layer);
#endif

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
