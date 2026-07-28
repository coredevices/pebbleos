/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "camera_remote.h"

#ifdef CONFIG_BT_HID_REMOTE

#include "applib/app.h"
#include "applib/app_timer.h"
#include "applib/fonts/fonts.h"
#include "applib/persist.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/bluetooth/ble_hid.h"
#include "pbl/services/i18n/i18n.h"
#include "process_state/app_state/app_state.h"
#include <bluetooth/hid_service.h>
#include <pbl/logging/logging.h>
#include <pbl/util/size.h>

#include <stdio.h>

// There is no subscription-state event to listen to, so poll instead: the phone
// often subscribes a second or two after the app is opened.
#define CAMERA_REMOTE_READINESS_POLL_MS (1000)

#define CAMERA_REMOTE_PICKER_HOLD_MS (500)

// A held Volume Up in the iOS Camera app starts a QuickTake video instead of
// taking a photo, so the shutter has to be a tap, not a hold. 20-50 ms works.
// This is camera policy and lives here; ble_hid only guarantees that a usage
// nobody releases is released eventually.
#define CAMERA_REMOTE_TAP_MS (30)

//! App-scoped persist keys. Key 1 held an index into a table of report bits in
//! the first prototype; both the table and its meaning are gone, so the usage
//! itself is stored under a new key rather than risk reading the old value.
#define CAMERA_REMOTE_PERSIST_KEY_RETIRED_INDEX (1)
#define CAMERA_REMOTE_PERSIST_KEY_USAGE (2)
#define CAMERA_REMOTE_PERSIST_KEY_PATH (3)

// The picker is the comparison surface the two report forms exist for: a long
// press on Select chooses the Consumer usage and the report it goes out on, so
// a bitmap bit and an Array selector can be tried against the same phone
// without a serial console. AC Back / AC Forward are above 0xFF on purpose, to
// show the full 10-bit range survives. The picker labels are untranslated on
// purpose. @see BleHidReportPath for what settling the comparison would remove.
typedef struct {
  const char *name;
  uint16_t usage;
} CameraRemoteCode;

static const CameraRemoteCode s_codes[] = {
    {"Volume Up", BLE_HID_USAGE_VOLUME_UP},
    {"Volume Down", BLE_HID_USAGE_VOLUME_DOWN},
    {"Snapshot", BLE_HID_USAGE_SNAPSHOT},
    {"Menu Left", 0x044},
    {"Menu Right", 0x045},
    {"AC Back", 0x224},
    {"AC Forward", 0x225},
};

//! Picker labels, indexed by BleHidReportPath.
static const char *const s_path_names[] = {"Report 1", "Report 2"};

//! Cached so the send path never touches flash.
static uint16_t s_usage;
static BleHidReportPath s_path;

static void prv_load_settings(void) {
  // The prototype's key is never read again, so drop it rather than leave it in
  // the app's persist file forever.
  if (persist_exists(CAMERA_REMOTE_PERSIST_KEY_RETIRED_INDEX)) {
    persist_delete(CAMERA_REMOTE_PERSIST_KEY_RETIRED_INDEX);
  }

  // A missing key and a failed read both read back as 0, which is neither a
  // valid usage nor a stored path other than the default, so both fall back
  // silently on a first run. The values reach a HID report and the table can
  // change between firmware versions, so check them rather than trust them.
  s_usage = s_codes[0].usage;
  s_path = BleHidReportPathBitmap;

  const int32_t stored_usage = persist_read_int(CAMERA_REMOTE_PERSIST_KEY_USAGE);
  if (stored_usage != 0) {
    bool known = false;
    for (unsigned i = 0; i < ARRAY_LENGTH(s_codes); ++i) {
      if (s_codes[i].usage == (uint16_t)stored_usage) {
        s_usage = s_codes[i].usage;
        known = true;
        break;
      }
    }
    if (!known) {
      PBL_LOG_WRN("Camera remote: unknown stored usage 0x%x, using the default",
                  (unsigned)stored_usage);
    }
  }

  const int32_t stored_path = persist_read_int(CAMERA_REMOTE_PERSIST_KEY_PATH);
  if ((stored_path >= 0) && (stored_path < (int32_t)ARRAY_LENGTH(s_path_names))) {
    s_path = (BleHidReportPath)stored_path;
  } else {
    PBL_LOG_WRN("Camera remote: stored path %d out of range, using the default",
                (int)stored_path);
  }
}

static void prv_save_setting(uint32_t key, int32_t value) {
  const status_t rv = persist_write_int(key, value);
  if (FAILED(rv)) {
    PBL_LOG_WRN("Camera remote: could not save setting %u (%d)", (unsigned)key, (int)rv);
  }
}

//! @return False if the picked report cannot carry the picked usage, which
//! ble_hid_consumer_press() rejects rather than quietly rerouting.
static bool prv_selection_is_valid(void) {
  return ((s_path != BleHidReportPathBitmap) || (ble_hid_consumer_usage_to_bit(s_usage) != 0));
}

static const char *prv_usage_name(uint16_t usage) {
  for (unsigned i = 0; i < ARRAY_LENGTH(s_codes); ++i) {
    if (s_codes[i].usage == usage) {
      return s_codes[i].name;
    }
  }
  return "?";
}

static const char *prv_status_label(status_t rv) {
  switch (rv) {
    case S_NO_ACTION_REQUIRED:
      // The same usage is already down, so nothing was queued and no second
      // photo is coming. PASSED() would call that "sent" and the user would be
      // left wondering why two presses gave one photo.
      return "held";
    case E_INVALID_ARGUMENT:
      // Covers both an out-of-range usage and a report that cannot carry it.
      return "bad code";
    case E_INVALID_OPERATION:
      return "not subd";
    case E_BUSY:
      return "busy";
    default:
      return PASSED(rv) ? "sent" : "error";
  }
}

typedef struct {
  Window window;
  TextLayer title_text;
  TextLayer body_text;
  TextLayer code_text;
  TextLayer path_text;
  TextLayer hint_text;
  TextLayer status_text;
  AppTimer *readiness_timer;
  //! Runs the release half of a tap; NULL when nothing is held.
  AppTimer *release_timer;
  uint16_t held_usage;
  //! Readiness currently on screen, invalid until the first update.
  bool shown_ready;
  bool shown_ready_valid;
  //! Back the text layers above, so they must outlive every draw.
  char code[28];
  char path[24];
  char status[24];

  Window picker_window;
  SimpleMenuLayer picker_menu;
  SimpleMenuItem picker_usage_items[ARRAY_LENGTH(s_codes)];
  SimpleMenuItem picker_path_items[ARRAY_LENGTH(s_path_names)];
  SimpleMenuSection picker_sections[2];
  char picker_subtitles[ARRAY_LENGTH(s_codes)][8];
} CameraRemoteAppData;

static void prv_update_readiness(CameraRemoteAppData *data) {
  // text_layer_set_text() always marks the layer dirty, so re-setting the same
  // string would redraw the window on every poll.
  const bool ready = ble_hid_is_ready(s_path);
  if (data->shown_ready_valid && (data->shown_ready == ready)) {
    return;
  }
  data->shown_ready = ready;
  data->shown_ready_valid = true;

  text_layer_set_text(&data->body_text,
                      ready ? i18n_get("Open the Camera app on your phone", data)
                            : i18n_get("Not connected. Pair your phone first.", data));
}

static void prv_update_selection(CameraRemoteAppData *data) {
  snprintf(data->code, sizeof(data->code), "%s 0x%03x", prv_usage_name(s_usage), (unsigned)s_usage);
  text_layer_set_text(&data->code_text, data->code);
  // Which report a press goes out on is the whole point of the picker, so say
  // it, and say when the combination cannot be sent instead of only failing on
  // press.
  snprintf(data->path, sizeof(data->path), "Path: %s%s", s_path_names[s_path],
           prv_selection_is_valid() ? "" : " (n/a)");
  text_layer_set_text(&data->path_text, data->path);
}

static void prv_readiness_timer_cb(void *context);

static void prv_schedule_readiness_timer(CameraRemoteAppData *data) {
  data->readiness_timer =
      app_timer_register(CAMERA_REMOTE_READINESS_POLL_MS, prv_readiness_timer_cb, data);
  if (!data->readiness_timer) {
    PBL_LOG_WRN("Camera remote: no readiness timer, the screen will not refresh");
  }
}

static void prv_readiness_timer_cb(void *context) {
  CameraRemoteAppData *data = context;
  prv_schedule_readiness_timer(data);
  // The low-latency request expires after MIN_LATENCY_MODE_TIMEOUT_HID_SECS;
  // re-arm it here so the link stays fast for as long as the app is open, not
  // just for a minute after the last press. The timer belongs to the app, not
  // to a window, so this keeps working while the picker is on top.
  ble_hid_set_low_latency(true);
  prv_update_readiness(data);
}

static void prv_cancel_readiness_timer(CameraRemoteAppData *data) {
  if (data->readiness_timer) {
    app_timer_cancel(data->readiness_timer);
    data->readiness_timer = NULL;
  }
}

//! Sends the release half of a tap. Idempotent, so leaving the app can call it
//! without knowing whether the timer already did.
//! @note The status is dropped and held_usage cleared either way. A failure
//! means ble_hid could not queue the work, and it retries that itself until it
//! either lifts the key or gives up loudly; a second release from here would
//! only be refused, and holding the usage would stop the app pressing again --
//! which is the one thing that can still clear a key ble_hid gave up on.
static void prv_release_held(CameraRemoteAppData *data) {
  if (data->release_timer) {
    app_timer_cancel(data->release_timer);
    data->release_timer = NULL;
  }
  if (data->held_usage != 0) {
    ble_hid_consumer_release(data->held_usage);
    data->held_usage = 0;
  }
}

static void prv_release_timer_cb(void *context) {
  CameraRemoteAppData *data = context;
  data->release_timer = NULL;
  prv_release_held(data);
}

static void prv_usage_selected(int index, void *context) {
  CameraRemoteAppData *data = context;
  s_usage = s_codes[index].usage;
  prv_save_setting(CAMERA_REMOTE_PERSIST_KEY_USAGE, s_usage);
  prv_update_selection(data);
  app_window_stack_remove(&data->picker_window, true /* animated */);
}

static void prv_path_selected(int index, void *context) {
  CameraRemoteAppData *data = context;
  s_path = (BleHidReportPath)index;
  prv_save_setting(CAMERA_REMOTE_PERSIST_KEY_PATH, s_path);
  prv_update_selection(data);
  // Readiness is per report, so the other report may well have a different one.
  prv_update_readiness(data);
  app_window_stack_remove(&data->picker_window, true /* animated */);
}

static void prv_picker_window_load(Window *window) {
  CameraRemoteAppData *data = window_get_user_data(window);

  for (unsigned i = 0; i < ARRAY_LENGTH(s_codes); ++i) {
    snprintf(data->picker_subtitles[i], sizeof(data->picker_subtitles[i]), "0x%03x",
             (unsigned)s_codes[i].usage);
    data->picker_usage_items[i] = (SimpleMenuItem){
        .title = s_codes[i].name,
        .subtitle = data->picker_subtitles[i],
        .callback = prv_usage_selected,
    };
  }
  for (unsigned i = 0; i < ARRAY_LENGTH(s_path_names); ++i) {
    data->picker_path_items[i] = (SimpleMenuItem){
        .title = s_path_names[i],
        .callback = prv_path_selected,
    };
  }
  data->picker_sections[0] = (SimpleMenuSection){
      .title = "Usage",
      .items = data->picker_usage_items,
      .num_items = ARRAY_LENGTH(data->picker_usage_items),
  };
  data->picker_sections[1] = (SimpleMenuSection){
      .title = "Report",
      .items = data->picker_path_items,
      .num_items = ARRAY_LENGTH(data->picker_path_items),
  };

  simple_menu_layer_init(&data->picker_menu, &window->layer.bounds, window, data->picker_sections,
                         ARRAY_LENGTH(data->picker_sections), data);
  // Open on the usage that is actually selected. set_selected_index() keeps the
  // current section, which is the first one right after init.
  for (unsigned i = 0; i < ARRAY_LENGTH(s_codes); ++i) {
    if (s_codes[i].usage == s_usage) {
      simple_menu_layer_set_selected_index(&data->picker_menu, i, false /* animated */);
      break;
    }
  }
  layer_add_child(&window->layer, simple_menu_layer_get_layer(&data->picker_menu));
}

static void prv_picker_window_unload(Window *window) {
  CameraRemoteAppData *data = window_get_user_data(window);
  simple_menu_layer_deinit(&data->picker_menu);
}

static void prv_select_click(ClickRecognizerRef recognizer, void *context) {
  CameraRemoteAppData *data = context;
  const status_t rv = ble_hid_consumer_press(s_usage, s_path);
  snprintf(data->status, sizeof(data->status), "%s 0x%03x", prv_status_label(rv),
           (unsigned)s_usage);
  // S_SUCCESS only, not PASSED(): ble_hid_consumer_press() answers
  // S_NO_ACTION_REQUIRED when the same usage and path are already held, and this
  // would then overwrite data->release_timer without cancelling the timer it
  // replaces. The first would still fire, clear the handle that by then refers
  // to the second, and release the key; the second would fire later against a
  // handle nothing owns, and prv_release_held() could cancel an already-fired
  // one. Leave release_timer alone and let the press that is already down keep
  // the release it came with.
  if (rv == S_SUCCESS) {
    data->held_usage = s_usage;
    data->release_timer = app_timer_register(CAMERA_REMOTE_TAP_MS, prv_release_timer_cb, data);
    if (!data->release_timer) {
      // ble_hid's safety timeout is seconds away and best effort at that, so
      // release now rather than leave the shutter held.
      PBL_LOG_WRN("Camera remote: no release timer, releasing immediately");
      prv_release_held(data);
    }
  }
  prv_update_readiness(data);
  text_layer_set_text(&data->status_text, data->status);
}

static void prv_select_long_click(ClickRecognizerRef recognizer, void *context) {
  CameraRemoteAppData *data = context;
  app_window_stack_push(&data->picker_window, true /* animated */);
}

static void prv_click_config(void *context) {
  // BACK keeps its default handler, which pops the window.
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click);
  window_long_click_subscribe(BUTTON_ID_SELECT, CAMERA_REMOTE_PICKER_HOLD_MS,
                              prv_select_long_click, NULL);
}

static void prv_window_load(Window *window) {
  CameraRemoteAppData *data = window_get_user_data(window);

  GRect bounds;
  layer_get_bounds(window_get_root_layer(window), &bounds);

  GRect title_frame = bounds;
  title_frame.size.h = 26;
  text_layer_init(&data->title_text, &title_frame);
  text_layer_set_font(&data->title_text, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(&data->title_text, GTextAlignmentCenter);
  text_layer_set_text(&data->title_text, i18n_get("Camera", data));
  layer_add_child(window_get_root_layer(window), text_layer_get_layer(&data->title_text));

  GRect body_frame = bounds;
  body_frame.origin.y = 26;
  body_frame.size.h = 38;
  text_layer_init(&data->body_text, &body_frame);
  text_layer_set_font(&data->body_text, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(&data->body_text, GTextAlignmentCenter);
  layer_add_child(window_get_root_layer(window), text_layer_get_layer(&data->body_text));

  GRect code_frame = bounds;
  code_frame.origin.y = 64;
  code_frame.size.h = 22;
  text_layer_init(&data->code_text, &code_frame);
  text_layer_set_font(&data->code_text, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
  text_layer_set_text_alignment(&data->code_text, GTextAlignmentCenter);
  layer_add_child(window_get_root_layer(window), text_layer_get_layer(&data->code_text));

  GRect path_frame = bounds;
  path_frame.origin.y = 86;
  path_frame.size.h = 20;
  text_layer_init(&data->path_text, &path_frame);
  text_layer_set_font(&data->path_text, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(&data->path_text, GTextAlignmentCenter);
  layer_add_child(window_get_root_layer(window), text_layer_get_layer(&data->path_text));

  GRect hint_frame = bounds;
  hint_frame.origin.y = 106;
  hint_frame.size.h = 18;
  text_layer_init(&data->hint_text, &hint_frame);
  text_layer_set_font(&data->hint_text, fonts_get_system_font(FONT_KEY_GOTHIC_14));
  text_layer_set_text_alignment(&data->hint_text, GTextAlignmentCenter);
  text_layer_set_text(&data->hint_text, "Hold Select: settings");
  layer_add_child(window_get_root_layer(window), text_layer_get_layer(&data->hint_text));

  GRect status_frame = bounds;
  status_frame.origin.y = bounds.size.h - 24;
  status_frame.size.h = 24;
  text_layer_init(&data->status_text, &status_frame);
  text_layer_set_font(&data->status_text, fonts_get_system_font(FONT_KEY_GOTHIC_14));
  text_layer_set_text_alignment(&data->status_text, GTextAlignmentCenter);
  text_layer_set_text(&data->status_text, data->status);
  layer_add_child(window_get_root_layer(window), text_layer_get_layer(&data->status_text));

  prv_update_readiness(data);
  prv_update_selection(data);
  prv_schedule_readiness_timer(data);

  // The remote is useless at the default connection interval; the poller
  // re-arms this, and it must be undone on unload or the link stays pinned in
  // a high-power state. Pushing the picker does not unload this window, so the
  // request survives the trip into the picker and back.
  ble_hid_set_low_latency(true);
}

static void prv_window_unload(Window *window) {
  CameraRemoteAppData *data = window_get_user_data(window);
  prv_cancel_readiness_timer(data);
  prv_release_held(data);
  ble_hid_set_low_latency(false);
}

static void prv_handle_init(void) {
  CameraRemoteAppData *data = app_malloc_check(sizeof(CameraRemoteAppData));
  data->readiness_timer = NULL;
  data->release_timer = NULL;
  data->held_usage = 0;
  data->shown_ready_valid = false;
  data->code[0] = '\0';
  data->path[0] = '\0';
  data->status[0] = '\0';
  app_state_set_user_data(data);

  prv_load_settings();

  window_init(&data->window, WINDOW_NAME("Camera Remote"));
  window_set_user_data(&data->window, data);
  window_set_window_handlers(&data->window, &(WindowHandlers){
                                                .load = prv_window_load,
                                                .unload = prv_window_unload,
                                            });
  window_set_click_config_provider_with_context(&data->window, prv_click_config, data);

  window_init(&data->picker_window, WINDOW_NAME("Camera Remote Code"));
  window_set_user_data(&data->picker_window, data);
  window_set_window_handlers(&data->picker_window, &(WindowHandlers){
                                                       .load = prv_picker_window_load,
                                                       .unload = prv_picker_window_unload,
                                                   });

  app_window_stack_push(&data->window, true /* animated */);
}

static void prv_handle_deinit(void) {
  CameraRemoteAppData *data = app_state_get_user_data();
  // Safety net in case the window never got unloaded.
  prv_cancel_readiness_timer(data);
  prv_release_held(data);
  ble_hid_set_low_latency(false);
  i18n_free_all(data);
  app_free(data);
}

static void prv_main(void) {
  prv_handle_init();

  app_event_loop();

  prv_handle_deinit();
}

const PebbleProcessMd *camera_remote_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_md = {
    .common = {
      .main_func = prv_main,
      // UUID: 78d71f3a-f372-40a8-a44a-38a01d958e7c
      .uuid = {0x78, 0xd7, 0x1f, 0x3a, 0xf3, 0x72, 0x40, 0xa8,
               0xa4, 0x4a, 0x38, 0xa0, 0x1d, 0x95, 0x8e, 0x7c},
    },
    .name = i18n_noop("Camera"),
  };
  return (const PebbleProcessMd *)&s_app_md;
}

#endif  // CONFIG_BT_HID_REMOTE
