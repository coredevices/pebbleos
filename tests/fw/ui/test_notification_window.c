/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/preferred_content_size.h"
#include "applib/ui/window_private.h"
#include "apps/system/settings/notifications_private.h"
#include "popups/notifications/notification_window.h"
#include "popups/notifications/notification_window_private.h"
#include "popups/notifications/notifications_presented_list.h"
#include "resource/timeline_resource_ids.auto.h"
#include "pbl/services/timeline/notification_layout.h"
#include "pbl/util/trig.h"

#include <stdio.h>

// Stubs
/////////////////////

#include "stubs_action_menu.h"
#include "stubs_alarm_layout.h"
// stubs_alerts.h intentionally omitted; see local replacements below
// stubs_alerts_preferences.h intentionally omitted; see local replacements below
#include "stubs_analytics.h"
#include "stubs_ancs_filtering.h"
#include "stubs_app_install_manager.h"
#include "stubs_app_state.h"
#include "stubs_app_timer.h"
#include "stubs_app_window_stack.h"
#include "stubs_bluetooth_persistent_storage.h"
#include "stubs_bootbits.h"
#include "stubs_buffer.h"
#include "stubs_calendar_layout.h"
#include "stubs_click.h"
#include "stubs_content_indicator.h"
#include "stubs_dialog.h"
#include "stubs_do_not_disturb.h"
#include "stubs_event_loop.h"
#include "stubs_event_service_client.h"
#include "stubs_evented_timer.h"
#include "stubs_generic_layout.h"
#include "stubs_health_layout.h"
#include "stubs_heap.h"
#include "stubs_i18n.h"
#include "stubs_imaging.h"
#include "stubs_ios_notif_pref_db.h"
#include "stubs_layer.h"
#include "stubs_light.h"
#include "stubs_logging.h"
#include "stubs_memory_layout.h"
#include "stubs_menu_cell_layer.h"
#include "stubs_modal_manager.h"
#include "stubs_mutex.h"
// stubs_notification_storage.h intentionally omitted; see local replacements below
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_process_info.h"
#include "stubs_pebble_tasks.h"
#include "stubs_peek_layer.h"
#include "stubs_pin_db.h"
#include "stubs_print.h"
#include "stubs_process_manager.h"
#include "stubs_prompt.h"
#include "stubs_regular_timer.h"
#include "stubs_reminder_db.h"
#include "stubs_reminders.h"
#include "stubs_serial.h"
#include "stubs_session.h"
#include "stubs_shell_prefs.h"
#include "stubs_simple_dialog.h"
#include "stubs_sleep.h"
#include "stubs_sports_layout.h"
#include "stubs_stringlist.h"
#include "stubs_syscall_internal.h"
#include "stubs_syscalls.h"
#include "stubs_task_watchdog.h"
#include "stubs_time.h"
#include "stubs_timeline.h"
#include "stubs_timeline_actions.h"
#include "stubs_timeline_item.h"
#include "stubs_timeline_layer.h"
#include "stubs_timeline_peek.h"
#include "stubs_vibes.h"
#include "stubs_vibe_client.h"
#include "stubs_vibe_score.h"
#include "stubs_vibe_score_info.h"
#include "stubs_weather_layout.h"
#include "stubs_window_manager.h"
#include "stubs_window_stack.h"

#include "pbl/services/notifications/alerts_preferences_private.h"

// Local replacements for stubs_alerts_preferences.h so the notification status
// bar style is settable per test (a strong override cannot share a TU with the
// header's WEAK definition).
static NotificationStatusBarStyle s_notification_status_bar_style =
    NotificationStatusBarStyle_Default;

NotificationStatusBarStyle alerts_preferences_get_notification_status_bar_style(void) {
  return s_notification_status_bar_style;
}

VibeScoreId alerts_preferences_get_vibe_score_for_client(VibeClient client) {
  return VibeScoreId_Invalid;
}

VibeIntensity alerts_preferences_get_vibe_intensity(void) {
  return VibeIntensityLow;
}

bool alerts_preferences_get_notification_alternative_design(void) {
  return false;
}

DndNotificationMode alerts_preferences_dnd_get_show_notifications(void) {
  return DndNotificationModeShow;
}

bool alerts_preferences_dnd_get_auto_dismiss(void) {
  return false;
}

bool alerts_preferences_get_notification_vibe_delay(void) {
  return false;
}

static bool s_respect_phone_silence;

bool alerts_preferences_get_respect_phone_silence(void) {
  return s_respect_phone_silence;
}

// Local replacements for stubs_alerts.h so the notification-added path can be
// exercised and the vibe gate observed per test.
#include "pbl/services/notifications/alerts_private.h"

static AlertMask s_alert_mask;
static bool s_should_notify;
static bool s_should_vibrate;
static int s_should_vibrate_call_count;

AlertMask alerts_get_mask(void) {
  return s_alert_mask;
}

void alerts_set_mask(AlertMask mask) {
  s_alert_mask = mask;
}

uint32_t alerts_get_notification_window_timeout_ms(void) {
  return 0;
}

void alerts_incoming_alert_analytics(void) {}

void alerts_set_notification_vibe_timestamp(void) {}

bool alerts_should_enable_backlight_for_type(AlertType type) {
  return false;
}

bool alerts_should_notify_for_type(AlertType type) {
  return s_should_notify;
}

bool alerts_should_vibrate_for_type(AlertType type) {
  s_should_vibrate_call_count++;
  return s_should_vibrate;
}

// Local replacements for stubs_notification_storage.h so a stored item can be
// read back by the quiet-delivery check.
#include "pbl/services/notifications/notification_storage.h"

static TimelineItem s_stored_notification;
static bool s_has_stored_notification;
static int s_notification_remove_count;

void notification_storage_init(void) {}

void notification_storage_lock(void) {}

void notification_storage_unlock(void) {}

void notification_storage_store(TimelineItem *notification) {}

bool notification_storage_notification_exists(const Uuid *id) {
  return false;
}

size_t notification_storage_get_len(const Uuid *uuid) {
  return 0;
}

bool notification_storage_get(const Uuid *id, TimelineItem *item_out) {
  if (!s_has_stored_notification || !uuid_equal(id, &s_stored_notification.header.id)) {
    return false;
  }
  *item_out = s_stored_notification;
  return true;
}

void notification_storage_set_status(const Uuid *id, uint8_t status) {}

bool notification_storage_get_status(const Uuid *id, uint8_t *status) {
  return false;
}

void notification_storage_remove(const Uuid *id) {
  s_notification_remove_count++;
}

bool notification_storage_find_ancs_notification_id(uint32_t ancs_uid, Uuid *uuid_out) {
  return false;
}

bool notification_storage_find_ancs_notification_by_timestamp(
    TimelineItem *notification, CommonTimelineItemHeader *header_out) {
  return false;
}

void notification_storage_rewrite(void (*iter_callback)(TimelineItem *notification,
    SerializedTimelineItemHeader *header, void *data), void *data) {}

void notification_storage_iterate(
    bool (*iter_callback)(void *data, SerializedTimelineItemHeader *header_id), void *data) {}

int16_t interpolate_int16(int32_t normalized, int16_t from, int16_t to) {
  return to;
}

uint32_t interpolate_uint32(int32_t normalized, uint32_t from, uint32_t to) {
  return to;
}

int64_t interpolate_moook(int32_t normalized, int64_t from, int64_t to) {
  return to;
}

uint32_t interpolate_moook_duration() {
  return 0;
}

int64_t interpolate_moook_soft(int32_t normalized, int64_t from, int64_t to,
                               int32_t num_frames_mid) {
  return to;
}

uint32_t interpolate_moook_soft_duration(int32_t num_frames_mid) {
  return 0;
}

// Fakes
/////////////////////

#include "fake_animation.h"
#include "fake_app_state.h"
#include "fake_content_indicator.h"
#include "fake_graphics_context.h"
#include "fake_spi_flash.h"
#include "../../fixtures/load_test_resources.h"

typedef struct NotificationWindowTestData {
  uint32_t icon_id;
  const char *app_name;
  const char *title;
  const char *subtitle;
  const char *location_name;
  const char *body;
  const char *timestamp;
  const char *reminder_timestamp;
  GColor primary_color;
  GColor background_color;
  bool show_notification_timestamp;
  bool is_reminder;
  struct {
    AttributeList attr_list;
    TimelineItem timeline_item;
  } statics;
} NotificationWindowTestData;

static NotificationWindowTestData s_test_data;

void clock_get_since_time(char *buffer, int buf_size, time_t timestamp) {
  if (buffer && s_test_data.timestamp) {
    strncpy(buffer, s_test_data.timestamp, (size_t)buf_size);
    buffer[buf_size - 1] = '\0';
  }
}

void clock_get_until_time(char *buffer, int buf_size, time_t timestamp, int max_relative_hrs) {
  if (buffer && s_test_data.reminder_timestamp) {
    strncpy(buffer, s_test_data.reminder_timestamp, (size_t)buf_size);
    buffer[buf_size - 1] = '\0';
  }
}

void clock_copy_time_string(char *buffer, uint8_t buf_size) {
  if (buffer) {
    strncpy(buffer, "12:00 PM", buf_size);
    buffer[buf_size - 1] = '\0';
  }
}

//! This function overrides the implementation in swap_layer.c as a way of providing the data
//! we want to display in each notification
LayoutLayer *prv_get_layout_handler(SwapLayer *swap_layer, int8_t rel_position,
                                    void *context) {
  // Only support one layout at a time for now
  if (rel_position != 0) {
    return NULL;
  }

  NotificationWindowData *data = context;

  AttributeList *attr_list = &s_test_data.statics.attr_list;
  attribute_list_add_resource_id(attr_list, AttributeIdIconTiny, s_test_data.icon_id);
  if (s_test_data.app_name) {
    attribute_list_add_cstring(attr_list, AttributeIdAppName, s_test_data.app_name);
  }
  if (s_test_data.title) {
    attribute_list_add_cstring(attr_list, AttributeIdTitle, s_test_data.title);
  }
  if (s_test_data.subtitle) {
    attribute_list_add_cstring(attr_list, AttributeIdSubtitle, s_test_data.subtitle);
  }
  if (s_test_data.location_name) {
    attribute_list_add_cstring(attr_list, AttributeIdLocationName, s_test_data.location_name);
  }
  if (s_test_data.body) {
    attribute_list_add_cstring(attr_list, AttributeIdBody, s_test_data.body);
  }
  if (!gcolor_is_invisible(s_test_data.primary_color)) {
    attribute_list_add_uint8(attr_list, AttributeIdPrimaryColor, s_test_data.primary_color.argb);
  }
  if (!gcolor_is_invisible(s_test_data.background_color)) {
    attribute_list_add_uint8(attr_list, AttributeIdBgColor, s_test_data.background_color.argb);
  }

  s_test_data.statics.timeline_item = (TimelineItem) {
    .header = (CommonTimelineItemHeader) {
      .layout = LayoutIdNotification,
      .type = s_test_data.is_reminder ? TimelineItemTypeReminder : TimelineItemTypeNotification,
    },
    .attr_list = *attr_list,
  };

  TimelineItem *item = &s_test_data.statics.timeline_item;

  NotificationLayoutInfo layout_info = (NotificationLayoutInfo) {
    .item = item,
    .show_notification_timestamp = s_test_data.show_notification_timestamp,
  };
  const LayoutLayerConfig config = {
    .frame = &data->window.layer.bounds,
    .attributes = &item->attr_list,
    .mode = LayoutLayerModeCard,
    .app_id = &data->notification_app_id,
    .context = &layout_info,
  };
  return notification_layout_create(&config);
}

static void prv_property_animation_grect_update(Animation *animation,
                                                const AnimationProgress progress) {
  PropertyAnimationPrivate *property_animation = (PropertyAnimationPrivate *)animation;
  if (property_animation) {
    layer_set_frame(property_animation->subject, &property_animation->values.to.grect);
  }
}

static const PropertyAnimationImplementation s_frame_layer_implementation = {
  .base.update = prv_property_animation_grect_update,
  .accessors = {
    .setter.grect = (const GRectSetter)layer_set_frame_by_value,
    .getter.grect = (const GRectGetter)layer_get_frame_by_value,
  },
};

//! Overrides the stub in stubs_animation.c to provide the proper plumbing for scrolling
PropertyAnimation *property_animation_create_layer_frame(
  struct Layer *layer, GRect *from_frame, GRect *to_frame) {
  PropertyAnimationPrivate *animation = (PropertyAnimationPrivate *)
    property_animation_create(&s_frame_layer_implementation, layer, from_frame, to_frame);
  if (from_frame) {
    animation->values.from.grect = *from_frame;
    PropertyAnimationImplementation *impl =
      (PropertyAnimationImplementation *)animation->animation.implementation;
    impl->accessors.setter.grect(animation->subject, animation->values.from.grect);
  }
  if (to_frame) {
    animation->values.to.grect = *to_frame;
  }
  return (PropertyAnimation *)animation;
}

// Helper Functions
/////////////////////

#include "../graphics/test_graphics.h"
#include "../graphics/util.h"

// Setup and Teardown
////////////////////////////////////

// To easily render multiple windows in a single canvas, we'll use an 8-bit bitmap for color
// displays (including round), but we can use the native format for black and white displays (1-bit)
#define CANVAS_GBITMAP_FORMAT PBL_IF_COLOR_ELSE(GBitmapFormat8Bit, GBITMAP_NATIVE_FORMAT)

// Overrides same function in graphics.c; we need to do this so we can pass in the GBitmapFormat
// we need to use for the unit test output canvas instead of relying on GBITMAP_NATIVE_FORMAT, which
// wouldn't work for Spalding since it uses GBitmapFormat8BitCircular
GBitmap* graphics_capture_frame_buffer(GContext *ctx) {
  PBL_ASSERTN(ctx);
  return graphics_capture_frame_buffer_format(ctx, CANVAS_GBITMAP_FORMAT);
}

// Overrides same function in graphics.c; we need to do this so we can release the framebuffer we're
// using even though its format doesn't match GBITMAP_NATIVE_FORMAT (see comment for mocked
// graphics_capture_frame_buffer() above)
bool graphics_release_frame_buffer(GContext *ctx, GBitmap *buffer) {
  PBL_ASSERTN(ctx);
  ctx->lock = false;
  framebuffer_dirty_all(ctx->parent_framebuffer);
  return true;
}

extern NotificationWindowData s_notification_window_data;
extern bool s_in_use;

void test_notification_window__initialize(void) {
  fake_app_state_init();
  load_system_resources_fixture();

  attribute_list_destroy_list(&s_test_data.statics.attr_list);
  s_test_data = (NotificationWindowTestData) {};
  s_notification_status_bar_style = NotificationStatusBarStyle_Default;
  s_respect_phone_silence = false;
  s_should_notify = false;
  s_should_vibrate = false;
  s_should_vibrate_call_count = 0;
  s_stored_notification = (TimelineItem) {};
  s_has_stored_notification = false;
  s_notification_remove_count = 0;
  notifications_presented_list_init();
  // Reset the notification window so each test gets a fresh prv_init_notification_window
  // call with the correct status-bar style.  Without this, s_in_use=true from a previous
  // test (e.g. big_bold setting LargeBold mode) would prevent re-initialization and leave
  // both the status-bar mode and swap-layer frame stale.
  s_in_use = false;
}

void test_notification_window__cleanup(void) {
}

// Helpers
//////////////////////

extern NotificationWindowData s_notification_window_data;

//! Static function in swap_layer.c used to scroll
void prv_attempt_scroll(SwapLayer *swap_layer, ScrollDirection direction, bool is_repeating);

static void prv_render_notification_window(unsigned int num_down_scrolls) {
  Window *window = &s_notification_window_data.window;

  // Set the window on screen so its load/appear handlers will be called
  window_set_on_screen(window, true, true);

  // Trigger a reload of the NotificationWindow's SwapLayer so it will be updated with the content
  // in s_test_data
  swap_layer_reload_data(&s_notification_window_data.swap_layer);

  // Scroll down the specified number of times
  SwapLayer *swap_layer = &s_notification_window_data.swap_layer;
  for (int i = 0; i < num_down_scrolls; i++) {
    prv_attempt_scroll(swap_layer, ScrollDirectionDown, false /* is_repeating */);
    fake_animation_complete(swap_layer->animation);
    swap_layer->animation = NULL;
  }

  // Force the display of the action button
  layer_set_hidden(&s_notification_window_data.action_button_layer, false);

  // Render the window
  window_render(window, fake_graphics_context_get_context());
}

//! @note This must be a multiple of 8 so that we are word-aligned when using a 1-bit bitmap.
#define GRID_CELL_PADDING 8

static void prv_prepare_canvas_and_render_notification_windows(unsigned int num_down_scrolls) {
  // Initialize the notification window module before rendering anything
  notification_window_init(false /* is_modal */);

  const unsigned int num_columns = SettingsContentSizeCount;
  const unsigned int num_rows = num_down_scrolls + 1;

  const int16_t bitmap_width = (DISP_COLS * num_columns) + (GRID_CELL_PADDING * (num_columns + 1));
  const int16_t bitmap_height =
      (int16_t)((num_rows == 1) ? DISP_ROWS :
                                  ((DISP_ROWS * num_rows) + (GRID_CELL_PADDING * (num_rows + 1))));
  const GSize bitmap_size = GSize(bitmap_width, bitmap_height);
  GBitmap *canvas_bitmap = gbitmap_create_blank(bitmap_size, CANVAS_GBITMAP_FORMAT);
  PBL_ASSERTN(canvas_bitmap);

  GContext *ctx = fake_graphics_context_get_context();
  ctx->dest_bitmap = *canvas_bitmap;
  // We modify the bitmap's data pointer below so save a reference to the original here
  uint8_t *saved_bitmap_addr = ctx->dest_bitmap.addr;
  const uint8_t bitdepth = gbitmap_get_bits_per_pixel(ctx->dest_bitmap.info.format);

  // Fill the bitmap with pink (on color) or white (on b&w) so it's easier to see errors
  const GColor out_of_bounds_color = PBL_IF_COLOR_ELSE(GColorShockingPink, GColorWhite);
  memset(canvas_bitmap->addr, out_of_bounds_color.argb,
         canvas_bitmap->row_size_bytes * canvas_bitmap->bounds.size.h);

  for (int settings_content_size = 0; settings_content_size < SettingsContentSizeCount;
       settings_content_size++) {
    const PreferredContentSize content_size =
        settings_content_size_to_preferred_size((SettingsContentSize)settings_content_size);
    system_theme_set_content_size(content_size);

    const int16_t x_offset =
        (int16_t)(GRID_CELL_PADDING + (settings_content_size * (GRID_CELL_PADDING + DISP_COLS)));

    for (int down_scrolls = 0; down_scrolls <= num_down_scrolls; down_scrolls++) {
      const int16_t y_offset =
          (int16_t)((num_rows == 1) ? 0 :
                    GRID_CELL_PADDING + (down_scrolls * (GRID_CELL_PADDING + DISP_ROWS)));
      // Set the GContext bitmap's data pointer to the position in the larger bitmap where we
      // want to draw this particular notification window
      ctx->dest_bitmap.addr =
          saved_bitmap_addr + (y_offset * ctx->dest_bitmap.row_size_bytes) +
            (x_offset * bitdepth / 8);

      prv_render_notification_window((unsigned int)down_scrolls);

      // On Round we end up drawing outside the visible screen bounds, so let's draw a circle where
      // those bounds are to help us visualize each copy of the screen
#if PBL_ROUND
      graphics_context_set_fill_color(ctx, GColorBlack);
      graphics_fill_radial(ctx, DISP_FRAME, GOvalScaleModeFitCircle, 1, 0, TRIG_MAX_ANGLE);
#endif
    }
  }

  // Restore the bitmap's original data pointer
  ctx->dest_bitmap.addr = saved_bitmap_addr;
}

// Tests
//////////////////////

void test_notification_window__title_body(void) {
  s_test_data = (NotificationWindowTestData) {
    .icon_id = TIMELINE_RESOURCE_NOTIFICATION_FACEBOOK_MESSENGER,
    .title = "Henry Levak",
    .body = "Nu, Shara. Where are my designs, blat?",
    .show_notification_timestamp = true,
    .timestamp = "Just now",
    .background_color = GColorPictonBlue,
  };
  const unsigned int num_down_scrolls =
      PBL_IF_RECT_ELSE((PreferredContentSizeDefault < PreferredContentSizeLarge) ? 1 : 0, 0);
  prv_prepare_canvas_and_render_notification_windows(num_down_scrolls);
  FAKE_GRAPHICS_CONTEXT_CHECK_DEST_BITMAP_FILE();
}

void test_notification_window__title_subtitle_body(void) {
  s_test_data = (NotificationWindowTestData) {
    .icon_id = TIMELINE_RESOURCE_NOTIFICATION_GOOGLE_INBOX,
    .title = "Henry Levak",
    .subtitle = "Henry Levak sent you a 1-1 message",
    .body = "Good morning to you my friend!",
    .background_color = GColorRed,
  };
  prv_prepare_canvas_and_render_notification_windows(PBL_IF_RECT_ELSE(2, 1) /* num_down_scrolls */);
  FAKE_GRAPHICS_CONTEXT_CHECK_DEST_BITMAP_FILE();
}

void test_notification_window__reminder(void) {
  s_test_data = (NotificationWindowTestData) {
    .icon_id = TIMELINE_RESOURCE_NOTIFICATION_REMINDER,
    .title = "Feed Humphrey",
    .location_name = "RWC Office",
    .body = "Only the best!",
    .reminder_timestamp = "In 15 minutes",
    .is_reminder = true,
  };
  const unsigned int num_down_scrolls =
      (PreferredContentSizeDefault >= PreferredContentSizeLarge) ? 0 : 1;
  prv_prepare_canvas_and_render_notification_windows(num_down_scrolls);
  FAKE_GRAPHICS_CONTEXT_CHECK_DEST_BITMAP_FILE();
}

void test_notification_window__body_icon(void) {
  s_test_data = (NotificationWindowTestData) {
    .icon_id = TIMELINE_RESOURCE_NOTIFICATION_GOOGLE_HANGOUTS,
    .title = "Kevin Conley",
    .subtitle = (PreferredContentSizeDefault >= PreferredContentSizeLarge) ? "New mail!" : NULL,
    .body = "❤",
    .background_color = GColorIslamicGreen,
  };
  prv_prepare_canvas_and_render_notification_windows(0 /* num_down_scrolls */);
  FAKE_GRAPHICS_CONTEXT_CHECK_DEST_BITMAP_FILE();
}

// Quiet delivery (phone-silent ANCS notifications)
//////////////////////

static const Uuid s_quiet_notif_id = {0x6b, 0x03, 0x2c, 0xd8, 0x0a, 0x2e, 0x4e, 0x5a,
                                      0x92, 0x22, 0x71, 0x8e, 0x4b, 0xb2, 0x8e, 0x11};

//! Store a notification and dispatch a NotificationAdded event for it.
static void prv_store_and_add_notification(const Uuid *id, bool ancs_notif, bool silent) {
  s_stored_notification = (TimelineItem) {
    .header = (CommonTimelineItemHeader) {
      .id = *id,
      .type = TimelineItemTypeNotification,
      .layout = LayoutIdNotification,
      .ancs_notif = ancs_notif,
      .silent = silent,
    },
  };
  s_has_stored_notification = true;

  PebbleSysNotificationEvent event = {
    .type = NotificationAdded,
    .notification_id = (Uuid *)id,
  };
  notification_window_handle_notification(&event);
}

static void prv_setup_quiet_delivery_test(void) {
  s_should_notify = true;
  s_should_vibrate = true;
  // Content for the layout created when the window loads the notification
  s_test_data = (NotificationWindowTestData) {
    .icon_id = TIMELINE_RESOURCE_NOTIFICATION_GENERIC,
    .title = "Quiet",
    .body = "Delivered silently on the phone",
  };
}

void test_notification_window__silent_ancs_suppressed_when_pref_on(void) {
  prv_setup_quiet_delivery_test();
  s_respect_phone_silence = true;

  // Platform-neutral: suppression works from header.silent alone, even without ancs_notif
  // (the companion-app / Android wire-flag case).
  prv_store_and_add_notification(&s_quiet_notif_id, false /* ancs_notif */, true /* silent */);

  // No popup: the window was never initialized and nothing was presented
  cl_assert(!s_in_use);
  cl_assert_equal_i(notifications_presented_list_count(), 0);
  // No vibe: the vibe gate was never consulted
  cl_assert_equal_i(s_should_vibrate_call_count, 0);
  // The item remains in storage
  TimelineItem item = {};
  cl_assert(notification_storage_get(&s_quiet_notif_id, &item));
  cl_assert_equal_i(s_notification_remove_count, 0);
}

void test_notification_window__non_silent_ancs_shown_when_pref_on(void) {
  prv_setup_quiet_delivery_test();
  s_respect_phone_silence = true;

  prv_store_and_add_notification(&s_quiet_notif_id, true /* ancs_notif */, false /* silent */);

  cl_assert(s_in_use);
  cl_assert_equal_i(notifications_presented_list_count(), 1);
  cl_assert(uuid_equal(notifications_presented_list_current(), &s_quiet_notif_id));
  cl_assert(s_should_vibrate_call_count > 0);
}

void test_notification_window__silent_ancs_shown_when_pref_off(void) {
  prv_setup_quiet_delivery_test();
  s_respect_phone_silence = false;

  prv_store_and_add_notification(&s_quiet_notif_id, true /* ancs_notif */, true /* silent */);

  cl_assert(s_in_use);
  cl_assert_equal_i(notifications_presented_list_count(), 1);
  cl_assert(uuid_equal(notifications_presented_list_current(), &s_quiet_notif_id));
  cl_assert(s_should_vibrate_call_count > 0);
}

void test_notification_window__big_bold(void) {
  s_notification_status_bar_style = NotificationStatusBarStyle_LargeBold;
  s_test_data = (NotificationWindowTestData) {
    .icon_id = TIMELINE_RESOURCE_NOTIFICATION_FACEBOOK_MESSENGER,
    .title = "Henry Levak",
    .body = "Nu, Shara. Where are my designs, blat?",
    .show_notification_timestamp = true,
    .timestamp = "Just now",
    .background_color = GColorPictonBlue,
  };
  const unsigned int num_down_scrolls =
      PBL_IF_RECT_ELSE((PreferredContentSizeDefault < PreferredContentSizeLarge) ? 1 : 0, 0);
  prv_prepare_canvas_and_render_notification_windows(num_down_scrolls);
  FAKE_GRAPHICS_CONTEXT_CHECK_DEST_BITMAP_FILE();
}
