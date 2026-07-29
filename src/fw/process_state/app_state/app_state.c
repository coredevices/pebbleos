/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "process_state/app_state/app_state.h"

#include "applib/app_message/app_message_internal.h"
#include "applib/event_service_client.h"
#include "applib/graphics/framebuffer.h"
#include "applib/graphics/graphics.h"
#include "applib/pbl_std/locale.h"
#include "applib/ui/animation_private.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/layer.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/touch_nav.h"
#include "applib/ui/click.h"
#include "applib/ui/click_internal.h"
#include "applib/ui/window_private.h"
#include "applib/ui/window_stack.h"
#include "applib/touch_service.h"
#include "applib/touch_service_private.h"
#include "pbl/services/touch/touch.h"
#include "pbl/drivers/button_id.h"
#include "applib/unobstructed_area_service.h"
#include "kernel/util/segment.h"
#include "process_management/app_install_types.h"
#include "process_management/app_manager.h"
#include "process_management/process_loader.h"
#include "process_management/process_manager.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/persist.h"
#include "syscall/syscall_internal.h"
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "tinymt32.h"

#include <string.h>

typedef struct {
  Heap heap;

  struct tm gmtime_tm;
  struct tm localtime_tm;
  char localtime_zone[TZ_LEN];

  tinymt32_t rand_seed;

  ProcessAppSDKType sdk_type;
  int16_t initial_obstruction_origin_y;

  ClickManager click_manager;

  AppInbox *app_message_inbox;

  EventServiceInfo app_outbox_subscription_info;

  ApplibInternalEventsInfo applib_internal_events_info;

  AnimationState animation_state;

  AppMessageCtx app_message_ctx;

  WindowStack window_stack;

  FrameBuffer framebuffer;

  GContext graphics_context;

  EventServiceInfo event_service_state;

  BLEAppState ble_app_state;

  AccelServiceState accel_state;

  CompassServiceConfig *compass_config;

  PluginServiceState plugin_service_state;

  void* user_data;

  LogState log_state;

  BatteryStateServiceState battery_state_service_state;

  BacklightServiceState backlight_service_state;

  TickTimerServiceState tick_timer_service_state;

  TouchServiceState touch_service_state;

  ConnectionServiceState connection_service_state;

  HealthServiceState health_service_state;

  LocaleInfo locale_info;

  ContentIndicatorsBuffer content_indicators_buffer;

  bool app_framebuffer_render_pending;

  AppFocusState app_focus_state;

  UnobstructedAreaState unobstructed_area_service_state;

  Layer* layer_tree_stack[LAYER_TREE_STACK_SIZE];

  WakeupHandler wakeup_handler;

  EventServiceInfo wakeup_event_info;

  SpeakerFinishedCallback speaker_finish_handler;
  void *speaker_finish_ctx;
  EventServiceInfo speaker_finish_event_info;

#ifdef CONFIG_TOUCH
  RecognizerList recognizer_list;
  RecognizerManager recognizer_manager;
  TouchNavState touch_nav_state;
  //! Whether this app participates in touch navigation. Set at launch to whether the current app is
  //! a system app; a third-party app can opt in at runtime via app_touch_navigation_enable().
  bool touch_nav_participating;
  //! Whether this app EXPLICITLY opted in via app_touch_navigation_enable(). An opted-in app
  //! follows the master "Touch" pref alone; mere participation (system apps) follows the system
  //! nav state (master AND the Touch Navigation sub-pref).
  bool touch_nav_opted_in;
#endif

  uint8_t *js_runtime_context_buffer;
  JsRuntimeContext *js_runtime_context;

  JsMemoryAPIContext *js_memory_api_context;

  AppGlance glance;

  TextRenderState text_render_state;

  bool text_perimeter_debugging_enabled;

  TimelineItemActionSource current_timeline_item_action_source;

  GBitmap *legacy2_framebuffer;
} AppState;

KERNEL_READONLY_DATA static AppState *s_app_state_ptr;

bool app_state_configure(MemorySegment *app_state_ram,
                         ProcessAppSDKType sdk_type,
                         int16_t obstruction_origin_y) {
  s_app_state_ptr = memory_segment_split(app_state_ram, NULL, sizeof(AppState));
  if (!s_app_state_ptr) {
    return false;
  }

  s_app_state_ptr->sdk_type = sdk_type;
  s_app_state_ptr->initial_obstruction_origin_y = obstruction_origin_y;

  if (GBITMAP_NATIVE_FORMAT != GBitmapFormat1Bit &&
      sdk_type == ProcessAppSDKType_Legacy2x) {
    // When running legacy2 aplite apps on basalt we actually have some space
    // after AppState that we don't need, because legacy2 aplite apps need to
    // support running on the smaller platform anyway. We can use this space for
    // doing legacy2 aplite-compatibility work. Note we don't have to worry
    // about 3.x aplite here because we don't support running 3.x aplite apps on
    // 3.x basalt platforms.

    s_app_state_ptr->legacy2_framebuffer = memory_segment_split(
        app_state_ram, NULL, sizeof(GBitmap));
    if (!s_app_state_ptr->legacy2_framebuffer) {
      return false;
    }

    uint16_t row_size = gbitmap_format_get_row_size_bytes(
        LEGACY_2X_DISP_COLS, GBitmapFormat1Bit);
    void *fb_data = memory_segment_split(app_state_ram, NULL,
                                         row_size * LEGACY_2X_DISP_ROWS);
    if (!fb_data) {
      return false;
    }

    *s_app_state_ptr->legacy2_framebuffer = (GBitmap) {
      .addr = fb_data,
      .row_size_bytes = row_size,
      .info.is_bitmap_heap_allocated = false,
      .info.format = GBitmapFormat1Bit,
      .info.version = GBITMAP_VERSION_0,
      .bounds = { { 0, 0 }, { LEGACY_2X_DISP_COLS, LEGACY_2X_DISP_ROWS } }
    };
  }
  return true;
}

#ifdef CONFIG_TOUCH
// Touch-nav bridge effects for the app task.
static bool prv_app_touch_nav_is_animating(void *ctx) {
  return window_stack_is_animating(app_state_get_window_stack());
}

static bool prv_app_touch_nav_top_overrides_back(void *ctx) {
  Window *top = app_window_stack_get_top_window();
  return top && top->overrides_back_button;
}

static bool prv_app_touch_nav_top_bridge_disabled(void *ctx) {
  Window *top = app_window_stack_get_top_window();
  const bool window_opt_out = top && top->touch_bridge_disabled;
  // The raw slot reflects only a genuine app raw subscription (touch_service_subscribe): the app
  // twin installs into the separate system slot, and the backlight / nav system-hold live in the
  // touch service layer, not this slot. So a set raw_handler means the app owns touch and must not
  // also receive synthesized buttons.
  const TouchServiceState *touch_state = app_state_get_touch_service_state();
  const bool app_has_raw_subscriber = touch_state && (touch_state->raw_handler != NULL);
  return touch_nav_app_bridge_disabled(window_opt_out, app_has_raw_subscriber);
}

static void prv_app_touch_nav_pop_top(void *ctx) {
  window_stack_pop_with_transition(app_state_get_window_stack(), NULL /* default pop transition */);
}

static void prv_app_touch_nav_emit_button(void *ctx, ButtonId button) {
  ClickManager *cm = app_state_get_click_manager();
  click_recognizer_handle_button_down(&cm->recognizers[button]);
  click_recognizer_handle_button_up(&cm->recognizers[button]);
}

static const TouchNavOps s_app_touch_nav_ops = {
  .is_animating = prv_app_touch_nav_is_animating,
  .top_overrides_back = prv_app_touch_nav_top_overrides_back,
  .top_bridge_disabled = prv_app_touch_nav_top_bridge_disabled,
  .pop_top = prv_app_touch_nav_pop_top,
  .emit_button = prv_app_touch_nav_emit_button,
  .idle_refresh = NULL,  // the app task has no idle-timeout refresh; that is the kernel's job
};

// Twin subscription effects for the app task. The subscribe/reconcile state machine lives in
// touch_nav.c; these ops bind it to the app task's touch service slot and nav state.
static bool prv_app_twin_pref_enabled(void *ctx) {
  return touch_nav_enabled();
}

static bool prv_app_twin_master_enabled(void *ctx) {
  // The master "Touch" switch IS the global touch kill: an opted-in app follows it alone.
  return touch_service_is_globally_enabled();
}

static void prv_app_twin_install_handler(void *ctx) {
  // touch_service_set_system_handler also no-ops for watchfaces, so they never receive emulation.
  touch_service_set_system_handler(touch_nav_dispatch, &s_app_state_ptr->touch_nav_state);
  // Mark the app twin live so the dispatch gate and touch-driven backlight treat this app as
  // navigating even when system nav (the Touch Navigation sub-pref) is off.
  touch_set_app_nav_active(true);
}

static void prv_app_twin_remove_handler(void *ctx) {
  touch_set_app_nav_active(false);
  recognizer_manager_cancel_and_reset(&s_app_state_ptr->recognizer_manager);
  touch_service_set_system_handler(NULL, NULL);
}

static const TouchNavTwinOps s_app_twin_ops = {
  .pref_enabled = prv_app_twin_pref_enabled,
  .master_enabled = prv_app_twin_master_enabled,
  .install_handler = prv_app_twin_install_handler,
  .remove_handler = prv_app_twin_remove_handler,
};
#endif

NOINLINE void app_state_init(void) {
  s_app_state_ptr->rand_seed.mat1 = 0; // Uninitialized

  click_manager_init(&s_app_state_ptr->click_manager);

#ifdef CONFIG_TOUCH
  recognizer_manager_init(&s_app_state_ptr->recognizer_manager);
  s_app_state_ptr->recognizer_manager.global_list = &s_app_state_ptr->recognizer_list;
  touch_nav_state_init(&s_app_state_ptr->touch_nav_state, &s_app_state_ptr->recognizer_manager,
                       &s_app_touch_nav_ops);
  // Participate only when this is positively a system/built-in app (negative install id). Third-party
  // app-DB apps (positive id) and unresolved/INSTALL_ID_INVALID (0) are inert by default and must opt
  // in via app_touch_navigation_enable(), so the unsafe direction (unknown -> on) cannot happen.
  s_app_state_ptr->touch_nav_participating =
      app_install_id_from_system(app_manager_get_current_app_id());
  s_app_state_ptr->touch_nav_opted_in = false;
#endif

  animation_private_state_init(&s_app_state_ptr->animation_state);

  app_message_init();

  app_outbox_init();

  // Set the correct framebuffer size depending on the SDK version
  GSize fb_size;
  app_manager_get_framebuffer_size(&fb_size);
  framebuffer_init(&s_app_state_ptr->framebuffer, &fb_size);

  framebuffer_clear(&s_app_state_ptr->framebuffer);

  const GContextInitializationMode init_mode =
      (s_app_state_ptr->sdk_type == ProcessAppSDKType_System) ? GContextInitializationMode_System :
                                                            GContextInitializationMode_App;
  graphics_context_init(&s_app_state_ptr->graphics_context,
                        &s_app_state_ptr->framebuffer, init_mode);


  ble_init_app_state();

  accel_service_state_init(app_state_get_accel_state());

  plugin_service_state_init(app_state_get_plugin_service());

  battery_state_service_state_init(app_state_get_battery_state_service_state());

  backlight_service_state_init(app_state_get_backlight_service_state());

  connection_service_state_init(app_state_get_connection_service_state());

  tick_timer_service_state_init(app_state_get_tick_timer_service_state());

  touch_service_state_init(app_state_get_touch_service_state());

#ifdef CONFIG_TOUCH
  // Route this app's touch events through the nav bridge when the master pref is on. No-op for
  // watchfaces (touch_service_set_system_handler declines them).
  app_touch_nav_subscribe();
#endif

  health_service_state_init(app_state_get_health_service_state());

  locale_init_app_locale(app_state_get_locale_info());

  content_indicator_init_buffer(app_state_get_content_indicators_buffer());

  unobstructed_area_service_init(app_state_get_unobstructed_area_state(),
                                 s_app_state_ptr->initial_obstruction_origin_y);

#if !defined(CONFIG_RECOVERY_FW)
  app_glance_service_init_glance(app_state_get_glance());
#endif

  s_app_state_ptr->current_timeline_item_action_source = TimelineItemActionSourceModalNotification;
}

NOINLINE void app_state_deinit(void) {
  animation_private_state_deinit(&s_app_state_ptr->animation_state);
  health_service_state_deinit(app_state_get_health_service_state());
  unobstructed_area_service_deinit(app_state_get_unobstructed_area_state());
}

Heap *app_state_get_heap(void) {
  return &s_app_state_ptr->heap;
}

struct tm *app_state_get_gmtime_tm(void) {
  return &s_app_state_ptr->gmtime_tm;
}
struct tm *app_state_get_localtime_tm(void) {
  return &s_app_state_ptr->localtime_tm;
}
char *app_state_get_localtime_zone(void) {
  return s_app_state_ptr->localtime_zone;
}
void *app_state_get_rand_ptr(void) {
  return &s_app_state_ptr->rand_seed;
}

AppInbox **app_state_get_app_message_inbox(void) {
  return &s_app_state_ptr->app_message_inbox;
}

EventServiceInfo *app_state_get_app_outbox_subscription_info(void) {
  return &s_app_state_ptr->app_outbox_subscription_info;
}

AnimationState* app_state_get_animation_state() {
  return &s_app_state_ptr->animation_state;
}

AppMessageCtx *app_state_get_app_message_ctx(void) {
  return &s_app_state_ptr->app_message_ctx;
}

BLEAppState* app_state_get_ble_app_state(void) {
  return &s_app_state_ptr->ble_app_state;
}

ClickManager* app_state_get_click_manager() {
  return &s_app_state_ptr->click_manager;
}

WindowStack* app_state_get_window_stack() {
  return &s_app_state_ptr->window_stack;
}

FrameBuffer* app_state_get_framebuffer() {
  return &s_app_state_ptr->framebuffer;
}

GContext* app_state_get_graphics_context() {
  return &s_app_state_ptr->graphics_context;
}

EventServiceInfo* app_state_get_event_service_state(void) {
  return &s_app_state_ptr->event_service_state;
}

void app_state_set_user_data(void *data) {
  s_app_state_ptr->user_data = data;
}

void* app_state_get_user_data(void) {
  return s_app_state_ptr->user_data;
}

AccelServiceState* app_state_get_accel_state(void) {
  return &s_app_state_ptr->accel_state;
}

CompassServiceConfig **app_state_get_compass_config(void) {
  return &s_app_state_ptr->compass_config;
}

PluginServiceState *app_state_get_plugin_service(void) {
  return &s_app_state_ptr->plugin_service_state;
}

LogState *app_state_get_log_state(void) {
  return &s_app_state_ptr->log_state;
}

BatteryStateServiceState *app_state_get_battery_state_service_state(void) {
  return &s_app_state_ptr->battery_state_service_state;
}

BacklightServiceState *app_state_get_backlight_service_state(void) {
  return &s_app_state_ptr->backlight_service_state;
}

TickTimerServiceState *app_state_get_tick_timer_service_state(void) {
  return &s_app_state_ptr->tick_timer_service_state;
}

TouchServiceState *app_state_get_touch_service_state(void) {
  return &s_app_state_ptr->touch_service_state;
}

ConnectionServiceState *app_state_get_connection_service_state(void) {
  return &s_app_state_ptr->connection_service_state;
}

HealthServiceState *app_state_get_health_service_state(void) {
  return &s_app_state_ptr->health_service_state;
}

ContentIndicatorsBuffer *app_state_get_content_indicators_buffer(void) {
  return &s_app_state_ptr->content_indicators_buffer;
}

LocaleInfo *app_state_get_locale_info(void) {
  return &s_app_state_ptr->locale_info;
}

bool *app_state_get_framebuffer_render_pending() {
  return &s_app_state_ptr->app_framebuffer_render_pending;
}

Layer** app_state_get_layer_tree_stack(void) {
  return s_app_state_ptr->layer_tree_stack;
}

AppFocusState *app_state_get_app_focus_state(void) {
  return &s_app_state_ptr->app_focus_state;
}

UnobstructedAreaState *app_state_get_unobstructed_area_state(void) {
  return &s_app_state_ptr->unobstructed_area_service_state;
}

AppGlance *app_state_get_glance(void) {
  return &s_app_state_ptr->glance;
}

WakeupHandler app_state_get_wakeup_handler(void) {
  return s_app_state_ptr->wakeup_handler;
}

void app_state_set_wakeup_handler(WakeupHandler handler) {
  s_app_state_ptr->wakeup_handler = handler;
}

EventServiceInfo *app_state_get_wakeup_event_info(void) {
  return &s_app_state_ptr->wakeup_event_info;
}

SpeakerFinishedCallback app_state_get_speaker_finish_handler(void) {
  return s_app_state_ptr->speaker_finish_handler;
}

void app_state_set_speaker_finish_handler(SpeakerFinishedCallback handler) {
  s_app_state_ptr->speaker_finish_handler = handler;
}

void *app_state_get_speaker_finish_ctx(void) {
  return s_app_state_ptr->speaker_finish_ctx;
}

void app_state_set_speaker_finish_ctx(void *ctx) {
  s_app_state_ptr->speaker_finish_ctx = ctx;
}

EventServiceInfo *app_state_get_speaker_finish_event_info(void) {
  return &s_app_state_ptr->speaker_finish_event_info;
}

GBitmap* app_state_legacy2_get_2bit_framebuffer(void) {
  PBL_ASSERTN(s_app_state_ptr->legacy2_framebuffer);
  return s_app_state_ptr->legacy2_framebuffer;
}

#ifdef CONFIG_TOUCH
RecognizerList *app_state_get_recognizer_list(void) {
  return &s_app_state_ptr->recognizer_list;
}

RecognizerManager *app_state_get_recognizer_manager(void) {
  return &s_app_state_ptr->recognizer_manager;
}

TouchNavState *app_state_get_touch_nav_state(void) {
  return &s_app_state_ptr->touch_nav_state;
}

void app_touch_nav_subscribe(void) {
  // Install the nav dispatcher into the touch service system slot only when the twin gate passes:
  // system nav for participating (system) apps, the master pref alone for explicitly opted-in
  // apps. The gate/decision lives in touch_nav.c (unit-testable via s_app_twin_ops).
  touch_nav_app_twin_subscribe(&s_app_twin_ops, s_app_state_ptr->touch_nav_participating,
                               s_app_state_ptr->touch_nav_opted_in);
}

void app_touch_nav_unsubscribe(void) {
  prv_app_twin_remove_handler(NULL);
}

void app_touch_nav_resync(void) {
  // Re-evaluate the gate for the running app after a pref flip: keeps an opted-in app subscribed
  // when only the Touch Navigation sub-pref turned off, removes it when the master pref did.
  touch_nav_app_twin_resync(&s_app_twin_ops, s_app_state_ptr->touch_nav_participating,
                            s_app_state_ptr->touch_nav_opted_in);
}

void app_touch_nav_set_participating(bool enable) {
  // Reconcile the subscription with the new opt-in state: subscribe on opt-in (a no-op while the
  // master pref is off, until the pref turns on and the app resubscribes), unsubscribe on opt-out.
  // Idempotent when nothing changes.
  touch_nav_app_twin_reconcile(&s_app_twin_ops, &s_app_state_ptr->touch_nav_participating,
                               &s_app_state_ptr->touch_nav_opted_in, enable);
}
#endif

JsRuntimeContext *app_state_get_js_runtime_context(void) {
  return s_app_state_ptr->js_runtime_context;
}

uint8_t *app_state_get_js_runtime_context_buffer(void) {
  return s_app_state_ptr->js_runtime_context_buffer;
}

void app_state_set_js_runtime_context(uint8_t *unaligned_buffer,
                                         JsRuntimeContext *js_runtime_context) {
  s_app_state_ptr->js_runtime_context_buffer = unaligned_buffer;
  s_app_state_ptr->js_runtime_context = js_runtime_context;
}

JsMemoryAPIContext *app_state_get_js_memory_api_context(void) {
  return s_app_state_ptr->js_memory_api_context;
}

void app_state_set_js_memory_api_context(JsMemoryAPIContext *context) {
  s_app_state_ptr->js_memory_api_context = context;
}

ApplibInternalEventsInfo *app_state_get_applib_internal_events_info(void) {
  return &s_app_state_ptr->applib_internal_events_info;
}

TextRenderState *app_state_get_text_render_state(void) {
  return &s_app_state_ptr->text_render_state;
}

bool app_state_get_text_perimeter_debugging_enabled(void) {
  return s_app_state_ptr->text_perimeter_debugging_enabled;
}

void app_state_set_text_perimeter_debugging_enabled(bool enabled) {
  s_app_state_ptr->text_perimeter_debugging_enabled = enabled;
}

TimelineItemActionSource app_state_get_current_timeline_item_action_source(void) {
  return s_app_state_ptr->current_timeline_item_action_source;
}

void app_state_set_current_timeline_item_action_source(TimelineItemActionSource current_source) {
  s_app_state_ptr->current_timeline_item_action_source = current_source;
}

// Serial Commands
///////////////////////////////////////////////////////////
#ifdef CONFIG_MALLOC_INSTRUMENTATION
void command_dump_malloc_app(void) {
  heap_dump_malloc_instrumentation_to_dbgserial(app_state_get_heap());
}
#endif
