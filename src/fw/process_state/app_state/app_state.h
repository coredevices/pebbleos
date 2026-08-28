/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/accel_service_private.h"
#include "applib/app_focus_service.h"
#include "applib/app_inbox.h"
#include "applib/app_message/app_message_internal.h"
#include "applib/app_wakeup.h"
#include "applib/backlight_service.h"
#include "applib/backlight_service_private.h"
#include "applib/battery_state_service.h"
#include "applib/battery_state_service_private.h"
#include "applib/bluetooth/ble_app_support.h"
#include "applib/compass_service_private.h"
#include "applib/connection_service.h"
#include "applib/connection_service_private.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/text_render.h"
#include "applib/health_service.h"
#include "applib/health_service_private.h"
#include "applib/pbl_std/locale.h"
#include "applib/plugin_service_private.h"
#include "applib/tick_timer_service.h"
#include "applib/tick_timer_service_private.h"
#include "applib/touch_service_private.h"
#include "applib/ui/animation_private.h"
#include "applib/ui/click_internal.h"
#include "applib/ui/content_indicator_private.h"
#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/speaker.h"
#include "applib/ui/window_stack_private.h"
#include "applib/unobstructed_area_service_private.h"
#include "logging/logging_private.h"
#include "pbl/services/app_glances/app_glance_service.h"
#include "pbl/services/timeline/timeline_actions.h"
#include "pbl/util/heap.h"
#include "pbl/util/list.h"

struct _reent;

typedef struct ApplibInternalEventsInfo {
  EventServiceInfo will_focus_event;
  EventServiceInfo button_down_event;
  EventServiceInfo button_up_event;
  EventServiceInfo legacy2_status_bar_change_event;
  int minute_of_last_legacy2_statusbar_change;
} ApplibInternalEventsInfo;

typedef struct AppFocusState {
  AppFocusHandlers handlers;
  EventServiceInfo will_focus_info;
  EventServiceInfo did_focus_info;
} AppFocusState;

typedef struct TextRenderState {
  SpecialCodepointHandlerCb special_codepoint_handler_cb;
  void *special_codepoint_handler_context;
} TextRenderState;


typedef struct AppStateInitParams {
  ProcessAppSDKType sdk_type;
  int16_t obstruction_origin_y;
} AppStateInitParams;


typedef struct MemorySegment MemorySegment;

typedef struct JsRuntimeContext JsRuntimeContext;
typedef struct JsMemoryAPIContext JsMemoryAPIContext;

//! Allocate memory in the process' address space for AppState data and
//! perform initial configuration.
bool app_state_configure(MemorySegment *app_state_ram,
                         ProcessAppSDKType sdk_type,
                         int16_t obstruction_origin_y);

//! Finish initializing AppState from within the running app task.
void app_state_init(void);

//! Clean up after ourselves nicely. Note that this may not be called if the app crashes.
void app_state_deinit(void);

Heap* app_state_get_heap(void);

AppInbox **app_state_get_app_message_inbox(void);

EventServiceInfo *app_state_get_app_outbox_subscription_info(void);

ApplibInternalEventsInfo *app_state_get_applib_internal_events_info(void);

AnimationState* app_state_get_animation_state(void);

AppMessageCtx *app_state_get_app_message_ctx(void);

BLEAppState* app_state_get_ble_app_state(void);

ClickManager* app_state_get_click_manager(void);

WindowStack* app_state_get_window_stack(void);

FrameBuffer* app_state_get_framebuffer(void);

GContext* app_state_get_graphics_context(void);

EventServiceInfo* app_state_get_event_service_state(void);

AccelServiceState* app_state_get_accel_state(void);

CompassServiceConfig **app_state_get_compass_config(void);

PluginServiceState *app_state_get_plugin_service(void);

LogState *app_state_get_log_state(void);

BatteryStateServiceState *app_state_get_battery_state_service_state(void);

BacklightServiceState *app_state_get_backlight_service_state(void);

TickTimerServiceState *app_state_get_tick_timer_service_state(void);

TouchServiceState *app_state_get_touch_service_state(void);

ConnectionServiceState *app_state_get_connection_service_state(void);

LocaleInfo *app_state_get_locale_info(void);

ContentIndicatorsBuffer *app_state_get_content_indicators_buffer(void);

HealthServiceState *app_state_get_health_service_state(void);

RecognizerList *app_state_get_recognizer_list(void);

struct RecognizerManager *app_state_get_recognizer_manager(void);

struct TouchNavState *app_state_get_touch_nav_state(void);

//! Subscribe the app task's touch-service system slot to the nav dispatcher (no-op unless the
//! master nav pref is on). Runs on the app task.
void app_touch_nav_subscribe(void);

//! Unsubscribe the app task's nav dispatcher and cancel any in-flight gesture. Runs on the app task.
void app_touch_nav_unsubscribe(void);

//! Re-evaluate the app twin's gate for the running app after a pref flip and install or remove the
//! nav dispatcher accordingly (keeps an opted-in app subscribed when only the Touch Navigation
//! sub-pref turned off).
void app_touch_nav_resync(void);

//! Privileged setter behind the app_touch_navigation_enable() SDK call (invoked via the
//! sys_app_touch_navigation_enable syscall). Sets this app's touch-nav participation and reconciles
//! the app twin's subscription with the master pref (subscribe when enabling with the pref on,
//! unsubscribe when disabling). Idempotent. Runs on the app task.
void app_touch_nav_set_participating(bool enable);

JsRuntimeContext *app_state_get_js_runtime_context(void);

uint8_t *app_state_get_js_runtime_context_buffer(void);

void app_state_set_js_runtime_context(uint8_t *unaligned_buffer,
                                         JsRuntimeContext *js_runtime_context);

JsMemoryAPIContext *app_state_get_js_memory_api_context(void);

void app_state_set_js_memory_api_context(JsMemoryAPIContext *context);

bool *app_state_get_framebuffer_render_pending();

void app_state_set_user_data(void *data);
void* app_state_get_user_data(void);

AppFocusState *app_state_get_app_focus_state(void);

UnobstructedAreaState *app_state_get_unobstructed_area_state(void);

AppGlance *app_state_get_glance(void);

struct Layer;
typedef struct Layer Layer;

Layer** app_state_get_layer_tree_stack(void);

WakeupHandler app_state_get_wakeup_handler(void);
void app_state_set_wakeup_handler(WakeupHandler handler);

EventServiceInfo *app_state_get_wakeup_event_info(void);

SpeakerFinishedCallback app_state_get_speaker_finish_handler(void);
void app_state_set_speaker_finish_handler(SpeakerFinishedCallback handler);
void *app_state_get_speaker_finish_ctx(void);
void app_state_set_speaker_finish_ctx(void *ctx);
EventServiceInfo *app_state_get_speaker_finish_event_info(void);

//! Retrieve a preallocated full screen 2bit framebuffer for use with 2.x apps that want to use the
//! capture_frame_buffer API. Note this memory is only valid when used with 2.x apps.
GBitmap* app_state_legacy2_get_2bit_framebuffer(void);

struct tm *app_state_get_gmtime_tm(void);
struct tm *app_state_get_localtime_tm(void);
char *app_state_get_localtime_zone(void);

void *app_state_get_rand_ptr(void);

TextRenderState *app_state_get_text_render_state(void);

bool app_state_get_text_perimeter_debugging_enabled(void);
void app_state_set_text_perimeter_debugging_enabled(bool enabled);

TimelineItemActionSource app_state_get_current_timeline_item_action_source(void);
void app_state_set_current_timeline_item_action_source(TimelineItemActionSource current_source);

bool app_state_get_action_bar_follows_display_orientation(void);
void app_state_set_action_bar_follows_display_orientation(bool follow);
