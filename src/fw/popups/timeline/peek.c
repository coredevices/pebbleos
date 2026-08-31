/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "peek_private.h"

#include "applib/ui/property_animation.h"
#include "process_management/app_manager.h"
#include "applib/ui/window_stack.h"
#include "applib/unobstructed_area_service.h"
#include "apps/system/timeline/common.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/modals/modal_manager.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>
#include "pbl/util/size.h"

#include <pebbleos/cron.h>

#define TIMELINE_PEEK_FRAME_HIDDEN GRect(0, DISP_ROWS, DISP_COLS, TIMELINE_PEEK_HEIGHT)
#define TIMELINE_PEEK_OUTER_BORDER_WIDTH PBL_IF_RECT_ELSE(2, 1)
#define TIMELINE_PEEK_MULTI_BORDER_WIDTH (1)
#define TIMELINE_PEEK_MULTI_CONTENT_HEIGHT PBL_IF_RECT_ELSE(2, 1)
#define TIMELINE_PEEK_MAX_CONCURRENT (2)

static TimelinePeek s_peek;

static unsigned int prv_get_concurrent_height(unsigned int num_concurrent) {
  // Height of the border and other concurrent contents
  return (TIMELINE_PEEK_OUTER_BORDER_WIDTH +
          (num_concurrent * (TIMELINE_PEEK_MULTI_BORDER_WIDTH +
                             TIMELINE_PEEK_MULTI_CONTENT_HEIGHT)));
}

unsigned int timeline_peek_get_concurrent_height(unsigned int num_concurrent) {
  return prv_get_concurrent_height(MIN(num_concurrent, TIMELINE_PEEK_MAX_CONCURRENT));
}

static void prv_draw_background(GContext *ctx, const GRect *frame_orig,
                                unsigned int num_concurrent) {
  GRect frame = *frame_orig;
#if PBL_RECT
  // Fill all the way to the bottom of the screen
  frame.size.h = DISP_ROWS - frame.origin.y;
#endif
  const GColor background_color = GColorWhite;
  graphics_context_set_fill_color(ctx, background_color);
  graphics_fill_rect(ctx, &frame);

  // Draw the icon background
  frame.origin.x += DISP_COLS - TIMELINE_PEEK_ICON_BOX_WIDTH;
  frame.size.w = TIMELINE_PEEK_ICON_BOX_WIDTH;
  graphics_context_set_fill_color(ctx, TIMELINE_FUTURE_COLOR);
  graphics_fill_rect(ctx, &frame);

  // Draw the top border and concurrent event indicators
  frame = *frame_orig;
  const GColor border_color = GColorBlack;
  for (unsigned int i = 0; i <= num_concurrent; i++) {
    const bool has_content = (i < num_concurrent);
    for (unsigned int type = 0; type < (has_content ? 2 : 1); type++) {
      const bool is_outer = (i == 0);
      const bool is_border = (type == 0);
      const int height = (is_outer && is_border) ? TIMELINE_PEEK_OUTER_BORDER_WIDTH :
                                       is_border ? TIMELINE_PEEK_MULTI_BORDER_WIDTH :
                                                   TIMELINE_PEEK_MULTI_CONTENT_HEIGHT;
      frame.size.h = height;
      graphics_context_set_fill_color(ctx, is_border ? border_color : background_color);
      graphics_fill_rect(ctx, &frame);
      frame.origin.y += height;
    }
  }

#if PBL_ROUND
  // Draw the bottom border
  frame = *frame_orig;
  frame.origin.y += frame.size.h - TIMELINE_PEEK_OUTER_BORDER_WIDTH;
  frame.size.h = TIMELINE_PEEK_OUTER_BORDER_WIDTH;
  graphics_context_set_fill_color(ctx, border_color);
  graphics_fill_rect(ctx, &frame);
#endif
}

void timeline_peek_draw_background(GContext *ctx, const GRect *frame,
                                   unsigned int num_concurrent) {
  prv_draw_background(ctx, frame, num_concurrent);
}

static void prv_timeline_peek_update_proc(Layer *layer, GContext *ctx) {
  TimelinePeek *peek = (TimelinePeek *)layer;
  const unsigned int num_concurrent = peek->peek_layout ?
      MIN(peek->peek_layout->info.num_concurrent, TIMELINE_PEEK_MAX_CONCURRENT) : 0;
  if (peek->removing_concurrent && (num_concurrent > 0)) {
    prv_draw_background(ctx, &TIMELINE_PEEK_FRAME_VISIBLE, num_concurrent - 1);
  }
  prv_draw_background(ctx, &peek->layout_layer.frame, num_concurrent);
}

static void prv_redraw(void *PBL_UNUSED data) {
  TimelinePeek *peek = &s_peek;
  layer_mark_dirty(&peek->layout_layer);
}

static void prv_cron_callback(CronJob *job, void *PBL_UNUSED data) {
  launcher_task_add_callback(prv_redraw, NULL);
  cron_job_schedule(job);
}

static CronJob s_timeline_peek_job = {
  .minute = CRON_MINUTE_ANY,
  .hour = CRON_HOUR_ANY,
  .mday = CRON_MDAY_ANY,
  .month = CRON_MONTH_ANY,
  .cb = prv_cron_callback,
};

static void prv_destroy_layout(void) {
  TimelinePeek *peek = &s_peek;
  if (!peek->peek_layout) {
    return;
  }

  layout_destroy(&peek->peek_layout->timeline_layout->layout_layer);
  timeline_item_destroy(peek->peek_layout->item);
  task_free(peek->peek_layout);
  peek->peek_layout = NULL;
}

static PeekLayout *prv_create_layout(TimelineItem *item, unsigned int num_concurrent) {
  PeekLayout *layout = task_zalloc_check(sizeof(PeekLayout));
  item = timeline_item_copy(item);
  layout->item = item;
  timeline_layout_init_info(&layout->info, item, time_util_get_midnight_of(rtc_get_time()));
  layout->info.num_concurrent = num_concurrent;
  const LayoutLayerConfig config = {
    .frame = &GRect(0, 0, DISP_COLS, TIMELINE_PEEK_HEIGHT),
    .attributes = &item->attr_list,
    .mode = LayoutLayerModePeek,
    .app_id = &item->header.parent_id,
    .context = &layout->info,
  };
  layout->timeline_layout = (TimelineLayout *)layout_create(item->header.layout, &config);
  return layout;
}

static void prv_set_layout(PeekLayout *layout) {
  prv_destroy_layout();
  TimelinePeek *peek = &s_peek;
  peek->peek_layout = layout;
  layer_add_child(&peek->layout_layer, &peek->peek_layout->timeline_layout->layout_layer.layer);
}

static void prv_unschedule_animation(TimelinePeek *peek) {
  animation_unschedule(peek->animation);
  peek->animation = NULL;
}

//! Returns true if we're running an upscaled legacy app in scaling mode.
static bool prv_is_upscaled_app(void) {
  GSize app_framebuffer_size;
  app_manager_get_framebuffer_size(&app_framebuffer_size);
  if (app_framebuffer_size.h == DISP_ROWS) {
    return false;  // Native app, not upscaled
  }
#if defined(CONFIG_APP_SCALING) && !defined(CONFIG_RECOVERY_FW)
  // On platforms that support scaling mode, check the preference
  return shell_prefs_get_legacy_app_render_mode() >= LegacyAppRenderMode_ScalingNearest;
#else
  return false;  // Other platforms use bezel mode only
#endif
}

static bool prv_should_use_unobstructed_area(void) {
  GSize app_framebuffer_size;
  app_manager_get_framebuffer_size(&app_framebuffer_size);
  // For upscaled apps in scaling mode, the peek always covers part of the scaled content,
  // so we need to notify the app.
  // For bezel mode or native apps, check if the gap is less than peek height.
  if (prv_is_upscaled_app()) {
    return true;
  }
  return (DISP_ROWS - app_framebuffer_size.h) < TIMELINE_PEEK_HEIGHT;
}

//! Scales a display Y coordinate to framebuffer coordinates for upscaled apps.
//! For native apps or bezel mode, returns the coordinate unchanged.
static int16_t prv_scale_y_to_framebuffer(int16_t display_y) {
  if (!prv_is_upscaled_app()) {
    return display_y;
  }
  GSize app_framebuffer_size;
  app_manager_get_framebuffer_size(&app_framebuffer_size);
  // Scale from display coordinates to framebuffer coordinates
  return (display_y * app_framebuffer_size.h) / DISP_ROWS;
}

static void prv_peek_frame_setup(Animation *animation) {
  PropertyAnimation *prop_anim = (PropertyAnimation *)animation;
  TimelinePeek *peek;
  property_animation_subject(prop_anim, (void *)&peek, false /* set */);
  GRect from_frame;
  property_animation_get_from_grect(prop_anim, &from_frame);
  GRect to_frame;
  property_animation_get_to_grect(prop_anim, &to_frame);
  if (prv_should_use_unobstructed_area()) {
    unobstructed_area_service_will_change(prv_scale_y_to_framebuffer(from_frame.origin.y),
                                          prv_scale_y_to_framebuffer(to_frame.origin.y));
  }
}

static void prv_peek_frame_update(Animation *animation, AnimationProgress progress) {
  PropertyAnimation *prop_anim = (PropertyAnimation *)animation;
  property_animation_update_grect(prop_anim, progress);
  TimelinePeek *peek;
  property_animation_subject(prop_anim, (void *)&peek, false /* set */);
  GRect to_frame;
  property_animation_get_to_grect(prop_anim, &to_frame);
  if (prv_should_use_unobstructed_area()) {
    unobstructed_area_service_change(
        prv_scale_y_to_framebuffer(peek->layout_layer.frame.origin.y),
        prv_scale_y_to_framebuffer(to_frame.origin.y),
        progress);
  }
}

static void prv_peek_frame_teardown(Animation *animation) {
  PropertyAnimation *prop_anim = (PropertyAnimation *)animation;
  GRect to_frame;
  property_animation_get_to_grect(prop_anim, &to_frame);
  if (prv_should_use_unobstructed_area()) {
    unobstructed_area_service_did_change(prv_scale_y_to_framebuffer(to_frame.origin.y));
  }
}

static GRect prv_peek_frame_getter(void *subject) {
  TimelinePeek *peek = subject;
  GRect frame;
  layer_get_frame(&peek->layout_layer, &frame);
  return frame;
}

static void prv_peek_frame_setter(void *subject, GRect frame) {
  TimelinePeek *peek = subject;
  layer_set_frame(&peek->layout_layer, &frame);
}

static const PropertyAnimationImplementation s_peek_prop_impl = {
  .base = {
    .setup = prv_peek_frame_setup,
    .update = prv_peek_frame_update,
    .teardown = prv_peek_frame_teardown,
  },
  .accessors = {
    .getter.grect = prv_peek_frame_getter,
    .setter.grect = prv_peek_frame_setter,
  },
};

static void prv_peek_anim_stopped(Animation *animation, bool finished, void *context) {
  TimelinePeek *peek = &s_peek;
  if (context) {
    // Replace the previous item with the next item
    PeekLayout *layout = context;
    prv_set_layout(layout);
    // Reset the frame
    layer_set_frame(&peek->layout_layer, &TIMELINE_PEEK_FRAME_VISIBLE);
  } else if (!peek->visible) {
    // If the peek was becoming hidden, destroy the timeline layout
    prv_destroy_layout();
  }
  peek->removing_concurrent = false;
}

static const AnimationHandlers s_peek_anim_handlers = {
  .stopped = prv_peek_anim_stopped,
};

static void prv_transition_frame(TimelinePeek *peek, bool visible, bool animated) {
  prv_unschedule_animation(peek);

  const bool last_visible = peek->visible;
  peek->visible = visible;
  GRect to_frame = visible ? TIMELINE_PEEK_FRAME_VISIBLE : TIMELINE_PEEK_FRAME_HIDDEN;
  if ((last_visible == visible) && grect_equal(&peek->layout_layer.frame, &to_frame)) {
    // No change
    return;
  }

  if (!animated) {
    layer_set_frame(&peek->layout_layer, &to_frame);
    return;
  }

  PropertyAnimation *prop_anim = property_animation_create(&s_peek_prop_impl, peek, NULL, NULL);
  property_animation_set_from_grect(prop_anim, &peek->layout_layer.frame);
  property_animation_set_to_grect(prop_anim, &to_frame);
  Animation *animation = property_animation_get_animation(prop_anim);
  animation_set_duration(animation, interpolate_moook_duration());
  animation_set_custom_interpolation(animation, interpolate_moook);
  animation_set_handlers(animation, s_peek_anim_handlers, NULL);

  peek->animation = animation;
  animation_schedule(animation);
}

#define EXTENDED_BOUNCE_BACK (2 * INTERPOLATE_MOOOK_BOUNCE_BACK)

static const int32_t s_extended_moook_out[] =
    {EXTENDED_BOUNCE_BACK, INTERPOLATE_MOOOK_BOUNCE_BACK, 2, 1, 0};
static const MoookConfig s_extended_moook_out_config = {
  .frames_out = s_extended_moook_out,
  .num_frames_out = ARRAY_LENGTH(s_extended_moook_out),
  .no_bounce_back = true,
};

static int64_t prv_interpolate_extended_moook_out(AnimationProgress progress, int64_t from,
                                                  int64_t to) {
  return interpolate_moook_custom(progress, from, to, &s_extended_moook_out_config);
}

static Animation *prv_create_transition_adding_concurrent(
    TimelinePeek *peek, PeekLayout *layout) {
  const int height_shrink = 20;
  GRect frame_normal = TIMELINE_PEEK_FRAME_VISIBLE;
  GRect frame_shrink = grect_inset(frame_normal, GEdgeInsets(0, 0, height_shrink, 0));
  // Starting with shrink instead of ending with it will flash white
  PropertyAnimation *white_prop_anim = property_animation_create_layer_frame(
      &peek->layout_layer, &frame_shrink, &frame_normal);
  Animation *white_animation = property_animation_get_animation(white_prop_anim);
  animation_set_duration(white_animation, ANIMATION_TARGET_FRAME_INTERVAL_MS);
  animation_set_handlers(white_animation, s_peek_anim_handlers, layout);

  GRect frame_bounce =
      grect_inset(frame_normal, GEdgeInsets(-EXTENDED_BOUNCE_BACK, 0, 0, 0));
  PropertyAnimation *bounce_prop_anim = property_animation_create_layer_frame(
      &peek->layout_layer, &frame_bounce, &frame_normal);
  Animation *bounce_animation = property_animation_get_animation(bounce_prop_anim);
  animation_set_duration(bounce_animation,
                         interpolate_moook_custom_duration(&s_extended_moook_out_config));
  animation_set_custom_interpolation(bounce_animation, prv_interpolate_extended_moook_out);
  return animation_sequence_create(white_animation, bounce_animation, NULL);
}

static const int32_t s_custom_moook_in[] = {0, 1, INTERPOLATE_MOOOK_BOUNCE_BACK};
static const MoookConfig s_custom_moook_in_config = {
  .frames_in = s_custom_moook_in,
  .num_frames_in = ARRAY_LENGTH(s_custom_moook_in),
};

static int64_t prv_interpolate_custom_moook_in(AnimationProgress progress, int64_t from,
                                               int64_t to) {
  return interpolate_moook_custom(progress, from, to, &s_custom_moook_in_config);
}

static int64_t prv_interpolate_moook_out(AnimationProgress progress, int64_t from, int64_t to) {
  return interpolate_moook_out(progress, from, to, 0 /* num_frames_from */,
                               false /* bounce_back */);
}

static Animation *prv_create_transition_removing_concurrent(
    TimelinePeek *peek, PeekLayout *layout) {
  PropertyAnimation *remove_prop_anim = property_animation_create_layer_frame(
      &peek->layout_layer, &TIMELINE_PEEK_FRAME_VISIBLE, &TIMELINE_PEEK_FRAME_HIDDEN);
  Animation *remove_animation = property_animation_get_animation(remove_prop_anim);
  // Cut out the last frame
  animation_set_duration(remove_animation,
                         interpolate_moook_custom_duration(&s_custom_moook_in_config));
  animation_set_custom_interpolation(remove_animation, prv_interpolate_custom_moook_in);
  animation_set_handlers(remove_animation, s_peek_anim_handlers, layout);

  GRect bounds_normal = { .size = TIMELINE_PEEK_FRAME_VISIBLE.size };
  GRect bounds_bounce = { .origin.y = TIMELINE_PEEK_HEIGHT, .size = bounds_normal.size };
  PropertyAnimation *bounce_prop_anim = property_animation_create_layer_bounds(
      &peek->layout_layer, &bounds_bounce, &bounds_normal);
  Animation *bounce_animation = property_animation_get_animation(bounce_prop_anim);
  animation_set_duration(bounce_animation, interpolate_moook_out_duration());
  animation_set_custom_interpolation(bounce_animation, prv_interpolate_moook_out);
  return animation_sequence_create(remove_animation, bounce_animation, NULL);
}

static void prv_transition_concurrent(TimelinePeek *peek, PeekLayout *layout) {
  const unsigned int old_num_concurrent = peek->peek_layout->info.num_concurrent;
  const unsigned int new_num_concurrent = layout->info.num_concurrent;
  if (uuid_equal(&peek->peek_layout->item->header.id, &layout->item->header.id) &&
      (old_num_concurrent == new_num_concurrent)) {
    // Either nothing changed or the item content changed, just set the layout
    prv_set_layout(layout);
    return;
  }

  prv_unschedule_animation(peek);

  Animation *animation = NULL;
  if (peek->peek_layout && (old_num_concurrent < new_num_concurrent)) {
    animation = prv_create_transition_adding_concurrent(peek, layout);
  } else {
    animation = prv_create_transition_removing_concurrent(peek, layout);
    peek->removing_concurrent = true;
  }

  peek->animation = animation;
  animation_schedule(animation);
}

static void prv_push_timeline_peek(void *unused) {
  timeline_peek_push();
}

void timeline_peek_init(void) {
  TimelinePeek *peek = &s_peek;
  *peek = (TimelinePeek) {
#ifndef CONFIG_SHELL_SDK
    .enabled = timeline_peek_prefs_get_enabled(),
#endif
  };
  window_init(&peek->window, WINDOW_NAME("Timeline Peek"));
  window_set_focusable(&peek->window, false);
  window_set_transparent(&peek->window, true);
  layer_set_update_proc(&peek->window.layer, prv_timeline_peek_update_proc);
  layer_init(&peek->layout_layer, &TIMELINE_PEEK_FRAME_HIDDEN);
  layer_add_child(&peek->window.layer, &peek->layout_layer);

  timeline_peek_set_show_before_time(timeline_peek_prefs_get_before_time() * SECONDS_PER_MINUTE);

  // Wait one event loop to show the timeline peek
  launcher_task_add_callback(prv_push_timeline_peek, NULL);
}

static void prv_set_visible(bool visible, bool animated) {
  TimelinePeek *peek = &s_peek;
  if (!peek->started && visible) {
    cron_job_schedule(&s_timeline_peek_job);
  } else {
    cron_job_unschedule(&s_timeline_peek_job);
  }
  prv_transition_frame(peek, visible, animated);
}

static bool prv_can_animate(void) {
  return app_manager_is_watchface_running();
}

void timeline_peek_set_visible(bool visible, bool animated) {
  TimelinePeek *peek = &s_peek;
#ifndef CONFIG_SHELL_SDK
  if (!peek->exists) {
    visible = false;
  }
#endif
  prv_set_visible((app_manager_is_watchface_running() && peek->enabled && visible),
                  (prv_can_animate() && animated));
}

void timeline_peek_set_item(TimelineItem *item, bool started, unsigned int num_concurrent,
                            bool first, bool animated) {
  TimelinePeek *peek = &s_peek;
  animated = (prv_can_animate() && animated);
  if (!animated) {
    // We are not animating and thus don't need to retain the layout
    prv_destroy_layout();
  }

  peek->exists = (item != NULL);
  peek->started = started;
  peek->first = first;
  timeline_peek_set_visible(peek->exists, animated);

  PeekLayout *layout = item ? prv_create_layout(item, num_concurrent) : NULL;
  if (animated && !peek->animation && peek->visible) {
    // Swap the layout in an animation
    prv_transition_concurrent(peek, layout);
  } else if (layout) {
    // Immediately set the new layout
    prv_set_layout(layout);
  }
}

void timeline_peek_dismiss(void) {
  TimelinePeek *peek = &s_peek;
  if (!peek->peek_layout) {
    return;
  }
  TimelineItem *item = peek->peek_layout->item;
  const status_t rv = pin_db_set_status_bits(&item->header.id, TimelineItemStatusDismissed);
  if (rv == S_SUCCESS) {
    timeline_event_refresh();
  } else {
    char uuid_buffer[UUID_STRING_BUFFER_LENGTH];
    uuid_to_string(&item->header.id, uuid_buffer);
    PBL_LOG_WRN("Failed to dismiss Timeline Peek event %s (status: %"PRIi32")",
            uuid_buffer, rv);
  }
}

int16_t timeline_peek_get_origin_y(void) {
  TimelinePeek *peek = &s_peek;
  return peek->layout_layer.frame.origin.y;
}

int16_t timeline_peek_get_obstruction_origin_y(void) {
  if (prv_should_use_unobstructed_area()) {
    return prv_scale_y_to_framebuffer(timeline_peek_get_origin_y());
  }
  // Not using unobstructed area - return framebuffer height to indicate no obstruction
  GSize app_framebuffer_size;
  app_manager_get_framebuffer_size(&app_framebuffer_size);
  return app_framebuffer_size.h;
}

void timeline_peek_get_item_id(TimelineItemId *item_id_out) {
  TimelinePeek *peek = &s_peek;
  *item_id_out = (peek->enabled && peek->visible && peek->exists && peek->peek_layout)
      ? peek->peek_layout->item->header.id : UUID_INVALID;
}

bool timeline_peek_is_first_event(void) {
  TimelinePeek *peek = &s_peek;
  return peek->first;
}

bool timeline_peek_is_future_empty(void) {
  TimelinePeek *peek = &s_peek;
  return peek->future_empty;
}

void timeline_peek_push(void) {
  TimelinePeek *peek = &s_peek;
  modal_window_push(&peek->window, ModalPriorityDiscreet, true);
}

void timeline_peek_pop(void) {
  TimelinePeek *peek = &s_peek;
  window_stack_remove(&peek->window, true);
}

void timeline_peek_set_enabled(bool enabled) {
  TimelinePeek *peek = &s_peek;
  peek->enabled = enabled;
  timeline_peek_set_visible(enabled, true /* animated */);
}

void timeline_peek_handle_peek_event(PebbleTimelinePeekEvent *event) {
  TimelinePeek *peek = &s_peek;
  peek->future_empty = event->is_future_empty;
  bool show = false;
  bool started = false;
  if (event->item_id != NULL) {
    switch (event->time_type) {
      case TimelinePeekTimeType_None:
      case TimelinePeekTimeType_SomeTimeNext:
      case TimelinePeekTimeType_WillEnd:
        break;
      case TimelinePeekTimeType_ShowWillStart:
        show = true;
        break;
      case TimelinePeekTimeType_ShowStarted:
        show = true;
        started = true;
        break;
    }
  }
  TimelineItem item = {};
  if (show) {
    const status_t rv = pin_db_get(event->item_id, &item);
    // We failed to read the pin since it may have been deleted immediately. We will probably
    // momentarily recover from another peek event resulting from the delete.
    show = (rv == S_SUCCESS);
  }
  if (show) {
    timeline_peek_set_item(&item, started, event->num_concurrent,
                           event->is_first_event, true /* animated */);
  } else {
    timeline_peek_set_item(NULL, false /* started */, 0 /* num_concurrent */,
                           false /* is_first_event */, true /* animated */);
  }
  timeline_item_free_allocated_buffer(&item);
}

void timeline_peek_handle_process_start(void) {
  timeline_peek_set_visible(true, false /* animated */);
}

void timeline_peek_handle_process_kill(void) {
  timeline_peek_set_visible(false, false /* animated */);
}

#if UNITTEST
TimelinePeek *timeline_peek_get_peek(void) {
  return &s_peek;
}
#endif
