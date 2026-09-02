/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "animation_private.h"

#include "animation_timing.h"
#include "property_animation_private.h"

#include "applib/legacy2/ui/animation_legacy2.h"
#include "applib/legacy2/ui/animation_private_legacy2.h"

#include "applib/app_logging.h"
#include "applib/applib_malloc.auto.h"

#include "process_state/app_state/app_state.h"

#include "kernel/kernel_applib_state.h"
#include "kernel/memory_layout.h"

#include "pbl/services/animation_service.h"

#include "system/passert.h"
#include "pbl/util/math.h"

#include <string.h>

KERNEL_READONLY_DATA static bool s_paused = false;

static void prv_run(AnimationState *state, uint32_t now, AnimationPrivate *top_level_animation,
                    uint32_t top_level_start_time, bool do_update);
#define PebbleTask_Current PebbleTask_Unknown

// -------------------------------------------------------------------------------------------
static AnimationState *prv_animation_state_get(PebbleTask task) {
  if (task == PebbleTask_Current) {
    task = pebble_task_get_current();
  }
  if (task == PebbleTask_App) {
    return app_state_get_animation_state();
  } else if (task == PebbleTask_KernelMain) {
    return kernel_applib_get_animation_state();
  } else {
    WTF;
  }
}

// -------------------------------------------------------------------------------------------
T_STATIC AnimationPrivate *prv_animation_get_current(void) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Unknown);
  return state->aux->current_animation;
}

InterpolateInt64Function animation_private_current_interpolate_override(void) {
  AnimationPrivate *animation = prv_animation_get_current();
  if (animation && animation->curve == AnimationCurveCustomInterpolationFunction) {
    return animation->custom_interpolation_function;
  }
  return NULL;
}

// ------------------------------------------------------------------------------------
static bool prv_handle_list_filter(ListNode* node, void* data) {
  AnimationPrivate* animation = (AnimationPrivate *)node;
  return animation->handle == data;
}


// ------------------------------------------------------------------------------------
// Find annotation by handle. If quiet is true, don't print out a log error message if we detect
// an invalid handle. Quiet mode is used by animation_unschedule and animation_is_scheduled.
static AnimationPrivate* prv_find_animation_by_handle(AnimationState *state, Animation *handle,
                                                      bool quiet) {
  if (!handle) {
    return NULL;
  }

  // Default to state for the current task
  if (!state) {
    state = prv_animation_state_get(PebbleTask_Current);
  }

  // Look for this animation by id. It could either be in the unscheduled or scheduled list
  ListNode* node = list_find(state->unscheduled_head, prv_handle_list_filter, (void*)handle);
  if (!node) {
    node = list_find(state->scheduled_head, prv_handle_list_filter, (void*)handle);
  }
  if (!node) {
    if (!quiet) {
      APP_LOG(APP_LOG_LEVEL_ERROR, "Animation %d does not exist", (int)handle);
    }
    return NULL;
  }
  return (AnimationPrivate *)node;
}


// -------------------------------------------------------------------------------------------
// Find animation by parent and child idx
typedef struct {
  AnimationPrivate *parent;
  uint8_t child_idx;
} ParentChildInfo;

static bool prv_parent_list_filter(ListNode* node, void* data) {
  AnimationPrivate* animation = (AnimationPrivate *)node;
  ParentChildInfo *info = (ParentChildInfo *)data;
  return animation->parent == info->parent && animation->child_idx == info->child_idx;
}

static AnimationPrivate* prv_find_animation_by_parent_child_idx(AnimationState *state,
            AnimationPrivate *parent, int child_idx) {
  if (!parent) {
    return NULL;
  }

  // Default to state for the current task
  if (!state) {
    state = prv_animation_state_get(PebbleTask_Current);
  }

  // Look for this animation by id. It could either be in the unscheduled or scheduled list
  ParentChildInfo info = (ParentChildInfo) {
    .parent = parent,
    .child_idx = child_idx
  };
  ListNode* node = list_find(state->unscheduled_head, prv_parent_list_filter, &info);
  if (!node) {
    node = list_find(state->scheduled_head, prv_parent_list_filter, &info);
  }
  if (!node) {
    return NULL;
  }
  return (AnimationPrivate *)node;
}

// -------------------------------------------------------------------------------------------
// Remove from being iterated after unscheduling. This must be called on any animation being
// unscheduled.
static void prv_iter_remove(AnimationState *state, AnimationPrivate *animation) {
  // If this animation is the iterator's next, bump the iterator
  if (state->aux->iter_next == (ListNode *)animation) {
    state->aux->iter_next = list_get_next((ListNode *)animation);
  }
}

// -------------------------------------------------------------------------------------------
// Remove from our list of allocated animations and free the memory
static void prv_unlink_and_free(AnimationState *state, AnimationPrivate *animation) {
  // It's an error if it's scheduled
  PBL_ASSERTN(list_contains(state->unscheduled_head, &animation->list_node));
  list_remove(&animation->list_node, &state->unscheduled_head /* &head */, NULL /* &tail */);

  ANIMATION_LOG_DEBUG("destroying %d (%p) ", (int)animation->handle, animation);
  applib_free(animation);
}


// -------------------------------------------------------------------------------------------
static int prv_scheduler_comparator(void *a, void *b) {
  AnimationPrivate *animation_a = (AnimationPrivate *)a;
  AnimationPrivate *animation_b = (AnimationPrivate *)b;
  return serial_distance32(animation_a->abs_start_time_ms, animation_b->abs_start_time_ms);
}


// -------------------------------------------------------------------------------------------
inline static uint32_t prv_get_ms_since_system_start(void) {
  return ((sys_get_ticks() * 1000 + RTC_TICKS_HZ / 2) / RTC_TICKS_HZ);
}


// -------------------------------------------------------------------------------------------
// Get the total duration of an animation, optionally considering the delay and play count.
// This recurses into children of sequence or spawn animations
static uint32_t prv_get_total_duration(AnimationState *state, AnimationPrivate *animation,
                                       bool include_delay, bool include_play_count) {
  uint32_t child_duration;
  uint32_t duration = 0;
  int child_idx;

  if (include_delay) {
    duration += animation->delay_ms;
  }

  if (animation->type == AnimationTypeSequence) {
    // For a sequence animation, add duration of each of the components
    for (child_idx = 0; child_idx < ANIMATION_MAX_CHILDREN; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, animation, child_idx);
      if (!child) {
        break;
      }
      child_duration = prv_get_total_duration(state, child, true, true);
      if (child_duration == PLAY_DURATION_INFINITE) {
        return PLAY_DURATION_INFINITE;
      }
      duration += child_duration;
    }


  } else if (animation->type == AnimationTypeSpawn) {
    // For a spawn animation, get the max of each component
    uint32_t  max_child_duration = 0;
    for (child_idx = 0; child_idx < ANIMATION_MAX_CHILDREN; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, animation, child_idx);
      if (!child) {
        break;
      }
      child_duration = prv_get_total_duration(state, child, true, true);
      if (child_duration == PLAY_DURATION_INFINITE) {
        return PLAY_DURATION_INFINITE;
      }
      max_child_duration = MAX(max_child_duration, child_duration);
    }
    duration += max_child_duration;

  } else {
    PBL_ASSERTN(animation->type == AnimationTypePrimitive);
    duration += animation->duration_ms;
  }

  if (include_play_count) {
    // Factor in the play count of this animation now
    if (animation->play_count == ANIMATION_PLAY_COUNT_INFINITE_STORED) {
      duration = PLAY_DURATION_INFINITE;
    } else {
      duration *= animation->play_count;
    }
  }

  return duration;
}


// -------------------------------------------------------------------------------------------
// Return true if animation is a descendent of the given parent
static bool prv_is_descendent_of(AnimationState *state, AnimationPrivate *animation,
                                 AnimationPrivate *parent) {
  // Follow the parents up
  while (animation) {
    // If no parent at all, can't be
    if (!animation->parent) {
      return false;
    }

    if (animation->parent == parent) {
      // Direct descendent
      return true;
    }

    // Get parent's parent
    animation = animation->parent;
  }
  return false;
}


// -------------------------------------------------------------------------------------------
static int32_t prv_get_elapsed(AnimationPrivate *animation, uint32_t now) {
  // Compute the absolute start time of this animation, backing it up by the
  // delay and any repeats we have already done
  uint32_t start_ms = animation->abs_start_time_ms;
  start_ms -= animation->times_played * (animation->duration_ms + animation->delay_ms);
  return serial_distance32(start_ms, now);
}


// -------------------------------------------------------------------------------------------
// Adjust the abs_start_time of this animation and all of its children. This is called during
// a set_elapsed operation.
static void prv_backup_start_time(AnimationState *state, AnimationPrivate *parent,
                                  const uint32_t delta) {
  AnimationPrivate *next;
  if (delta == 0) {
    return;
  }

  AnimationPrivate *animation = (AnimationPrivate *) state->scheduled_head;
  while (animation) {
    // Since we are reducing the start times, each of the animations we operate on will be
    // moved earlier in the list. Get the next pointer now before we possibly move it.
    next = (AnimationPrivate*) list_get_next(&animation->list_node);

    // Note that we have to iterate through all scheduled nodes and see if each is a descendent.
    // We can't follow the children of parent_h by searching using an incrementing child_idx
    // because one or more of the children may have already run and destroyed themselves.
    if (animation == parent || prv_is_descendent_of(state, animation, parent)) {
      animation->abs_start_time_ms -= delta;

      // Put back into sorted order
      list_remove(&animation->list_node, &state->scheduled_head, NULL /* &tail */);
      state->scheduled_head = list_sorted_add(state->scheduled_head, &animation->list_node,
                                              prv_scheduler_comparator, true /*ascending*/);
    }
    animation = next;
  }
}


// -------------------------------------------------------------------------------------------
static void prv_reschedule_timer(AnimationState *state, uint32_t rate_control_delay_ms) {
  AnimationPrivate *animation = (AnimationPrivate *)state->scheduled_head;
  if (animation == NULL) {
    return;
  }
  const uint32_t now = prv_get_ms_since_system_start();
  const int32_t delta_ms = serial_distance32(now, animation->abs_start_time_ms);
  const uint32_t interval_ms = MAX(delta_ms, 0) + rate_control_delay_ms;

  // The animation service will call animation_private_timer_callback() when the timer fires
  animation_service_timer_schedule(interval_ms);
}


// -------------------------------------------------------------------------------------------
bool prv_animation_is_scheduled(AnimationState* state, AnimationPrivate *animation) {
  return (animation->abs_start_time_ms != 0);
}

// -------------------------------------------------------------------------------------------
// Return true if animation is mutable
static bool prv_is_mutable(AnimationState *state, AnimationPrivate *animation) {
  return (animation && !animation->immutable && !animation->parent &&
          !prv_animation_is_scheduled(state, animation));
}


// -------------------------------------------------------------------------------------------
// Determine if any of an animation's descendents are scheduled
static bool prv_animation_children_scheduled(AnimationState *state, AnimationPrivate *animation) {
  if (animation->type != AnimationTypePrimitive) {
    // For a complex animation, check each component
    for (int child_idx = 0; child_idx < ANIMATION_MAX_CHILDREN; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, animation, child_idx);
      if (!child) {
        break;
      }
      if (child->type == AnimationTypePrimitive) {
        if (prv_animation_is_scheduled(state, child)) {
          return true;
        }
      } else if (prv_animation_is_scheduled(state, child)
                 || prv_animation_children_scheduled(state, child)) {
        return true;
      }
    }
  }
  return false;
}

// -------------------------------------------------------------------------------------------
// Unschedule of an animation and optional destroy, recurses into children of sequence or spawn
// animations. When this method is called on children of an animation, allow_auto_destroy is
// false unless the top-level animation has already been unscheduled.
void prv_unschedule_animation(AnimationState *state, AnimationPrivate *animation,
    const bool finished, bool allow_auto_destroy, bool force_destroy, bool teardown) {
  if (animation->type != AnimationTypePrimitive) {
    // For a complex animation, unschedule each of the components
    for (int child_idx = 0; child_idx < ANIMATION_MAX_CHILDREN; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, animation, child_idx);
      if (!child) {
        break;
      }
      prv_unschedule_animation(state, child, finished, allow_auto_destroy, force_destroy, teardown);
    }
  }

  if (!prv_animation_is_scheduled(state, animation)) {
    // When we unschedule a top-level animation, we call prv_unschedule_animation() on each of
    // the children, which gives us a chance to destroy them. Children are not allowed to destroy
    // themselves.
    if (animation->calling_end_handlers) {
      // We don't want to tear down an animation that is executing stopped handlers.
      animation->defer_delete = true;
    } else {
      if (teardown && animation->did_setup) {
        // When children unschedule themselves after running, their teardown isn't allowed to run
        // (because the parent might repeat). So, this is a chance to finally run the child's
        // teardown handler.
        if (animation->implementation->teardown != NULL) {
          animation->implementation->teardown(animation->handle);
        }
        animation->did_setup = false;
      }
      if (force_destroy || (allow_auto_destroy && animation->auto_destroy)) {
        prv_unlink_and_free(state, animation);
      }
    }
    return;
  }

  // Unschedule the passed in animation
  ANIMATION_LOG_DEBUG("unscheduling %d (%p)", (int)animation->handle, animation);
  PBL_ASSERTN(animation->implementation != NULL);

  const bool was_old_head = (&animation->list_node == state->scheduled_head);

  // Remove from being iterated
  prv_iter_remove(state, animation);

  // Move from the scheduled to the unscheduled list
  PBL_ASSERTN(list_contains(state->scheduled_head, &animation->list_node));
  list_remove(&animation->list_node, &state->scheduled_head, NULL);
  state->unscheduled_head = list_insert_before(state->unscheduled_head, &animation->list_node);

  // Reschedule the timer if we're removing the head animation:
  if (was_old_head && state->scheduled_head != NULL) {
    prv_reschedule_timer(state, 0);
  }

  // Reset these fields, before calling .stopped(), so that this animation
  // instance can be rescheduled again in the .stopped() handler, if needed.
  animation->abs_start_time_ms = 0;
  animation->is_completed = false;
  animation->times_played = 0;
  bool did_start = animation->started;
  animation->started = false;
  if (force_destroy) {
    // Setting this flag prevents the stopped handler from being able to reschedule it again
    animation->being_destroyed = true;
  }

  // Call the stopped and teardown handlers
  animation->calling_end_handlers = true;
  if (animation->handlers.stopped && did_start) {
    animation->handlers.stopped(animation->handle, finished, animation->context);
  }
  if (teardown && animation->did_setup) {
    if (animation->implementation->teardown != NULL) {
      animation->implementation->teardown(animation->handle);
    }
    animation->did_setup = false;
  }
  animation->calling_end_handlers = false;

#ifdef UNITTEST
  // Make sure this animation didn't get deleted as a side effect of running the stopped handler
  PBL_ASSERTN(list_contains(state->unscheduled_head, &animation->list_node)
              || list_contains(state->scheduled_head, &animation->list_node));
#endif

  if (force_destroy || animation->defer_delete
      || ((allow_auto_destroy && animation->auto_destroy)
           && !prv_animation_is_scheduled(state, animation))) {
    // It's possible the stopped handler rescheduled, so check before we destroy it
    prv_unlink_and_free(state, animation);
  }
}


// -------------------------------------------------------------------------------------------
// Low level schedule of an animation, no recursion
static void prv_schedule_low_level_animation(AnimationState* state, const uint32_t now,
                                             AnimationPrivate *animation, int32_t add_delay_ms) {
  animation->abs_start_time_ms = now + animation->delay_ms + add_delay_ms;
  if (animation->abs_start_time_ms == 0) {
    // 0 means not scheduled
    animation->abs_start_time_ms = 1;
  }
  if (!animation->did_setup) {
    if (animation->implementation->setup != NULL) {
      animation->implementation->setup(animation->handle);
    }
    animation->did_setup = true;
  }

  const bool old_head_is_animating = state->scheduled_head
              ? (((AnimationPrivate *)state->scheduled_head)->abs_start_time_ms <= now)
              : false;

  // Move from the unscheduled to the scheduled list
  PBL_ASSERTN(list_contains(state->unscheduled_head, &animation->list_node));
  list_remove(&animation->list_node, &state->unscheduled_head /* &head */, NULL /* &tail */);
  const bool ascending = true;
  state->scheduled_head = list_sorted_add(state->scheduled_head, &animation->list_node,
                                              prv_scheduler_comparator, ascending);

  const bool has_new_head = (&animation->list_node == state->scheduled_head);
  if (has_new_head) {
    // Only reschedule the timer if the previous head animation wasn't running yet:
    if (old_head_is_animating == false) {
      prv_reschedule_timer(state, 0);
    }
  }

  ANIMATION_LOG_DEBUG("scheduled %d (%p) to run at (%d). delay:%d, duration:%d",
        (int)animation->handle, animation, (int)animation->abs_start_time_ms,
        (int)(animation->delay_ms), (int)(animation->duration_ms));
}


// -------------------------------------------------------------------------------------------
// High level schedule of an animation, recurses into children of sequence or spawn animations
static bool prv_schedule_animation(AnimationState* state, const uint32_t now,
                                   AnimationPrivate *animation, int32_t add_delay_ms) {
  bool success = true;
  int child_idx;
  PBL_ASSERTN(animation != NULL);

  if (animation->play_count == 0) {
    // Play count of 0, no need to schedule it.
    return true;
  }

  // Don't allow an animation to be rescheduled (like from the stopped handler) if it is
  // being destroyed
  if (animation->being_destroyed) {
    return false;
  }

  ANIMATION_LOG_DEBUG("scheduling %d (%p) to run in %d ms (%d)", (int)animation->handle, animation,
        (int)(animation->delay_ms + add_delay_ms), (int)(now + animation->delay_ms + add_delay_ms));

  uint32_t earliest_start_time = now;

  if (animation->type == AnimationTypeSequence) {
    // For a sequence animation, schedule each of the components with increasing delays
    int32_t delay = animation->delay_ms + add_delay_ms;

    // Figure out and store our total duration (used by the scheduler to tell when it's done)
    animation->duration_ms = prv_get_total_duration(state, animation, false /*delay*/,
                                                    false /*play_count*/);

    for (child_idx = 0; child_idx < ANIMATION_MAX_CHILDREN; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, animation, child_idx);
      if (!child) {
        break;
      }
      uint32_t duration = prv_get_total_duration(state, child, true /*delay*/, true /*play_count*/);

      // It is allowed that the first child may have already been scheduled and played a bit. If
      // this is the case, backup the start time by reducing delay accordingly.
      if (child_idx == 0 && child->abs_start_time_ms) {
        // Remove the sequence's delay, we will shift all of the delay into the first child
        animation->delay_ms = 0;
        int32_t child_position_inc_delay = prv_get_elapsed(child, now) + child->delay_ms;
        earliest_start_time = now - child_position_inc_delay;
        delay = serial_distance32(now, earliest_start_time + duration);
      } else {
        success = prv_schedule_animation(state, now, child, delay);
        if (!success) {
          break;
        }
        delay += duration;
      }
      if (duration == PLAY_DURATION_INFINITE) {
        break;
      }
    }

  } else if (animation->type == AnimationTypeSpawn) {
    // For a spawn animation, schedule each of the components in parallel

    // If any of the children have already been scheduled, then we need to back up our start time
    // of the spawn accordingly and adjust the delay_ms field of every child such that
    // prv_get_total_duration() reflects the overall duration of the spawn correctly.
    uint32_t latest_end_time = now;
    for (child_idx = 0; true; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, animation, child_idx);
      if (!child) {
        break;
      }
      uint32_t child_duration = prv_get_total_duration(state, child, true /*delay*/,
                                                       true /*play_count*/);
      uint32_t child_end_time;
      if (child->abs_start_time_ms) {
        // Already scheduled
        int32_t child_position_inc_delay = prv_get_elapsed(child, now)  + child->delay_ms;
        uint32_t child_start_time = now - child_position_inc_delay;
        if (serial_distance32(child_start_time, earliest_start_time) > 0) {
          // computes (earliest_start_time - child->abs_start_time_ms)
          earliest_start_time = child_start_time;
        }
        child_end_time = child_start_time + child_duration;
      } else {
        child_end_time = now + animation->delay_ms + add_delay_ms + child_duration;
      }
      if (serial_distance32(child_end_time, latest_end_time) < 0) {
        // computes (latest_end_time - child_end_time)
        latest_end_time = child_end_time;
      }
    }

    // Schedule the children that have not been scheduled yet. If any have already been scheduled,
    // adjust the delays of all children to make it look the same as if the spawn had been
    // scheduled in the past with no children scheduled yet.
    int32_t delay = animation->delay_ms + add_delay_ms;
    for (child_idx = 0; success; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, animation, child_idx);
      if (!child) {
        // No more children
        break;
      }

      if (now != earliest_start_time) {
        // We need to adjust the delays of each child since one or more were already scheduled
        uint32_t child_start;
        if (child->abs_start_time_ms) {
          child_start = child->abs_start_time_ms - child->delay_ms;
        } else {
          success = prv_schedule_animation(state, now, child, delay);
          child_start = now + delay + child->delay_ms;
        }
        child->delay_ms = serial_distance32(earliest_start_time, child_start);
      } else {
        if (!child->abs_start_time_ms) {
          success = prv_schedule_animation(state, now, child, delay);
        }
      }
    }

    // Set the duration now, after we've possibly adjusted the children delays to compensate
    // for already scheduled children.
    animation->duration_ms = prv_get_total_duration(state, animation, false /*delay*/,
                                                    false /*play_count*/);

  } else {
    PBL_ASSERTN(animation->type == AnimationTypePrimitive);
    PBL_ASSERTN(animation->implementation->update != NULL);
  }

  if (now != earliest_start_time) {
    // This is a complex animation that has a child that was already scheduled. We must pretend that
    // the top-level animation started at earliest_start_time. add_delay_ms may end up being
    // negative here if the child already started.
    animation->delay_ms = 0;
    add_delay_ms = serial_distance32(now, earliest_start_time);
  }

  // Schedule the parent node
  prv_schedule_low_level_animation(state, now, animation, add_delay_ms);

  return success;
}

static AnimationProgress prv_get_distance_normalized(const AnimationPrivate *animation,
                                                     AnimationProgress time_normalized_raw) {
  AnimationProgress time_normalized;
  if (animation->reverse) {
    time_normalized = ANIMATION_NORMALIZED_MAX - time_normalized_raw;
  } else {
    time_normalized = time_normalized_raw;
  }
  AnimationProgress distance_normalized;
  if (animation->curve >= AnimationCurveCustomFunction) {
    if (animation->curve == AnimationCurveCustomFunction && animation->custom_curve_function) {
      distance_normalized = animation->custom_curve_function(time_normalized);
    } else {
      // Just use the unchanged time if curve is AnimationCurveCustomInterpolation or there is no
      // custom curve function assigned.
      distance_normalized = time_normalized;
    }
  } else {
    distance_normalized = animation_timing_curve(time_normalized, animation->curve);
  }
  return distance_normalized;
}

// -------------------------------------------------------------------------------------------
void animation_private_update(AnimationState *state, AnimationPrivate *animation,
                              AnimationProgress progress_raw) {
  PBL_ASSERTN(animation);

  if (state == NULL) {
    state = prv_animation_state_get(PebbleTask_Current);
  }

  const AnimationProgress distance_normalized = prv_get_distance_normalized(animation,
                                                                            progress_raw);

  state->aux->current_animation = animation;
  animation->implementation->update(animation->handle, distance_normalized);
  state->aux->current_animation = NULL;
}

static uint32_t prv_get_time_normalized_raw(const AnimationPrivate *animation, uint32_t now) {
  const int32_t rel_ms_running = serial_distance32(animation->abs_start_time_ms, now);

  // The caller should already have checked that this animation is active
  PBL_ASSERTN(rel_ms_running >= 0);

  uint32_t time_normalized_raw;
  if (animation->duration_ms == ANIMATION_DURATION_INFINITE) {
    time_normalized_raw = ANIMATION_NORMALIZED_MIN;
  } else if (animation->duration_ms == 0) {
    time_normalized_raw = ANIMATION_NORMALIZED_MAX;
  } else {
    // animation->duration_ms/2 added in for round to nearest
    time_normalized_raw = (ANIMATION_NORMALIZED_MAX * rel_ms_running + animation->duration_ms/2)
                           / animation->duration_ms;
    time_normalized_raw = MIN(time_normalized_raw, ANIMATION_NORMALIZED_MAX);
  }
  return time_normalized_raw;
}

AnimationProgress animation_private_get_animation_progress(const AnimationPrivate *animation) {
  // FIXME PBL-25497: Make this function less fragile
  // Calling prv_get_ms_since_system_start() here means this function will have different return
  // values if it's called multiple times, all of which will be different from the value actually
  // passed to the animations .update() function
  const uint32_t now = prv_get_ms_since_system_start();
  const uint32_t time_normalized_raw = prv_get_time_normalized_raw(animation, now);
  return prv_get_distance_normalized(animation, time_normalized_raw);
}

bool animation_get_progress(Animation *animation_h, AnimationProgress *progress_out) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h,
                                                             false /* quiet */);
  if (!animation || !prv_animation_is_scheduled(state, animation) || !progress_out) {
    return false;
  }

  *progress_out = animation_private_get_animation_progress(animation);
  return true;
}

// -------------------------------------------------------------------------------------------
// Execute the callbacks (update (optional), started, stopped) for a given animation at the given
// timestamp. Returns true if this is a parent ready to unschedule itself but can't because
// the children have not finished yet.
static bool prv_run_animation(AnimationState *state, AnimationPrivate *animation,
                              const uint32_t now, bool do_update) {
  bool blocked_on_children_complete = false;

  // Play count of 0 should have never been scheduled.
  PBL_ASSERTN(animation->play_count != 0);

  // If this is the animation's first frame, call the 'started' handler:
  if (animation->handlers.started && !animation->started) {
    animation->handlers.started(animation->handle, animation->context);
  }
  animation->started = true;

  uint32_t time_normalized_raw = prv_get_time_normalized_raw(animation, now);
  bool completed = (time_normalized_raw == ANIMATION_NORMALIZED_MAX);

  // Call the update procedure?
  if (do_update || (completed && !animation->is_completed)) {
    animation_private_update(state, animation, time_normalized_raw);
  }


  // If completed, either reschedule it now if it needs to be repeated or unschedule it (which
  // results in a call to the stopped handler)
  if (completed && !animation->is_completed) {
    animation->is_completed = true;
    animation->times_played++;

    if (animation->times_played < animation->play_count) {
      // We need to repeat it. The prv_unschedule_animation() method zeros out times_played, so we
      // need to restore it again after scheduling.
      uint16_t times_played = animation->times_played;

      // Schedule this at duration past the previous start time
      uint32_t new_start_time = animation->abs_start_time_ms + animation->duration_ms;

      prv_unschedule_animation(state, animation, true /*finished */, false /*allow_destroy*/,
                               false /*force_destroy*/, false /*teardown*/);
      prv_schedule_animation(state, new_start_time, animation, 0 /*add_delay*/);
      animation->times_played = times_played;
    }
  }

  if (animation->is_completed) {
    // We're done with this animation, we can unschedule it if all of its children have
    // also been unscheduled. If the children have not completed yet, we keep it scheduled but
    // with the is_completed flag set so that we can check it again next interval.
    if (!prv_animation_children_scheduled(state, animation)) {
      // Once all our children have completed, we can safely unschedule ourselves
      prv_unschedule_animation(state, animation, animation->is_completed,
                               !animation->parent /*allow_destroy*/, false /*force_destroy*/,
                               !animation->parent /*teardown*/);
    } else {
      blocked_on_children_complete = true;
    }
  }
  return blocked_on_children_complete;
}


// -------------------------------------------------------------------------------------------
// @param state our context
// @param now the time we are running to. When called from animation_set_elapsed, this will
//      be in the future, otherwise it will be the current time
// @param top_level_animation only used by animation_set_elapsed, this is the top-level parent
//      that we are setting the elapsed of.
// @param top_level_start_time when the top_level animation started playing (used for debug
//      logging only)
static void prv_run(AnimationState *state, uint32_t now, AnimationPrivate *top_level_animation,
                    uint32_t top_level_start_time, bool do_update) {
  for (int i = 0; i < 2; i++) {
    bool have_blocked_parents = false;
    // We run through the animations up to 2 times. If during the first run we detect that some
    // parents want to unschedule but couldn't because they still have children running, then we
    // run again so that the parents can check again if their children finished on the first run.
    AnimationPrivate *animation = (AnimationPrivate *) state->scheduled_head;
    while (animation) {
  #ifdef UNITTEST
      // This is to ensure unit test fails in case of bad behaviour - no need to execute in
      // in normal FW since this case should not exist with defer_delete check
      AnimationPrivate *animation_p = animation_private_animation_find(animation->handle);
      PBL_ASSERTN(animation_p != NULL);
      // Make sure this is an animation in the scheduled list
      PBL_ASSERTN(list_contains(state->scheduled_head, &animation->list_node));
  #endif

      const int32_t rel_ms_running = serial_distance32(animation->abs_start_time_ms, now);
      if (rel_ms_running < 0) {
        // Animations are ordered by abs_start_time_ms.
        // We've reached an animation that should not start yet, so
        // everything after and including this animation shouldn't run yet.
        break;
      }

      // Get a pointer to next now, because after possible unscheduling, this animation may change
      // into a node of the unscheduled list or become freed
      state->aux->iter_next = list_get_next(&animation->list_node);

      // If only running from a specific top-level animation, see if this animation is the target
      // one or one of it's children, and if so advance it
      if (!top_level_animation || animation == top_level_animation
          || prv_is_descendent_of(state, animation, top_level_animation)) {
        ANIMATION_LOG_DEBUG("advancing animation %d to %"PRIu32" ms", (int)animation->handle,
                              now - top_level_start_time);
        // Run this animation. Record if this is a parent ready to unschedule itself but
        // still waiting for one of its children.
        have_blocked_parents |= prv_run_animation(state, animation, now, do_update);
      }

      // Next one
      animation = (AnimationPrivate *)state->aux->iter_next;
    }

    // If no blocked parents, we can exit right away
    if (!have_blocked_parents) {
      break;
    }
  }

  // We are done iterating
  state->aux->iter_next = NULL;
}


// -------------------------------------------------------------------------------------------
typedef Animation *(*CreateFromArrayFunc)(Animation **animation_array,
                      uint32_t array_len);

static Animation *prv_call_using_vargs(CreateFromArrayFunc func, Animation *animation_a,
                  Animation *animation_b, Animation *animation_c, va_list args) {
  const int max_args = ANIMATION_MAX_CREATE_VARGS;
  typedef Animation *AnimationPtr;
  AnimationPtr animation_array[max_args];
  int array_len = 2;

  // A and B must not be NULL
  if (!animation_a || !animation_b) {
    return false;
  }
  animation_array[0] = animation_a;
  animation_array[1] = animation_b;

  if (animation_c) {
    // If c is not NULL, we need to figure out the array length
    animation_array[array_len++] = animation_c;
    while (array_len < max_args) {
      void *arg = va_arg(args, void *);
      if (arg == NULL) {
        break;
      }
      animation_array[array_len++] = arg;
    }
  }

  // Create from an array
  return func(animation_array, array_len);
}


// -------------------------------------------------------------------------------------------
// Complex animations don't perform any logic in their update callback
static void prv_complex_animation_update(Animation * animation, uint32_t distance) {
}
static const AnimationImplementation s_complex_implementation = {
  .update = (AnimationUpdateImplementation) prv_complex_animation_update,
};

// -------------------------------------------------------------------------------------------
static Animation *prv_complex_init(Animation *parent_h, Animation **animation_array,
                                   uint32_t array_len, AnimationType type) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (array_len > ANIMATION_MAX_CHILDREN) {
    // Exceed max # of children allowed?
    return NULL;
  }

  bool success = true;

  AnimationPrivate *parent = animation_private_animation_find(parent_h);
  parent->type = type;

  // Keep track of which children we added so we can restore them in case of error
  bool used_children[array_len];
  memset(used_children, 0, sizeof(bool) * array_len);

  // Set the parent on each of the components
  uint32_t child_idx = 0;
  for (uint32_t i = 0; i < array_len; i++) {
    AnimationPrivate *component = prv_find_animation_by_handle(state, animation_array[i],
                                                               false /*quiet*/);
    if (!component) {
      // It is OK to pass in already destroyed children.
      continue;
    }

    // The 2nd and subsequent children of a sequence must NOT be already scheduled. Also fail if
    // child already has a parent
    if (component->parent
        || ((type == AnimationTypeSequence) && (i > 0) && (component->abs_start_time_ms))) {
      success = false;
      break;
    }
    component->parent = parent;
    component->child_idx = child_idx++;
    used_children[i] = true;
  }

  if (!success) {
    for (uint32_t i = 0; i < array_len; i++) {
      if (!used_children[i]) {
        continue;
      }
      // Undo setting of the parent and child_idx on the components we modified.
      AnimationPrivate *component = prv_find_animation_by_handle(state, animation_array[i],
                                                                 false /*quiet*/);
      if (component) {
        component->parent = NULL;
        component->child_idx = 0;
      }
    }
    prv_unlink_and_free(state, parent);
    parent_h = NULL;
  }

  return parent_h;
}


// -------------------------------------------------------------------------------------------
static Animation *prv_complex_create(Animation **animation_array, uint32_t array_len,
                                     AnimationType type) {
  if (array_len > ANIMATION_MAX_CHILDREN) {
    // Exceed max # of children allowed?
    return NULL;
  }

  AnimationPrivate *parent = applib_type_malloc(AnimationPrivate);
  if (!parent) {
    return NULL;
  }
  Animation *parent_h = animation_private_animation_init(parent);
  parent->implementation = &s_complex_implementation;

  return prv_complex_init(parent_h, animation_array, array_len,
                          type);
}

// -------------------------------------------------------------------------------------------
static Animation *prv_animation_clone(AnimationState *state, AnimationPrivate *from) {
  Animation *clone_h;
  AnimationPrivate *clone = NULL;
  bool success = true;

  // If this is a complex animation, create the children
  if (from->type != AnimationTypePrimitive) {
    // Count the children
    int num_children;
    for (num_children = 0; num_children < ANIMATION_MAX_CHILDREN; num_children++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, from, num_children);
      if (!child) {
        break;
      }
    }

    // Allocate array to hold the children and allocate each of them.
    Animation *children[num_children];
    for (int child_idx = 0; child_idx < num_children; child_idx++) {
      AnimationPrivate *child = prv_find_animation_by_parent_child_idx(state, from, child_idx);
      children[child_idx] = prv_animation_clone(state, child);

      // Bail if we couldn't create the child
      if (children[child_idx] == NULL) {
        num_children = child_idx;
        success = false;
        break;
      }
    }

    // Allocate the complex animation parent
    if (success) {
      clone_h = prv_complex_create(children, num_children, from->type);
      clone = prv_find_animation_by_handle(state, clone_h, false /*quiet*/);
    }
    if (!clone) {
      for (int i = 0; i < num_children; i++) {
        animation_destroy(children[i]);
      }
      return NULL;
    }

  } else {
    if (from->is_property_animation) {
      PropertyAnimationPrivate *prop = property_animation_private_clone(
                                                                (PropertyAnimationPrivate *)from);
      if (prop) {
        clone = &prop->animation;
        clone->is_property_animation = true;
      }
    } else {
      clone = applib_type_malloc(AnimationPrivate);
    }
    if (!clone) {
      return NULL;
    }
    clone_h = animation_private_animation_init(clone);
  }

  // Copy the values into the clone
  clone->implementation = from->implementation;
  clone->handlers = from->handlers;
  clone->context = from->context;
  clone->delay_ms = from->delay_ms;
  clone->duration_ms = from->duration_ms;
  clone->play_count = from->play_count;
  clone->curve = from->curve;
  clone->auto_destroy = from->auto_destroy;
  clone->reverse = from->reverse;
  clone->custom_curve_function = from->custom_curve_function;

  return clone_h;
}


// -------------------------------------------------------------------------------------------
void animation_private_state_init(AnimationState *state) {
#ifndef UNITTEST
  _Static_assert(sizeof(AnimationState) <= sizeof(AnimationLegacy2Scheduler),
        "Animation state larger than allowed for 2.0 compatibility");
#endif

  // If this a legacy 2.0 application, instantiate the 2.0 legacy animation support
#ifndef CONFIG_RECOVERY_FW
  if (process_manager_compiled_with_legacy2_sdk()) {
    animation_legacy2_private_init_scheduler((AnimationLegacy2Scheduler *)state);
    return;
  }
#endif

  // Allocate the auxiliary information
  AnimationAuxState *aux_state = applib_type_malloc(AnimationAuxState);
  PBL_ASSERTN(aux_state);
  *aux_state = (AnimationAuxState) {
    // To aid for debugging, let's start each task off at a different handle offset. Eventually they
    // will collide but it is not required that each task have globally unique handles
    .next_handle = pebble_task_get_current() * 100000000,
    .last_delay_ms = ANIMATION_RENDER_FRAME_INTERVAL_MS,
    .last_frame_time_ms = prv_get_ms_since_system_start()
  };

  *state = (AnimationState) {
    .signature = ANIMATION_STATE_3_X_SIGNATURE,
    .aux = aux_state,
  };
}

// -------------------------------------------------------------------------------------------
void animation_private_state_deinit(AnimationState *state) {

  if (!process_manager_compiled_with_legacy2_sdk()) {
    applib_free(state->aux);
  }
}


// -------------------------------------------------------------------------------------------
// Return true if the animation globals were instantiated using the legacy 2.x animation
// manager
bool animation_private_using_legacy_2(AnimationState *state) {
  if (state == NULL) {
    state = prv_animation_state_get(PebbleTask_Current);
  }
  return (state->signature != ANIMATION_STATE_3_X_SIGNATURE);
}


// -------------------------------------------------------------------------------------------
// Return the animation pointer for the given handle
AnimationPrivate *animation_private_animation_find(Animation *handle) {
  return prv_find_animation_by_handle(NULL, handle, false /*quiet*/);
}


// -------------------------------------------------------------------------------------------
Animation *animation_private_animation_init(AnimationPrivate *animation) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);

  *animation = (AnimationPrivate) {
    .handle = (Animation *)(uintptr_t)(++state->aux->next_handle),
    .duration_ms = ANIMATION_DEFAULT_DURATION_MS,
    .play_count = 1,
    .curve = AnimationCurveDefault,
    .auto_destroy = true
  };
  PBL_ASSERTN(animation->handle);

  state->unscheduled_head = list_insert_before(state->unscheduled_head, &animation->list_node);
  ANIMATION_LOG_DEBUG("creating %d (%p)", (int)animation->handle, animation);
  return (Animation *)(animation->handle);
}


// -------------------------------------------------------------------------------------------
void animation_private_timer_callback(void *context) {
  AnimationState *state = (AnimationState *)context;
  const uint32_t now = prv_get_ms_since_system_start();

  // Tell the timer that we received the event it sent
  animation_service_timer_event_received();

  if(!s_paused){
    // Run all animations for this time interval
    prv_run(state, now, NULL /*top-level animation*/, 0/*top-level start time*/, true/*do_update*/);
  }

  // Frame rate control:
  const int32_t frame_interval_ms = serial_distance32(state->aux->last_frame_time_ms, now);
  const int32_t error_ms = frame_interval_ms - ANIMATION_RENDER_FRAME_INTERVAL_MS;
  const int32_t theoretic_delay_ms = state->aux->last_delay_ms - error_ms;
  const uint32_t delay_ms = CLIP(theoretic_delay_ms, (int32_t) 0,
                                (int32_t) ANIMATION_RENDER_FRAME_INTERVAL_MS);

  prv_reschedule_timer(state, delay_ms);
  state->aux->last_delay_ms = delay_ms;
  state->aux->last_frame_time_ms = now;
}


// -------------------------------------------------------------------------------------------
Animation *animation_create(void) {
  if (animation_private_using_legacy_2(NULL)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return (Animation *)animation_legacy2_create();
  }
  AnimationPrivate *animation = applib_type_malloc(AnimationPrivate);
  if (!animation) {
    return NULL;
  }

  return animation_private_animation_init(animation);
}


// -------------------------------------------------------------------------------------------
bool animation_destroy(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_destroy((AnimationLegacy2 *)animation_h);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation || animation->parent) {
    // Only top-level animations can be destroyed
    return false;
  }

  // If we're being called from the stopped or teardown handler, set the defer_delete flag.
  // This will inform us to delete the animation once we return back to the animation code
  // from the handler.
  if (animation->calling_end_handlers) {
    animation->defer_delete = true;
    return true;
  }

  // Set this flag so that no one can reschedule it while we're trying to destroy it
  //  (like it's stopped handler)
  animation->being_destroyed = true;

  // Unschedule and destroy it
  prv_unschedule_animation(state, animation, false /*finished*/, false /*allow_auto_destroy*/,
                           true /*force_destroy*/, true /*teardown*/);
  return true;
}


// -------------------------------------------------------------------------------------------
bool animation_set_auto_destroy(Animation *animation_h, bool auto_destroy) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    PBL_ASSERTN(!auto_destroy);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  animation->auto_destroy = auto_destroy;
  return true;
}


// -------------------------------------------------------------------------------------------
bool animation_schedule(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_schedule((AnimationLegacy2 *)animation_h);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation || animation->parent) {
    // Not allowed to schedule an animation that has a parent.
    return false;
  }

  // Unschedule if it's already scheduled, or if the play_count is 0 (in which case we allow it to
  // be auto-destroyed).
  if (animation->abs_start_time_ms || animation->play_count == 0) {
    const bool allow_auto_destroy = (animation->play_count == 0);
    prv_unschedule_animation(state, animation, false /*finished=false*/, allow_auto_destroy,
                             false /*force_destroy*/, true /*teardown*/);
  }

  // Schedule it
  bool success = prv_schedule_animation(state, prv_get_ms_since_system_start(), animation,
                                        0 /*add_delay*/);
  return success;
}


// -------------------------------------------------------------------------------------------
bool animation_unschedule(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_unschedule((AnimationLegacy2 *)animation_h);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, true /*quiet*/);
  if (!animation || animation->parent) {
    return false;
  }

  prv_unschedule_animation(state, animation, false /*finished=false*/, true /*allow_auto_destroy*/,
                           false /*force_destroy*/, true /*teardown*/);
  return true;
}


// -------------------------------------------------------------------------------------------
void animation_unschedule_all(void) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_unschedule_all();
    return;
  }

  while (state->scheduled_head) {
    AnimationPrivate *animation = (AnimationPrivate *)state->scheduled_head;

    // We can only unschedule top-level animations
    while (animation) {
      if (!animation->parent) {
        break;
      }
      animation = (AnimationPrivate *) list_get_next(&animation->list_node);
    }
    // There had to be at least 1 top-level animation
    PBL_ASSERTN(animation);
    prv_unschedule_animation(state, animation, false /*finished*/, true /*allow_auto_destroy*/,
                             false /*force_destroy*/, true /*teardown*/);
  }
}


// -------------------------------------------------------------------------------------------
bool animation_is_scheduled(Animation *animation_h) {
  if (!animation_h) {
    return false;
  }

  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return animation_legacy2_is_scheduled((AnimationLegacy2 *)animation_h);
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, true /*quiet*/);
  if (!animation) {
    return false;
  }

  return prv_animation_is_scheduled(state, animation);
}


// -------------------------------------------------------------------------------------------
bool animation_set_handlers(Animation *animation_h, AnimationHandlers handlers,
                            void *context) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    AnimationLegacy2Handlers *legacy_handlers = (AnimationLegacy2Handlers *)&handlers;
    animation_legacy2_set_handlers((AnimationLegacy2 *)animation_h, *legacy_handlers, context);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  animation->context = context;
  animation->handlers = handlers;
  return true;
}


// -------------------------------------------------------------------------------------------
AnimationHandlers animation_get_handlers(Animation *animation_h) {
  if (animation_private_using_legacy_2(NULL)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    AnimationHandlers *handlers = (AnimationHandlers *)&((AnimationLegacy2 *)animation_h)->handlers;
    return *handlers;
  }
  AnimationPrivate *animation = prv_find_animation_by_handle(NULL, animation_h, false /*quiet*/);
  if (!animation) {
    return (AnimationHandlers) {0};
  }

  return animation->handlers;
}


// -------------------------------------------------------------------------------------------
bool animation_set_implementation(Animation *animation_h,
                                  const AnimationImplementation *implementation) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_set_implementation((AnimationLegacy2 *)animation_h,
                                      (const AnimationLegacy2Implementation *)implementation);
    return true;
  }


  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  animation->implementation = implementation;
  return true;
}

// -------------------------------------------------------------------------------------------
const AnimationImplementation* animation_get_implementation(Animation *animation_h) {
  if (animation_private_using_legacy_2(NULL)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return (const AnimationImplementation*)((AnimationLegacy2 *)animation_h)->implementation;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(NULL, animation_h, false /*quiet*/);
  if (!animation) {
    return NULL;
  }
  return animation->implementation;
}

// -------------------------------------------------------------------------------------------
void *animation_get_context(Animation *animation_h) {
  if (animation_private_using_legacy_2(NULL)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return ((AnimationLegacy2 *)animation_h)->context;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(NULL, animation_h, false /*quiet*/);
  if (!animation) {
    return NULL;
  }
  return animation->context;
}

// -------------------------------------------------------------------------------------------
bool animation_set_delay(Animation *animation_h, uint32_t delay_ms) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_set_delay((AnimationLegacy2 *)animation_h, delay_ms);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  animation->delay_ms = delay_ms;
  return true;
}


// -------------------------------------------------------------------------------------------
uint32_t animation_get_delay(Animation *animation_h) {
  if (animation_private_using_legacy_2(NULL)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return ((AnimationLegacy2 *)animation_h)->delay_ms;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(NULL, animation_h, false /*quiet*/);
  if (!animation) {
    return 0;
  }

  return animation->delay_ms;
}

// -------------------------------------------------------------------------------------------
uint32_t animation_get_abs_start_time_ms(Animation *animation_h) {
  if (animation_private_using_legacy_2(NULL)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return ((AnimationLegacy2 *)animation_h)->abs_start_time_ms;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(NULL, animation_h, false /*quiet*/);
  if (!animation) {
    return 0;
  }

  return animation->abs_start_time_ms;
}

// -------------------------------------------------------------------------------------------
bool animation_set_duration(Animation *animation_h, uint32_t duration_ms) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_set_duration((AnimationLegacy2 *)animation_h, duration_ms);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)
      || animation->type != AnimationTypePrimitive) {
    return false;
  }

  animation->duration_ms = duration_ms;
  return true;
}


// -------------------------------------------------------------------------------------------
uint32_t animation_get_duration(Animation *animation_h, bool include_delay,
                                bool include_play_count) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return ((AnimationLegacy2 *)animation_h)->duration_ms;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation) {
    return 0;
  }

  return prv_get_total_duration(state, animation, include_delay, include_play_count);
}


// -------------------------------------------------------------------------------------------
bool animation_set_curve(Animation *animation_h, AnimationCurve curve) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_set_curve((AnimationLegacy2 *)animation_h, curve);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  PBL_ASSERTN(curve < AnimationCurveCustomFunction);
  animation->curve = curve;
  return true;
}

// -------------------------------------------------------------------------------------------
AnimationCurve animation_get_curve(Animation *animation_h) {
  if (animation_private_using_legacy_2(NULL)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return ((AnimationLegacy2 *)animation_h)->curve;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(NULL, animation_h, false /*quiet*/);
  if (!animation) {
    return AnimationCurveDefault;
  }

  return animation->curve;
}

// -------------------------------------------------------------------------------------------
static bool prv_animation_set_custom_function(Animation *animation_h, AnimationCurve curve,
                                              void *function) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    if (curve != AnimationCurveCustomFunction) {
      // 2.x doesn't support AnimationCurveCustomInterpolationFunction
      return false;
    }

    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    animation_legacy2_set_custom_curve((AnimationLegacy2 *)animation_h, function);
    return true;
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  animation->custom_function = function;
  if (function) {
    animation->curve = curve;
  } else {
    animation->curve = AnimationCurveDefault;
  }
  return true;
}

// -------------------------------------------------------------------------------------------
bool animation_set_custom_curve(Animation *animation_h,
                                AnimationCurveFunction curve_function) {
  return prv_animation_set_custom_function(animation_h,
                                           AnimationCurveCustomFunction,
                                           curve_function);
}

// -------------------------------------------------------------------------------------------
bool animation_set_custom_interpolation(Animation *animation_h,
                                        InterpolateInt64Function interpolate_function) {
  return prv_animation_set_custom_function(animation_h,
                                           AnimationCurveCustomInterpolationFunction,
                                           interpolate_function);
}

// -------------------------------------------------------------------------------------------
static void *prv_animation_get_custom_function(Animation *animation_h, AnimationCurve curve) {
  if (animation_private_using_legacy_2(NULL)) {
    if (curve != AnimationCurveCustomFunction) {
      // 2.x doesn't support AnimationCurveCustomInterpolationFunction
      return NULL;
    }
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    return animation_legacy2_get_custom_curve((AnimationLegacy2 *)animation_h);
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(NULL, animation_h, false /*quiet*/);
  if (!animation) {
    return NULL;
  }

  return (animation->curve == curve) ? animation->custom_function : NULL;
}

// -------------------------------------------------------------------------------------------
AnimationCurveFunction animation_get_custom_curve(Animation *animation_h) {
  return prv_animation_get_custom_function(animation_h, AnimationCurveCustomFunction);
}

// -------------------------------------------------------------------------------------------
InterpolateInt64Function animation_get_custom_interpolation(Animation *animation_h) {
  return prv_animation_get_custom_function(animation_h, AnimationCurveCustomInterpolationFunction);
}

// -------------------------------------------------------------------------------------------
bool animation_set_immutable(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation) {
    return false;
  }

  animation->immutable = true;
  return true;
}

// -------------------------------------------------------------------------------------------
bool animation_is_immutable(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));
  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation) {
    return false;
  }

  return animation->immutable;
}

// -------------------------------------------------------------------------------------------
bool animation_set_reverse(Animation *animation_h, bool reverse) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  if (animation_private_using_legacy_2(state)) {
    // We need to enable other applib modules like scroll_layer, menu_layer, etc. which are
    // compiled to use the 3.0 animation API to work with 2.0 apps.
    PBL_ASSERTN(!reverse);
  }

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  // NOTE: We still need to implement reverse for sequence and spawn animations.
  // tracked in this JIRA: https://pebbletechnology.atlassian.net/browse/PBL-14838
  animation->reverse = reverse;
  return true;
}


// -------------------------------------------------------------------------------------------
bool animation_get_reverse(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));
  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation) {
    return false;
  }

  return animation->reverse;
}

// -------------------------------------------------------------------------------------------
bool animation_set_play_count(Animation *animation_h, uint32_t play_count) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));
  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!prv_is_mutable(state, animation)) {
    return false;
  }

  if (play_count == ANIMATION_PLAY_COUNT_INFINITE) {
    animation->play_count = ANIMATION_PLAY_COUNT_INFINITE_STORED;
  } else if (play_count > ANIMATION_PLAY_COUNT_INFINITE_STORED) {
    // We can't support play counts greater than ANIMATION_PLAY_COUNT_INFINITE_STORED
    return false;
  } else {
    animation->play_count = play_count;
  }
  return true;
}


// -------------------------------------------------------------------------------------------
uint32_t animation_get_play_count(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));
  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation) {
    return false;
  }

  if (animation->play_count == ANIMATION_PLAY_COUNT_INFINITE_STORED) {
    return ANIMATION_PLAY_COUNT_INFINITE;
  } else {
    return animation->play_count;
  }
}


// -------------------------------------------------------------------------------------------
bool animation_set_elapsed(Animation *parent_h, uint32_t elapsed_ms) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));

  AnimationPrivate *parent = prv_find_animation_by_handle(state, parent_h, false /*quiet*/);
  if (!parent || !prv_animation_is_scheduled(state, parent) || parent->parent) {
    // Can only set the elapsed of a top-level animation, and then, only after it has
    // been scheduled.
    return false;
  }

  // First, we need to compute the absolute start time of this animation, backing it up by the
  // delay and any repeats we have already done
  uint32_t start_ms = parent->abs_start_time_ms;
  start_ms -= parent->times_played * (parent->duration_ms + parent->delay_ms);

  // Loop through animation and all of it's children until the "virtual now" catches up to
  // the desired elapsed
  const uint32_t now = prv_get_ms_since_system_start();

  uint32_t virtual_now = now;
  const uint32_t target_now = start_ms + elapsed_ms;

  while (serial_distance32(virtual_now, target_now) >= 0) {
    prv_run(state, virtual_now, parent, start_ms, (virtual_now == target_now) /*do_update*/);

    // Advance virtual now
    uint32_t remaining = serial_distance32(virtual_now, target_now);
    if (remaining == 0) {
      break;
    }
    virtual_now += MIN(ANIMATION_TARGET_FRAME_INTERVAL_MS, remaining);
  }

  // Now, go and backup the abs_start_time_ms of the animations we skipped ahead on
  prv_backup_start_time(state, parent, virtual_now - now);
  return true;
}


// -------------------------------------------------------------------------------------------
bool animation_get_elapsed(Animation *animation_h, int32_t *elapsed_ms) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation || !prv_animation_is_scheduled(state, animation) || !elapsed_ms) {
    return false;
  }

  *elapsed_ms = prv_get_elapsed(animation, prv_get_ms_since_system_start());
  return true;
}


// -------------------------------------------------------------------------------------------
Animation *animation_sequence_create_from_array(Animation **animation_array, uint32_t array_len) {
  PBL_ASSERTN(!animation_private_using_legacy_2(NULL));
  Animation *seq = prv_complex_create(animation_array, array_len, AnimationTypeSequence);
  return seq;
}


// -------------------------------------------------------------------------------------------
Animation *animation_sequence_init_from_array(Animation *parent, Animation **animation_array,
                                              uint32_t array_len) {
  PBL_ASSERTN(!animation_private_using_legacy_2(NULL));
  return prv_complex_init(parent, animation_array, array_len, AnimationTypeSequence);
}


// -------------------------------------------------------------------------------------------
Animation *animation_sequence_create(Animation *animation_a, Animation *animation_b,
                                     Animation *animation_c, ...) {
  PBL_ASSERTN(!animation_private_using_legacy_2(NULL));
  va_list args;
  va_start(args, animation_c);
  Animation *animation = prv_call_using_vargs(animation_sequence_create_from_array, animation_a,
                            animation_b, animation_c, args);
  va_end(args);
  return animation;
}


// -------------------------------------------------------------------------------------------
Animation *animation_spawn_create_from_array(Animation **animation_array, uint32_t array_len) {
  PBL_ASSERTN(!animation_private_using_legacy_2(NULL));
  Animation *spawn = prv_complex_create(animation_array, array_len, AnimationTypeSpawn);
  return spawn;
}


// -------------------------------------------------------------------------------------------
Animation *animation_spawn_create(Animation *animation_a, Animation *animation_b,
                                  Animation *animation_c, ...) {
  PBL_ASSERTN(!animation_private_using_legacy_2(NULL));
  va_list args;
  va_start(args, animation_c);
  Animation *animation = prv_call_using_vargs(animation_spawn_create_from_array, animation_a,
                            animation_b, animation_c, args);
  va_end(args);
  return animation;
}


// -------------------------------------------------------------------------------------------
Animation *animation_clone(Animation *animation_h) {
  AnimationState *state = prv_animation_state_get(PebbleTask_Current);
  PBL_ASSERTN(!animation_private_using_legacy_2(state));

  AnimationPrivate *animation = prv_find_animation_by_handle(state, animation_h, false /*quiet*/);
  if (!animation) {
    return false;
  }

  return prv_animation_clone(state, animation);
}

// ------------------------------------------------------------------------------------------
static void prv_dump_animations(ListNode *node, bool is_scheduled, char *buffer, int buffer_size) {
  while (node) {
    AnimationPrivate *animation = (AnimationPrivate *)node;

    dbgserial_putstr_fmt(buffer, buffer_size,
        "<%p> { sch: %s, handle = %p, abs_start_time_ms = %"PRIu32", delay = %"PRIu32", "
        "duration = %"PRIu32", curve = %i, run = %p }",
        animation, is_scheduled ? "yes" : "no", animation->handle,
        (uint32_t)animation->abs_start_time_ms,
        (uint32_t)animation->delay_ms, (uint32_t)animation->duration_ms,
        animation->curve,
        animation->implementation->update);

    node = node->next;
  }
}

static void prv_dump_legacy_animations(ListNode *head, char *buffer, int buffer_size) {
  AnimationLegacy2 *animation = (AnimationLegacy2 *)head;

  while (animation) {
    dbgserial_putstr_fmt(buffer, buffer_size,
        "<%p> { sch: yes, start handle = %p, stop handle = %p,"
        "abs_start_time_ms = %"PRIu32", delay = %"PRIu32", "
        "duration = %"PRIu32", curve = %i, run = %p }",
        animation, animation->handlers.started, animation->handlers.stopped,
        animation->abs_start_time_ms,
        animation->delay_ms, animation->duration_ms,
        animation->curve,
        animation->implementation->update);

    animation = (AnimationLegacy2 *)list_get_next(&animation->list_node);
  }
}

// -------------------------------------------------------------------------------------------
static void prv_dump_scheduler(char* buffer, int buffer_size, AnimationState* state) {
  pbl_irq_lock();
  if (animation_private_using_legacy_2(state)) {
    AnimationLegacy2Scheduler *legacy_state = (AnimationLegacy2Scheduler *)state;
    prv_dump_legacy_animations(legacy_state->head, buffer, buffer_size);
  } else {
    prv_dump_animations(state->scheduled_head, true, buffer, buffer_size);
    prv_dump_animations(state->unscheduled_head, false, buffer, buffer_size);
  }
  pbl_irq_unlock();
}


// -------------------------------------------------------------------------------------------
void animation_private_pause(void) {
  s_paused = true;
}


// -------------------------------------------------------------------------------------------
void animation_private_resume(void) {
  s_paused = false;
}


// -------------------------------------------------------------------------------------------
void command_animations_info(void) {
  char buffer[128];
  dbgserial_putstr_fmt(buffer, sizeof(buffer), "Now: %"PRIu32, prv_get_ms_since_system_start());

  dbgserial_putstr_fmt(buffer, sizeof(buffer), "Kernel Animations:");
  prv_dump_scheduler(buffer, sizeof(buffer), kernel_applib_get_animation_state());

  dbgserial_putstr_fmt(buffer, sizeof(buffer), "App Animations:");
  prv_dump_scheduler(buffer, sizeof(buffer), app_state_get_animation_state());
}


// -------------------------------------------------------------------------------------------
void command_pause_animations(void) {
  animation_private_pause();
}


// -------------------------------------------------------------------------------------------
void command_resume_animations(void) {
  animation_private_resume();
}
