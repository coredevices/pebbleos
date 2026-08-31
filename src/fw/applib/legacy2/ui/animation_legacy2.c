/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "animation_private_legacy2.h"
#include "animation_legacy2.h"

#include "applib/ui/animation_timing.h"
#include "process_state/app_state/app_state.h"
#include "kernel/kernel_applib_state.h"
#include "kernel/pbl_malloc.h"
#include "system/passert.h"
#include "pbl/util/math.h"
#include "pbl/util/order.h"

///////////////////
// Base AnimationLegacy2
//

static const uint32_t ANIMATION_TARGET_FRAME_INTERVAL_MS_LEGACY2 = 40; // 25 Hz

static void animation_legacy2_private_run(AnimationLegacy2Scheduler *animation_legacy2_scheduler);

// Unfortunately, the fields of the AnimationLegacy2 struct were made part of
// the Pebble SDK public interface, and some apps statically allocated
// AnimationLegacy2 or PropertyAnimation structs. Therefore, the size of an
// AnimationLegacy2 struct can never change without breaking apps. Thankfully,
// some bits of the AnimationLegacy2 struct were left unused due to padding and
// unnecessary use of bitfields. To be able to implement the custom
// animation curves feature, a function pointer needed to be added into
// the struct. There are only 30 unallocated bits in total in the
// struct: 28 bits of padding at the end, and two bits in the
// AnimationCurve enum (AnimationLegacy2.curve = 0b1xx encodes custom function,
// leaving lowest two bits free).
//
// Out of the 32 bits that make up a function pointer, only 31 bits need
// to be encoded. The least-significant bit will always be 1, indicating
// that the function is in Thumb-mode. Since we have only 30 bits free
// in the struct, this means that we need to drop at least one bit from
// the pointer, restricting us from being able to store a pointer to a
// function anywhere in one half of the total address space. Since
// current Pebble hardware (at the time of writing) can only have code
// in one of a few small ranges, a pointer can be packed into much fewer
// than 28 bits while still being able to address a function anywhere in
// memory that exists.
//
// For reference, those ranges are:
// 0x0000 0000 - 0x0001 FFFF   - Internal Flash, remapped at 0x0
// 0x0800 0000 - 0x0801 FFFF   - Internal Flash
// 0x2000 0000 - 0x2002 FFFF   - Internal SRAM
//
// When such a time comes that more ranges are required, the spare two
// bits in AnimationLegacy2.curve can be utilized.
#ifndef UNITTEST
_Static_assert(sizeof(AnimationLegacy2) <= 40, "Breaking back-compatibility!");
_Static_assert(sizeof(AnimationLegacy2Scheduler) <= 16, "Breaking back-compatibility!");
#endif


// Pack a function pointer into 28 bits. We do this by dropping bits
// 1, 26, 30 and 31 and packing the remainder together.
static uintptr_t prv_custom_curve_ptr_pack(AnimationCurveFunction ptr) {
  uintptr_t bits = (uintptr_t)ptr;
  uint8_t top_byte = bits >> 24;
  PBL_ASSERTN((top_byte & 0b11000100) == 0); // Function pointer outside of packable range!
  top_byte = ((top_byte & 0b00111000) >> 1) | (top_byte & 0b11);
  bits = (top_byte << 24) | (bits & 0xffffff);
  bits >>= 1;
  return bits;
}

// Unpack a function pointer previously packed by prv_custom_curve_ptr_pack
static AnimationCurveFunction prv_custom_curve_ptr_unpack(uintptr_t bits) {
  bits = (bits << 1) | 1;  // Set the Thumb bit on the function pointer
  uint8_t top_byte = bits >> 24;
  top_byte = ((top_byte & 0b11100) << 1) | (top_byte & 0b11);
  bits = (top_byte << 24) | (bits & 0xffffff);
  return (AnimationCurveFunction)bits;
}

struct AnimationLegacy2 *animation_legacy2_create(void) {
  AnimationLegacy2 *animation = task_malloc(sizeof(AnimationLegacy2));
  if (animation) {
    animation_legacy2_init(animation);
  }
  return (animation);
}

void animation_legacy2_destroy(AnimationLegacy2 *animation) {
  if (animation == NULL) {
    return;
  }
  animation_legacy2_unschedule(animation);
  task_free(animation);
}

void animation_legacy2_init(struct AnimationLegacy2 *animation) {
  PBL_ASSERTN(animation != NULL);
  *animation = (AnimationLegacy2){};
  animation->duration_ms = 250;
  animation->curve = AnimationCurveEaseInOut;
  animation->handlers = (AnimationLegacy2Handlers) { NULL, NULL };
  animation->context = NULL;
  animation->is_completed = false;
}

static int animation_legacy2_scheduler_comparator(AnimationLegacy2 *animation_legacy2_a,
                                                  AnimationLegacy2 *animation_legacy2_b) {
  return serial_distance32(animation_legacy2_a->abs_start_time_ms,
                           animation_legacy2_b->abs_start_time_ms);
}

static AnimationLegacy2Scheduler* animation_legacy2_scheduler_data_for_app_ctx_idx(
      AppTaskCtxIdx idx) {
  if (idx == AppTaskCtxIdxApp) {
    return (AnimationLegacy2Scheduler *)app_state_get_animation_state();
  }
  return (AnimationLegacy2Scheduler *)kernel_applib_get_animation_state();
}

static AnimationLegacy2Scheduler* get_current_scheduler(void) {
  if (pebble_task_get_current() == PebbleTask_App) {
    return (AnimationLegacy2Scheduler *)app_state_get_animation_state();
  }
  return (AnimationLegacy2Scheduler *)kernel_applib_get_animation_state();
}

inline static uint32_t animation_legacy2_get_ms_since_system_start(void) {
  return (sys_get_ticks() * 1000 / RTC_TICKS_HZ);
}

static void animation_legacy2_timer_callback(
                          AnimationLegacy2Scheduler *animation_legacy2_scheduler) {
  animation_legacy2_scheduler->timer_handle = NULL;
  animation_legacy2_private_run(animation_legacy2_scheduler);
}

static void animation_legacy2_reschedule_timer(
            AnimationLegacy2Scheduler *animation_legacy2_scheduler,
            uint32_t rate_control_delay_ms) {
  AnimationLegacy2 *animation = (AnimationLegacy2 *) animation_legacy2_scheduler->head;
  if (animation == NULL) {
    return;
  }
  const uint32_t now = animation_legacy2_get_ms_since_system_start();
  const int32_t delta_ms = serial_distance32(now, animation->abs_start_time_ms);
  const uint32_t interval_ms = MAX(delta_ms, 0) + rate_control_delay_ms;

  if (animation_legacy2_scheduler->timer_handle != NULL) {
    app_timer_reschedule(animation_legacy2_scheduler->timer_handle, interval_ms);
    // Ignore the return value of reschedule. If it fails it probably means the callback is already
    // fired and we're waiting for the handler to be called. This will end up rescheduling us for
    // the right time once that gets handled.
  } else {
    animation_legacy2_scheduler->timer_handle =
        app_timer_register(interval_ms, (AppTimerCallback) animation_legacy2_timer_callback,
                           animation_legacy2_scheduler);
    PBL_ASSERTN(animation_legacy2_scheduler->timer_handle != NULL);
  }
}

static void animation_legacy2_private_schedule(AnimationLegacy2 *animation,
          AnimationLegacy2Scheduler* animation_legacy2_scheduler) {
  PBL_ASSERTN(animation != NULL);
  PBL_ASSERTN(animation->implementation->update != NULL);

  if (animation_legacy2_is_scheduled(animation)) {
    animation_legacy2_unschedule(animation);
  }

  const uint32_t now = animation_legacy2_get_ms_since_system_start();
  animation->abs_start_time_ms = now + animation->delay_ms;
  if (animation->implementation->setup != NULL) {
    animation->implementation->setup(animation);
  }

  const bool old_head_is_animating = animation_legacy2_scheduler->head
        ? (((AnimationLegacy2 *)animation_legacy2_scheduler->head)->abs_start_time_ms <= now)
        : false;
  const bool ascending = true;
  animation_legacy2_scheduler->head = list_sorted_add(animation_legacy2_scheduler->head,
          &animation->list_node, (Comparator) animation_legacy2_scheduler_comparator, ascending);
  const bool has_new_head = (&animation->list_node == animation_legacy2_scheduler->head);
  if (has_new_head) {
    // Only reschedule the timer if the previous head animation wasn't running yet:
    if (old_head_is_animating == false) {
      animation_legacy2_reschedule_timer(animation_legacy2_scheduler, 0);
    }
  }
}

void animation_legacy2_private_unschedule(AnimationLegacy2 *animation,
        AnimationLegacy2Scheduler *animation_legacy2_scheduler, const bool finished) {
  if (animation == NULL || !animation_legacy2_is_scheduled(animation)) {
    return;
  }

  PBL_ASSERTN(animation->implementation != NULL);

  const bool was_old_head = (animation == (AnimationLegacy2 *) animation_legacy2_scheduler->head);
  list_remove(&animation->list_node, &animation_legacy2_scheduler->head, NULL);
  // Reschedule the timer if we're removing the head animation:
  if (was_old_head && animation_legacy2_scheduler->head != NULL) {
    animation_legacy2_reschedule_timer(animation_legacy2_scheduler, 0);
  }
  // Reset these fields, before calling .stopped(), so that this animation
  // instance can be rescheduled again in the .stopped() handler, if needed.
  animation->abs_start_time_ms = 0;
  animation->is_completed = false;
  if (animation->handlers.stopped) {
    animation->handlers.stopped(animation, finished, animation->context);
  }
  if (animation->implementation->teardown != NULL) {
    animation->implementation->teardown(animation);
  }
}

void animation_legacy2_private_unschedule_all(AppTaskCtxIdx idx) {
  AnimationLegacy2Scheduler *animation_legacy2_scheduler =
                                  animation_legacy2_scheduler_data_for_app_ctx_idx(idx);
  AnimationLegacy2 *animation = (AnimationLegacy2 *) animation_legacy2_scheduler->head;
  while (animation) {
    AnimationLegacy2 *next = (AnimationLegacy2 *) list_get_next(&animation->list_node);
    animation_legacy2_private_unschedule(animation, animation_legacy2_scheduler, false);
    animation = next;
  }
}

void animation_legacy2_private_init_scheduler(
                          AnimationLegacy2Scheduler *animation_legacy2_scheduler) {
  *animation_legacy2_scheduler = (AnimationLegacy2Scheduler) {
    .timer_handle = NULL,
    .last_delay_ms = ANIMATION_TARGET_FRAME_INTERVAL_MS_LEGACY2,
    .last_frame_time = animation_legacy2_get_ms_since_system_start()
  };
}

void animation_legacy2_schedule(AnimationLegacy2 *animation) {
  animation_legacy2_private_schedule(animation, get_current_scheduler());
}

void animation_legacy2_unschedule(AnimationLegacy2 *animation) {
  animation_legacy2_private_unschedule(animation, get_current_scheduler(), false);
}

void animation_legacy2_unschedule_all(void) {
  animation_legacy2_private_unschedule_all(AppTaskCtxIdxApp);
}

bool animation_legacy2_is_scheduled(AnimationLegacy2 *animation) {
  AnimationLegacy2Scheduler *animation_legacy2_scheduler = get_current_scheduler();
  return list_contains(animation_legacy2_scheduler->head, &animation->list_node);
}

static void animation_legacy2_private_run(AnimationLegacy2Scheduler *animation_legacy2_scheduler) {
  AnimationLegacy2 *animation = (AnimationLegacy2 *) animation_legacy2_scheduler->head;

  const uint32_t now = animation_legacy2_get_ms_since_system_start();

  while (animation) {
    const int32_t rel_ms_running = serial_distance32(animation->abs_start_time_ms, now);

    if (rel_ms_running < 0) {
      // AnimationLegacy2s are ordered by abs_start_time_ms.
      // We've reached an animation that should not start yet, so
      // everything after and including this animation shouldn't run yet.
      break;
    }

    // Get a pointer to next now, because after unscheduling this animation won't have a next.
    AnimationLegacy2 *next = (AnimationLegacy2*) list_get_next(&animation->list_node);

    if (animation->is_completed) {
      // Unschedule + call animation.stopped callback:
      const bool finished = true;
      animation_legacy2_private_unschedule(animation, animation_legacy2_scheduler, finished);
    } else {
      // If this is the animation's first frame, call the 'started' handler:
      if (animation->handlers.started &&
          animation->abs_start_time_ms > animation_legacy2_scheduler->last_frame_time) {
        animation->handlers.started(animation, animation->context);
      }

      const uint32_t time_normalized_raw = (animation->duration_ms != 0)
                ? ((ANIMATION_NORMALIZED_MAX * rel_ms_running) / animation->duration_ms)
                : ANIMATION_NORMALIZED_MAX;
      const uint32_t time_normalized = MIN(time_normalized_raw, ANIMATION_NORMALIZED_MAX);
      uint32_t distance_normalized;
      if (animation->curve >= AnimationCurveCustomFunction) {
        distance_normalized = prv_custom_curve_ptr_unpack(
            animation->custom_curve_function)(time_normalized);
      } else {
        distance_normalized = animation_timing_curve(time_normalized, animation->curve);
      }
      animation->implementation->update(animation, distance_normalized);

      const bool completed = (time_normalized == ANIMATION_NORMALIZED_MAX);
      if (completed) {
        if (animation->duration_ms != ANIMATION_DURATION_INFINITE) {
          // Leave the animation on the list for now, we'll unschedule it the next time,
          // so it's guaranteed the animation.stopped callback gets fired after the
          // (render) events caused by this last update have been processed:
          animation->is_completed = true;
        }
      }
    }

    animation = next;
  };

  // Frame rate control:
  const int32_t frame_interval_ms = serial_distance32(
                                        animation_legacy2_scheduler->last_frame_time, now);
  const int32_t error_ms = frame_interval_ms - ANIMATION_TARGET_FRAME_INTERVAL_MS_LEGACY2;
  const int32_t theoretic_delay_ms = animation_legacy2_scheduler->last_delay_ms - error_ms;
  const uint32_t delay_ms = CLIP(theoretic_delay_ms, (int32_t) 0,
                                  (int32_t) ANIMATION_TARGET_FRAME_INTERVAL_MS_LEGACY2);

  animation_legacy2_reschedule_timer(animation_legacy2_scheduler, delay_ms);
  animation_legacy2_scheduler->last_delay_ms = delay_ms;
  animation_legacy2_scheduler->last_frame_time = now;
}

void animation_legacy2_set_handlers(AnimationLegacy2 *animation, AnimationLegacy2Handlers handlers,
                                    void *context) {
  PBL_ASSERTN(animation->abs_start_time_ms == 0); // can't set after animation has been added
  animation->context = context;
  animation->handlers = handlers;
}

void animation_legacy2_set_implementation(AnimationLegacy2 *animation,
                                const AnimationLegacy2Implementation *implementation) {
  PBL_ASSERTN(animation->abs_start_time_ms == 0); // can't set after animation has been added
  animation->implementation = implementation;
}

void *animation_legacy2_get_context(AnimationLegacy2 *animation) {
  return animation->context;
}

void animation_legacy2_set_delay(AnimationLegacy2 *animation, uint32_t delay_ms) {
  PBL_ASSERTN(animation->abs_start_time_ms == 0); // can't set after animation has been added
  animation->delay_ms = delay_ms;
}

void animation_legacy2_set_duration(AnimationLegacy2 *animation, uint32_t duration_ms) {
  PBL_ASSERTN(animation->abs_start_time_ms == 0); // can't set after animation has been added
  animation->duration_ms = duration_ms;
}

void animation_legacy2_set_curve(AnimationLegacy2 *animation, AnimationCurve curve) {
  PBL_ASSERTN(animation->abs_start_time_ms == 0); // can't set after animation has been added
  PBL_ASSERTN(curve < AnimationCurveCustomFunction);
  animation->curve = curve;
}

void animation_legacy2_set_custom_curve(AnimationLegacy2 *animation,
                                AnimationCurveFunction curve_function) {
  animation->curve = AnimationCurveCustomFunction;
  animation->custom_curve_function = prv_custom_curve_ptr_pack(curve_function);
}

AnimationCurveFunction animation_legacy2_get_custom_curve(AnimationLegacy2 *animation) {
  return prv_custom_curve_ptr_unpack(animation->custom_curve_function);
}

static void dump_scheduler(char* buffer, int buffer_size,
                           AnimationLegacy2Scheduler* animation_legacy2_scheduler) {
  AnimationLegacy2 *animation = (AnimationLegacy2 *) animation_legacy2_scheduler->head;
  while (animation) {
    dbgserial_putstr_fmt(buffer, buffer_size,
        "<%p> { abs_start_time_ms = %"PRIu32", delay = %"PRIu32", duration = %"PRIu32", "
        "curve = %i, run = %p }",
        animation, animation->abs_start_time_ms, animation->delay_ms, animation->duration_ms,
        animation->curve, animation->implementation->update);

    animation = (AnimationLegacy2 *)list_get_next(&animation->list_node);
  }
}

void command_legacy2_animations_info(void) {
  char buffer[128];
  dbgserial_putstr_fmt(buffer, sizeof(buffer), "Now: %"PRIu32,
                      animation_legacy2_get_ms_since_system_start());

  dbgserial_putstr_fmt(buffer, sizeof(buffer), "Kernel AnimationLegacy2s:");
  dump_scheduler(buffer, sizeof(buffer),
        (AnimationLegacy2Scheduler *)kernel_applib_get_animation_state());

  dbgserial_putstr_fmt(buffer, sizeof(buffer), "App AnimationLegacy2s:");
  dump_scheduler(buffer, sizeof(buffer),
        (AnimationLegacy2Scheduler *)app_state_get_animation_state());
}
