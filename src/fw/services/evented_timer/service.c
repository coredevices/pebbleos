/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/evented_timer.h"

#include "kernel/events.h"
#include "pbl/kernel/mutex.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/new_timer/new_timer.h"
#include "system/passert.h"
#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "process_management/app_manager.h"

PBL_LOG_MODULE_DEFINE(service_evented_timer, CONFIG_SERVICE_EVENTED_TIMER_LOG_LEVEL);

typedef struct EventedTimer {
  ListNode list_node;

  //! The TimerID type used for sys_timers is a non-repeating integer that we also use as our key for finding
  //!  EventedTimers by id.
  TimerID sys_timer_id;

  EventedTimerCallback callback;
  void* callback_data;

  PebbleTask target_task;

  bool expired; // This gets set when the timer's callback runs
  bool repeating;
} EventedTimer;

//! The list of all the timers that have been created.
static ListNode* s_timer_list_head;

static PBL_MUTEX_DEFINE(s_mutex);

// ------------------------------------------------------------------------------------
// Find timer by id
static bool prv_id_list_filter(ListNode* node, void* data) {
  EventedTimer* timer = (EventedTimer*)node;
  return timer->sys_timer_id == (TimerID)data;
}

static EventedTimer* prv_find_timer(TimerID timer_id)
{
  if (timer_id == EVENTED_TIMER_INVALID_ID) {
    return NULL;
  }

  // Look for this timer in our linked list
  ListNode* node = list_find(s_timer_list_head, prv_id_list_filter, (void*)(intptr_t)timer_id);

  return (EventedTimer *)node;
}


//! Retrieves details for a given timer handle and copies them out to user supplied memory.
//! This gets executed on the client's task and is called directly from the callback we put onto the
//! client task's event queue.
//! It accesses the privileged contents of the timer from the client's unprivileged task.
//! This call deletes the system timer and removes it from the timer list before returning unless
//! it is a repeating timer.
DEFINE_SYSCALL(void, sys_evented_timer_consume, TimerID timer_id, EventedTimerCallback* out_cb,
                                                void** out_cb_data) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(out_cb, sizeof(*out_cb));
    syscall_assert_userspace_buffer(out_cb_data, sizeof(*out_cb_data));
  }

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  EventedTimer *timer = prv_find_timer(timer_id);

  // It's possible that the client made a call to delete the timer just
  // after the timer executed (from the timer  task) and posted the PEBBLE_CALLBACK_EVENT
  // to the client's event queue. In this case, the timer will no
  // longer be in our timers list by the time the event arrives and gets processed here.
  if (!timer) {
    pbl_mutex_unlock(&s_mutex);
    *out_cb = 0;
    return;
  }

  // Only allow consuming a timer that belongs to the calling task. Without this
  // check an unprivileged app could probe TimerIDs to find a system timer and
  // read back its kernel callback pointer + data (KASLR-defeating info leak),
  // and additionally cancel that timer (DoS).
  if (PRIVILEGE_WAS_ELEVATED && timer->target_task != pebble_task_get_current()) {
    pbl_mutex_unlock(&s_mutex);
    *out_cb = 0;
    return;
  }

  *out_cb = timer->callback;
  *out_cb_data = timer->callback_data;

  if (timer->repeating) {
    pbl_mutex_unlock(&s_mutex);
  } else {
    list_remove(&timer->list_node, &s_timer_list_head, NULL);
    pbl_mutex_unlock(&s_mutex);
    new_timer_delete(timer->sys_timer_id);
    kernel_free(timer);
  }
}


//! Wrapper for the user supplied callback. We installed this callback by posting a PEBBLE_CALLBACK_EVENT
//! to the client's event queue. This gets executed on the target task.
static void prv_evented_timer_event_callback(void* data) {
  // Note this may be running on the app task, so we have to jump through hoops to read kernel memory.

  TimerID timer_id = (TimerID) data;

  EventedTimerCallback timer_cb;
  void* timer_cb_data;

  // Get the user supplied callback pointer and data, remove the timer from our list, and delete it.
  sys_evented_timer_consume(timer_id, &timer_cb, &timer_cb_data);

  if (!timer_cb) {
    // We've already cancelled this timer, just abort.
    return;
  }

  timer_cb(timer_cb_data);
}


//! Called on the timer task. From here we need to generate a callback on the client's task.
static void prv_sys_timer_callback(void* cb_data) {
  PBL_ASSERT_TASK(PebbleTask_NewTimers);
  TimerID id = (TimerID)cb_data;

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  EventedTimer *timer = prv_find_timer(id);
  if (!timer) {
    // If there's no timer in the list, that means we've been cancelled already.
    // When we cancel a timer, we immediately  free the EventedTimer* and
    // call new_time_delete() to delete the timer. It's possible however that this
    // callback got entered after we grabbed the mutex but before we issued the
    // new_timer_delete(), in which case we  would have been blocked at the beginning
    // of this method trying to grab the mutex while the timer was deleted.
    // To handle this, we check here to see if the EventedTimer got deleted on us and
    // return immediately if so.
    pbl_mutex_unlock(&s_mutex);
    return;
  }

  timer->expired = !timer->repeating;

  pbl_mutex_unlock(&s_mutex);

  PebbleEvent e = {
    .type = PEBBLE_CALLBACK_EVENT,
    .callback = {
      .callback = prv_evented_timer_event_callback,
      .data = (void*)(intptr_t)id,
    }
  };

  switch (timer->target_task) {
    case PebbleTask_KernelMain:
      event_put(&e);
      break;
    case PebbleTask_App:
    case PebbleTask_Worker:
      process_manager_send_event_to_process(timer->target_task, &e);
      break;
    default:
      PBL_CROAK("Invalid task %s", pebble_task_get_name(pebble_task_get_current()));
    }
}


// ========================================================================================================
// External API

void evented_timer_init(void) {
}

void evented_timer_clear_process_timers(PebbleTask task) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  ListNode* iter = s_timer_list_head;
  while (iter) {
    EventedTimer* timer = (EventedTimer*) iter;
    ListNode* next = list_get_next(iter);

    if (timer->target_task == task) {
      list_remove(iter, &s_timer_list_head, NULL);
      // The delete operation will stop it for us
      new_timer_delete(timer->sys_timer_id);
      kernel_free(timer);
    }

    iter = next;
  }

  pbl_mutex_unlock(&s_mutex);
}

EventedTimerID evented_timer_register_or_reschedule(EventedTimerID timer_id, uint32_t timeout_ms,
    EventedTimerCallback callback, void *data) {
  if (timer_id != EVENTED_TIMER_INVALID_ID && evented_timer_reschedule(timer_id, timeout_ms)) {
    return timer_id;
  }
  return evented_timer_register(timeout_ms, false, callback, data);
}

EventedTimerID evented_timer_register(uint32_t timeout_ms,
                                      bool repeating,
                                      EventedTimerCallback callback,
                                      void* data) {
  PebbleTask current_task = pebble_task_get_current();
  PBL_ASSERT(current_task == PebbleTask_KernelMain || current_task == PebbleTask_App
              || current_task == PebbleTask_Worker,
      "Invalid task: %s", pebble_task_get_name(current_task));

  // Handle a lazy client. Timers are useful for handling things "not right now, but soon".
  if (timeout_ms == 0) {
    timeout_ms = 1;
  }

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  EventedTimer* new_timer = kernel_malloc_check(sizeof(EventedTimer));

  *new_timer = (EventedTimer) {
    .list_node = { 0 },
    .sys_timer_id = TIMER_INVALID_ID,  // We set this below
    .callback = callback,
    .callback_data = data,
    .target_task = current_task,
    .repeating = repeating,
    .expired = false
  };

  new_timer->sys_timer_id = new_timer_create();
  PBL_ASSERTN(new_timer->sys_timer_id != TIMER_INVALID_ID);

  s_timer_list_head = list_prepend(s_timer_list_head, &new_timer->list_node);

  uint32_t flags = repeating ? TIMER_START_FLAG_REPEATING : 0;
  bool success = new_timer_start(new_timer->sys_timer_id, timeout_ms, prv_sys_timer_callback,
                                 (void*)(intptr_t)new_timer->sys_timer_id, flags);
  PBL_ASSERTN(success);

  pbl_mutex_unlock(&s_mutex);
  return new_timer->sys_timer_id;
}


bool evented_timer_reschedule(EventedTimerID timer_id, uint32_t timeout_ms) {
  if (timeout_ms == 0) {
    timeout_ms = 1;
  }
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  // This will detect an invalid timer ID, or one that already ran on the client's task and got deleted
  //  already
  EventedTimer* timer = prv_find_timer(timer_id);
  if (!timer) {
    PBL_LOG_DBG("Attempting to reschedule an invalid timer (id=%u)",
            (unsigned)timer_id);
    pbl_mutex_unlock(&s_mutex);
    return false;
  }

  PBL_ASSERT(timer->target_task == pebble_task_get_current(), "%u vs %u",
      timer->target_task, pebble_task_get_current());

  // This will detect if the timer callback has already executed on the timer task.
  // If the timer is still in our  timer's list but is expired,
  // it means the event posted by the timer task has not yet arrived at the
  // client's task.
  if (timer->expired) {
    pbl_mutex_unlock(&s_mutex);
    return false;
  }

  // At this point, we are assured that the timer callback either has not yet run
  // or that the callback is currently blocked trying to grab s_mutex.
  // new_timer_start() will reliably tell us if it was able to reschedule
  // the timer before the callback got entered.
  // If it returns false, it means the callback was entered before it was able to reschedule it.
  uint32_t flags = timer->repeating ? TIMER_START_FLAG_REPEATING :
                                      TIMER_START_FLAG_FAIL_IF_EXECUTING;
  bool success = new_timer_start(timer->sys_timer_id, timeout_ms, prv_sys_timer_callback,
                                 (void*)(intptr_t)timer->sys_timer_id, flags);

  pbl_mutex_unlock(&s_mutex);
  return success;
}


void evented_timer_cancel(EventedTimerID timer_id) {
  if (timer_id == EVENTED_TIMER_INVALID_ID) {
    return;
  }

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  // Find this timer and validate it
  EventedTimer* timer = prv_find_timer(timer_id);
  if (!timer) {
    PBL_LOG_DBG("Attempting to cancel an invalid timer (id=%u)", (unsigned)timer_id);
    pbl_mutex_unlock(&s_mutex);
    return;
  }

  new_timer_delete(timer->sys_timer_id);    // This automatically stops the timer for us first
  list_remove(&timer->list_node, &s_timer_list_head, NULL);
  kernel_free(timer);

  pbl_mutex_unlock(&s_mutex);
}

bool evented_timer_exists(EventedTimerID timer_id){
  return prv_find_timer(timer_id) != NULL;
}

bool evented_timer_is_current_task(EventedTimerID timer_id){
  EventedTimer* timer = prv_find_timer(timer_id);
  PBL_ASSERTN(timer);
  return timer->target_task == pebble_task_get_current();
}

void evented_timer_reset(void) {
  s_timer_list_head = 0;
}

void *evented_timer_get_data(EventedTimerID timer_id) {
  EventedTimer* timer = prv_find_timer(timer_id);
  if (timer) {
    return timer->callback_data;
  } else {
    return NULL;
  }
}
