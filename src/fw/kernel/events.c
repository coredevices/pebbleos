/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "events.h"

#include <pbl/logging/logging.h>
#include "system/passert.h"

#include "kernel/pbl_malloc.h"

#include "pbl/services/app_outbox_service.h"
#include "syscall/syscall.h"

#include "pbl/kernel/msgq.h"
#include "pbl/kernel/poll.h"

#include <stdint.h>
#include <stdbool.h>

#define MAX_KERNEL_EVENTS 32
#define MAX_FROM_APP_EVENTS 10
#define MAX_FROM_WORKER_EVENTS 5
#define MAX_FROM_KERNEL_MAIN_EVENTS 14

static PBL_MSGQ_DEFINE(s_kernel_event_queue, sizeof(PebbleEvent), MAX_KERNEL_EVENTS);
static PBL_MSGQ_DEFINE(s_from_app_event_queue, sizeof(PebbleEvent), MAX_FROM_APP_EVENTS);
static PBL_MSGQ_DEFINE(s_from_worker_event_queue, sizeof(PebbleEvent), MAX_FROM_WORKER_EVENTS);

// The following conventions insure that the s_from_kernel_event_queue queue will always have sufficient space and that
// KernelMain will never deadlock trying to send an event to itself:
// 1.) KernelMain must never enqueue more than MAX_FROM_KERNEL_MAIN_EVENTS events to itself while processing another
//     event.
// 2.) The ONLY task that posts events to s_from_kernel_event_queue is the KernelMain task.
// 3.) Whenever KernelMain wants to post an event to itself, it MUST use this queue.
// 4.) The KernelMain task will always service this queue first, before servicing the kernel or from_app queues.
static PBL_MSGQ_DEFINE(s_from_kernel_event_queue, sizeof(PebbleEvent), MAX_FROM_KERNEL_MAIN_EVENTS);

// This queue set contains the s_kernel_event_queue, s_from_app_event_queue, and s_from_worker_event_queue queues
static PBL_POLL_GROUP_DEFINE(s_system_event_queue_set);

uint32_t s_current_event;

// Breakdown of kernel event queue entries by callback function pointer.
// Visible in coredumps to diagnose EventQueueFull reboots.
#define CALLBACK_TRACKER_MAX_ENTRIES 16
typedef struct {
  uintptr_t callback;
  uint8_t count;
} CallbackTrackerEntry;
static CallbackTrackerEntry s_callback_tracker[CALLBACK_TRACKER_MAX_ENTRIES];

static void prv_callback_tracker_push(uintptr_t cb) {
  // Try to find existing entry or an empty slot
  int empty_slot = -1;
  for (int i = 0; i < CALLBACK_TRACKER_MAX_ENTRIES; i++) {
    if (s_callback_tracker[i].callback == cb) {
      s_callback_tracker[i].count++;
      return;
    }
    if (empty_slot < 0 && s_callback_tracker[i].count == 0) {
      empty_slot = i;
    }
  }
  if (empty_slot >= 0) {
    s_callback_tracker[empty_slot].callback = cb;
    s_callback_tracker[empty_slot].count = 1;
  }
  // If full, silently drop — the per-type count is still accurate
}

static void prv_callback_tracker_pop(uintptr_t cb) {
  for (int i = 0; i < CALLBACK_TRACKER_MAX_ENTRIES; i++) {
    if (s_callback_tracker[i].callback == cb) {
      if (--s_callback_tracker[i].count == 0) {
        s_callback_tracker[i].callback = 0;
      }
      return;
    }
  }
}

#define EVENT_DEBUG 0

#if EVENT_DEBUG
static void prv_queue_dump(struct pbl_msgq *queue) {
  PebbleEvent event;
  PBL_LOG_DBG("Dumping queue:");
  while (pbl_msgq_get(queue, &event, PBL_NO_WAIT) == 0) {
    PBL_LOG_DBG("Event type: %u", event.type);
  }
  for(;;);
}
#endif

void events_init(void) {
  // This assert is to make sure we don't accidentally bloat our PebbleEvent unecessarily. If you hit this
  // assert and you have a good reason for making the event bigger, feel free to relax the restriction.
  //PBL_LOG_DBG("PebbleEvent size is %u", sizeof(PebbleEvent));
  // FIXME:
  _Static_assert(sizeof(PebbleEvent) <= 12,
                 "You made the PebbleEvent bigger! It should be no more than 12");

  pbl_poll_group_add(&s_system_event_queue_set, &s_kernel_event_queue);
  pbl_poll_group_add(&s_system_event_queue_set, &s_from_app_event_queue);
  pbl_poll_group_add(&s_system_event_queue_set, &s_from_worker_event_queue);
}

//! Get the from_process queue for a specific task
struct pbl_msgq *event_get_to_kernel_queue(PebbleTask task) {
  if (task == PebbleTask_App) {
    return &s_from_app_event_queue;
  } else if (task == PebbleTask_Worker) {
    return &s_from_worker_event_queue;
  } else if (task == PebbleTask_KernelMain) {
    return &s_from_kernel_event_queue;
  } else if ((task == PebbleTask_NewTimers) || (task == PebbleTask_KernelBackground)) {
    return &s_kernel_event_queue;
  } else {
    WTF;
    return NULL;
  }
}


//! Decode a bit more information out about an event and pack it into a uint32_t
static uint32_t prv_get_fancy_type_from_event(const PebbleEvent *event) {
  if (event->type == PEBBLE_CALLBACK_EVENT) {
    return (uint32_t) event->callback.callback;
  }
  return event->type;
}

static void prv_log_kernel_queue_contents(void) {
  for (int i = 0; i < CALLBACK_TRACKER_MAX_ENTRIES; i++) {
    if (s_callback_tracker[i].count > 0) {
      PBL_LOG_ERR("  callback %p: %d", (void *)s_callback_tracker[i].callback,
                  s_callback_tracker[i].count);
    }
  }
}

static void prv_log_event_put_failure(const char *queue_name, uintptr_t saved_lr, const PebbleEvent *event) {
  PBL_LOG_ERR("Error, %s queue full. Type %u", queue_name, event->type);
  prv_log_kernel_queue_contents();

  RebootReason reason = {
    .code = RebootReasonCode_EventQueueFull,
    .event_queue = {
      .push_lr = saved_lr,
      .current_event = s_current_event,
      .dropped_event = prv_get_fancy_type_from_event(event)
    }
  };
  reboot_reason_set(&reason);
}

static bool prv_event_put_isr(struct pbl_msgq *queue, const char* queue_type, uintptr_t saved_lr,
                                  PebbleEvent* event) {
  PBL_ASSERTN(queue);

  if (pbl_msgq_put(queue, event, PBL_NO_WAIT) != 0) {
    prv_log_event_put_failure(queue_type, saved_lr, event);

#ifdef CONFIG_NO_WATCHDOG
    while (1);
#endif

    reset_due_to_software_failure();
  }

  if (queue == &s_kernel_event_queue && event->type == PEBBLE_CALLBACK_EVENT) {
    prv_callback_tracker_push((uintptr_t)event->callback.callback);
  }

  return false;
}

static bool prv_try_event_put(struct pbl_msgq *queue, PebbleEvent *event) {
  PBL_ASSERTN(queue);
  bool success = (pbl_msgq_put(queue, event, PBL_MSEC(3000)) == 0);
  if (success && queue == &s_kernel_event_queue && event->type == PEBBLE_CALLBACK_EVENT) {
    prv_callback_tracker_push((uintptr_t)event->callback.callback);
  }
  return success;
}

static void prv_event_put(struct pbl_msgq *queue,
                          const char* queue_type,
                          uintptr_t saved_lr,
                          PebbleEvent* event) {
  PBL_ASSERTN(queue);

  if (pbl_msgq_put(queue, event, PBL_MSEC(3000)) != 0) {
    // We waited a reasonable amount of time here before failing. We don't want to wait too long because
    // if the queue really is stuck we'll just get a watchdog reset, which will be harder to debug than
    // just dieing here. However, we want to wait a non-zero amount of time to provide for a little bit
    // of backup to occur before killing ourselves.

    prv_log_event_put_failure(queue_type, saved_lr, event);

#if EVENT_DEBUG
    prv_queue_dump(queue);
#endif

    reset_due_to_software_failure();
  }

  if (queue == &s_kernel_event_queue && event->type == PEBBLE_CALLBACK_EVENT) {
    prv_callback_tracker_push((uintptr_t)event->callback.callback);
  }
}

void event_deinit(PebbleEvent* event) {
  void **buffer = event_get_buffer(event);
  if (buffer && *buffer) {
    kernel_free(*buffer);
    *buffer = NULL;
  }
}

void event_put(PebbleEvent* event) {
  // Caller LR; more reliable than reading lr register from a deeper helper.
  uintptr_t saved_lr = (uintptr_t)__builtin_return_address(0);
  // If we are posting from the KernelMain task, use the dedicated s_from_kernel_event_queue queue for that
  // See comments above where s_from_kernel_event_queue is declared.
  if (pebble_task_get_current() == PebbleTask_KernelMain) {
    return prv_event_put(&s_from_kernel_event_queue, "from_kernel", saved_lr, event);
  } else {
    return prv_event_put(&s_kernel_event_queue, "kernel", saved_lr, event);
  }
}

bool event_put_isr(PebbleEvent* event) {
  uintptr_t saved_lr = (uintptr_t)__builtin_return_address(0);

  return prv_event_put_isr(&s_kernel_event_queue, "kernel", saved_lr, event);
}

void event_put_from_process(PebbleTask task, PebbleEvent* event) {
  uintptr_t saved_lr = (uintptr_t)__builtin_return_address(0);

  struct pbl_msgq *queue = event_get_to_kernel_queue(task);
  prv_event_put(queue, "from app", saved_lr, event);
}

bool event_try_put_from_process(PebbleTask task, PebbleEvent* event) {
  struct pbl_msgq *queue = event_get_to_kernel_queue(task);
  return prv_try_event_put(queue, event);
}

bool event_take_timeout(PebbleEvent* event, int timeout_ms) {
  s_current_event = 0;

  // We must prioritize the from_kernel queue and always empty that first in order to avoid deadlocks in
  // KernelMain. See comments at top of file where s_from_kernel_event_queue is declared.

  // Check the from_kernel queue first to see if we posted any events to ourself.
  bool result = (pbl_msgq_get(&s_from_kernel_event_queue, event, PBL_NO_WAIT) == 0);
  if (result) {
    s_current_event = prv_get_fancy_type_from_event(event);
    return true;
  }

  // Wait for either the from_app, from_worker, or kernel queue to be ready.
  struct pbl_msgq *activated_queue = pbl_poll_group_wait(&s_system_event_queue_set,
                                                         PBL_MSEC(timeout_ms));
  if (!activated_queue) {
    return false;
  }

  // Always service the kernel queue first. This prevents a misbehaving app from starving us.
  // If we're a little lazy servicing the app, the app will just block itself when the queue gets full.
  if (pbl_msgq_get(&s_kernel_event_queue, event, PBL_NO_WAIT) == 0) {
    if (event->type == PEBBLE_CALLBACK_EVENT) {
      prv_callback_tracker_pop((uintptr_t)event->callback.callback);
    }
  } else {
    // Process the activated queue. This insures that events are handled in FIFO order from the app and worker
    // tasks. Note that sometimes the activated_queue can be the s_kernel_event_queue, even though
    // the above receive returned no event
    if (activated_queue == &s_from_app_event_queue || activated_queue == &s_from_worker_event_queue) {
      result = (pbl_msgq_get(activated_queue, event, PBL_NO_WAIT) == 0);
    }
    if (!result) {
      result = (pbl_msgq_get(&s_from_app_event_queue, event, PBL_NO_WAIT) == 0);
    }
    if (!result) {
      result = (pbl_msgq_get(&s_from_worker_event_queue, event, PBL_NO_WAIT) == 0);
    }

    // If there was nothing in the queue, return false. We are misusing the poll group by pulling events out
    //  from the s_kernel_event_queue queue before it's activated so likely, the activated queue was
    //  s_kernel_event_queue.
    if (!result) {
      return false;
    }
  }

  s_current_event = prv_get_fancy_type_from_event(event);
  return true;
}

void **event_get_buffer(PebbleEvent *event) {
  switch (event->type) {
    case PEBBLE_SYS_NOTIFICATION_EVENT:
      if (event->sys_notification.type == NotificationActionResult) {
        return (void **)&event->sys_notification.action_result;
      } else if ((event->sys_notification.type == NotificationAdded) ||
                 (event->sys_notification.type == NotificationRemoved) ||
                 (event->sys_notification.type == NotificationActedUpon)) {
        return (void **)&event->sys_notification.notification_id;
      }
      break;

    case PEBBLE_BLOBDB_EVENT:
      return (void **)&event->blob_db.key;

    case PEBBLE_BT_PAIRING_EVENT:
      if (event->bluetooth.pair.type ==
          PebbleBluetoothPairEventTypePairingUserConfirmation) {
        return (void **)&event->bluetooth.pair.confirmation_info;
      }
      break;

    case PEBBLE_APP_LAUNCH_EVENT:
      return (void **)&event->launch_app.data;

    case PEBBLE_VOICE_SERVICE_EVENT:
      return (void **)&event->voice_service.data;

    case PEBBLE_REMINDER_EVENT:
      return (void **)&event->reminder.reminder_id;

    case PEBBLE_BLE_GATT_CLIENT_EVENT:
      if (event->bluetooth.le.gatt_client.subtype == PebbleBLEGATTClientEventTypeServiceChange) {
        return (void **)(&event->bluetooth.le.gatt_client_service.info);
      }
      break;
#ifdef CONFIG_MFG
    case PEBBLE_HRM_EVENT:
      if (event->hrm.event_type == HRMEvent_CTR) {
        return (void **)(&event->hrm.ctr);
      } else if (event->hrm.event_type == HRMEvent_Leakage) {
        return (void **)(&event->hrm.leakage);
      }
      break;
#endif
    case PEBBLE_APP_GLANCE_EVENT:
      return (void **)&event->app_glance.app_uuid;

    case PEBBLE_TIMELINE_PEEK_EVENT:
      return (void **)&event->timeline_peek.item_id;

    default:
      break; // Nothing to do!
  }

  return NULL;
}

void event_cleanup(PebbleEvent* event) {
  event_deinit(event);

#ifndef CONFIG_RELEASE
  // Hopefully this will catch some use after free evil
  *event = (PebbleEvent){};
#endif
}

void event_reset_from_process_queue(PebbleTask task) {
  struct pbl_msgq *reset_queue;
  if (task == PebbleTask_App) {
    reset_queue = &s_from_app_event_queue;
  } else if (task == PebbleTask_Worker) {
    reset_queue = &s_from_worker_event_queue;
  } else {
    WTF;
  }
  event_queue_cleanup_and_reset(reset_queue);
}

struct pbl_msgq *event_kernel_to_kernel_event_queue(void) {
  return &s_from_kernel_event_queue;
}

void event_queue_cleanup_and_reset(struct pbl_msgq *queue) {
  int num_events_in_queue = pbl_msgq_num_used(queue);
  PebbleEvent event;
  for (int i = 0; i < num_events_in_queue; ++i) {
    PBL_ASSERTN(pbl_msgq_get(queue, &event, PBL_NO_WAIT) == 0);
    // event service does some book-keeping about events, notify it that we're dropping these.
    sys_event_service_cleanup(&event);
#if !defined(CONFIG_RECOVERY_FW)
    // app outbox service messages need to be cleaned up:
    app_outbox_service_cleanup_event(&event);
#endif
    // cleanup the event, free associated memory if applicable
    event_cleanup(&event);
  }

  pbl_msgq_purge(queue);
}
