/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/event_service_client.h"
#include "kernel/pbl_malloc.h"
#include "kernel/pebble_tasks.h"
#include "process_management/app_manager.h"
#include "process_management/worker_manager.h"
#include "pbl/kernel/mutex.h"
#include "pbl/services/event_service.h"
#include "syscall/syscall_internal.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include "FreeRTOS.h"
#include "queue.h"

#include <string.h>

PBL_LOG_MODULE_DEFINE(service_event_service, CONFIG_SERVICE_EVENT_SERVICE_LOG_LEVEL);

typedef struct {
  int num_subscribers;
  QueueHandle_t subscribers[NumPebbleTask];
  EventServiceAddSubscriberCallback add_subscriber_callback;
  EventServiceRemoveSubscriberCallback remove_subscriber_callback;
} EventServiceEntry;

typedef struct {
  ListNode list_node;
  void *ptr;
  // There is an intent bit for each task + a special "claimed" bit
  uint16_t intents_pending;
} EventServiceBuffer;

static const uint16_t CLAIMED_BIT = (1 << NumPebbleTask);

static EventServiceBuffer *s_event_service_buffers = NULL;

// We dynamically allocate one of these for every service UUID that either a client subscribes to or a service
// publishes an event to.
typedef struct {
  ListNode  list_node;
  uint16_t  service_index;                    // index of the service
  Uuid      uuid;                             // UUID
} EventPluginUUIDEntry;

static uint16_t s_next_service_index = 0;
static ListNode s_plugin_list;
// This mutex guards the s_plugin_list linked list
static PBL_MUTEX_DEFINE(s_plugin_list_mutex);

// There's an event service for each event so that
// System apps can also use the service
static EventServiceEntry *s_event_services[PEBBLE_NUM_EVENTS];

static void prv_event_service_unsubscribe(PebbleSubscriptionEvent *subscription) {
  EventServiceEntry *service = s_event_services[subscription->event_type];

  if (s_event_services[subscription->event_type] == NULL) {
    // service does not exist
    PBL_LOG_WRN("Attempted to unsubscribe from %d, no service found",
        subscription->event_type);
    return;
  }

  if (service->subscribers[subscription->task] == NULL) {
    // not subscribed
    PBL_LOG_WRN("Attempted to unsubscribe from %d, not subscribed",
        subscription->event_type);
    return;
  }

  PBL_ASSERTN(service->num_subscribers > 0);
  --service->num_subscribers;
  service->subscribers[subscription->task] = NULL;
  if (service->remove_subscriber_callback != NULL) {
    service->remove_subscriber_callback(subscription->task);
  }
}


static void prv_event_service_subscribe(PebbleSubscriptionEvent *subscription) {
  EventServiceEntry *service = s_event_services[subscription->event_type];

  if (service == NULL) {
    // No event service for this event type, create one
    event_service_init(subscription->event_type, NULL, NULL);
    service = s_event_services[subscription->event_type];
  }

  if (service->subscribers[subscription->task]) {
    // already subscribed ?
    PBL_LOG_DBG("already subscribed");
    return;
  }

  if (service->add_subscriber_callback != NULL) {
    service->add_subscriber_callback(subscription->task);
  }

  service->subscribers[subscription->task] = subscription->event_queue;
  ++service->num_subscribers;
}

void event_service_subscribe_from_kernel_main(PebbleSubscriptionEvent *subscription) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);
  prv_event_service_subscribe(subscription);
}

static bool prv_event_service_send_event(QueueHandle_t queue, PebbleEvent *e) {
  PBL_ASSERTN(queue != NULL);
  bool success = (xQueueSendToBack(queue, e, 0) == pdTRUE);
  return success;
}

void event_service_handle_subscription(PebbleSubscriptionEvent *subscription) {
  if (subscription->subscribe) {
    prv_event_service_subscribe(subscription);
  } else {
    prv_event_service_unsubscribe(subscription);
  }
}


void event_service_clear_process_subscriptions(PebbleTask task) {
  EventServiceEntry *service;

  for (int i = 0; i < PEBBLE_NUM_EVENTS; i++) {
    if ((service = s_event_services[i]) == NULL) {
      continue;
    }
    if (service->subscribers[task] == NULL) {
      continue;
    }

    PebbleSubscriptionEvent event = {
      .subscribe = false,
      .task = task,
      .event_type = i,
    };
    prv_event_service_unsubscribe(&event);
  }
}

void event_service_system_init(void) {
}

void event_service_init(PebbleEventType type, EventServiceAddSubscriberCallback add_subscriber_callback,
    EventServiceRemoveSubscriberCallback remove_subscriber_callback) {
  if(s_event_services[type] != NULL) {
    // an event service was already inited, free it
    kernel_free(s_event_services[type]);
  }

  s_event_services[type] = kernel_malloc(sizeof(EventServiceEntry));
  memset(s_event_services[type], 0, sizeof(*s_event_services[type]));
  s_event_services[type]->add_subscriber_callback = add_subscriber_callback;
  s_event_services[type]->remove_subscriber_callback = remove_subscriber_callback;
}

bool event_service_is_running(PebbleEventType event_type) {
  if (s_event_services[event_type] == NULL) {
    return (false);
  }
  if (s_event_services[event_type]->num_subscribers > 0) {
    return (true);
  }

  return (false);
}

static bool prv_task_is_masked_out(PebbleEvent *e, PebbleTask task) {
  const PebbleTaskBitset task_bit = (1 << task);
  return (e->task_mask & task_bit);
}

static bool prv_steal_buffer(void *buf, EventServiceEntry *service, PebbleEvent *e) {
  uint16_t intents_pending =  0;

  for (int i = 0; i < NumPebbleTask; i++) {
    if (!prv_task_is_masked_out(e, i) && service->subscribers[i]) {
      const PebbleTaskBitset task_bit = (1 << i);
      intents_pending |= task_bit;
    }
  }

  if (intents_pending) {
    EventServiceBuffer *esb = kernel_zalloc_check(sizeof(EventServiceBuffer));
    esb->ptr = buf;
    esb->intents_pending = intents_pending;
    list_init(&esb->list_node);
    s_event_service_buffers = (EventServiceBuffer *)list_prepend(
        (ListNode *)s_event_service_buffers, (ListNode *)esb);
    return true; // we stole the buffer
  } else {
    return false;
  }
}

void event_service_handle_event(PebbleEvent *e) {
  EventServiceEntry *service = s_event_services[e->type];
  if (service == NULL) {
    return;
  }

  bool stolen = false;
  void **buf_ptr = event_get_buffer(e);
  if (buf_ptr && *buf_ptr) {
    stolen = prv_steal_buffer(*buf_ptr, service, e); // FIXME arguments?
  }

  PebbleTask cur_task = pebble_task_get_current();
  for (unsigned i = 0; i < NumPebbleTask; i++) {
    if (!prv_task_is_masked_out(e, i) && service->subscribers[i]) {
      if (i == cur_task) {
        // We will handle this inline, but that has to happen after we copy it into the queues
        // because handling it inline could modify the event
        continue;
      } else {
        if (!prv_event_service_send_event(service->subscribers[i], e)) {
          PBL_LOG_ERR("Queue full! %d not delivered to task %d!",
                  (int)e->type, (int)i);
#ifndef CONFIG_RELEASE
          // For 3rd party apps, just close them. For a 1st party app or other task, reboot
          // the watch
          if (i == PebbleTask_App && app_manager_get_current_app_md()->is_unprivileged) {
            app_manager_close_current_app(false);
          } else if (i == PebbleTask_Worker &&
                     worker_manager_get_current_worker_md()->is_unprivileged) {
            worker_manager_close_current_worker(false);
          } else {
            PBL_ASSERTN(0);
          }
#endif
        }
      }
    }
  }

  if (!prv_task_is_masked_out(e, cur_task) && service->subscribers[cur_task]) {
    // we are on the current task, so we just tell the client to handle it inline
    event_service_client_handle_event(e);
  }

  if (stolen) {
    // we stole the buffer from the event, NULL it out
    *buf_ptr = NULL;
  }
}

// -------------------------------------------------------------------------------------------------
static bool prv_buffer_find(ListNode *found_node, void *data) {
  EventServiceBuffer *esb = (EventServiceBuffer *)found_node;
  return (esb->ptr == data);
}

bool event_service_is_known_buffer(const void *buf) {
  if (buf == NULL) {
    return false;
  }
  // Cast away const for list_find's callback signature; prv_buffer_find only compares.
  return list_find((ListNode *)s_event_service_buffers, prv_buffer_find,
                   (void *)buf) != NULL;
}

static EventServiceBuffer* prv_get_esb_for_event(PebbleEvent *e) {
  void **buf_ptr = event_get_buffer(e);
  EventServiceBuffer *esb = NULL;
  if (buf_ptr && *buf_ptr) {
    esb = (EventServiceBuffer *)list_find((ListNode *)s_event_service_buffers,
                                          prv_buffer_find,
                                          *buf_ptr);
  }
  return esb;
}

void* event_service_claim_buffer(PebbleEvent *e) {
  EventServiceBuffer *esb = prv_get_esb_for_event(e);
  if (esb) {
    if (esb->intents_pending & CLAIMED_BIT) {
      // For now only 1 claim at a time is needed, so lets keep things simple and just support that
      PBL_LOG_WRN("Buffer already claimed");
      return NULL;
    } else {
      esb->intents_pending |= CLAIMED_BIT;
      return esb;
    }
  }
  return NULL;
}

void event_service_free_claimed_buffer(void *ref) {
  if (!ref) {
    return;
  }

  EventServiceBuffer *esb = ref;

  if (esb->intents_pending & CLAIMED_BIT) {
    // If other events still need the buffer removing the claim marker will make things
    // get cleaned up as usual.
    uint16_t intents_pending =  __sync_and_and_fetch(&esb->intents_pending, ~CLAIMED_BIT);

    if (!intents_pending) {
      list_remove((ListNode *)esb, (ListNode **)&s_event_service_buffers, NULL);
      if (esb->ptr) {
        kernel_free(esb->ptr);
        esb->ptr = NULL;
      }
      kernel_free(esb);
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
static bool prv_service_filter(ListNode *node, void *tp) {
  EventPluginUUIDEntry *info = (EventPluginUUIDEntry *)node;
  return uuid_equal(&info->uuid, (Uuid *)tp);
}

// ---------------------------------------------------------------------------------------------------------------
// TODO: We need to prune out entries from this list when they are no longer needed
// TODO: The applib should force a restriction on the number of plugin service UUIDs that an app can subscribe
//          to at once.
static int16_t prv_get_plugin_index(const Uuid *uuid) {
  int16_t result = -1;

  pbl_mutex_lock(&s_plugin_list_mutex, PBL_FOREVER);

  // Look for this service UUID
  ListNode *found;
  ListNode *list = &s_plugin_list;
  found = list_find(list, prv_service_filter, (void*)uuid);
  if (found) {
    result = ((EventPluginUUIDEntry *)found)->service_index;
    goto unlock;
  }

  EventPluginUUIDEntry *entry = kernel_zalloc_check(sizeof(EventPluginUUIDEntry));
  entry->service_index = ++s_next_service_index;
  entry->uuid = *uuid;
  list_append(list, &entry->list_node);
  result = entry->service_index;

  char uuid_buffer[UUID_STRING_BUFFER_LENGTH];
  uuid_to_string(uuid, uuid_buffer);
  PBL_LOG_DBG("Registered plug-in service %s as index %d", uuid_buffer, result);

unlock:
  pbl_mutex_unlock(&s_plugin_list_mutex);
  return result;
}


//! @param uuid the UUID of the plugin service, or NULL to use uuid of the current process
//! @return non-negative service index, or -1 if error
DEFINE_SYSCALL(int16_t, sys_event_service_get_plugin_service_index, const Uuid * uuid) {
  if (PRIVILEGE_WAS_ELEVATED && uuid != NULL) {
    syscall_assert_userspace_buffer(uuid, sizeof(*uuid));
  }
  if (uuid == NULL) {
    uuid = &sys_process_manager_get_current_process_md()->uuid;
  }
  return prv_get_plugin_index(uuid);
}

DEFINE_SYSCALL(void, sys_event_service_cleanup, PebbleEvent *e) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(e, sizeof(PebbleEvent));
  }

  EventServiceBuffer *esb = prv_get_esb_for_event(e);

  if (esb) {
    uint16_t task_bit = 1 << pebble_task_get_current();
    uint16_t intents_pending =  __sync_and_and_fetch(&esb->intents_pending, ~task_bit);
    if (intents_pending) {
      // zero out buf_ptr so it won't be freed by cleanup. Other tasks are still waiting to use it
      void **buf_ptr = event_get_buffer(e);
      *buf_ptr = NULL;
    } else {
      // free the EventServiceBuffer and free the data
      list_remove((ListNode *)esb, (ListNode **)&s_event_service_buffers, NULL);
      kernel_free(esb);

      event_deinit(e);
    }
  }
}
