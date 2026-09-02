/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app_message/app_message_internal.h"
#include "kernel/pbl_malloc.h"
#include "pbl/kernel/mutex.h"
#include "process_management/process_manager.h"
#include "pbl/services/app_message/app_message_sender.h"
#include "pbl/services/app_outbox_service.h"
#include "syscall/syscall.h"
#include "syscall/syscall_internal.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/list.h"

PBL_LOG_MODULE_DEFINE(service_app_outbox_service, CONFIG_SERVICE_APP_OUTBOX_SERVICE_LOG_LEVEL);

static PBL_MUTEX_DEFINE(s_app_outbox_mutex);

typedef struct {
  AppOutboxMessage *head;
  AppOutboxMessageHandler message_handler;
  size_t consumer_data_length;
  PebbleTask consumer_task;
} AppOutboxConsumer;

//! Array with the consuming kernel services that have been registered at run-time:
static AppOutboxConsumer s_app_outbox_consumer[NumAppOutboxServiceTag];

////////////////////////////////////////////////////////////////////////////////////////////////////
// Declarations of permitted senders:

typedef struct {
  AppOutboxSentHandler sent_handler;
  size_t max_length;
  uint32_t max_pending_messages;
} AppOutboxSenderDef;

extern void app_message_outbox_handle_app_outbox_message_sent(AppOutboxStatus status, void *cb_ctx);

#ifdef UNITTEST
extern void test_app_outbox_sent_handler(AppOutboxStatus status, void *cb_ctx);
#endif

//! Constant array defining the allowed handlers and their restrictions:
static const AppOutboxSenderDef s_app_outbox_sender_defs[] = {
  [AppOutboxServiceTagAppMessageSender] = {
    .sent_handler = app_message_outbox_handle_app_outbox_message_sent,
    .max_length = (sizeof(AppMessageAppOutboxData) + APP_MSG_HDR_OVRHD_SIZE + APP_MSG_8K_DICT_SIZE),
    .max_pending_messages = 1,
  },
#ifdef UNITTEST
  [AppOutboxServiceTagUnitTest] = {
    .sent_handler = test_app_outbox_sent_handler,
    .max_length = 1,
    .max_pending_messages = 2,
  },
#endif
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Syscalls

static const AppOutboxSenderDef *prv_find_def_and_tag_by_handler(AppOutboxSentHandler sent_handler,
                                                                 AppOutboxServiceTag *tag_out) {
  for (AppOutboxServiceTag tag = 0; tag < NumAppOutboxServiceTag; ++tag) {
    if (s_app_outbox_sender_defs[tag].sent_handler == sent_handler) {
      if (tag_out) {
        *tag_out = tag;
      }
      return &s_app_outbox_sender_defs[tag];
    }
  }
  if (tag_out) {
    *tag_out = AppOutboxServiceTagInvalid;
  }
  return NULL;
}

static void app_outbox_service_send(const uint8_t *data, size_t length,
                                    AppOutboxSentHandler sent_handler, void *cb_ctx);

DEFINE_SYSCALL(void, sys_app_outbox_send, const uint8_t *data, size_t length,
               AppOutboxSentHandler sent_handler, void *cb_ctx) {
  if (PRIVILEGE_WAS_ELEVATED) {
    // Check that data is in app space:
    syscall_assert_userspace_buffer(data, length);
  }

  const AppOutboxSenderDef *def = prv_find_def_and_tag_by_handler(sent_handler, NULL);
  if (!def) {
    PBL_LOG_ERR("AppOutbox sent_handler not allowed <%p>", sent_handler);
    syscall_failed();
  }

  const size_t max_length = def->max_length;
  if (length > max_length) {
    PBL_LOG_ERR("AppOutbox max_length exceeded %"PRIu32" vs %"PRIu32,
            (uint32_t)length, (uint32_t)max_length);
    syscall_failed();
  }
  app_outbox_service_send(data, length, sent_handler, cb_ctx);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers

static void prv_lock(void) {
  // Using one "global" lock for all app outboxes.
  // If needed, we could easily give each app outbox its own mutex, but it seems overkill right now.
  pbl_mutex_lock(&s_app_outbox_mutex, PBL_FOREVER);
}

static void prv_unlock(void) {
  pbl_mutex_unlock(&s_app_outbox_mutex);
}

static AppOutboxConsumer *prv_consumer_for_tag(AppOutboxServiceTag tag) {
  if (tag == AppOutboxServiceTagInvalid) {
    return NULL;
  }
  AppOutboxConsumer *consumer = &s_app_outbox_consumer[tag];
  if (consumer->message_handler == NULL) {
    return NULL;
  }
  return consumer;
}

static void prv_schedule_sent_handler(AppOutboxSentHandler sent_handler,
                                      void *cb_ctx, AppOutboxStatus status) {
  if (!sent_handler) {
    return;
  }
  PebbleEvent event = {
    .type = PEBBLE_APP_OUTBOX_SENT_EVENT,
    .app_outbox_sent = {
      .sent_handler = sent_handler,
      .cb_ctx = cb_ctx,
      .status = status,
    },
  };
  process_manager_send_event_to_process(PebbleTask_App, &event);
}

//! @note This executes on App Task
static void prv_schedule_consumer_message_handler(AppOutboxConsumer *consumer,
                                                  AppOutboxMessage *message) {
  void (*callback)(void *) = (__typeof__(callback))consumer->message_handler;
  PebbleEvent event = {
    .type = PEBBLE_APP_OUTBOX_MSG_EVENT,
    .app_outbox_msg = {
      .callback = callback,
      .data = message,
    },
  };
  sys_send_pebble_event_to_kernel(&event);
}

static uint32_t prv_num_pending_messages(const AppOutboxConsumer *consumer) {
  return list_count((ListNode *)consumer->head);
}

static AppOutboxConsumer *prv_find_consumer_with_message(const AppOutboxMessage *message) {
  AppOutboxMessage *head = (AppOutboxMessage *)list_get_head((ListNode *)message);
  for (AppOutboxServiceTag tag = 0; tag < NumAppOutboxServiceTag; ++tag) {
    if (s_app_outbox_consumer[tag].head == head) {
      return &s_app_outbox_consumer[tag];
    }
  }
  return NULL;
}

void prv_cleanup_pending_messages(AppOutboxConsumer *consumer, bool should_call_sent_handler) {
  AppOutboxMessage *message = consumer->head;
  consumer->head = NULL;
  while (message) {
    if (should_call_sent_handler) {
      prv_schedule_sent_handler(message->sent_handler, message->cb_ctx,
                                        AppOutboxStatusConsumerDoesNotExist);
    }

    AppOutboxMessage *next = (AppOutboxMessage *)message->node.next;
    message->node = (ListNode) {};
    // Don't free it, it's the responsibility of the consumer to eventually call
    // app_outbox_service_consume_message(), which will free the message!
    message = next;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Exported functions

void app_outbox_service_register(AppOutboxServiceTag tag,
                                 AppOutboxMessageHandler message_handler,
                                 PebbleTask consumer_task,
                                 size_t consumer_data_length) {
  prv_lock();
  {
    PBL_ASSERTN(!prv_consumer_for_tag(tag));
    AppOutboxConsumer *consumer = &s_app_outbox_consumer[tag];
    consumer->message_handler = message_handler;
    consumer->consumer_data_length = consumer_data_length;
    consumer->consumer_task = consumer_task;
  }
  prv_unlock();
}

void app_outbox_service_unregister(AppOutboxServiceTag service_tag) {
  prv_lock();
  {
    prv_cleanup_pending_messages(&s_app_outbox_consumer[service_tag],
                         true /* should_call_sent_handler */);
    s_app_outbox_consumer[service_tag].message_handler = NULL;
  }
  prv_unlock();
}

//! @note This executes on App Task
//! Should only get called through the syscall, sys_app_outbox_send
static void app_outbox_service_send(const uint8_t *data, size_t length,
                                    AppOutboxSentHandler sent_handler, void *cb_ctx) {
  AppOutboxStatus status = AppOutboxStatusSuccess;
  prv_lock();
  {
    AppOutboxServiceTag tag;
    const AppOutboxSenderDef *def = prv_find_def_and_tag_by_handler(sent_handler, &tag);
    AppOutboxConsumer *consumer = prv_consumer_for_tag(tag);
    if (!consumer) {
      status = AppOutboxStatusConsumerDoesNotExist;
      goto finally;
    }

    if (prv_num_pending_messages(consumer) >= def->max_pending_messages) {
      status = AppOutboxStatusOutOfResources;
      goto finally;
    }

    const size_t consumer_data_length = consumer->consumer_data_length;
    AppOutboxMessage *message =
        (AppOutboxMessage *)kernel_zalloc(sizeof(AppOutboxMessage) + consumer_data_length);
    if (!message) {
      status = AppOutboxStatusOutOfMemory;
      goto finally;
    }

    *message = (AppOutboxMessage) {
      .data = data,
      .length = length,
      .sent_handler = sent_handler,
      .cb_ctx = cb_ctx,
    };

    consumer->head = (AppOutboxMessage *)list_prepend((ListNode *)consumer->head,
                                                      (ListNode *)message);

    prv_schedule_consumer_message_handler(consumer, message);
  }
finally:
  if (AppOutboxStatusSuccess != status) {
    prv_schedule_sent_handler(sent_handler, cb_ctx, status);
  }
  prv_unlock();
}

bool app_outbox_service_is_message_cancelled(AppOutboxMessage *message) {
  prv_lock();
  bool cancelled = !prv_find_consumer_with_message(message);
  prv_unlock();
  return cancelled;
}

void app_outbox_service_consume_message(AppOutboxMessage *message, AppOutboxStatus status) {
  prv_lock();
  {
    if (app_outbox_service_is_message_cancelled(message)) {
      // Don't call the sent_handler
      goto finally;
    }
    AppOutboxConsumer *consumer = prv_find_consumer_with_message(message);
    PBL_ASSERTN(consumer);
    list_remove(&message->node, (ListNode **)&consumer->head, NULL);
    prv_schedule_sent_handler(message->sent_handler, message->cb_ctx, status);
  }
finally:
  kernel_free(message);
  prv_unlock();
}

void app_outbox_service_cleanup_all_pending_messages(void) {
  prv_lock();
  for (AppOutboxServiceTag tag = 0; tag < NumAppOutboxServiceTag; ++tag) {
    AppOutboxConsumer *consumer = &s_app_outbox_consumer[tag];
    prv_cleanup_pending_messages(consumer, false /* should_call_sent_handler */);
  }
  prv_unlock();
}

void app_outbox_service_cleanup_event(PebbleEvent *event) {
  if (event->type != PEBBLE_APP_OUTBOX_MSG_EVENT) {
    return;
  }
  // Call consume directly to clean up the message, it's not valid anyway:
  app_outbox_service_consume_message((AppOutboxMessage *)event->app_outbox_msg.data,
                                     AppOutboxStatusSuccess /* ignored */);
}

void app_outbox_service_init(void) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Unit Test Interfaces

void app_outbox_service_deinit(void) {
  app_outbox_service_cleanup_all_pending_messages();
  memset(&s_app_outbox_consumer, 0, sizeof(s_app_outbox_consumer));
}

uint32_t app_outbox_service_max_pending_messages(AppOutboxServiceTag tag) {
  return s_app_outbox_sender_defs[tag].max_pending_messages;
}

uint32_t app_outbox_service_max_message_length(AppOutboxServiceTag tag) {
  return s_app_outbox_sender_defs[tag].max_length;
}
