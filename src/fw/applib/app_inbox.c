/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_inbox.h"

#include "applib/applib_malloc.auto.h"
#include "pbl/services/app_inbox_service.h"
#include "syscall/syscall.h"
#include <pbl/logging/logging.h>

AppInbox *app_inbox_create_and_register(size_t buffer_size, uint32_t min_num_messages,
                                        AppInboxMessageHandler message_handler,
                                        AppInboxDroppedHandler dropped_handler) {
  if (!message_handler) {
    return NULL;
  }
  if (!buffer_size) {
    return NULL;
  }
  if (!min_num_messages) {
    return NULL;
  }
  buffer_size += (min_num_messages * sizeof(AppInboxMessageHeader));
  uint8_t *buffer = applib_zalloc(buffer_size);
  if (!buffer) {
    PBL_LOG_ERR("Not enough memory to allocate App Inbox of size %"PRIu32,
            (uint32_t)buffer_size);
    return NULL;
  }
  if (!sys_app_inbox_service_register(buffer, buffer_size, message_handler, dropped_handler)) {
    applib_free(buffer);
    return NULL;
  }
  return (AppInbox *) buffer;
}

uint32_t app_inbox_destroy_and_deregister(AppInbox *app_inbox) {
  uint8_t *buffer = (uint8_t *)app_inbox;
  uint32_t num_messages_lost = sys_app_inbox_service_unregister(buffer);
  applib_free(buffer);
  return num_messages_lost;
}

void app_inbox_consume(AppInboxConsumerInfo *consumer_info) {
  sys_app_inbox_service_consume(consumer_info);
}
