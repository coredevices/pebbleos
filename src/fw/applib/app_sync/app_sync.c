/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app_sync/app_sync.h"
#include "syscall/syscall.h"
#include "system/passert.h"

static void delegate_errors(AppSync *s, DictionaryResult dict_result,
                            AppMessageResult app_message_result) {
  if (dict_result == DICT_OK && app_message_result == APP_MSG_OK) {
    return;
  }
  if (s->callback.error) {
    s->callback.error(dict_result, app_message_result, s->callback.context);
  }
}

static void update_key_callback(const uint32_t key, const Tuple *new_tuple,
                                const Tuple *old_tuple, void *context) {
  AppSync *s = context;
  if (s->callback.value_changed) {
    s->callback.value_changed(key, new_tuple, old_tuple, s->callback.context);
  }
}

static void pass_initial_values_app_task_callback(void *data) {
  AppSync *s = data;
  Tuple *tuple = dict_read_first(&s->current_iter);
  while (tuple) {
    update_key_callback(tuple->key, tuple, NULL, s);
    tuple = dict_read_next(&s->current_iter);
  }
}

static void update_callback(DictionaryIterator *updated_iter, void *context) {
  AppSync *s = context;
  uint32_t size = s->buffer_size;
  const bool update_existing_keys_only = true;
  DictionaryResult result = dict_merge(&s->current_iter, &size,
                                       updated_iter,
                                       update_existing_keys_only,
                                       update_key_callback, s);
  delegate_errors(s, result, APP_MSG_OK);
}

static void out_failed_callback(DictionaryIterator *failed, AppMessageResult reason,
                                void *context) {
  AppSync *s = context;
  delegate_errors(s, DICT_OK, reason);
}

static void in_dropped_callback(AppMessageResult reason, void *context) {
  AppSync *s = context;
  delegate_errors(s, DICT_OK, reason);
}

// FIXME PBL-1709: this should return an AppMessageResult ...
void app_sync_init(AppSync *s,
                   uint8_t *buffer, const uint16_t buffer_size,
                   const Tuplet * const keys_and_initial_values, const uint8_t count,
                   AppSyncTupleChangedCallback tuple_changed_callback,
                   AppSyncErrorCallback error_callback,
                   void *context) {
  PBL_ASSERTN(buffer != NULL);
  PBL_ASSERTN(buffer_size > 0);
  s->buffer = buffer;
  s->buffer_size = buffer_size;
  s->callback.value_changed = tuple_changed_callback;
  s->callback.error = error_callback;
  s->callback.context = context;
  uint32_t in_out_size = buffer_size;
  const DictionaryResult dict_result = dict_serialize_tuplets_to_buffer_with_iter(
    &s->current_iter, keys_and_initial_values, count, s->buffer, &in_out_size);
  app_message_set_context(s);
  app_message_register_outbox_sent(update_callback);
  app_message_register_outbox_failed(out_failed_callback);
  app_message_register_inbox_received(update_callback);
  app_message_register_inbox_dropped(in_dropped_callback);
  sys_current_process_schedule_callback(pass_initial_values_app_task_callback, s);
  delegate_errors(s, dict_result, APP_MSG_OK);
}

void app_sync_deinit(AppSync *s) {
  app_message_set_context(NULL);
  app_message_register_outbox_sent(NULL);
  app_message_register_outbox_failed(NULL);
  app_message_register_inbox_received(NULL);
  app_message_register_inbox_dropped(NULL);
  s->current = NULL;
}

AppMessageResult app_sync_set(AppSync *s, const Tuplet * const updated_keys_and_values,
                              const uint8_t count) {
  DictionaryIterator *iter;
  AppMessageResult result = app_message_outbox_begin(&iter);
  if (iter == NULL) {
    return result;
  }
  for (unsigned int i = 0; i < count; ++i) {
    dict_write_tuplet(iter, &updated_keys_and_values[i]);
  }
  dict_write_end(iter);
  return app_message_outbox_send();
}

const Tuple * app_sync_get(const AppSync *s, const uint32_t key) {
  return dict_find(&s->current_iter, key);
}
