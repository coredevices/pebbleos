/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

// Fakes
/////////////
#include "fake_system_task.h"
#include "fake_regular_timer.h"
#include "fake_blobdb.h"

// Stubs
/////////////
#include "stubs_pbl_malloc.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_session.h"

// FW Includes
///////////////
#include "pbl/services/blob_db/api.h"
#include "pbl/services/blob_db/util.h"
#include "pbl/services/blob_db/sync.h"
#include "pbl/util/size.h"


// Writebacks counter
////////////////////////

static int s_num_writebacks;
static int s_num_until_timeout;

void blob_db_endpoint_send_sync_done(BlobDBId db_id) {
  return;
}

static void prv_handle_response_from_phone(void *data) {
  s_num_writebacks++;
  BlobDBSyncSession *session = data;
  blob_db_sync_next(session);
}

static void prv_generate_responses_from_phone(void) {
  while (fake_system_task_count_callbacks()) {
    fake_system_task_callbacks_invoke_pending();
  }
}



BlobDBToken blob_db_endpoint_send_writeback(BlobDBId db_id,
                                            time_t last_updated,
                                            const void *key,
                                            int key_len,
                                            const void *val,
                                            int val_len) {
  BlobDBSyncSession *session = blob_db_sync_get_session_for_id(db_id);
  cl_assert(session != NULL);
  if (s_num_until_timeout != 0 && s_num_writebacks >= s_num_until_timeout) {
    // Don't respond - simulates timeout (message lost/no response from phone)
  } else {
    system_task_add_callback(prv_handle_response_from_phone, session);
  }

  return 12345;
}

BlobDBToken blob_db_endpoint_send_write(BlobDBId db_id,
                                        time_t last_updated,
                                        const void *key,
                                        int key_len,
                                        const void *val,
                                        int val_len) {
  return 0;
}

// Tests
////////////////////////

void test_blob_db_sync__initialize(void) {
  fake_blob_db_set_id(BlobDBIdTest);
  blob_db_init_dbs();
  s_num_until_timeout = 0;
  s_num_writebacks = 0;
}

void test_blob_db_sync__cleanup(void) {
  blob_db_flush(BlobDBIdTest);
  fake_system_task_callbacks_cleanup();
}

void test_blob_db_sync__no_dirty(void) {
  uint8_t ids[NumBlobDBs];
  uint8_t num_ids;
  blob_db_get_dirty_dbs(ids, &num_ids);
  cl_assert_equal_i(num_ids, 0);
  cl_assert(blob_db_get_dirty_list(BlobDBIdTest) == NULL);

  // insert one
  char *key = "key";
  char *value = "value";
  cl_assert_equal_i(S_SUCCESS, blob_db_insert(BlobDBIdTest,
                                              (uint8_t *)key,
                                              strlen(key),
                                              (uint8_t *)value,
                                              strlen(value)));
  blob_db_get_dirty_dbs(ids, &num_ids);
  cl_assert_equal_i(num_ids, 1);
  cl_assert(blob_db_get_dirty_list(BlobDBIdTest) != NULL);

  // mark it synced
  cl_assert_equal_i(S_SUCCESS, blob_db_mark_synced(BlobDBIdTest, (uint8_t *)key, strlen(key)));
  blob_db_get_dirty_dbs(ids, &num_ids);
  cl_assert_equal_i(num_ids, 0);
  cl_assert(blob_db_get_dirty_list(BlobDBIdTest) == NULL);
}

static bool prv_list_key_comparator(ListNode *cur_node, void *data) {
  BlobDBDirtyItem *dirty_item = (BlobDBDirtyItem *)cur_node;
  char *key = data;
  return (memcmp(dirty_item->key, key, dirty_item->key_len) == 0);
}

void test_blob_db_sync__dirty_list(void) {
  uint8_t ids[NumBlobDBs];
  uint8_t num_ids;
  blob_db_get_dirty_dbs(ids, &num_ids);
  cl_assert_equal_i(num_ids, 0);
  BlobDBDirtyItem *dirty_list = blob_db_get_dirty_list(BlobDBIdTest);
  cl_assert(dirty_list == NULL);
  blob_db_util_free_dirty_list(dirty_list);

  char *keys[] = { "key1", "key2", "key3", "key4", "key5" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2", "val3", "val4", "val5" };
  int value_len = strlen(values[0]);

  // insert all keys
  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
  }

  // check the dirty list
  blob_db_get_dirty_dbs(ids, &num_ids);
  cl_assert_equal_i(num_ids, 1);
  dirty_list = blob_db_get_dirty_list(BlobDBIdTest);
  cl_assert_equal_i(list_count(&dirty_list->node), ARRAY_LENGTH(keys));

  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    cl_assert(list_find(&dirty_list->node, prv_list_key_comparator, keys[i]) != NULL);
  }

  blob_db_util_free_dirty_list(dirty_list);

  // mark one as synced and re-check
  for (int synced_idx = 1; synced_idx < ARRAY_LENGTH(keys); ++synced_idx) {
    blob_db_mark_synced(BlobDBIdTest, (uint8_t *)keys[synced_idx - 1], key_len);
    dirty_list = blob_db_get_dirty_list(BlobDBIdTest);
    cl_assert_equal_i(list_count(&dirty_list->node), ARRAY_LENGTH(keys) - synced_idx);
    for (int i = synced_idx; i < ARRAY_LENGTH(keys); ++i) {
      cl_assert(list_find(&dirty_list->node, prv_list_key_comparator, keys[i]) != NULL);
    }
    blob_db_util_free_dirty_list(dirty_list);
  }
}

void test_blob_db_sync__sync_all(void) {
  char *keys[] = { "key1", "key2", "key3", "key4", "key5" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2", "val3", "val4", "val5" };
  int value_len = strlen(values[0]);

  // insert all keys
  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
  }

  cl_assert(blob_db_sync_db(BlobDBIdTest) == S_SUCCESS);
  prv_generate_responses_from_phone();

  cl_assert_equal_i(s_num_writebacks, 5);
}

void test_blob_db_sync__sync_oom(void) {
  char *keys[] = { "key1", "key2", "key3", "key4", "key5" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2", "val3", "val4", "val5" };
  int value_len = strlen(values[0]);

  // insert all keys
  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
  }

  cl_assert(blob_db_sync_db(BlobDBIdTest) == S_SUCCESS);

  // We have built the dirty list, add more entries.
  // This mimics us performing writes while the sync is ongoing or not having enough memory to
  // build the initial list
  char *extra_keys[] = { "keyA", "keyB" };
  char *extra_values[] = { "valA", "valB" };
  for (int i = 0; i < ARRAY_LENGTH(extra_keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)extra_keys[i], key_len,
                   (uint8_t *)extra_values[i], value_len);
  }

  prv_generate_responses_from_phone();

  cl_assert_equal_i(s_num_writebacks, 7);
}

void test_blob_db_sync__sync_some(void) {
  char *keys[] = { "key1", "key2", "key3", "key4", "key5" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2", "val3", "val4", "val5" };
  int value_len = strlen(values[0]);

  // insert all keys, mark some as synced
  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
    // choose two that are not consecutive, that leaves 3 still to be synced
    if (i == 1 || i == 3) {
      blob_db_mark_synced(BlobDBIdTest, (uint8_t *)keys[i], key_len);
    }
  }

  cl_assert(blob_db_sync_db(BlobDBIdTest) == S_SUCCESS);
  prv_generate_responses_from_phone();

  cl_assert_equal_i(s_num_writebacks, 3);
}

void test_blob_db_sync__timeout_and_retry(void) {
  char *keys[] = { "key1", "key2", "key3", "key4", "key5" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2", "val3", "val4", "val5" };
  int value_len = strlen(values[0]);

  // insert all keys, mark some as synced
  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
  }

  s_num_until_timeout = 3;
  s_num_writebacks = 0;
  cl_assert(blob_db_sync_db(BlobDBIdTest) == S_SUCCESS);
  prv_generate_responses_from_phone();
  cl_assert_equal_i(s_num_writebacks, s_num_until_timeout);

  // Cancel the timed-out session so we can start a fresh sync
  BlobDBSyncSession *session = blob_db_sync_get_session_for_id(BlobDBIdTest);
  cl_assert(session != NULL);
  blob_db_sync_cancel(session);

  s_num_until_timeout = 0;
  cl_assert(blob_db_sync_db(BlobDBIdTest) == S_SUCCESS);
  prv_generate_responses_from_phone();
  cl_assert_equal_i(s_num_writebacks, 5);
}

void test_blob_db_sync__sync_while_syncing(void) {
  char *keys[] = { "key1", "key2", "key3", "key4", "key5" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2", "val3", "val4", "val5" };
  int value_len = strlen(values[0]);

  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
  }

  cl_assert(blob_db_sync_db(BlobDBIdTest) == S_SUCCESS);

  // We should throw an error if we get a sync while a db sync in in progress
  cl_assert(blob_db_sync_db(BlobDBIdTest) == E_BUSY);

  // Generate some responses so the sync session gets cleaned up
  prv_generate_responses_from_phone();
}

static BlobDBSyncSession *prv_start_sync_with_two_keys(void) {
  char *keys[] = { "key1", "key2" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2" };
  int value_len = strlen(values[0]);

  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(BlobDBIdTest, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
  }

  cl_assert(blob_db_sync_db(BlobDBIdTest) == S_SUCCESS);
  BlobDBSyncSession *session = blob_db_sync_get_session_for_id(BlobDBIdTest);
  cl_assert(session != NULL);
  return session;
}

// The regular timers fire on the timer task and hop to KernelBG before touching
// the session. If the sync completes or is cancelled in that window the session
// is freed, so the deferred half must detect that instead of dereferencing it.
void test_blob_db_sync__stale_timeout_callback_after_cancel(void) {
  BlobDBSyncSession *session = prv_start_sync_with_two_keys();

  // Drop the harness' pending phone response; exercise the timeout path alone.
  fake_system_task_callbacks_cleanup();

  fake_regular_timer_trigger(&session->timeout_timer);
  cl_assert_equal_i(fake_system_task_count_callbacks(), 1);

  blob_db_sync_cancel(session);
  cl_assert(blob_db_sync_get_session_for_id(BlobDBIdTest) == NULL);

  s_num_writebacks = 0;
  fake_system_task_callbacks_invoke_pending();

  // The stale callback must not resurrect the session or send anything.
  cl_assert_equal_i(s_num_writebacks, 0);
  cl_assert(blob_db_sync_get_session_for_id(BlobDBIdTest) == NULL);
}

void test_blob_db_sync__stale_abandon_callback_after_cancel(void) {
  // Keep the phone silent: a response would complete the sync and disarm the
  // abandon timer before we can exercise it.
  s_num_until_timeout = 1;
  s_num_writebacks = 1;

  BlobDBSyncSession *session = prv_start_sync_with_two_keys();
  cl_assert_equal_i(fake_system_task_count_callbacks(), 0);

  // A timeout schedules the abandon timer.
  fake_regular_timer_trigger(&session->timeout_timer);
  fake_system_task_callbacks_invoke_pending();
  cl_assert(regular_timer_is_scheduled(&session->abandon_timer));
  fake_regular_timer_trigger(&session->abandon_timer);
  cl_assert_equal_i(fake_system_task_count_callbacks(), 1);

  // Session goes away before the abandon callback runs; it must not free twice.
  blob_db_sync_cancel(session);
  fake_system_task_callbacks_invoke_pending();
  cl_assert(blob_db_sync_get_session_for_id(BlobDBIdTest) == NULL);
}

// A stale id must never resolve to a *different* session that happened to reuse
// the freed allocation.
void test_blob_db_sync__stale_callback_does_not_match_new_session(void) {
  BlobDBSyncSession *session = prv_start_sync_with_two_keys();
  const uint32_t first_id = session->session_id;

  fake_system_task_callbacks_cleanup();
  fake_regular_timer_trigger(&session->timeout_timer);
  blob_db_sync_cancel(session);

  // A fresh session for the same db, potentially at the same address.
  BlobDBSyncSession *new_session = prv_start_sync_with_two_keys();
  cl_assert(new_session->session_id != first_id);

  s_num_writebacks = 0;
  fake_system_task_callbacks_invoke_pending();

  // The new session syncs to completion; the stale timeout must not have
  // driven it (which would double-send the first item).
  cl_assert_equal_i(s_num_writebacks, 2);
}

static void prv_fill_stop_return_session(BlobDBId id) {
  char *keys[] = { "key1", "key2", "key3", "key4", "key5" };
  int key_len = strlen(keys[0]);
  char *values[] = { "val1", "val2", "val3", "val4", "val5" };
  int value_len = strlen(values[0]);

  fake_blob_db_set_id(id);
  blob_db_init_dbs();
  for (int i = 0; i < ARRAY_LENGTH(keys); ++i) {
    blob_db_insert(id, (uint8_t *)keys[i], key_len, (uint8_t *)values[i], value_len);
  }
  s_num_writebacks = 0;
  cl_assert(blob_db_sync_db(id) == S_SUCCESS);
}

void test_blob_db_sync__find_session(void) {
  // create a few sync sessions
  prv_fill_stop_return_session(BlobDBIdTest);
  prv_fill_stop_return_session(BlobDBIdPins);
  prv_fill_stop_return_session(BlobDBIdReminders);

  // check we can conjure them by id
  BlobDBSyncSession *test_session = blob_db_sync_get_session_for_id(BlobDBIdTest);
  cl_assert(test_session);
  cl_assert_equal_i(test_session->db_id, BlobDBIdTest);
  BlobDBSyncSession *pins_session = blob_db_sync_get_session_for_id(BlobDBIdPins);
  cl_assert(pins_session);
  cl_assert_equal_i(pins_session->db_id, BlobDBIdPins);
  BlobDBSyncSession *reminders_session = blob_db_sync_get_session_for_id(BlobDBIdReminders);
  cl_assert(reminders_session);
  cl_assert_equal_i(reminders_session->db_id, BlobDBIdReminders);

  test_session->current_token = 1;
  pins_session->current_token = 2;
  reminders_session->current_token = 3;

  // check we can conjure them by token
  cl_assert(test_session == blob_db_sync_get_session_for_token(1));
  cl_assert(pins_session == blob_db_sync_get_session_for_token(2));
  cl_assert(reminders_session == blob_db_sync_get_session_for_token(3));

  // Cancel the sync sessions so they get cleaned up
  blob_db_sync_cancel(test_session);
  blob_db_sync_cancel(pins_session);
  blob_db_sync_cancel(reminders_session);

  // reset fake blob db so cleanup doesn't assert
  fake_blob_db_set_id(BlobDBIdTest);
  blob_db_init_dbs();
}

