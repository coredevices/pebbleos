/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/sync.h"
#include "pbl/services/blob_db/endpoint.h"
#include "pbl/services/blob_db/util.h"

#include "kernel/pbl_malloc.h"
#include "pbl/services/comm_session/session.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "pbl/util/list.h"

#include <stdlib.h>

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);


#define SYNC_TIMEOUT_SECONDS 30
#define SYNC_ABANDON_TIMEOUT_SECONDS (5 * 60)  // 5 minutes to fully abandon

static BlobDBSyncSession *s_sync_sessions = NULL;

//! Ids are handed out monotonically and never reused, so a stale id can never
//! resolve to a different session that happens to reuse the same allocation.
static uint32_t s_next_session_id = 1;

static void prv_send_writeback(BlobDBSyncSession *session);

static bool prv_session_uid_filter_callback(ListNode *node, void *data) {
  BlobDBSyncSession *session = (BlobDBSyncSession *)node;
  return session->session_id == (uint32_t)(uintptr_t)data;
}

//! The regular timers below hop to KernelBG before touching the session, and by
//! then the sync may have finished or been cancelled and the session freed.
//! Resolve the id against the live list instead of trusting a raw pointer.
static BlobDBSyncSession *prv_find_live_session(void *session_id) {
  return (BlobDBSyncSession *)list_find((ListNode *)s_sync_sessions,
                                        prv_session_uid_filter_callback,
                                        session_id);
}

static bool prv_session_id_filter_callback(ListNode *node, void *data) {
  BlobDBId db_id = (BlobDBId)data;
  BlobDBSyncSession *session = (BlobDBSyncSession *)node;
  if (session->session_type == BlobDBSyncSessionTypeRecord) {
    return false;
  }
  return session->db_id == db_id;
}

static bool prv_session_token_filter_callback(ListNode *node, void *data) {
  uint16_t token = (uint16_t)(uintptr_t)data;
  BlobDBSyncSession *session = (BlobDBSyncSession *)node;
  return session->current_token == token;
}

static void prv_abandon_kernelbg_callback(void *data) {
  BlobDBSyncSession *session = prv_find_live_session(data);
  if (!session) {
    // Sync finished or was cancelled between the timer firing and this callback
    return;
  }
  PBL_LOG_WRN("Blob DB Sync abandoned after extended timeout");
  blob_db_sync_cancel(session);
}

static void prv_abandon_timer_callback(void *data) {
  system_task_add_callback(prv_abandon_kernelbg_callback, data);
}

static void prv_timeout_kernelbg_callback(void *data) {
  BlobDBSyncSession *session = prv_find_live_session(data);
  if (!session) {
    return;
  }

  // Start the abandon timer if not already running
  if (!regular_timer_is_scheduled(&session->abandon_timer)) {
    regular_timer_add_multisecond_callback(&session->abandon_timer, SYNC_ABANDON_TIMEOUT_SECONDS);
  }

  // Retry sending the current item
  PBL_LOG_WRN("Blob DB Sync timeout, retrying (db %d)", session->db_id);
  session->state = BlobDBSyncSessionStateIdle;
  prv_send_writeback(session);
}

static void prv_timeout_timer_callback(void *data) {
  system_task_add_callback(prv_timeout_kernelbg_callback, data);
}

static void prv_send_writeback(BlobDBSyncSession *session) {
  // we want to write-back the first item in the dirty list
  BlobDBDirtyItem *dirty_item = session->dirty_list;
  int item_size = blob_db_get_len(session->db_id, dirty_item->key, dirty_item->key_len);
  if (item_size == 0) {
    // item got removed during the sync. Go to the next one
    blob_db_sync_next(session);
    return;
  }

  if (!comm_session_get_system_session()) {
    PBL_LOG_DBG("Cancelling sync: No route to phone");
    blob_db_sync_cancel(session);
    return;
  }

  // read item into a temporary buffer
  void *item_buf = kernel_malloc_check(item_size);
  status_t status = blob_db_read(session->db_id,
                                 dirty_item->key,
                                 dirty_item->key_len,
                                 item_buf, item_size);
  // Both blob_db_sync_next() and blob_db_sync_cancel() can free the session, so
  // neither it nor dirty_item may be touched after calling them.
  if (status == E_DOES_NOT_EXIST) {
    // item was removed
    kernel_free(item_buf);
    blob_db_sync_next(session);
    return;
  } else if (!PASSED(status)) {
    // something went terribly wrong
    PBL_LOG_ERR("Failed to read blob DB during sync. Error code: 0x%"PRIx32, status);
    kernel_free(item_buf);
    blob_db_sync_cancel(session);
    return;
  }

  regular_timer_add_multisecond_callback(&session->timeout_timer, SYNC_TIMEOUT_SECONDS);

  // only one writeback in flight at a time
  session->state = BlobDBSyncSessionStateWaitingForAck;


  if (session->session_type == BlobDBSyncSessionTypeDB) {
    session->current_token = blob_db_endpoint_send_writeback(session->db_id,
                                                             dirty_item->last_updated,
                                                             dirty_item->key,
                                                             dirty_item->key_len,
                                                             item_buf,
                                                             item_size);
  } else {
    session->current_token = blob_db_endpoint_send_write(session->db_id,
                                                         dirty_item->last_updated,
                                                         dirty_item->key,
                                                         dirty_item->key_len,
                                                         item_buf,
                                                         item_size);
  }

  kernel_free(item_buf);
}

BlobDBSyncSession* prv_create_sync_session(BlobDBId db_id, BlobDBDirtyItem *dirty_list,
                                           BlobDBSyncSessionType session_type) {
  BlobDBSyncSession *session = kernel_zalloc_check(sizeof(BlobDBSyncSession));
  session->state = BlobDBSyncSessionStateIdle;
  session->db_id = db_id;
  session->dirty_list = dirty_list;
  session->session_type = session_type;
  session->session_id = s_next_session_id++;
  if (s_next_session_id == 0) {
    s_next_session_id = 1;  // 0 is reserved as "no session"
  }
  void *session_id_data = (void *)(uintptr_t)session->session_id;
  session->timeout_timer = (const RegularTimerInfo) {
    .cb = prv_timeout_timer_callback,
    .cb_data = session_id_data,
  };
  session->abandon_timer = (const RegularTimerInfo) {
    .cb = prv_abandon_timer_callback,
    .cb_data = session_id_data,
  };
  s_sync_sessions = (BlobDBSyncSession *)list_prepend((ListNode *)s_sync_sessions,
                                                      (ListNode *)session);

  return session;
}

//! Will not return sessions for individual records
BlobDBSyncSession *blob_db_sync_get_session_for_id(BlobDBId db_id) {
  return (BlobDBSyncSession *)list_find((ListNode *)s_sync_sessions,
                                        prv_session_id_filter_callback,
                                        (void *)(uintptr_t)db_id);
}

BlobDBSyncSession *blob_db_sync_get_session_for_token(uint16_t token) {
  return (BlobDBSyncSession *)list_find((ListNode *)s_sync_sessions,
                                        prv_session_token_filter_callback,
                                        (void *)(uintptr_t)token);
}

status_t blob_db_sync_db(BlobDBId db_id) {
  if (db_id >= NumBlobDBs) {
    return E_INVALID_ARGUMENT;
  }
  PBL_LOG_DBG("Starting BlobDB db sync: %d", db_id);

  BlobDBDirtyItem *dirty_list = blob_db_get_dirty_list(db_id);
  if (!dirty_list) {
    blob_db_endpoint_send_sync_done(db_id);
    return S_NO_ACTION_REQUIRED;
  }

  BlobDBSyncSession *session = blob_db_sync_get_session_for_id(db_id);
  if (session) {
    // already have a session in progress!
    return E_BUSY;
  }

  session = prv_create_sync_session(db_id, dirty_list, BlobDBSyncSessionTypeDB);

  prv_send_writeback(session);

  return S_SUCCESS;
}

status_t blob_db_sync_record(BlobDBId db_id, const void *key, int key_len, time_t last_updated) {
  if (db_id >= NumBlobDBs) {
    return E_INVALID_ARGUMENT;
  }

  BlobDBSyncSession *session = blob_db_sync_get_session_for_id(db_id);
  if (session) {
    // This will get picked up by the current session when it is done with its dirty list
    return S_SUCCESS;
  }

  char buffer[key_len + 1];
  strncpy(buffer, (const char *)key, key_len);
  buffer[key_len] = '\0';
  PBL_LOG_DBG("Starting BlobDB record sync: <%s>", buffer);

  BlobDBDirtyItem *dirty_list = kernel_zalloc_check(sizeof(BlobDBDirtyItem) + key_len);
  list_init((ListNode *)dirty_list);
  dirty_list->last_updated = last_updated;
  dirty_list->key_len = key_len;
  memcpy(dirty_list->key, key, key_len);

  session = prv_create_sync_session(db_id, dirty_list, BlobDBSyncSessionTypeRecord);

  prv_send_writeback(session);

  return S_SUCCESS;
}

void blob_db_sync_cancel(BlobDBSyncSession *session) {
  PBL_LOG_DBG("Cancelling session %d sync", session->db_id);
  if (regular_timer_is_scheduled(&session->timeout_timer)) {
    regular_timer_remove_callback(&session->timeout_timer);
  }
  if (regular_timer_is_scheduled(&session->abandon_timer)) {
    regular_timer_remove_callback(&session->abandon_timer);
  }
  blob_db_util_free_dirty_list(session->dirty_list);
  list_remove((ListNode *)session, (ListNode **)&s_sync_sessions, NULL);
  kernel_free(session);
}

void blob_db_sync_next(BlobDBSyncSession *session) {
  PBL_LOG_DBG("blob_db_sync_next");

  // Cancel abandon timer - we got a successful response, so connection is working
  if (regular_timer_is_scheduled(&session->abandon_timer)) {
    regular_timer_remove_callback(&session->abandon_timer);
  }

  BlobDBDirtyItem *dirty_item = session->dirty_list;
  blob_db_mark_synced(session->db_id, dirty_item->key, dirty_item->key_len);

  // we're done with this item, we pop it off the stack
  list_remove((ListNode *)dirty_item, (ListNode **)&session->dirty_list, NULL);
  kernel_free(dirty_item);

  if (session->dirty_list) {
    prv_send_writeback(session);
  } else {
    // Check if new records became dirty while syncing the current list
    // New records could have been added while we were syncing OR
    // the list could be incomplete because we ran out of memory
    session->dirty_list = blob_db_get_dirty_list(session->db_id);
    if (session->dirty_list) {
      prv_send_writeback(session);
    } else {
      PBL_LOG_DBG("Finished syncing db %d, session type: %d", session->db_id,
                                                                          session->session_type);
      if (regular_timer_is_scheduled(&session->timeout_timer)) {
        regular_timer_remove_callback(&session->timeout_timer);
      }
      if (regular_timer_is_scheduled(&session->abandon_timer)) {
        regular_timer_remove_callback(&session->abandon_timer);
      }
      if (session->session_type == BlobDBSyncSessionTypeDB) {
        // Only send the sync done when syncing an entire db
        blob_db_endpoint_send_sync_done(session->db_id);
      }
      list_remove((ListNode *)session, (ListNode **)&s_sync_sessions, NULL);
      kernel_free(session);
    }
  }
}

