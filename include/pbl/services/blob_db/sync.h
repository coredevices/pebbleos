/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "api.h"
#include "endpoint.h"

#include "pbl/services/regular_timer.h"

typedef enum {
  BlobDBSyncSessionStateIdle = 0,
  BlobDBSyncSessionStateWaitingForAck = 1,
} BlobDBSyncSessionState;

typedef enum {
  BlobDBSyncSessionTypeDB,
  BlobDBSyncSessionTypeRecord,
} BlobDBSyncSessionType;

typedef struct {
  ListNode node;
  //! Unique, never reused. Deferred timer callbacks carry this instead of a
  //! session pointer so they can detect a session that has already been freed.
  uint32_t session_id;
  BlobDBSyncSessionState state;
  BlobDBId db_id;
  BlobDBDirtyItem *dirty_list;
  RegularTimerInfo timeout_timer;
  RegularTimerInfo abandon_timer;
  BlobDBToken current_token;
  BlobDBSyncSessionType session_type;
} BlobDBSyncSession;

//! Start sync-ing a blobdb.
//! @param db_id the BlobDBId of the database to sync
status_t blob_db_sync_db(BlobDBId db_id);

//! Start sync-ing a key within a blobdb.
//! @param db_id the BlobDBId of the database to sync
//! @param key the key to sync
//! @param key_len the length of the key to sync
//! @param last_updated the last-updated timestamp of the record
status_t blob_db_sync_record(BlobDBId db_id, const void *key, int key_len, time_t last_updated);

//! Get the sync session for a given ID. Will NOT return sessions for individual records
//! returns NULL if no sync is in progress
BlobDBSyncSession *blob_db_sync_get_session_for_id(BlobDBId db_id);

//! Get the sync session currently waiting for a response with the given token
//! return NULL if no sync is in progress
BlobDBSyncSession *blob_db_sync_get_session_for_token(BlobDBToken token);

//! Mark current item as synced and sync the next one
void blob_db_sync_next(BlobDBSyncSession *session);

//! Cancel the sync in progress. Pending items will be synced next time.
void blob_db_sync_cancel(BlobDBSyncSession *session);
