/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "api_types.h"

#include <stdint.h>
#include <stdbool.h>

#include "system/status_codes.h"
#include "pbl/util/attributes.h"
#include "pbl/util/list.h"
#include "util/time/time.h"

//! The BlobDB API is a single consistent API to a number of key/value stores on the watch.
//! It is used in conjunction with the BlobDB endpoint.
//! Key/Value stores that are meant to be used with the endpoint need to implement this API
//! by implementing each of the Impl functions (see below).
//! A BlobDB is not guaranteed to persist across reboots, but it is guaranteed to
//! have executed a command when it returns a success code.
//! If you want to route commands to your BlobDB implementation API, you need
//! to add it to the \ref BlobDBId enum and to the BlobDBs list (\ref s_blob_dbs) in api.c

typedef enum PACKED {
  BlobDBIdTest = 0x00,
  BlobDBIdPins = 0x01,
  BlobDBIdApps = 0x02,
  BlobDBIdReminders = 0x03,
  BlobDBIdNotifs = 0x04,
  BlobDBIdWeather = 0x05,
  BlobDBIdiOSNotifPref = 0x06,
  BlobDBIdPrefs = 0x07,
  BlobDBIdContacts = 0x08,
  BlobDBIdWatchAppPrefs = 0x09,
  BlobDBIdHealth = 0x0A,
  BlobDBIdAppGlance = 0x0B,
  BlobDBIdSettings = 0x0C,
  NumBlobDBs,
} BlobDBId;
_Static_assert(sizeof(BlobDBId) == 1, "BlobDBId is larger than 1 byte");

//! A linked list of blob DB items that need to be synced
typedef struct {
  ListNode node;
  time_t last_updated;
  int key_len; //!< length of the key, in bytes
  uint8_t key[]; //!< key_len-size byte array of key data
} BlobDBDirtyItem;

//! A Blob DB's initialization routine.
//! This function will be called at boot when all blob dbs are init-ed
typedef void (*BlobDBInitImpl)(void);

//! Implements the insert API. Note that this function should be blocking.
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \param val a pointer to the value data
//! \param val_len the length of the value, in bytes
//! \returns S_SUCCESS if the key/val pair was successfully inserted
//! and an error code otherwise (See \ref StatusCode)
typedef status_t (*BlobDBInsertImpl)
    (const uint8_t *key, int key_len, const uint8_t *val, int val_len);

//! Implements the get length API.
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \returns the length in bytes of the value for key on success
//! and an error code otherwise (See \ref StatusCode)
typedef int (*BlobDBGetLenImpl)
    (const uint8_t *key, int key_len);

//! Implements the read API. Note that this function should be blocking.
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \param[out] val_out a pointer to a buffer of size val_len
//! \param val_len the length of the value to be copied, in bytes
//! \returns S_SUCCESS if the value for key was successfully read,
//! and an error code otherwise (See \ref StatusCode)
typedef status_t (*BlobDBReadImpl)
    (const uint8_t *key, int key_len, uint8_t *val_out, int val_len);

//! Implements the delete API. Note that this function should be blocking.
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \returns S_SUCCESS if the key/val pair was successfully deleted
//! and an error code otherwise (See \ref StatusCode)
typedef status_t (*BlobDBDeleteImpl)
    (const uint8_t *key, int key_len);

//! Implements the flush API. Note that this function should be blocking.
//! \returns S_SUCCESS if all key/val pairs were successfully deleted
//! and an error code otherwise (See \ref StatusCode)
typedef status_t (*BlobDBFlushImpl)(void);

//! Implements the IsDirty API.
//! \param[out] is_dirty_out reference to a boolean that will be set depending on the DB state
//! \note if the function does not return S_SUCCESS, the state of the boolean is undefined.
//! \returns S_SUCCESS if the query succeeded, an error code otherwise
typedef status_t (*BlobDBIsDirtyImpl)(bool *is_dirty_out);

//! Implements the GetDirtyList API.
//! \return a linked list of \ref BlobDBDirtyItem with a node per out-of-sync item.
//! \note There is no limit how large this list can get. Handle OOM scenarios gracefully!
typedef BlobDBDirtyItem *(*BlobDBGetDirtyListImpl)(void);

//! Implements the MarkSynced API.
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \returns S_SUCCESS if the item was marked synced, an error code otherwise
typedef status_t (*BlobDBMarkSyncedImpl)(const uint8_t *key, int key_len);

//! Implements the Compact API. Reclaims unused space in the underlying
//! settings file. Note that this function should be blocking; only blob DBs
//! backed by a settings_file need to implement this.
//! \returns S_SUCCESS on success, an error code otherwise
typedef status_t (*BlobDBCompactImpl)(void);

//! Emits a Blob DB event.
//! \param type The type of event to emit
//! \param db_id the ID of the blob DB
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
void blob_db_event_put(BlobDBEventType type, BlobDBId db_id, const uint8_t *key, int key_len);

//! Call the BlobDBInitImpl for all the databases
void blob_db_init_dbs(void);

//! Call the BlobDBCompactImpl for every database that implements one. Used to
//! reclaim space in growable settings_file-backed databases. Must be called
//! after blob_db_init_dbs(). Safe to call from a system task callback; do not
//! call from the kernel main loop as compaction performs disk I/O.
void blob_db_compact_growable_dbs(void);

//! Call the BlobDBIsDirtyImpl for each database, and fill the 'ids' list
//! with all the dirty DB ids
//! \param[out] ids an array of BlobDbIds of size NumBlobDBs or more.
//! \param[out] num_ids an array of BlobDbIds of size NumBlobDBs or more.
//! \note The unused entries will be set to 0.
void blob_db_get_dirty_dbs(uint8_t *ids, uint8_t *num_ids);

//! Insert a key/val pair in a blob DB.
//! See \ref BlobDBReadImpl
//! \param db_id the ID of the blob DB
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \param val a pointer to the value data
//! \param val_len the length of the value, in bytes
status_t blob_db_insert(BlobDBId db_id,
    const uint8_t *key, int key_len, const uint8_t *val, int val_len);

//! Get the length of the value in a blob DB for a given key.
//! See \ref BlobDBGetLenImpl
//! \param db_id the ID of the blob DB
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
int blob_db_get_len(BlobDBId db_id,
    const uint8_t *key, int key_len);

//! Get the value of length val_len for a given key
//! \param db_id the ID of the blob DB
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \param val_out a buffer to store the value data
//! \param val_len the length of the value, in bytes
//! See \ref BlobDBReadImpl
status_t blob_db_read(BlobDBId db_id,
    const uint8_t *key, int key_len, uint8_t *val_out, int val_len);

//! Delete the key/val pair in a blob DB for a given key
//! \param db_id the ID of the blob DB
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! See \ref BlobDBDeleteImpl
status_t blob_db_delete(BlobDBId db_id,
    const uint8_t *key, int key_len);

//! Delete all key/val pairs in a blob DB.
//! \param db_id the ID of the blob DB
//! See \ref BlobDBFlushImpl
status_t blob_db_flush(BlobDBId db_id);

//! Get the list of items in a given blob DB that have yet to be synced.
//! Items originating from the phone are always marked as synced.
//! \note Use the APIs in sync.h to initiate a sync.
//! \param db_id the ID of the blob DB
//! \see BlobDBGetDirtyListImpl
BlobDBDirtyItem *blob_db_get_dirty_list(BlobDBId db_id);

//! Mark an item in a blob DB as having been synced
//! \note This API is used upon receiving an ACK from the phone during sync
//! \param db_id the ID of the blob DB
//! \param key a pointer to the key data
//! \param key_len the length of the key, in bytes
//! \see BlobDBMarkSyncedImpl
status_t blob_db_mark_synced(BlobDBId db_id, uint8_t *key, int key_len);
