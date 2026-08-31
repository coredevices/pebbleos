/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/events.h"

#include <stddef.h>

#define SHELL_PREFS_FILE_NAME "shellpref"
// TODO: PBL-42170 Improve settings file to better utilize large sector sizes
// We want to use a full page, but requesting such a size can end up using two pages
#define SHELL_PREFS_FILE_LEN (2048)


//! Update the backing store for the given preference.
//! @param[in] key the preference's name, as defined in prefs.c
//! @param[in] key_len the length of key
//! @param[in] value pointer to the new value
//! @param[in] value_len the length of the value
//! @return true on success, false if failure
bool prefs_private_write_backing(const uint8_t *key, size_t key_len, const void *value,
                               int value_len);

//! Get the length of a preference's value as stored in the backing store
//! @param[in] key the preference's name, as defined in prefs.c
//! @param[in] key_len the length of key
//! @return the >0 length upon success, or 0 if the preference was not found
int prefs_private_get_backing_len(const uint8_t *key, size_t key_len);

//! Read the value of a preference from the backing store.
//! @param[in] key the preference's name, as defined in prefs.c
//! @param[in] key_len the length of key
//! @param[out] value the value will be written into this pointer
//! @param[in] value_len the length of the value
//! @return true on success, false if failure
bool prefs_private_read_backing(const uint8_t *key, size_t key_len, void *value, int value_len);

//! Lock the prefs mutex. Must be paired with prefs_private_unlock().
void prefs_private_lock(void);

//! Unlock the prefs mutex. Must be paired with prefs_private_lock().
void prefs_private_unlock(void);

//! Process a blobDB event issued for the prefs DB (BlobDBIdPrefs). For BlobDBEventTypeInsert
//! events, this  method will update the internal global copy of that preference based on the
//! new value that was placed into the backing store.
//! @param[in] event pointer to the blob DB event
void prefs_private_handle_blob_db_event(PebbleBlobDBEvent *event);
