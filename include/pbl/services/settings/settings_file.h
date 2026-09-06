/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "settings_raw_iter.h"

#include <time.h>

// Deleted records have their key stick around for at least DELETED_LIFETIME
// before they can be garbage collected from the file in which they are
// contained, that way they have time to propagate to all devices we end up
// synchronizing with. For more information, refer to the sync protocol proposal:
// https://pebbletechnology.atlassian.net/wiki/pages/viewpage.action?pageId=26837564
//
// FIXME: See PBL-18945
#define DELETED_LIFETIME (0 * SECONDS_PER_DAY)

//! A SettingsFile is just a simple binary key-value store. Keys can be strings,
//! uint32_ts, or arbitrary bytes. Values are similarly flexible. All
//! operations are atomic, so a reboot in the middle of changing the value for a
//! key will always either complete, returning the new value upon reboot, or
//! will just return the old value.
//! It also supports bidirectional syncronization between the phone & watch,
//! using timestamps to resolve conflicts.
//! Note that although all operations are atomic, they are not thread-safe. If
//! you will be accessing a SettingsFile from multiple threads, make sure you
//! use locks!

// NOTE: These fields are internal, modify them at your own risk!
typedef struct SettingsFile {
  SettingsRawIter iter;
  char *name;

  //! Maximum total space which can be used by this settings_file before a
  //! compaction will be forced. (Must be >= max_used_space)
  int max_space_total;

  //! Maximum space that can be used by valid records within this settings_file.
  //! Once this has been exceeded, attempting to add more keys or values will
  //! fail.
  int max_used_space;

  //! The current allocation budget for physical file size. For growable files,
  //! this starts at a small initial value and grows toward max_used_space on
  //! demand. For non-growable files, this equals max_used_space.
  int alloc_used_space;

  //! The floor for alloc_used_space. Compact will not shrink below this. For
  //! growable files this is the initial_alloc_size requested at open time;
  //! for non-growable files this equals max_used_space (no shrink).
  int min_alloc_used_space;

  //! Amount of space in the settings_file that is currently dead, i.e.
  //! has been written to with some data, but that data is no longer valid.
  //! (overwritten records get added to this)
  int dead_space;

  //! Amount of space in the settings_file that is currently used by valid
  //! records.
  int used_space;

  //! When this file as a whole was last_modified.
  //! Defined as records.max(&:last_modified)
  uint32_t last_modified;

  //! The position of the current record in the iteration (if any). Necessary
  //! so that clients can read other records in the middle of iteration (i.e.
  //! settings_file_each()/settings_file_rewrite()),  without messing up the
  //! state of the iteration. Set to 0 if not in use.
  int cur_record_pos;
} SettingsFile;


//! max_used_space should be >= 5317 for persist files to make sure we can
//! always fit all of the records in the worst case (if the programmer stored
//! nothing but booleans).
//! Note: If the settings file already exists, the max_used_space parameter is
//! ignored. We could change this if the need arises.
status_t settings_file_open(SettingsFile *file, const char *name,
                            int max_used_space);
status_t settings_file_open_growable(SettingsFile *file, const char *name,
                                     int max_used_space, int initial_alloc_size);
void settings_file_close(SettingsFile *file);

bool settings_file_exists(SettingsFile *file, const void *key, size_t key_len);
status_t settings_file_delete(SettingsFile *file,
                              const void *key, size_t key_len);

int settings_file_get_len(SettingsFile *file, const void *key, size_t key_len);
//! val_out_len must exactly match the length of the record on disk.
status_t settings_file_get(SettingsFile *file, const void *key, size_t key_len,
                           void *val_out, size_t val_out_len);
status_t settings_file_set(SettingsFile *file, const void *key, size_t key_len,
                           const void *val, size_t val_len);

//! Set a record with a specific timestamp instead of the current time.
//! This is useful when rewriting files and preserving original timestamps.
//! @param file the settings_file in which to set the value
//! @param key the key to set
//! @param key_len the length of the key
//! @param val the value to set
//! @param val_len the length of the value
//! @param timestamp the timestamp to use for the record
status_t settings_file_set_with_timestamp(SettingsFile *file, const void *key, size_t key_len,
                                          const void *val, size_t val_len, uint32_t timestamp);

//! Mark a record as synced. The flag will remain until the record is overwritten
//! @param file the settings_file that contains the record
//! @param key the key to the settings file. Note: keys can be up to 127 bytes
//! @param key_len the length of the key
status_t settings_file_mark_synced(SettingsFile *file, const void *key, size_t key_len);

//! Mark all records as dirty (not synced) by rewriting the file.
//! This is used to trigger a full sync of all settings.
//! @param file the settings_file to mark dirty
//! @note This rewrites the entire file, which can be slow for large files.
status_t settings_file_mark_all_dirty(SettingsFile *file);

//! Callback invoked when a setting is changed via settings_file_set
//! @param file the settings file that was modified
//! @param key the key that was set
//! @param key_len the length of the key
//! @param last_modified the timestamp of the change
typedef void (*SettingsFileChangeCallback)(SettingsFile *file, const void *key, int key_len,
                                           time_t last_modified);

//! Register a callback to be invoked when settings change
//! @param callback the callback to register (NULL to unregister)
void settings_file_set_change_callback(SettingsFileChangeCallback callback);

//! set a byte in a setting. This can only be used a byte at a time to guarantee
//! atomicity. Do not use to modify several bytes in a row!
//! Note that only the reset bits will be applied (it writes flash directly)
status_t settings_file_set_byte(
    SettingsFile *file, const void *key, size_t key_len,
    size_t offset, uint8_t byte);



  //////////////////
 // Each/rewrite //
//////////////////
typedef void (*SettingsFileGetter)(SettingsFile *file,
                                   void *buf, size_t buf_len);

typedef struct {
  uint32_t last_modified;
  SettingsFileGetter get_key;
  int key_len;
  SettingsFileGetter get_val;
  int val_len;
  bool dirty; // has the dirty flag set
} SettingsRecordInfo;

//! Callback used for using settings_file_each.
//! The bool returned is used to control the iteration.
//! - If a callback returns true, the iteration continues
//! - If a callback returns false, the iteration stops.
typedef bool (*SettingsFileEachCallback)(SettingsFile *file,
                                         SettingsRecordInfo *info,
                                         void *context);
//! Calls cb for each and every entry within the given file.
//! Note that you cannot modify the settings file while iterating. If you want
//! to do this, try settings_file_rewrite instead. (you can read other entries
//! without fault).
status_t settings_file_each(SettingsFile *file, SettingsFileEachCallback cb,
                            void *context);


typedef void (*SettingsFileRewriteCallback)(SettingsFile *old_file,
                                            SettingsFile *new_file,
                                            SettingsRecordInfo *info,
                                            void *context);
//! Opens a new SettingsFile with the same name as the original SettingsFile,
//! in overwrite mode. This new file is passed into the given
//! SettingsFileRewriteCallback, which is called for each entry within the
//! original file. If you desire to preserve a key/value pair, you must write
//! it to the new file.
status_t settings_file_rewrite(SettingsFile *file,
                               SettingsFileRewriteCallback cb,
                               void *context);


//! Callback used for using settings_file_rewrite_filtered.
//! The bool returned is used to control whether or not the record is included in the file
//! after compaction. This callback is not allowed to use any other settings_file calls.
//! - If callback returns true, the record is included
//! - If callback returns false, the record is not included
typedef bool (*SettingsFileRewriteFilterCallback)(void *key, size_t key_len, void *value,
                                                  size_t value_len, void *context);

//! Opens a new SettingsFile with the same name as the original SettingsFile,
//! in overwrite mode. Any records from the old file which pass through the filter_cb with
//! a true result are included into the new file. This call is much faster than using
//! settings_file_rewrite if all you are doing is excluding specific records from the old file.
status_t settings_file_rewrite_filtered(SettingsFile *file,
                                        SettingsFileRewriteFilterCallback filter_cb, void *context);

//! Compact the file: rewrite all live records and, for growable files, drop
//! alloc_used_space toward min_alloc_used_space. Useful for shrinking growable
//! settings files that grew under load and have since had records removed.
status_t settings_file_compact(SettingsFile *file);
