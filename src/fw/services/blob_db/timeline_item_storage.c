/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/timeline_item_storage.h"

#include <pbl/drivers/rtc.h>
#include "kernel/pbl_malloc.h"
#include "pbl/services/settings/settings_raw_iter.h"
#include <pbl/logging/logging.h>
#include "util/units.h"

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

#define MAX_CHILDREN_PER_PIN 3

// FIRM-1649: temporary instrumentation to catch long mutex hold/wait on the
// timeline item storage mutex. Suspected root cause of multi-second KernelMain
// stalls during QuickView/alarm-save flows is settings_file compaction running
// under this mutex on KernelBackground.
#define FIRM_1649_MUTEX_WARN_MS 100

static RtcTicks prv_storage_lock(TimelineItemStorage *storage, const char *op) {
  RtcTicks before = rtc_get_ticks();
  mutex_lock(storage->mutex);
  RtcTicks after = rtc_get_ticks();
  uint32_t wait_ms = (uint32_t)(((after - before) * 1000) / RTC_TICKS_HZ);
  if (wait_ms >= FIRM_1649_MUTEX_WARN_MS) {
    PBL_LOG_WRN("FIRM-1649: %s waited %"PRIu32"ms for %s mutex",
                op, wait_ms, storage->name);
  }
  return after;
}

static void prv_storage_unlock(TimelineItemStorage *storage, RtcTicks lock_ticks,
                               const char *op) {
  uint32_t hold_ms =
      (uint32_t)(((rtc_get_ticks() - lock_ticks) * 1000) / RTC_TICKS_HZ);
  mutex_unlock(storage->mutex);
  if (hold_ms >= FIRM_1649_MUTEX_WARN_MS) {
    PBL_LOG_WRN("FIRM-1649: %s held %s mutex for %"PRIu32"ms",
                op, storage->name, hold_ms);
  }
}

typedef struct {
  Uuid parent_id;
  Uuid children_ids[MAX_CHILDREN_PER_PIN];
  int num_children;
  bool find_all;
} FindChildrenInfo;

typedef struct {
  time_t current;
  time_t best;
  Uuid id;
  uint32_t max_age;
  TimelineItemStorageFilterCallback filter_cb;
  bool found;
} NextInfo;

typedef struct {
  time_t earliest;
} GcInfo;

typedef struct {
  bool empty;
} AnyInfo;

// callback for settings_file_each that finds the first item by timestamp
static bool prv_each_first_item(SettingsFile *file, SettingsRecordInfo *info,
  void *context) {
  if (info->val_len < (int)sizeof(SerializedTimelineItemHeader) ||
      info->key_len != UUID_SIZE) { // deleted or malformed values
    if (info->key_len != UUID_SIZE) {
      PBL_LOG_WRN("Found reminder with invalid key size %d; ignoring.",
        info->key_len);
    }
    return true;
  }

  SerializedTimelineItemHeader hdr;
  info->get_val(file, &hdr, sizeof(SerializedTimelineItemHeader));
  // Restore flags & status
  hdr.common.flags = ~hdr.common.flags;
  hdr.common.status = ~hdr.common.status;

  NextInfo *next_info = (NextInfo *)context;
  time_t timestamp = timeline_item_get_tz_timestamp(&hdr.common);

  // check if filter callback exists, then check if we should keep the item.
  bool filtered_out = false;
  if (next_info->filter_cb && !next_info->filter_cb(&hdr, context)) {
    filtered_out = true;
  }

  if (!filtered_out &&
      (timestamp < next_info->best || !next_info->found) &&
      (timestamp >= (int)(next_info->current - next_info->max_age))) {
    next_info->found = true;
    next_info->best = timestamp;
    info->get_key(file, (uint8_t *)&next_info->id, sizeof(Uuid));
  }

  return true; // continue iterating
}

static bool prv_each_any_item(SettingsFile *file, SettingsRecordInfo *info, void *context) {
  if (info->val_len < (int)sizeof(SerializedTimelineItemHeader) ||
      info->key_len != UUID_SIZE) {
    return true; // continue looking
  }

  AnyInfo *anyinfo = context;
  anyinfo->empty = false;

  return false; // we found a valid entry
}

static bool prv_each_find_children(SettingsFile *file, SettingsRecordInfo *info,
  void *context) {
  FindChildrenInfo *find_info = (FindChildrenInfo *)context;
  if (info->val_len < (int)sizeof(SerializedTimelineItemHeader) ||
      info->key_len != UUID_SIZE) {
    // malformed values; deleted values have their lengths set to 0
    if (info->key_len != UUID_SIZE) {
      PBL_LOG_WRN("Found malformed item with invalid key/val sizes; ignoring.");
    }
    return true;
  }

  SerializedTimelineItemHeader hdr;
  info->get_val(file, &hdr, sizeof(SerializedTimelineItemHeader));
  // Restore flags & status
  hdr.common.flags = ~hdr.common.flags;
  hdr.common.status = ~hdr.common.status;
  if (uuid_equal(&find_info->parent_id, &hdr.common.parent_id)) {
    find_info->children_ids[find_info->num_children++] = hdr.common.id;
  }

  // continue iterating if more children are expected and we want them all
  return ((find_info->num_children < MAX_CHILDREN_PER_PIN) && find_info->find_all);
}

///////////////////////////////////
// Public API
///////////////////////////////////

bool timeline_item_storage_is_empty(TimelineItemStorage *storage) {
  bool rv = true;

  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  AnyInfo any_info = { .empty = true };
  status_t status = settings_file_each(&storage->file, prv_each_any_item, &any_info);
  if (status) {
    goto cleanup;
  }

  rv = any_info.empty;

cleanup:
  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

status_t timeline_item_storage_next_item(TimelineItemStorage *storage, Uuid *id_out,
    TimelineItemStorageFilterCallback filter_cb) {
  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  NextInfo next_info = {0};
  next_info.current = rtc_get_time();
  next_info.max_age = storage->max_item_age;
  next_info.filter_cb = filter_cb;

  status_t rv = settings_file_each(&storage->file, prv_each_first_item, &next_info);
  if (rv) {
    goto cleanup;
  }

  if (!next_info.found) {
    rv = S_NO_MORE_ITEMS;
    goto cleanup;
  }

  *id_out = next_info.id;
  rv = S_SUCCESS;

cleanup:
  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}


bool timeline_item_storage_exists_with_parent(TimelineItemStorage *storage, const Uuid *parent_id) {
  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  FindChildrenInfo info = {
    .parent_id = *parent_id,
    .num_children = 0,
    .find_all = false,
  };
  status_t rv = settings_file_each(&storage->file, prv_each_find_children, &info);
  if (rv) {
    goto cleanup;
  }

  if (info.num_children) {
    rv = S_SUCCESS;
  } else {
    rv = S_NO_MORE_ITEMS;
  }

cleanup:
  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv == S_SUCCESS;
}

status_t timeline_item_storage_delete_with_parent(
    TimelineItemStorage *storage,
    const Uuid *parent_id,
    TimelineItemStorageChildDeleteCallback child_delete_cb) {
  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  FindChildrenInfo info = {
    .parent_id = *parent_id,
    .num_children = 0,
    .find_all = true,
  };
  status_t rv = settings_file_each(&storage->file, prv_each_find_children, &info);
  if (rv) {
    goto cleanup;
  }

  for (int i = 0; i < info.num_children; ++i) {
    const void *key = &info.children_ids[i];
    rv = settings_file_delete(&storage->file, key, sizeof(Uuid));

    if (rv != S_SUCCESS) {
      goto cleanup;
    }

    if (child_delete_cb) {
      child_delete_cb((Uuid *)key);
    }
  }

cleanup:
  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

//! Caution: CommonTimelineItemHeader .flags & .status are stored inverted and not auto-restored
status_t timeline_item_storage_each(TimelineItemStorage *storage,
    TimelineItemStorageEachCallback each, void *data) {
  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);
  status_t rv = settings_file_each(&storage->file, each, data);
  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

void timeline_item_storage_init(TimelineItemStorage *storage,
    char *filename, uint32_t max_size, uint32_t max_age) {
  *storage = (TimelineItemStorage){
    .name = filename,
    .max_size = max_size,
    .max_item_age = max_age,
    .mutex = mutex_create(),
  };
  status_t rv = settings_file_open_growable(&storage->file, storage->name,
                                            storage->max_size, KiBYTES(8));
  if (FAILED(rv)) {
    PBL_LOG_ERR("Unable to create settings file %s, rv = %"PRId32 "!",
            filename, rv);
  }
}

void timeline_item_storage_deinit(TimelineItemStorage *storage) {
  settings_file_close(&storage->file);
}

status_t timeline_item_storage_compact(TimelineItemStorage *storage) {
  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);
  status_t rv = settings_file_compact(&storage->file);
  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

status_t timeline_item_storage_insert(TimelineItemStorage *storage,
    const uint8_t *key, int key_len, const uint8_t *val, int val_len, bool mark_as_synced) {
  if (key_len != UUID_SIZE ||
      val_len > SETTINGS_VAL_MAX_LEN ||
      val_len < (int)sizeof(SerializedTimelineItemHeader)) {
    return E_INVALID_ARGUMENT;
  }

  // Check that the layout has the correct items
  if (!timeline_item_verify_layout_serialized(val, val_len)) {
    PBL_LOG_WRN("Timeline item does not have the correct attributes");
    return E_INVALID_ARGUMENT;
  }

  SerializedTimelineItemHeader *hdr = (SerializedTimelineItemHeader *)val;
  // verify that the item isn't too old
  time_t now = rtc_get_time();
  time_t timestamp = timeline_item_get_tz_timestamp(&hdr->common);
  time_t end_timestamp = timestamp + hdr->common.duration * SECONDS_PER_MINUTE;
  if (end_timestamp < (int)(now - storage->max_item_age)) {
    PBL_LOG_WRN("Rejecting stale timeline item %ld seconds old",
      now - timestamp);
    return E_INVALID_OPERATION;
  }

  // FIXME: PBL-39523 timeline_item_storage_insert modifies a const buffer
  // Invert flags & status to store on flash
  hdr->common.flags = ~hdr->common.flags;
  hdr->common.status = ~hdr->common.status;

  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);
  status_t rv = settings_file_set(&storage->file, key, key_len, val, val_len);

  // Restore flags & status
  hdr->common.flags = ~hdr->common.flags;
  hdr->common.status = ~hdr->common.status;

  if (mark_as_synced) {
    settings_file_mark_synced(&storage->file, key, key_len);
  }

  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

int timeline_item_storage_get_len(TimelineItemStorage *storage,
    const uint8_t *key, int key_len) {
  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  status_t rv = settings_file_get_len(&storage->file, key, key_len);

  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

status_t timeline_item_storage_read(TimelineItemStorage *storage,
    const uint8_t *key, int key_len, uint8_t *val_out, int val_len) {
  if (key_len != UUID_SIZE) {
    return E_INVALID_ARGUMENT;
  }

  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  status_t rv = settings_file_get(&storage->file, key, key_len, val_out, val_len);

  // Restore flags & status
  SerializedTimelineItemHeader *hdr = (SerializedTimelineItemHeader *)val_out;
  hdr->common.flags = ~hdr->common.flags;
  hdr->common.status = ~hdr->common.status;

  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

status_t timeline_item_storage_get_from_settings_record(SettingsFile *file,
                                                        SettingsRecordInfo *info,
                                                        TimelineItem *item) {
  uint8_t *read_buf = kernel_zalloc_check(info->val_len);
  info->get_val(file, read_buf, info->val_len);

  SerializedTimelineItemHeader *header = (SerializedTimelineItemHeader *)read_buf;
  // Restore flags & status
  header->common.flags = ~header->common.flags;
  header->common.status = ~header->common.status;
  uint8_t *payload = read_buf + sizeof(SerializedTimelineItemHeader);
  status_t rv = timeline_item_deserialize_item(item, header, payload) ? S_SUCCESS : E_INTERNAL;

  kernel_free(read_buf);
  return rv;
}

status_t timeline_item_storage_set_status_bits(TimelineItemStorage *storage,
    const uint8_t *key, int key_len, uint8_t status) {
  if (key_len != UUID_SIZE) {
    return E_INVALID_ARGUMENT;
  }

  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  int offset = offsetof(SerializedTimelineItemHeader, common.status);
  // Invert status to store on flash
  status = ~status;
  status_t rv = settings_file_set_byte(&storage->file, key, key_len, offset, status);

  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

status_t timeline_item_storage_delete(TimelineItemStorage *storage,
  const uint8_t *key, int key_len) {
  if (key_len != UUID_SIZE) {
    return E_INVALID_ARGUMENT;
  }

  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  status_t rv = settings_file_delete(&storage->file, key, key_len);

  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

status_t timeline_item_storage_mark_synced(TimelineItemStorage *storage,
                                           const uint8_t *key, int key_len) {
  if (key_len == 0) {
    return E_INVALID_ARGUMENT;
  }

  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);

  status_t rv = settings_file_mark_synced(&storage->file, key, key_len);

  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}

static void prv_flush_rewrite_cb(SettingsFile *old,
                                 SettingsFile *new,
                                 SettingsRecordInfo *info,
                                 void *context) {
  if ((unsigned)info->key_len != sizeof(Uuid) ||
      (unsigned)info->val_len < sizeof(SerializedTimelineItemHeader)) {
    // invalid
    return;
  }

  SerializedTimelineItemHeader hdr;
  info->get_val(old, &hdr, sizeof(SerializedTimelineItemHeader));
  // Restore flags & status
  hdr.common.flags = ~hdr.common.flags;
  hdr.common.status = ~hdr.common.status;

  // keep watch-only items
  if (hdr.common.flags & TimelineItemFlagFromWatch) {
    // fetch the whole item
    uint8_t *val = kernel_malloc_check(info->val_len);
    uint8_t *key = kernel_malloc_check(info->key_len);
    info->get_val(old, val, info->val_len);
    info->get_key(old, key, info->key_len);

    // Don't restore flags & status here - we're writing it back immediately.

    // write it to the new file
    settings_file_set(new, key, info->key_len, val, info->val_len);
    kernel_free(key);
    kernel_free(val);
  }
}

status_t timeline_item_storage_flush(TimelineItemStorage *storage) {
  RtcTicks lock_ticks = prv_storage_lock(storage, __func__);
  status_t rv = settings_file_rewrite(&storage->file, prv_flush_rewrite_cb, NULL);
  prv_storage_unlock(storage, lock_ticks, __func__);
  return rv;
}
