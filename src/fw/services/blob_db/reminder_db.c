/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/reminder_db.h"
#include "pbl/services/blob_db/sync.h"
#include "pbl/services/blob_db/sync_util.h"
#include "pbl/services/blob_db/timeline_item_storage.h"

#include "pbl/util/uuid.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/timeline/reminders.h"
#include "system/passert.h"
#include <pbl/logging/logging.h>
#include "util/units.h"

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

#define REMINDER_DB_FILE_NAME "reminderdb"
#define REMINDER_DB_MAX_SIZE KiBYTES(40)
#define MAX_REMINDER_SIZE SETTINGS_VAL_MAX_LEN
#define MAX_REMINDER_AGE (15 * SECONDS_PER_MINUTE)

typedef struct {
  TimelineItemStorageFilterCallback filter_cb;
  time_t timestamp;
  const char *title;
  TimelineItem *reminder_out;
  bool match;
} ReminderInfo;

static TimelineItemStorage s_storage;

static status_t prv_read_item_header(TimelineItem *item_out, TimelineItemId *id) {
  SerializedTimelineItemHeader hdr = {{{0}}};
  status_t rv = reminder_db_read((uint8_t *)id, sizeof(TimelineItemId), (uint8_t *)&hdr,
    sizeof(SerializedTimelineItemHeader));
  timeline_item_deserialize_header(item_out, &hdr);
  return rv;
}

/////////////////////////
// Reminder DB specific API
/////////////////////////

status_t reminder_db_delete_with_parent(const TimelineItemId *parent_id) {
  return (timeline_item_storage_delete_with_parent(&s_storage, parent_id,
                                                   reminders_handle_reminder_removed));
}

status_t reminder_db_read_item(TimelineItem *item_out, TimelineItemId *id) {
  size_t size = reminder_db_get_len((uint8_t *)id, sizeof(TimelineItemId));
  if (size == 0) {
    return E_DOES_NOT_EXIST;
  }
  uint8_t *read_buf = kernel_malloc_check(size);
  status_t rv = reminder_db_read((uint8_t *)id, sizeof(TimelineItemId), read_buf, size);
  if (rv != S_SUCCESS) {
    goto cleanup;
  }

  SerializedTimelineItemHeader *header = (SerializedTimelineItemHeader *)read_buf;
  uint8_t *payload = read_buf + sizeof(SerializedTimelineItemHeader);
  if (!timeline_item_deserialize_item(item_out, header, payload)) {
    rv = E_INTERNAL;
    goto cleanup;
  }

  kernel_free(read_buf);
  return S_SUCCESS;

cleanup:
  kernel_free(read_buf);
  return rv;
}

// Only keep reminders that have not been fired yet.
static bool prv_reminder_filter(SerializedTimelineItemHeader *hdr, void *context) {
  return ((TimelineItemStatusReminded & hdr->common.status) == 0);
}

status_t reminder_db_next_item_header(TimelineItem *next_item_out) {
  PBL_LOG_DBG("Finding next item in queue.");
  TimelineItemId id;
  status_t rv = timeline_item_storage_next_item(&s_storage, &id, prv_reminder_filter);
  if (rv) {
    return rv;
  }
  rv = prv_read_item_header(next_item_out, &id);
  return rv;
}

static bool prv_timestamp_title_compare_func(SettingsFile *file, SettingsRecordInfo *info,
                                             void *context) {
  // Check entry is valid
  if (info->key_len != UUID_SIZE || info->val_len == 0) {
    return true; // continue iteration
  }

  // Compare timestamps (this should omit most reminders)
  ReminderInfo *reminder_info = (ReminderInfo *)context;
  SerializedTimelineItemHeader header;
  info->get_val(file, (uint8_t *)&header, sizeof(SerializedTimelineItemHeader));
  if (reminder_info->timestamp != header.common.timestamp) {
    return true; // continue iteration
  }

  // Read the full reminder to compare text
  TimelineItem *reminder = reminder_info->reminder_out;
  if (timeline_item_storage_get_from_settings_record(file, info, reminder) != S_SUCCESS) {
    return true; // continue iteration
  }

  const char *title = attribute_get_string(&reminder->attr_list, AttributeIdTitle, "");
  if (strcmp(title, reminder_info->title) != 0) {
    timeline_item_free_allocated_buffer(reminder);
    return true; // continue iteration
  }

  if (reminder_info->filter_cb && !reminder_info->filter_cb(&header, context)) {
    timeline_item_free_allocated_buffer(reminder);
    return true; // continue iteration
  }

  reminder_info->match = true;
  return false; // stop iteration
}

bool reminder_db_find_by_timestamp_title(time_t timestamp, const char *title,
                                         TimelineItemStorageFilterCallback filter_cb,
                                         TimelineItem *reminder_out) {
  PBL_ASSERTN(reminder_out);

  ReminderInfo reminder_info = {
    .filter_cb = filter_cb,
    .timestamp = timestamp,
    .title = title,
    .reminder_out = reminder_out,
    .match = false
  };

  timeline_item_storage_each(&s_storage, prv_timestamp_title_compare_func, &reminder_info);

  return reminder_info.match;
}

static status_t prv_insert_reminder(const uint8_t *key, int key_len,
                                    const uint8_t *val, int val_len, bool mark_synced) {
  const SerializedTimelineItemHeader *hdr = (const SerializedTimelineItemHeader *)val;
  const bool has_reminded = hdr->common.reminded;

  status_t rv = timeline_item_storage_insert(&s_storage, key, key_len, val, val_len, mark_synced);

  char uuid_buffer[UUID_STRING_BUFFER_LENGTH];
  uuid_to_string((Uuid *)key, uuid_buffer);
  PBL_LOG_DBG("Reminder added: %s", uuid_buffer);

  if (rv == S_SUCCESS) {
    if (has_reminded) {
      reminders_handle_reminder_updated(&hdr->common.id);
    } else {
      rv = reminders_update_timer();
    }
  }
  return rv;
}

status_t reminder_db_insert_item(TimelineItem *item) {
  if (item->header.type != TimelineItemTypeReminder) {
    return E_INVALID_ARGUMENT;
  }

  size_t payload_size = timeline_item_get_serialized_payload_size(item);
  uint8_t *buffer = kernel_malloc_check(sizeof(SerializedTimelineItemHeader) + payload_size);
  timeline_item_serialize_header(item, (SerializedTimelineItemHeader *) buffer);
  timeline_item_serialize_payload(item, buffer + sizeof(SerializedTimelineItemHeader),
    payload_size);

  // only for items without attributes as of right now
  // Records inserted by the watch are dirty and need to be synced to the phone
  const bool mark_synced = false;
  status_t rv = prv_insert_reminder((uint8_t *)&item->header.id, sizeof(TimelineItemId),
    buffer, sizeof(SerializedTimelineItemHeader) + payload_size, mark_synced);

  blob_db_sync_record(BlobDBIdReminders, (uint8_t *)&item->header.id, sizeof(TimelineItemId),
                      rtc_get_time());

  kernel_free(buffer);
  return rv;
}

static status_t prv_reminder_db_delete_common(const uint8_t *key, int key_len) {
  status_t rv = timeline_item_storage_delete(&s_storage, key, key_len);
  if (rv == S_SUCCESS) {
    reminders_update_timer();
  }

  return rv;
}

status_t reminder_db_delete_item(const TimelineItemId *id, bool send_event) {
  return (send_event ? reminder_db_delete :
                prv_reminder_db_delete_common)((uint8_t *)id, sizeof(TimelineItemId));
}

bool reminder_db_is_empty(void) {
  return timeline_item_storage_is_empty(&s_storage);
}

status_t reminder_db_set_status_bits(const TimelineItemId *id, uint8_t status) {
  return timeline_item_storage_set_status_bits(&s_storage, (uint8_t *)id,
                                               sizeof(ReminderId), status);
}

/////////////////////////
// Blob DB API
/////////////////////////

void reminder_db_init(void) {
  timeline_item_storage_init(&s_storage,
                             REMINDER_DB_FILE_NAME,
                             REMINDER_DB_MAX_SIZE,
                             MAX_REMINDER_AGE);
  reminders_init();
}

void reminder_db_deinit(void) {
  timeline_item_storage_deinit(&s_storage);
}

status_t reminder_db_compact(void) {
  return timeline_item_storage_compact(&s_storage);
}

status_t reminder_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  // Records inserted from the phone are synced
  const bool mark_synced = true;
  return prv_insert_reminder(key, key_len, val, val_len, mark_synced);
}

int reminder_db_get_len(const uint8_t *key, int key_len) {
  return timeline_item_storage_get_len(&s_storage, key, key_len);
}

status_t reminder_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len) {
  return timeline_item_storage_read(&s_storage, key, key_len, val_out, val_out_len);
}

status_t reminder_db_delete(const uint8_t *key, int key_len) {
  status_t rv = prv_reminder_db_delete_common(key, key_len);
  reminders_handle_reminder_removed((Uuid *) key);

  return rv;
}

status_t reminder_db_flush(void) {
  return timeline_item_storage_flush(&s_storage);
}

status_t reminder_db_is_dirty(bool *is_dirty_out) {
  *is_dirty_out = false;
  return timeline_item_storage_each(&s_storage, sync_util_is_dirty_cb, is_dirty_out);
}

BlobDBDirtyItem* reminder_db_get_dirty_list(void) {
  BlobDBDirtyItem *dirty_list = NULL;
  timeline_item_storage_each(&s_storage, sync_util_build_dirty_list_cb, &dirty_list);

  return dirty_list;
}

status_t reminder_db_mark_synced(const uint8_t *key, int key_len) {
  PBL_LOG_DBG("reminder_db_mark_synced");
  return timeline_item_storage_mark_synced(&s_storage, key, key_len);
}
