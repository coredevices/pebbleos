/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/blob_db/api.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/blob_db/reminder_db.h"
#include "pbl/services/blob_db/sync.h"
#include "pbl/services/blob_db/sync_util.h"
#include "pbl/services/blob_db/timeline_item_storage.h"

#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "pbl/services/app_cache.h"
#include "pbl/services/timeline/timeline.h"
#include <pbl/logging/logging.h>
#include "util/units.h"
#include "pbl/util/uuid.h"

PBL_LOG_MODULE_DECLARE(service_blob_db, CONFIG_SERVICE_BLOB_DB_LOG_LEVEL);

#define PIN_DB_MAX_AGE (3 * SECONDS_PER_DAY) // so we get at two full past days in there
#define PIN_DB_FILE_NAME "pindb"
#define PIN_DB_MAX_SIZE KiBYTES(40) // TODO [FBO] variable size / reasonable value

static TimelineItemStorage s_pin_db_storage;

/////////////////////////
// Pin DB specific API
/////////////////////////

status_t pin_db_delete_with_parent(const TimelineItemId *parent_id) {
  return (timeline_item_storage_delete_with_parent(&s_pin_db_storage, parent_id, NULL));
}

//! Caution: CommonTimelineItemHeader .flags & .status are stored inverted and not auto-restored
status_t pin_db_each(SettingsFileEachCallback each, void *data) {
  return timeline_item_storage_each(&s_pin_db_storage, each, data);
}

static status_t prv_insert_serialized_item(const uint8_t *key, int key_len, const uint8_t *val,
                                           int val_len, bool mark_synced) {
  CommonTimelineItemHeader *hdr = (CommonTimelineItemHeader *)val;
  if (hdr->layout == LayoutIdNotification || hdr->layout == LayoutIdReminder) {
    // pins do not support these layouts
    return E_INVALID_ARGUMENT;
  }

  status_t rv = timeline_item_storage_insert(&s_pin_db_storage, key, key_len,
                                             val, val_len, mark_synced);

  if (rv == S_SUCCESS) {
    TimelineItemId parent_id = ((CommonTimelineItemHeader *)val)->parent_id;
    if (timeline_get_private_data_source(&parent_id)) {
      goto done;
    }
    // Not a private data source, must be a PBW
    AppInstallId install_id = app_install_get_id_for_uuid(&parent_id);
    // can't add a pin for a not installed app!
    if (install_id == INSTALL_ID_INVALID) {
      // String initialized on the heap to reduce stack usage
      char *parent_id_string = kernel_malloc_check(UUID_STRING_BUFFER_LENGTH);
      uuid_to_string(&parent_id, parent_id_string);
      PBL_LOG_ERR("Pin insert for a pin with no app installed, parent id: %s",
              parent_id_string);
      kernel_free(parent_id_string);
      goto done;
    }
    // Bump the app's priority by telling the cache we're using it
    if (app_cache_entry_exists(install_id)) {
      app_cache_app_launched(install_id);
      goto done;
    }
    // System apps don't need to be fetched / are always installed
    if (app_install_id_from_system(install_id)) {
      goto done;
    }
    // The app isn't cached. Fetch it!
    PebbleEvent e = {
      .type = PEBBLE_APP_FETCH_REQUEST_EVENT,
      .app_fetch_request = {
        .id = install_id,
        .with_ui = false,
        .fetch_args = NULL,
      },
    };
    event_put(&e);
  }

done:
  return rv;
}

static status_t prv_insert_item(TimelineItem *item, bool emit_event) {
  if (item->header.type != TimelineItemTypePin) {
    return E_INVALID_ARGUMENT;
  }

  // allocate a buffer big enough for serialized item
  size_t payload_size = timeline_item_get_serialized_payload_size(item);
  uint8_t *buffer = kernel_malloc_check(sizeof(SerializedTimelineItemHeader) + payload_size);
  uint8_t *write_ptr = buffer;

  // serialize the header
  timeline_item_serialize_header(item, (SerializedTimelineItemHeader *) write_ptr);
  write_ptr += sizeof(SerializedTimelineItemHeader);

  // serialize the attributes / actions
  size_t bytes_serialized = timeline_item_serialize_payload(item, write_ptr, payload_size);
  status_t rv;
  if (bytes_serialized != payload_size) {
    rv = E_INVALID_ARGUMENT;
    goto cleanup;
  }

  // Only pins from the reminders app should be dirty and synced to the phone
  Uuid reminders_data_source_uuid = UUID_REMINDERS_DATA_SOURCE;
  const bool mark_synced = !uuid_equal(&item->header.parent_id, &reminders_data_source_uuid);
  rv = prv_insert_serialized_item(
      (uint8_t *)&item->header.id, sizeof(TimelineItemId),
      buffer, sizeof(SerializedTimelineItemHeader) + payload_size, mark_synced);
  if (rv == S_SUCCESS && emit_event) {
    blob_db_event_put(BlobDBEventTypeInsert, BlobDBIdPins, (uint8_t *)&item->header.id,
                      sizeof(TimelineItemId));
  }

  if (!mark_synced) {
    blob_db_sync_record(BlobDBIdPins, (uint8_t *)&item->header.id, sizeof(TimelineItemId),
                        rtc_get_time());
  }

cleanup:
  kernel_free(buffer);
  return rv;
}

status_t pin_db_insert_item(TimelineItem *item) {
  return prv_insert_item(item, true /* emit_event */);
}

status_t pin_db_insert_item_without_event(TimelineItem *item) {
  return prv_insert_item(item, false /* emit_event */);
}
status_t pin_db_set_status_bits(const TimelineItemId *id, uint8_t status) {
  return timeline_item_storage_set_status_bits(&s_pin_db_storage, (uint8_t *)id, sizeof(*id),
                                               status);
}

status_t pin_db_get(const TimelineItemId *id, TimelineItem *pin) {
  int size = pin_db_get_len((uint8_t *)id, UUID_SIZE);
  if (size <= 0) {
    return E_DOES_NOT_EXIST;
  }
  uint8_t *read_buf = task_malloc_check(size);
  status_t status = pin_db_read((uint8_t *)id, UUID_SIZE, read_buf, size);
  if (status != S_SUCCESS) {
    goto cleanup;
  }

  SerializedTimelineItemHeader *header = (SerializedTimelineItemHeader *)read_buf;
  uint8_t *payload = read_buf + sizeof(SerializedTimelineItemHeader);
  if (!timeline_item_deserialize_item(pin, header, payload)) {
    status = E_INTERNAL;
    goto cleanup;
  }

  task_free(read_buf);
  return (S_SUCCESS);

cleanup:
  task_free(read_buf);
  return status;
}

bool pin_db_exists_with_parent(const TimelineItemId *parent_id) {
  return timeline_item_storage_exists_with_parent(&s_pin_db_storage, parent_id);
}

status_t pin_db_read_item_header(TimelineItem *item_out, TimelineItemId *id) {
  SerializedTimelineItemHeader hdr = {{{0}}};
  status_t rv = pin_db_read((uint8_t *)id, sizeof(TimelineItemId), (uint8_t *)&hdr,
    sizeof(SerializedTimelineItemHeader));
  timeline_item_deserialize_header(item_out, &hdr);
  return rv;
}

status_t pin_db_next_item_header(TimelineItem *next_item_out,
                                 TimelineItemStorageFilterCallback filter) {
  TimelineItemId id;
  status_t rv = timeline_item_storage_next_item(&s_pin_db_storage, &id, filter);
  if (rv) {
    return rv;
  }
  rv = pin_db_read_item_header(next_item_out, &id);
  return rv;
}

/////////////////////////
// Blob DB API
/////////////////////////

void pin_db_init(void) {
  s_pin_db_storage = (TimelineItemStorage){};
  timeline_item_storage_init(&s_pin_db_storage,
                             PIN_DB_FILE_NAME,
                             PIN_DB_MAX_SIZE,
                             PIN_DB_MAX_AGE);
}

void pin_db_deinit(void) {
  timeline_item_storage_deinit(&s_pin_db_storage);
}

status_t pin_db_compact(void) {
  return timeline_item_storage_compact(&s_pin_db_storage);
}

bool pin_db_has_entry_expired(time_t pin_end_timestamp) {
  return (pin_end_timestamp < (rtc_get_time() - PIN_DB_MAX_AGE));
}

status_t pin_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len) {
  // Records inserted from the phone are already synced
  const bool mark_synced = true;
  return prv_insert_serialized_item(key, key_len, val, val_len, mark_synced);
}

int pin_db_get_len(const uint8_t *key, int key_len) {
  return timeline_item_storage_get_len(&s_pin_db_storage, key, key_len);
}

status_t pin_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_len) {
  return timeline_item_storage_read(&s_pin_db_storage, key, key_len, val_out, val_len);
}

status_t pin_db_delete(const uint8_t *key, int key_len) {
  status_t rv = timeline_item_storage_delete(&s_pin_db_storage, key, key_len);
  if (rv == S_SUCCESS) {
    //! remove reminders that are children of this pin
    reminder_db_delete_with_parent((TimelineItemId *)key);
  }

  return rv;
}

status_t pin_db_flush(void) {
  return timeline_item_storage_flush(&s_pin_db_storage);
}

status_t pin_db_is_dirty(bool *is_dirty_out) {
  *is_dirty_out = false;
  return timeline_item_storage_each(&s_pin_db_storage, sync_util_is_dirty_cb, is_dirty_out);
}

BlobDBDirtyItem* pin_db_get_dirty_list(void) {
  BlobDBDirtyItem *dirty_list = NULL;
  timeline_item_storage_each(&s_pin_db_storage, sync_util_build_dirty_list_cb, &dirty_list);

  return dirty_list;
}

status_t pin_db_mark_synced(const uint8_t *key, int key_len) {
  return timeline_item_storage_mark_synced(&s_pin_db_storage, key, key_len);
}
