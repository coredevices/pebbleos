/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/uuid.h"

#include "kernel/events.h"

void notification_storage_init(void);

// Use notification_storage_lock and notification_storage_unlock to perform multiple
// actions on the storage that should not be interrupted

//! Recursively lock storage mutex
void notification_storage_lock(void);

//! Recursively unlock storage mutex
void notification_storage_unlock(void);

//! Store a notification to flash
void notification_storage_store(TimelineItem* notification);

//! Check if a notification exists in storage
bool notification_storage_notification_exists(const Uuid *id);

size_t notification_storage_get_len(const Uuid *uuid);

//! Get a notification from flash. The allocated_buffer of the returned notification must be freed
//! When no longer in use
bool notification_storage_get(const Uuid *id, TimelineItem *item_out);

//! Set the status of a stored notification
void notification_storage_set_status(const Uuid *id, uint8_t status);

//! Get the status for a stored notification, returns false if not found
bool notification_storage_get_status(const Uuid *id, uint8_t *status);

//! Remove a notification from storage (mark it for deletion)
void notification_storage_remove(const Uuid *id);

//! Find a notification in storage with a matching ANCS UID
bool notification_storage_find_ancs_notification_id(uint32_t ancs_uid, Uuid *uuid_out);

//! Finds a notification that is identical to the specified one by first searching the timestamps
//! and then comparing the actions and attributes
//! @param notification Notification to match with
//! @param header_out Header of matching notification
//! @return true if matching notification found, else false
bool notification_storage_find_ancs_notification_by_timestamp(
    TimelineItem *notification, CommonTimelineItemHeader *header_out);

//! Iterates over all of the notifications in the storage, calling the iterator callback with
//! the header ID of each one
//! NOTE: Do NOT call into other notification storage functions from the iterator callback. It will
//! cause corruption of notification storage
void notification_storage_iterate(bool (*iter_callback)(void *data,
    SerializedTimelineItemHeader *header_id), void *data);

//! Iterates over all the notifications calling the callback with the passed data.
//! Overwrites the notifications and rewrites them to disk.
//! This is essentially a noop if the callback doesn't alter the data.
void notification_storage_rewrite(void (*iter_callback)(TimelineItem *notification,
    SerializedTimelineItemHeader *header, void *data), void *data);

//! Clear out all notifications and reset all state immediately.
void notification_storage_reset_and_init(void);

#if UNITTEST
//! Clear out all notifications and reset all state. Used for unit testing.
void notification_storage_reset(void);
#endif
