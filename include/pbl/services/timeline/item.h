/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "attribute.h"
#include "layout_layer.h"

#include "pbl/util/attributes.h"
#include "pbl/util/uuid.h"

#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX_PIN_TITLE_LENGTH  (50)

#define TIMELINE_INVALID_ACTION_ID  (0xFF)

typedef Uuid TimelineItemId;

//! Enumeration to extract individual statuses from a TimelineItem's status.
typedef enum {
  TimelineItemStatusRead = 1 << 0,
  TimelineItemStatusDeleted = 1 << 1,
  TimelineItemStatusActioned = 1 << 2,
  TimelineItemStatusReminded = 1 << 3,
  TimelineItemStatusDismissed = 1 << 4,
  TimelineItemStatusUnused = ~((1 << 5) - 1)
} TimelineItemStatus;

//! Enumeration to extract individual flags from a TimelineItem's flags.
typedef enum {
  TimelineItemFlagVisible = 1 << 0,
  TimelineItemFlagFloating = 1 << 1,
  TimelineItemFlagAllDay = 1 << 2,
  TimelineItemFlagFromWatch = 1 << 3,
  TimelineItemFlagFromANCS = 1 << 4,
  TimelineItemFlagPersistent = 1 << 5,
  TimelineItemFlagSilent = 1 << 6,
  // This should always be the bitmask for unused bits.
  TimelineItemFlagUnused = ~((1 << 7) - 1)
} TimelineItemFlag;

//! Enumeration of the different types of actions a TimelineItem can have.
typedef enum {
  TimelineItemActionTypeUnknown = 0x00,
  TimelineItemActionTypeAncsNegative = 0x01, // Formerly ANCS Dismiss
  TimelineItemActionTypeGeneric = 0x02,
  TimelineItemActionTypeResponse = 0x03,
  TimelineItemActionTypeDismiss = 0x04,
  TimelineItemActionTypeHttp = 0x05,
  TimelineItemActionTypeSnooze = 0x06,
  TimelineItemActionTypeOpenWatchApp = 0x07,
  TimelineItemActionTypeEmpty = 0x08,
  TimelineItemActionTypeRemove = 0x09,
  TimelineItemActionTypeOpenPin = 0x0A,
  TimelineItemActionTypeAncsPositive = 0x0B,
  TimelineItemActionTypeAncsDial = 0x0C,
  TimelineItemActionTypeAncsResponse = 0x0D,
  TimelineItemActionTypeInsightResponse = 0x0E,
  TimelineItemActionTypeAncsDelete = 0x0F,
  TimelineItemActionTypeComplete = 0x10,
  TimelineItemActionTypePostpone = 0x11,
  TimelineItemActionTypeRemoteRemove = 0x12,
  TimelineItemActionTypeAncsGeneric = 0x13,
  TimelineItemActionTypeBLEHRMStopSharing = 0x14,
} TimelineItemActionType;

//! Attribute identifiers for icons in the resource pack
typedef enum {
  TimelineItemIconIdCrossmark = 1,
  TimelineItemIconIdCheckmark,
  TimelineItemIconIdSentMail,
  TimelineItemIconIdSentMessage,
  TimelineItemIconIdPhoneCheckmark
} TimelineItemIconId;

//! Types of timeline items
typedef enum {
  TimelineItemTypeUnknown = 0,
  TimelineItemTypeNotification,
  TimelineItemTypePin,
  TimelineItemTypeReminder,
  TimelineItemTypeOutOfRange
} TimelineItemType;

typedef struct {
  uint8_t id;
  TimelineItemActionType type;
  AttributeList attr_list;
} TimelineItemAction;

typedef struct {
  uint8_t num_actions;
  TimelineItemAction *actions;
} TimelineItemActionGroup;

typedef struct PACKED {
  //! Unique identifier for this item.  Controlled by the watch.  Needed for responding
  //! to the phone to satisfy actions actuated on the watch.
  TimelineItemId id;
  union {
    //! ANCS UID associated with the TimelineItem if the item was received from an
    //! IOS device, but not through the Pebble IOS application.
    uint32_t ancs_uid;
    //! Unique identifier referencing the parent of this item.
    TimelineItemId parent_id;
  };
  //! The time (in UTC and in seconds) at which the TimelineItem occurs.
  time_t timestamp;
  //! The amount of time (in minutes) past the timestamp for which to display
  //! the TimelineItem (if it is a pin) in the NOW section of the timeline.  If
  //! timestamp + duration < current_time, then it is no longer in NOW.
  uint16_t duration;
  //! The timeline item type
  TimelineItemType type:8;
  //! These flags are set by the datasource and/or mobile application and are one-way
  //! flags indicating how the pin interacts with the user.
  union {
    //! Note: Once written to flash, the TimelineItem would have to be re-written
    //! entirely to revert the value.
    struct {
      //! Indicates whether the item is visible or not.
      uint8_t visible:1;
      //! Indicates whether this is a floating timezone item (implementation TBD)
      uint8_t is_floating:1;
      //! Indicates whether the item is an all day event.
      //! All day events should have a timestamp at midnight.
      uint8_t all_day:1;
      //! Indicates that this item was added by the watch and shouldn't get flushed
      uint8_t from_watch:1;
      //! Indicates that this notification was added by ANCS (iOS)
      uint8_t ancs_notif:1;
      //! Reserved: mirrors TimelineItemFlagPersistent, set by the phone. Not used on the watch.
      uint8_t persistent_flag:1;
      //! Indicates the notification was delivered silently on the phone (iOS ANCS EventFlagSilent,
      //! or set by a companion app on the wire flags byte).
      uint8_t silent:1;
    };
    uint8_t flags;
  };
  union {
    //! Note: Once written to flash, the TimelineItem would have to be re-written
    //! entirely to revert the value.
    struct {
      //! Indicates that the item has been read (only used for Notifications).
      uint8_t read:1;
      //! Indicates that the item has been deleted
      uint8_t deleted:1;
      //! Indicates that the item has been actioned on.
      uint8_t actioned:1;
      //! Indicates whether the reminder has been reminded.
      uint8_t reminded:1;
      //! Indicates whether the item has been dismissed
      uint8_t dismissed:1;
      //! Indicates whether the item is persistent (only used for Timeline Peek / Quick View)
      uint8_t persistent:1;
    };
    uint8_t status;
  };
  //! Layout for this TimelineItem when rendered in a view. Determines
  //! how the attributes are rendered.
  LayoutId layout:8;
} CommonTimelineItemHeader;

//! A TimelineItem is one of {Reminder, Notification, Pin}.  To determine which
//! type this corresponds to, we use the flags specified when the TimelineItem
//! is created.  This controls whether the item appears in the timeline or in
//! the notifications application.
typedef struct {
  CommonTimelineItemHeader header;
  AttributeList attr_list;
  TimelineItemActionGroup action_group;
  uint8_t *allocated_buffer;
} TimelineItem;

typedef struct PACKED {
  CommonTimelineItemHeader common;
  uint16_t payload_length;
  //! Number of attributes that determine how the view/pin look when they are rendered.
  uint8_t num_attributes;
  //! Number of actions associated with this TimelineItem.  These are the actions that appear when
  //! the view for this item is rendered.
  uint8_t num_actions;
} SerializedTimelineItemHeader;

//! Create a \ref TimelineItem
//! @param timestamp          timestamp of the item
//! @param duration           duration of the item (in minutes)
//! @param type               type of the item
//! @param layout             layout of the item for the timeline
//! @param attr_list          pointer to an attribute list. The attribute list will be copied
//! into the timeline item's allocated_buffer so it's okay to free the existing attribute list
//! @param action_group       pointer to an action group
//! @return pointer to a heap-allocated item
//! Heap-allocate and populated a \ref TimelineItem
TimelineItem *timeline_item_create_with_attributes(time_t timestamp, uint16_t duration,
                                                   TimelineItemType type, LayoutId layout,
                                                   AttributeList *attr_list,
                                                   TimelineItemActionGroup *action_group);

//! @param num_attributes   number of non-action attributes
//! @param num_actions      number of actions
//! @param attributes_per_action    an array of counts for the number of attributes per action
//!                                 in order corresponding to action order
//! @param required_size_for_strings    total size of all attribute strings
//! @param string_buffer    pointer to pointer to the start of the string buffer (where attribute
//!                         strings must be appended. The alloc string will set \c *string_buffer
//!                         if \c string_buffer is not NULL)
TimelineItem *timeline_item_create(int num_attributes, int num_actions,
    uint8_t attributes_per_action[], size_t required_size_for_strings, uint8_t **string_buffer);

//! Deserialize a \ref TimelineItem
//! @param item               storage for the item
//! @param num_attributes     number of notification attributes
//! @param num_actions        number of notification actions
//! @param data               serialized data buffer
//! @param size               size of the serial data buffer
//! @param string_alloc_size  size of string buffer that will be allocated for the notification
//! @param string_buffer      pointer to string buffer; string buffer location set by function
//! @return false if parse error or failure to allocate the required memory occurred
bool timeline_item_create_from_serial_data(TimelineItem *item, uint8_t num_attributes,
                                           uint8_t num_actions, const uint8_t *data, size_t size,
                                           size_t *string_alloc_size, uint8_t **string_buffer);

//! Copy a \ref TimelineItem
//! @param src The item to copy
//! @return pointer to a heap-allocated item which is a copy of src
TimelineItem *timeline_item_copy(TimelineItem *src);

//! Convert a stream of data into a \ref TimelineItem
//! @param item_out pointer to a TimelineItem to store the item into
//! @param header pointer to the item's header data
//! @param payload pointer to the item's payload data
//! @return true if the function succeeds, false otherwise
bool timeline_item_deserialize_item(TimelineItem *item_out,
                                    const SerializedTimelineItemHeader* header,
                                    const uint8_t *payload);

//! Serialize some of a timeline item's metadata into a SerializedTimelineItemHeader
//! @param item a pointer to the \ref TimelineItem to serialize
//! @param[out] header a pointer to the serialized header to populate
void timeline_item_serialize_header(TimelineItem *item, SerializedTimelineItemHeader *header);

//! Deserialize a SerializedTimelineItemHeader and populate a TimelineItem from it
//! @param[out] item a pointer to the TimelineItem to populate
//! @param header a pointer to the serialized header data
void timeline_item_deserialize_header(TimelineItem *item,
                                      const SerializedTimelineItemHeader *header);

//! Get the UTC-to-localtime adjusted timestamp of a serialized or deserialized header.
//! Just returns the timestamp if the item is not an all day event or floating.
//! @param hdr pointer to the header of the item
//! @return the potentially adjusted timestamp
time_t timeline_item_get_tz_timestamp(CommonTimelineItemHeader *hdr);

//! Serialize a notification's attributes and actions
//! @param item a pointer to the item to serialize
//! @param buffer a pointer to the buffer to write to
//! @param buffer_size the size of the buffer in bytes
//! @returns the number of bytes written to \c buffer
size_t timeline_item_serialize_payload(TimelineItem *item, uint8_t *buffer,
    size_t buffer_size);

//! Calculate the required size for a buffer to store an item's actions & attributes
size_t timeline_item_get_serialized_payload_size(TimelineItem *item);

//! Deserialize actions & attributes from payload
//! @param item pointer to the item to deserialize
//! @param string_buffer buffer to store the strings in
//! @param string_buffer_size size of the string buffer in bytes
//! @param payload serialized payload buffer
//! @param payload_size size of the payload buffer in bytes
//! @return true on success
bool timeline_item_deserialize_payload(TimelineItem *item,
    char *string_buffer, size_t string_buffer_size, const uint8_t *payload, size_t payload_size);

//! Clean up a dynamically allocated item.
void timeline_item_destroy(TimelineItem* item);

//! Frees allocated buffer and NULLs pointer to it.
//! NOTE: Internal pointers to attributes are invalid after calling this function
//!   suitable for TimelineItem structs that are not dynamically allocated
void timeline_item_free_allocated_buffer(TimelineItem *item);

bool timeline_item_verify_layout_serialized(const uint8_t *val, int val_len);

//! Find an action given it's id
//! @param item the item containing the action
//! @param action_id the ID of the action
//! @return a pointer to the action, NULL if not found
const TimelineItemAction *timeline_item_find_action_with_id(const TimelineItem *item,
                                                            uint8_t action_id);

//! Find an action given it's type
//! @param item the item containing the action
//! @param type the Type of the action
//! @return a pointer to the action, NULL if not found
TimelineItemAction *timeline_item_find_action_by_type(const TimelineItem *item,
                                                      TimelineItemActionType type);

//! Return true if the given action is a dismiss action
bool timeline_item_action_is_dismiss(const TimelineItemAction *action);

//! Return true if the given action is an ANCS action
bool timeline_item_action_is_ancs(const TimelineItemAction *action);

TimelineItemAction *timeline_item_find_dismiss_action(const TimelineItem *item);

//! Find the reply action in a timeline item (can be either regular response or ANCS response)
//! @param item the item containing the action
//! @return a pointer to the action, NULL if not found
TimelineItemAction *timeline_item_find_reply_action(const TimelineItem *item);


//! Find the reply action in an action group (can be either regular response or ANCS response)
//! @param action_group the action group to search
//! @return a pointer to the action, NULL if not foun
TimelineItemAction *timeline_item_action_group_find_reply_action(
    const TimelineItemActionGroup *action_group);

//! Return true if the TimelineItem is received from an ANCS message (iOS devices)
bool timeline_item_is_ancs_notif(const TimelineItem *item);
