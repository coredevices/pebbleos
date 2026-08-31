/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/attributes.h"
#include "util/pstring.h"
#include "pbl/util/size.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//! Dumping ground for ANCS types

//! Invalid ANCS UID. This is not officially invalid, but a representation is necessary, and this
//! is the most unlikely UID that an iOS device would use.
#define INVALID_UID UINT32_MAX

typedef bool (*AttrDictCompletePredicate)(const uint8_t* data, const size_t length, bool* out_error);

typedef enum {
  EventIDNotificationAdded = 0,
  EventIDNotificationModified = 1,
  EventIDNotificationRemoved = 2,
} EventID;

typedef enum {
  EventFlagSilent = (1 << 0),
  EventFlagImportant = (1 << 1),
  EventFlagPreExisting = (1 << 2),
  EventFlagPositiveAction = (1 << 3),
  EventFlagNegativeAction = (1 << 4),
  EventFlagMultiMedia = (1 << 5),
  EventFlagReserved = ~((1 << 6) - 1),
} EventFlags;

typedef enum {
  ActionIDPositive = 0,
  ActionIDNegative = 1,
} ActionId;

typedef enum {
  CategoryIDOther = 0,
  CategoryIDIncomingCall = 1,
  CategoryIDMissedCall = 2,
  CategoryIDVoicemail = 3,
  CategoryIDSocial = 4,
  CategoryIDSchedule = 5,
  CategoryIDEmail = 6,
  CategoryIDNews = 7,
  CategoryIDHealthAndFitness = 8,
  CategoryIDBusinessAndFinance = 9,
  CategoryIDLocation = 10,
  CategoryIDEntertainment = 11,
} CategoryID;

//! Notification Source's "Notification" format
typedef struct PACKED {
  EventID event_id:8;
  EventFlags event_flags:8;
  CategoryID category_id:8;
  uint8_t category_count; //<! FIXME PBL-1619: signed?
  uint32_t uid;
} NSNotification;

typedef enum {
  CommandIDGetNotificationAttributes = 0,
  CommandIDGetAppAttributes = 1,
  CommandIDPerformNotificationAction = 2,

  CommandIdInvalid
} CommandID;

//! Header for Control Point (CP) and Data Source (DS) messages
typedef struct PACKED {
  CommandID command_id:8;
  uint8_t data[];
} CPDSMessage;

typedef struct PACKED {
  CommandID command_id:8;
  uint32_t notification_uid;
  uint8_t attributes_data[];
} GetNotificationAttributesMsg;

typedef struct PACKED {
  CommandID command_id:8;
  char app_id[];
  // uint8_t attributes_data[] follows after the zero-terminated app_id string,
  // but it's not possible to express this in a C struct.
} GetAppAttributesMsg;

typedef struct PACKED {
  CommandID command_id:8;
  uint32_t notification_uid;
  uint8_t action_id;
} PerformNotificationActionMsg;

typedef enum {
  NotificationAttributeIDAppIdentifier = 0,
  NotificationAttributeIDTitle = 1, //<! Must be followed by a 2-bytes max length param
  NotificationAttributeIDSubtitle = 2, //<! Must be followed by a 2-bytes max length param
  NotificationAttributeIDMessage = 3, //<! Must be followed by a 2-bytes max length param
  NotificationAttributeIDMessageSize = 4,
  NotificationAttributeIDDate = 5,
  NotificationAttributeIDPositiveActionLabel = 6,
  NotificationAttributeIDNegativeActionLabel = 7,
} NotificationAttributeID;

typedef enum {
  AppAttributeIDDisplayName = 0,
} AppAttributeID;

typedef enum {
  FetchedAttributeFlagOptional = (1 << 0),
} FetchedAttributeFlag;

typedef struct {
  uint8_t id;
  uint8_t max_length;
  uint8_t flags;
} FetchedAttribute;

typedef enum {
  FetchedNotifAttributeIndexAppID = 0,
  FetchedNotifAttributeIndexTitle,
  FetchedNotifAttributeIndexSubtitle,
  FetchedNotifAttributeIndexMessage,
  FetchedNotifAttributeIndexMessageSize,
  FetchedNotifAttributeIndexDate,
  FetchedNotifAttributeIndexPositiveActionLabel,
  FetchedNotifAttributeIndexNegativeActionLabel,
} FetchedNotifAttributeIndex;

// FIXME AS: APP ID max length determined by looking through installed apps on iOS. Not sure what actual maximum is
#define APP_ID_MAX_LENGTH (60)
// Long enough that notification filtering rules can match text near the end
// of verbose titles (e.g. Discord's "Sender (#channel · Server Name)").
// Must fit FetchedAttribute.max_length (uint8_t).
#define TITLE_MAX_LENGTH (128)
#define SUBTITLE_MAX_LENGTH (40)
#define MESSAGE_MAX_LENGTH (200)
#define MESSAGE_SIZE_MAX_LENGTH (3)
#define DATE_LENGTH (15)
#define ACTION_MAX_LENGTH (10)
#define MAX_NUM_ACTIONS (2)
#define NOTIFICATION_ATTRIBUTES_MAX_BUFFER_LENGTH \
          (APP_ID_MAX_LENGTH + TITLE_MAX_LENGTH + SUBTITLE_MAX_LENGTH + \
           MESSAGE_MAX_LENGTH + MESSAGE_SIZE_MAX_LENGTH + DATE_LENGTH + \
          (ACTION_MAX_LENGTH * MAX_NUM_ACTIONS))

#define APP_DISPLAY_NAME_MAX_LENGTH (200)

static const FetchedAttribute s_fetched_notif_attributes[] = {
  [FetchedNotifAttributeIndexAppID] = {
    .id = NotificationAttributeIDAppIdentifier,
    .flags = 0,
    .max_length = 0
  },
  [FetchedNotifAttributeIndexTitle] = {
    .id = NotificationAttributeIDTitle,
    .flags = 0,
    .max_length = TITLE_MAX_LENGTH
  },
  [FetchedNotifAttributeIndexSubtitle] = {
    .id = NotificationAttributeIDSubtitle,
    .flags = 0,
    .max_length = SUBTITLE_MAX_LENGTH
  },
  [FetchedNotifAttributeIndexMessage] = {
    .id = NotificationAttributeIDMessage,
    .flags = 0,
    .max_length = MESSAGE_MAX_LENGTH
  },
  [FetchedNotifAttributeIndexMessageSize] = {
    .id = NotificationAttributeIDMessageSize,
    .flags = FetchedAttributeFlagOptional,
    .max_length = 0,
  },
  [FetchedNotifAttributeIndexDate] = {
    .id = NotificationAttributeIDDate,
    .flags = 0,
    .max_length = DATE_LENGTH
  },
  [FetchedNotifAttributeIndexPositiveActionLabel] = {
    .id = NotificationAttributeIDPositiveActionLabel,
    .flags = FetchedAttributeFlagOptional,
    .max_length = 0
  },
  [FetchedNotifAttributeIndexNegativeActionLabel] = {
    .id = NotificationAttributeIDNegativeActionLabel,
    .flags = FetchedAttributeFlagOptional,
    .max_length = 0
  },
};

#define NUM_FETCHED_NOTIF_ATTRIBUTES (ARRAY_LENGTH(s_fetched_notif_attributes))

typedef enum {
  FetchedAppAttributeIndexDisplayName = 0,
} FetchedAppAttributeIndex;

static const FetchedAttribute s_fetched_app_attributes[] = {
  [FetchedAppAttributeIndexDisplayName] = {
    .id = AppAttributeIDDisplayName,
  },
};

#define NUM_FETCHED_APP_ATTRIBUTES (ARRAY_LENGTH(s_fetched_app_attributes))

typedef struct PACKED {
  uint8_t id;
  union {
    PascalString16 pstr;
    struct {
      uint16_t length;
      uint8_t value[]; //<! Not null terminated!
    };
  };
} ANCSAttribute;

//! Enum with ANCS boolean properties
//! When a certain ANCS notification qualifies, it is passed along with relevant properties
//! These are for internal ANCS client use and not specified by the ANCS spec
typedef enum {
  ANCSProperty_None = 0,
  ANCSProperty_MissedCall = (1 << 0),
  ANCSProperty_IncomingCall = (1 << 1),
  ANCSProperty_VoiceMail = (1 << 2),
  ANCSProperty_MultiMedia = (1 << 3),
  ANCSProperty_iOS9 = (1 << 4),
} ANCSProperty;
