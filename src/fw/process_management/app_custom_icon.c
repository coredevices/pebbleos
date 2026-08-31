/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_custom_icon.h"

#include "app_install_manager_private.h"

#include "apps/system_app_ids.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/comm_session/session.h"
#include <pbl/logging/logging.h>
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"

#include <string.h>
#include <stddef.h>

// We no longer have icons in the launcher, so we don't really need this anymore.
// However, we may decide to put it back in, so let's keep the code around just in case.
#define ALLOW_SET_ICON 0

typedef enum {
  SPORTS = 0x00,
  GOLF = 0x01,
  NUM_APP_TYPES,

  UNKNOWN_APP_TYPE,
  APP_TYPE_MASK = 0x1,
} CustomizableAppType;

typedef enum {
  NAME_FIELD = 0x00,
  ICON_FIELD = 0x80,
  FIELD_MASK = 0x80,
} FieldId;

typedef struct PACKED {
  //! OR'ed value of (CustomizableAppType | FieldId). No C bitfields here, because order is compiler-specific.
  uint8_t app_type_and_field_bits;

  union {
    char name[0];
    struct {
      uint16_t row_size_bytes;
      uint16_t info_flags; //<! See GBitmap::info_flags
      GRect bounds;
      uint8_t image_data[];
    } icon;
  };
} AppCustomizeMessage;

typedef struct {
  char *name;
  GBitmap icon;
  uint8_t *image_data;
  CustomizableAppType app_type:4;
} AppCustomizeInfo;

static AppCustomizeInfo s_info[NUM_APP_TYPES];

static void do_callbacks(const CustomizableAppType app_type) {
  if (app_type == UNKNOWN_APP_TYPE) {
    return;
  }

  AppInstallId app_id = (app_type == SPORTS) ? APP_ID_SPORTS : APP_ID_GOLF;
  app_install_do_callbacks(APP_ICON_NAME_UPDATED, app_id, NULL, NULL, NULL);
}

#if ALLOW_SET_ICON
static void set_icon(const CustomizableAppType app_type, const uint16_t row_size_bytes,
    const uint16_t info_flags, const GRect bounds,
    const uint8_t image_data[], const uint16_t image_data_length) {

  AppCustomizeInfo *info = &s_info[app_type];
  const uint16_t desired_size = row_size_bytes * bounds.size.h;
  const uint16_t available_size = MIN(desired_size, image_data_length);
  if (info->image_data &&
      memcmp(info->image_data, image_data, available_size) == 0 &&
      info->icon.bounds.origin.x == bounds.origin.x &&
      info->icon.bounds.origin.y == bounds.origin.y &&
      info->icon.bounds.size.w == bounds.size.w &&
      info->icon.bounds.size.h == bounds.size.h &&
      info->icon.info_flags == info_flags &&
      info->icon.row_size_bytes == row_size_bytes) {
    // Already equal to current icon
    return;
  }
  if (info->image_data) {
    kernel_free(info->image_data);
  }
  info->image_data = (uint8_t *) kernel_malloc(available_size);
  memcpy(info->image_data, image_data, available_size);
  info->icon.addr = info->image_data;
  info->icon.bounds = bounds;
  info->icon.row_size_bytes = row_size_bytes;
  info->icon.info_flags = info_flags;
  do_callbacks(app_type);
}
#endif

static void set_name(const CustomizableAppType app_type, const char *name, const uint16_t length) {
  AppCustomizeInfo *info = &s_info[app_type];
  if (info->name &&
      strlen(info->name) == length &&
      strncmp(info->name, name, length) == 0) {
    // Already equal to current name
    return;
  }
  if (info->name) {
    kernel_free(info->name);
  }
  info->name = (char *) kernel_malloc(length + 1);
  memcpy(info->name, name, length);
  info->name[length] = 0;
  do_callbacks(app_type);
}

void customizable_app_protocol_msg_callback(CommSession *session, const uint8_t* data, size_t length) {
  AppCustomizeMessage *message = (AppCustomizeMessage *) data;
  const FieldId field_id = message->app_type_and_field_bits & FIELD_MASK;
  const CustomizableAppType app_type = message->app_type_and_field_bits & APP_TYPE_MASK;
  switch (field_id) {
    case NAME_FIELD: {
      const uint16_t name_length = length - offsetof(AppCustomizeMessage, name);
      set_name(app_type, message->name, name_length);
      break;
    }
#if ALLOW_SET_ICON
    case ICON_FIELD: {
      const uint16_t image_data_length = length - offsetof(AppCustomizeMessage, icon.image_data);
      set_icon(app_type, message->icon.row_size_bytes, message->icon.info_flags, message->icon.bounds, message->icon.image_data, image_data_length);
      break;
    }
#endif
    default: return;
  }
  (void)session;
}

static CustomizableAppType get_app_type_for_app_id(AppInstallId app_id) {
  if (app_id == APP_ID_SPORTS) {
    return SPORTS;
  } else if (app_id == APP_ID_GOLF) {
    return GOLF;
  }
  return UNKNOWN_APP_TYPE;
}

// Retrieve the custom name or return NULL
const char *app_custom_get_title(AppInstallId app_id) {
  CustomizableAppType app_type = get_app_type_for_app_id(app_id);

  const char *name = NULL;
  if (app_type != UNKNOWN_APP_TYPE && s_info[app_type].name != NULL) {
    name = s_info[app_type].name;
  } else {
    return NULL;
  }

  return name;
}

const GBitmap *app_custom_get_icon(AppInstallId app_id) {
  CustomizableAppType app_type = get_app_type_for_app_id(app_id);

  if (app_type != UNKNOWN_APP_TYPE && s_info[app_type].image_data != NULL) {
    return &s_info[app_type].icon;
  }
  return NULL;
}
