/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include "pbl/util/uuid.h"
#include "process_management/app_install_manager.h"
#include "process_management/pebble_process_info.h"
#include "system/status_codes.h"
#include "pbl/util/attributes.h"

//! App database entry for BlobDB. First pass is very basic. The list will expand as more features
//! and requirements are implemented.
typedef struct PACKED {
  Uuid          uuid;
  uint32_t      info_flags;
  uint32_t      icon_resource_id;
  Version       app_version;
  Version       sdk_version;
  GColor8       app_face_bg_color;
  uint8_t       template_id;
  char          name[APP_NAME_SIZE_BYTES];
} AppDBEntry;

//! Used in app_db_enumerate_entries
typedef void(*AppDBEnumerateCb)(AppInstallId install_id, AppDBEntry *entry, void *data);

/* AppDB Functions */

int32_t app_db_get_next_unique_id(void);

AppInstallId app_db_get_install_id_for_uuid(const Uuid *uuid);

status_t app_db_get_app_entry_for_uuid(const Uuid *uuid, AppDBEntry *entry);

status_t app_db_get_app_entry_for_install_id(AppInstallId app_id, AppDBEntry *entry);

void app_db_enumerate_entries(AppDBEnumerateCb cb, void *data);

/* AppDB AppInstallId Implementation */

bool app_db_exists_install_id(AppInstallId app_id);

/* BlobDB Implementation */

void app_db_init(void);

status_t app_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len);

int app_db_get_len(const uint8_t *key, int key_len);

status_t app_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len);

status_t app_db_delete(const uint8_t *key, int key_len);

status_t app_db_flush(void);

status_t app_db_compact(void);

/* TEST */
AppInstallId app_db_check_next_unique_id(void);
