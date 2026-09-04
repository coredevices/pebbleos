/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/app_glances/app_glance_service.h"
#include "system/status_codes.h"
#include "util/time/time.h"
#include "pbl/util/uuid.h"

#include <stdint.h>

// -------------------------------------------------------------------------------------------------
// AppGlanceDB Implementation

status_t app_glance_db_insert_glance(const Uuid *uuid, const AppGlance *glance);

status_t app_glance_db_read_glance(const Uuid *uuid, AppGlance *glance_out);

status_t app_glance_db_read_creation_time(const Uuid *uuid, time_t *time_out);

status_t app_glance_db_delete_glance(const Uuid *uuid);

// -------------------------------------------------------------------------------------------------
// BlobDB API Implementation

void app_glance_db_init(void);

status_t app_glance_db_flush(void);

//! Compact and shrink the on-disk settings file. Forces growable files that
//! grew before the growable change landed (or under heavy load) to drop back
//! toward the initial allocation.
status_t app_glance_db_compact(void);

status_t app_glance_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len);

int app_glance_db_get_len(const uint8_t *key, int key_len);

status_t app_glance_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len);

status_t app_glance_db_delete(const uint8_t *key, int key_len);
