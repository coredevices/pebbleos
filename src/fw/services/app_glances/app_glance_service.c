/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/app_glances/app_glance_service.h"

#include "applib/app_glance.h"
#include "applib/event_service_client.h"
#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "process_management/app_install_manager.h"
#include "pbl/services/blob_db/app_glance_db.h"
#include "syscall/syscall_internal.h"
#include "system/passert.h"
#include "system/status_codes.h"
#include "pbl/util/math.h"

//! Return true to continue iteration and false to stop it.
typedef bool (*SliceForEachCb)(AppGlanceSliceInternal *slice, void *context);

static void prv_slice_for_each(AppGlance *glance, SliceForEachCb cb, void *context) {
  if (!glance || !cb) {
    return;
  }

  for (unsigned int slice_index = 0;
       slice_index < MIN(glance->num_slices, APP_GLANCE_DB_MAX_SLICES_PER_GLANCE); slice_index++) {
    AppGlanceSliceInternal *current_slice = &glance->slices[slice_index];
    // Stop iterating if the client's callback function returns false
    if (!cb(current_slice, context)) {
      break;
    }
  }
}

typedef struct FindCurrentSliceData {
  time_t current_time;
  AppGlanceSliceInternal *current_slice;
} FindCurrentSliceData;

//! The "current" slice is the slice with an expiration_time closest to the current time while
//! still being after the current time
static bool prv_find_current_glance(AppGlanceSliceInternal *slice, void *context) {
  FindCurrentSliceData *data = context;
  PBL_ASSERTN(data);

  // First check if this slice never expires; the zero value of APP_GLANCE_SLICE_NO_EXPIRATION
  // won't work with the comparisons we perform below
  if (slice->expiration_time == APP_GLANCE_SLICE_NO_EXPIRATION) {
    // We'll only use a never-expiring slice if we haven't set a slice yet
    if (!data->current_slice) {
      data->current_slice = slice;
    }
    // Continue iterating through the slices
    return true;
  }

  const int time_until_slice_expires = slice->expiration_time - data->current_time;

  // Continue iterating through the slices if this slice expires in the past
  if (time_until_slice_expires <= 0) {
    return true;
  }

  // If we don't have a current slice or the current slice we have is a never-expiring slice, we can
  // go ahead and use this slice now but continue iterating to try to find an earlier expiring slice
  // in the list
  if (!data->current_slice ||
      (data->current_slice->expiration_time == APP_GLANCE_SLICE_NO_EXPIRATION)) {
    data->current_slice = slice;
    return true;
  }

  const int time_until_current_slice_expires =
      data->current_slice->expiration_time - data->current_time;

  // If this slice expires earlier than our current slice, use this slice as the new current slice
  if (time_until_slice_expires < time_until_current_slice_expires) {
    data->current_slice = slice;
  }

  // Continue iterating to try to find an earlier slice
  return true;
}

static void prv_glance_event_put(const Uuid *app_uuid) {
  Uuid *app_uuid_copy = kernel_zalloc_check(sizeof(Uuid));
  *app_uuid_copy = *app_uuid;

  PebbleEvent e = (PebbleEvent) {
    .type = PEBBLE_APP_GLANCE_EVENT,
    .app_glance = (PebbleAppGlanceEvent) {
      .app_uuid = app_uuid_copy,
    },
  };

  event_put(&e);
}

//////////////////////
// Event handlers
// NOTE: These events are handled on KernelMain (app_glance_service_init called from
// services_normal_init)
//////////////////////

static void prv_blob_db_event_handler(PebbleEvent *event, void *context) {
  const PebbleBlobDBEvent *blob_db_event = &event->blob_db;
  const BlobDBId blob_db_id = blob_db_event->db_id;

  if (blob_db_id != BlobDBIdAppGlance) {
    // We only care about app glance changes
    return;
  }

  prv_glance_event_put((Uuid *)blob_db_event->key);
}

static void prv_handle_app_cache_event(PebbleEvent *e, void *context) {
  if (e->app_cache_event.cache_event_type == PebbleAppCacheEvent_Removed) {
    Uuid app_uuid;
    app_install_get_uuid_for_install_id(e->app_cache_event.install_id, &app_uuid);
    app_glance_db_delete_glance(&app_uuid);
  }
}

//////////////////////
// Public API
//////////////////////

void app_glance_service_init_glance(AppGlance *glance) {
  if (!glance) {
    return;
  }
  *glance = (AppGlance) {};
}

void app_glance_service_init(void) {

  static EventServiceInfo s_blob_db_event_info = {
    .type = PEBBLE_BLOBDB_EVENT,
    .handler = prv_blob_db_event_handler,
  };
  event_service_client_subscribe(&s_blob_db_event_info);

  static EventServiceInfo s_app_cache_event_info = {
    .type = PEBBLE_APP_CACHE_EVENT,
    .handler = prv_handle_app_cache_event,
  };
  event_service_client_subscribe(&s_app_cache_event_info);
}

bool app_glance_service_get_current_slice(const Uuid *app_uuid, AppGlanceSliceInternal *slice_out) {
  if (!slice_out) {
    return false;
  }

  bool success;
  // Try to read the app's glance, first checking the cache
  AppGlance *app_glance = kernel_zalloc_check(sizeof(*app_glance));
  const status_t rv = app_glance_db_read_glance(app_uuid, app_glance);
  if (rv != S_SUCCESS) {
    success = false;
    goto cleanup;
  }

  // Iterate over the slices to find the current slice (which might be NULL if there aren't any
  // slices or if all of the slices have expired)
  FindCurrentSliceData find_current_slice_data = (FindCurrentSliceData) {
    .current_time = rtc_get_time(),
  };
  prv_slice_for_each(app_glance, prv_find_current_glance, &find_current_slice_data);
  if (!find_current_slice_data.current_slice) {
    success = false;
    goto cleanup;
  }

  // Copy the current slice data to slice_out
  *slice_out = *find_current_slice_data.current_slice;
  success = true;

cleanup:
  kernel_free(app_glance);

  return success;
}

DEFINE_SYSCALL(bool, sys_app_glance_update, const Uuid *uuid, const AppGlance *glance) {
  if (PRIVILEGE_WAS_ELEVATED) {
    syscall_assert_userspace_buffer(uuid, sizeof(*uuid));
    syscall_assert_userspace_buffer(glance, sizeof(*glance));
  }
  const bool success = (app_glance_db_insert_glance(uuid, glance) == S_SUCCESS);
  if (success) {
    prv_glance_event_put(uuid);
  }
  return success;
}
