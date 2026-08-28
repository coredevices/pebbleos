/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/services_normal.h"

#include <string.h>

#include "applib/event_service_client.h"
#include <pbl/drivers/rtc.h>
#include "kernel/events.h"
#include "process_management/app_install_manager.h" // FIXME: This should really be in services/
#include "process_management/launcher_app_message.h" // FIXME: This should really be in services/
#include "pbl/services/activity/activity.h"
#include "pbl/services/alarms/alarm.h"
#include "pbl/services/app_cache.h"
#include "pbl/services/app_fetch_endpoint.h"
#include "pbl/services/app_glances/app_glance_service.h"
#include "pbl/services/blob_db/api.h"
#include "pbl/services/blob_db/endpoint_private.h"
#include "pbl/services/data_logging/data_logging_service.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/protobuf_log/protobuf_log.h"
#include "pbl/services/music_endpoint.h"
#include "pbl/services/music_internal.h"
#include "pbl/services/notifications/alerts_private.h"
#include "pbl/services/notifications/notifications.h"
#include "pbl/services/persist.h"
#include "pbl/services/phone_call.h"
#include "pbl/services/process_management/app_order_storage.h"
#include "pbl/services/send_text_service.h"
#include "pbl/services/speaker/speaker_service.h"
#include "pbl/services/stationary.h"
#include "pbl/services/timeline/event.h"
#include "pbl/services/wakeup.h"
#include "pbl/services/weather/weather_service.h"
#include "pbl/services/runlevel_impl.h"

#ifdef CONFIG_ORIENTATION_MANAGER
#include "pbl/services/orientation_manager.h"
#endif

#include "pbl/services/activity/activity.h"
#include "pbl/services/voice/voice.h"

#include "pbl/util/size.h"

// Minimum valid time: January 1, 2020 00:00:00 UTC (timestamp: 1577836800)
// This represents the minimum time we consider valid for activity initialization
#define MIN_VALID_TIME_TIMESTAMP 1577836800

// State for deferred activity initialization
static bool s_activity_init_deferred = false;
static EventServiceInfo s_time_event_info;

static void prv_time_set_event_handler(PebbleEvent *e, void *context) {
  // Check if time is now valid and activity init was deferred
  if (s_activity_init_deferred && rtc_get_time() >= MIN_VALID_TIME_TIMESTAMP) {
    // Time is now valid, initialize activity
    s_activity_init_deferred = false;
    
    // Unsubscribe from time events
    event_service_client_unsubscribe(&s_time_event_info);
    
    activity_init();
    // If the user had tracking enabled before init was deferred, start tracking now so we
    // don't miss steps when initialization happens after boot.
    if (activity_prefs_tracking_is_enabled()) {
      // Ensure the runlevel-enabled flag is set for the activity service so the usual
      // enable/disable machinery will attempt to start tracking. This covers the case
      // where services_set_runlevel() already ran before activity was initialized.
      activity_set_enabled(true);
      activity_start_tracking(false /* test_mode */);
    }
  }
}

static bool prv_is_time_valid_for_activity_init(void) {
  return rtc_get_time() >= MIN_VALID_TIME_TIMESTAMP;
}

void services_normal_early_init(void) {
  pfs_init(true);
}

void services_normal_init(void) {
  persist_service_init();

  app_install_manager_init();

  blob_db_init_dbs();
  app_cache_init();
  phone_call_service_init();
  music_init();
  alarm_init();
  timeline_event_init();
  dls_init();

  wakeup_init();

  app_order_storage_init();

  // Check if time is valid before initializing activity
  if (prv_is_time_valid_for_activity_init()) {
    activity_init();
  } else {
    // Defer activity initialization until time is set properly
    s_activity_init_deferred = true;

    // Subscribe to time set events
    s_time_event_info = (EventServiceInfo) {
      .type = PEBBLE_SET_TIME_EVENT,
      .handler = prv_time_set_event_handler,
    };
    event_service_client_subscribe(&s_time_event_info);
  }

  notifications_init();
  alerts_init();
  send_text_service_init();
  protobuf_log_init();

  weather_service_init();

  speaker_service_init();

#ifdef CONFIG_MIC
  voice_init();
#endif

  app_glance_service_init();
}

static struct ServiceRunLevelSetting s_runlevel_settings[] = {
  {
    .set_enable_fn = wakeup_enable,
    .enable_mask = R_Stationary | R_Normal,
  },
  {
    .set_enable_fn = alarm_service_enable_alarms,
    .enable_mask = R_LowPower | R_Stationary | R_Normal,
  },
  {
    .set_enable_fn = activity_set_enabled,
    .enable_mask = R_Stationary | R_Normal,
  },
  {
    .set_enable_fn = stationary_run_level_enable,
    .enable_mask = R_Stationary | R_Normal,
  },
  {
    .set_enable_fn = dls_set_send_enable_run_level,
    .enable_mask = R_Normal,
  },
  {
    .set_enable_fn = blob_db_enabled,
    .enable_mask = R_Normal,
  },
#ifdef CONFIG_ORIENTATION_MANAGER
  {
    .set_enable_fn = orientation_manager_enable,
    // FirmwareUpdate would otherwise call enable(false) and un-rotate the
    // display, so the progress UI looks inverted in Left-Handed Mode.
    .enable_mask = R_LowPower | R_Stationary | R_FirmwareUpdate | R_Normal,
  }
#endif
};

void services_normal_set_runlevel(RunLevel runlevel) {
  for (size_t i = 0; i < ARRAY_LENGTH(s_runlevel_settings); ++i) {
    struct ServiceRunLevelSetting *service = &s_runlevel_settings[i];
    service->set_enable_fn(((1 << runlevel) & service->enable_mask) != 0);
  }
}
