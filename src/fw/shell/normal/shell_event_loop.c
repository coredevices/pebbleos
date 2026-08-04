/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <kernel/events.h>
#include "shell/shell_event_loop.h"
#include "shell/prefs_private.h"

#include "apps/system_app_ids.h"
#include "apps/system/app_fetch_ui.h"
#include "apps/system/settings/quick_launch.h"
#include "apps/system/timeline/timeline.h"
#include "kernel/low_power.h"
#include "kernel/pbl_malloc.h"
#include "popups/alarm_popup.h"
#include "popups/bluetooth_pairing_ui.h"
#include "popups/notifications/notification_window.h"
#include "popups/timeline/peek.h"
#include "process_management/app_install_manager.h"
#include "process_management/app_manager.h"
#include "process_management/process_manager.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/blob_db/api.h"
#include "pbl/services/bluetooth/bluetooth_persistent_storage.h"
#include "pbl/services/shared_prf_storage/shared_prf_storage.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/workout_service.h"
#include "pbl/services/app_inbox_service.h"
#include "pbl/services/app_outbox_service.h"
#include "pbl/services/music.h"
#include "pbl/services/music_endpoint.h"
#include "pbl/services/notifications/do_not_disturb.h"
#include "pbl/services/stationary.h"
#include "pbl/services/system_task.h"
#include "pbl/services/timeline/event.h"
#include "shell/normal/app_idle_timeout.h"
#include "shell/normal/battery_ui.h"
#include "shell/normal/quick_launch.h"
#include "shell/normal/watchface.h"
#include "shell/prefs.h"
#include <pbl/logging/logging.h>


extern void shell_prefs_init(void);

// Force-compact every growable settings DB on first boot after upgrade. 
// Pre-growable-files devices keep their original full-size settings file allocation forever otherwise, 
// which on devices with many installed apps shows up as launcher scroll lag.
static void prv_settings_dbs_compaction_migration_cb(void *data) {
  blob_db_compact_growable_dbs();
  shell_prefs_set_settings_dbs_compacted_v1(true);
}

static void prv_maybe_run_settings_dbs_compaction_migration(void) {
  if (shell_prefs_get_settings_dbs_compacted_v1()) {
    return;
  }
  system_task_add_callback(prv_settings_dbs_compaction_migration_cb, NULL);
}

void shell_event_loop_init(void) {
  shell_prefs_init();
  prv_maybe_run_settings_dbs_compaction_migration();
  notification_window_service_init();
  app_inbox_service_init();
  app_outbox_service_init();
  app_message_sender_init();
  watchface_init();
  timeline_peek_init();
  // Start activity tracking if enabled
  if (activity_prefs_tracking_is_enabled()) {
    activity_start_tracking(false /*test_mode*/);
  }
  workout_service_init();

  bool factory_reset_or_first_use = !shared_prf_storage_get_getting_started_complete();
  // We are almost done booting, welcome the user if applicable. This _must_ occur before setting
  // the getting started completed below.
  if (factory_reset_or_first_use) {
    bt_persistent_storage_set_unfaithful(true);
  }

  // As soon as we boot normally for the first time, we've therefore completed first use mode and
  // we don't need to go through it again until we factory reset.
  shared_prf_storage_set_getting_started_complete(true /* complete */);
}

void shell_event_loop_handle_event(PebbleEvent *e) {
  switch (e->type) {
    case PEBBLE_APP_FETCH_REQUEST_EVENT:
      app_manager_handle_app_fetch_request_event(&e->app_fetch_request);
      return;

    case PEBBLE_ALARM_CLOCK_EVENT:
      stationary_wake_up();
      alarm_popup_push_window(&e->alarm_clock);
      return;

    case PEBBLE_BT_PAIRING_EVENT:
      bluetooth_pairing_ui_handle_event(&e->bluetooth.pair);
      return;

    case PEBBLE_APP_WILL_CHANGE_FOCUS_EVENT:
      if (e->app_focus.in_focus) {
        app_idle_timeout_resume();
      } else {
        app_idle_timeout_pause();
      }
      return;

    case PEBBLE_SYS_NOTIFICATION_EVENT:
      // This handles incoming Notifications and actions on Notifications and Reminders
      notification_window_handle_notification(&e->sys_notification);
      return;

    case PEBBLE_CALENDAR_EVENT:
      do_not_disturb_handle_calendar_event(&e->calendar);
      return;

    case PEBBLE_TIMELINE_PEEK_EVENT:
      timeline_peek_handle_peek_event(&e->timeline_peek);
      return;

    case PEBBLE_BLOBDB_EVENT:
    {
      // Calendar should only handle pin_db events
      PebbleBlobDBEvent *blobdb_event = &e->blob_db;
      if (blobdb_event->db_id == BlobDBIdPins) {
        timeline_event_handle_blobdb_event();
      } else if (blobdb_event->db_id == BlobDBIdPrefs) {
        prefs_private_handle_blob_db_event(blobdb_event);
      }
      return;
    }

    case PEBBLE_DO_NOT_DISTURB_EVENT:
      notification_window_handle_dnd_event(&e->do_not_disturb);
      return;

    case PEBBLE_REMINDER_EVENT:
      // This handles incoming Reminders
      notification_window_handle_reminder(&e->reminder);
      return;

    case PEBBLE_BATTERY_STATE_CHANGE_EVENT:
      battery_ui_handle_state_change_event(e->battery_state.new_state);
      return;

    case PEBBLE_COMM_SESSION_EVENT:
      music_endpoint_handle_mobile_app_event(&e->bluetooth.comm_session_event);
      return;

    // Sent by the comm layer once we get a response from the mobile app to a phone version request
    case PEBBLE_REMOTE_APP_INFO_EVENT:
      music_endpoint_handle_mobile_app_info_event(&e->bluetooth.app_info_event);
      return;

    case PEBBLE_MEDIA_EVENT:
      if (e->media.playback_state == MusicPlayStatePlaying) {
        app_install_mark_prioritized(APP_ID_MUSIC, true /* can_expire */);
      }
      return;

    case PEBBLE_HEALTH_SERVICE_EVENT:
      workout_service_health_event_handler(&e->health_event);
      return;

    case PEBBLE_ACTIVITY_EVENT:
      do_not_disturb_handle_activity_event(&e->activity_event);
      workout_service_activity_event_handler(&e->activity_event);
      return;

    case PEBBLE_WORKOUT_EVENT: {
      // If a workout is ongoing, keep the app at the top of the launcher.
      // When a workout is stopped it will return to it's normal position after the
      // default timeout.
      PebbleWorkoutEvent *workout_e = &e->workout;
      bool can_expire = true;
      switch (workout_e->type) {
        case PebbleWorkoutEvent_Started:
        case PebbleWorkoutEvent_Paused:
          can_expire = false;
          break;
        case PebbleWorkoutEvent_Stopped:
          can_expire = true;
          break;
        case PebbleWorkoutEvent_FrontendOpened:
        case PebbleWorkoutEvent_FrontendClosed:
          break;
      }
      app_install_mark_prioritized(APP_ID_WORKOUT, can_expire);
      workout_service_workout_event_handler(workout_e);
      return;
    }
    default:
      break; // don't care
  }
}

