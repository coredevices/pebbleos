/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "utils.h"
#include "workout.h"

#include "kernel/pbl_malloc.h"
#include "resource/timeline_resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/workout_service.h"
#include "pbl/services/timeline/timeline.h"
#include "system/passert.h"

static TimelineItem *prv_create_abandoned_workout_notification(void) {
  const char *msg = i18n_noop("Still sweating? Your workout is active and will be ended soon. "
                              "Open the workout to keep it going.");

  AttributeList notif_attr_list = {0};
  attribute_list_add_uint32(&notif_attr_list, AttributeIdIconTiny, TIMELINE_RESOURCE_ACTIVITY);
  attribute_list_add_cstring(&notif_attr_list, AttributeIdBody, i18n_get(msg, &notif_attr_list));
  attribute_list_add_uint8(&notif_attr_list, AttributeIdBgColor,
                           PBL_IF_COLOR_ELSE(GColorYellowARGB8, GColorDarkGrayARGB8));

  AttributeList dismiss_attr_list = {0};
  attribute_list_add_cstring(&dismiss_attr_list, AttributeIdTitle,
                             i18n_get("Dismiss", &notif_attr_list));

  AttributeList end_workout_attr_list = {0};
  attribute_list_add_cstring(&end_workout_attr_list, AttributeIdTitle,
                             i18n_get("End Workout", &notif_attr_list));
  attribute_list_add_uint32(&end_workout_attr_list, AttributeIdLaunchCode,
                            WorkoutLaunchArg_EndWorkout);

  AttributeList open_workout_attr_list = {0};
  attribute_list_add_cstring(&open_workout_attr_list, AttributeIdTitle,
                             i18n_get("Open Workout", &notif_attr_list));

  const int num_actions = 3;
  TimelineItemActionGroup action_group = {
    .num_actions = num_actions,
    .actions = (TimelineItemAction[]) {
      {
        .id = 0,
        .type = TimelineItemActionTypeDismiss,
        .attr_list = dismiss_attr_list,
      },
      {
        .id = 1,
        .type = TimelineItemActionTypeOpenWatchApp,
        .attr_list = end_workout_attr_list,
      },
      {
        .id = 2,
        .type = TimelineItemActionTypeOpenWatchApp,
        .attr_list = open_workout_attr_list,
      },
    },
  };

  const time_t now_utc = rtc_get_time();

  // Note: it's fine if this returns null, since the parent functions will check for a null pointer
  TimelineItem *item = timeline_item_create_with_attributes(now_utc, 0,
                                                            TimelineItemTypeNotification,
                                                            LayoutIdNotification, &notif_attr_list,
                                                            &action_group);

  i18n_free_all(&notif_attr_list);
  attribute_list_destroy_list(&notif_attr_list);
  attribute_list_destroy_list(&end_workout_attr_list);
  attribute_list_destroy_list(&open_workout_attr_list);

  return item;
}

void workout_utils_send_abandoned_workout_notification(void) {
  TimelineItem *item = prv_create_abandoned_workout_notification();
  if (item) {
    item->header.from_watch = true;
    item->header.parent_id = (Uuid)UUID_WORKOUT_DATA_SOURCE;
    notifications_add_notification(item);
    timeline_item_destroy(item);
  }
}

const char* workout_utils_get_name_for_activity(ActivitySessionType type) {
  switch (type) {
    case ActivitySessionType_Open:
      /// Workout Label
      return i18n_noop("Workout");
    case ActivitySessionType_Walk:
      /// Walk Label
      return i18n_noop("Walk");
    case ActivitySessionType_Run:
      /// Run Label
      return i18n_noop("Run");
    case ActivitySessionType_Sleep:
    case ActivitySessionType_RestfulSleep:
    case ActivitySessionType_Nap:
    case ActivitySessionType_RestfulNap:
    case ActivitySessionType_None:
    case ActivitySessionTypeCount:
      // ActivitySessionType_Invalid should have the same value
      break;
  }

  WTF;
}

const char* workout_utils_get_detection_text_for_activity(ActivitySessionType type) {
  switch (type) {
    case ActivitySessionType_Open:
      /// Workout automatically detected dialog text
      return i18n_noop("Workout\nDetected");
    case ActivitySessionType_Walk:
      /// Walk automatically detected dialog text
      return i18n_noop("Walk\nDetected");
    case ActivitySessionType_Run:
      /// Run automatically detected dialog text
      return i18n_noop("Run\nDetected");
    case ActivitySessionType_Sleep:
    case ActivitySessionType_RestfulSleep:
    case ActivitySessionType_Nap:
    case ActivitySessionType_RestfulNap:
    case ActivitySessionType_None:
    case ActivitySessionTypeCount:
      // ActivitySessionType_Invalid should have the same value
      break;
  }

  WTF;
}

bool workout_utils_find_ongoing_activity_session(ActivitySession *session_out) {
  bool found_session = false;

  uint32_t num_sessions = ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT;
  ActivitySession *sessions = app_zalloc_check(sizeof(ActivitySession) *
                                               ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT);
  activity_get_sessions(&num_sessions, sessions);

  for (int i = num_sessions; i >= 0; i--) {
    if (workout_service_is_workout_type_supported(sessions[i].type) && sessions[i].ongoing) {
      if (session_out) {
        memcpy(session_out, &sessions[i], sizeof(ActivitySession));
      }
      found_session = true;
      break;
    }
  }

  app_free(sessions);

  return found_session;
}
