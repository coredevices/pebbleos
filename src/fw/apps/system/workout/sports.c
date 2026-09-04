/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/app.h"
#include "applib/app_comm.h"
#include "applib/app_sync/app_sync.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/activity/activity_algorithm.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/i18n/i18n.h"
#include <pbl/logging/logging.h>
#include "pbl/util/size.h"

#include "active.h"
#include "controller.h"
#include "metrics.h"

#include <stdio.h>


enum {
  SPORTS_TIME_KEY           = 0x0, // TUPLE_CSTRING
  SPORTS_DISTANCE_KEY       = 0x1, // TUPLE_CSTRING
  SPORTS_DATA_KEY           = 0x2, // TUPLE_CSTRING
  SPORTS_UNITS_KEY          = 0x3, // TUPLE_UINT(8)
  SPORTS_ACTIVITY_STATE_KEY = 0x4, // TUPLE_UINT(8)
  SPORTS_LABEL_KEY          = 0x5, // TUPLE_UINT(8)
  SPORTS_HRM_KEY            = 0x6, // TUPLE_UINT(8)
  SPORTS_CUSTOM_LABEL_KEY   = 0x7, // TUPLE_CSTRING
  SPORTS_CUSTOM_VALUE_KEY   = 0x8, // TUPLE_CSTRING
};

enum {
  STATE_INIT_VALUE    = 0x00,
  STATE_RUNNING_VALUE = 0x01,
  STATE_PAUSED_VALUE  = 0x02,
  STATE_END_VALUE     = 0x03,
};

typedef struct {
  Window *window;
  WorkoutActiveWindow *active_window;
  WorkoutController workout_controller;

  AppSync sync;
  uint8_t sync_buffer[148];

  uint8_t current_bpm;
  char duration_string[20];
  char distance_string[20];
  char pace_string[20];
  char custom_label_string[20];
  char custom_value_string[20];
  bool is_paused;
  bool supports_third_party_hr;

  WorkoutMetricType pace_speed_metric;
} SportsAppData;

#define DEFAULT_PACE_SPEED_METRIC WorkoutMetricType_Pace

////////////////////
// App
//

// Activity change callback
static void prv_health_service_event_handler(HealthEventType event, void *context) {
  SportsAppData *sports_app_data = context;
  if (event == HealthEventHeartRateUpdate) {
    sports_app_data->current_bpm = health_service_peek_current_value(HealthMetricHeartRateBPM);;
  }
}

static void prv_sync_error_callback(DictionaryResult dict_error, AppMessageResult app_message_error,
                                void *context) {
  PBL_LOG_DBG("Sports error! dict: %u, app msg: %u", dict_error,
          app_message_error);
}

static void prv_update_scrollable_metrics(SportsAppData *data) {
  WorkoutMetricType scrollable_metrics[3];

  int num_scrollable_metrics = 0;
  scrollable_metrics[num_scrollable_metrics++] = data->pace_speed_metric;

  const bool has_builtin_hrm = activity_is_hrm_present() && activity_prefs_heart_rate_is_enabled();
  if (has_builtin_hrm || data->supports_third_party_hr) {
    scrollable_metrics[num_scrollable_metrics++] = WorkoutMetricType_Hr;
  }

  const bool has_custom_metric = data->custom_label_string[0] != '\0' &&
                                 data->custom_value_string[0] != '\0';
  if (has_custom_metric) {
    scrollable_metrics[num_scrollable_metrics++] = WorkoutMetricType_Custom;
  }

  workout_active_update_scrollable_metrics(data->active_window,
                                           num_scrollable_metrics,
                                           scrollable_metrics);
}

static void prv_sync_tuple_changed_callback(uint32_t key, const Tuple *new_tuple,
                                            const Tuple *old_tuple, void *context) {
  SportsAppData *data = context;

  switch (key) {
    case SPORTS_DATA_KEY:
      strncpy(data->pace_string, new_tuple->value->cstring, sizeof(data->pace_string));
      break;
    case SPORTS_DISTANCE_KEY:
      strncpy(data->distance_string, new_tuple->value->cstring, sizeof(data->distance_string));
      break;
    case SPORTS_TIME_KEY:
      strncpy(data->duration_string, new_tuple->value->cstring, sizeof(data->duration_string));
      break;
    case SPORTS_LABEL_KEY:
    {
      const bool is_pace = MIN(new_tuple->value->uint8, 1);
      WorkoutMetricType metric_type = is_pace ? WorkoutMetricType_Pace : WorkoutMetricType_Speed;
      if (metric_type != data->pace_speed_metric) {
        data->pace_speed_metric = metric_type;
        prv_update_scrollable_metrics(data);
      }
      break;
    }
    case SPORTS_UNITS_KEY:
      break;
    case SPORTS_HRM_KEY:
    {
      // This returns if the SPORTS_HRM_KEY value has not changed from the default 0 value
      if (new_tuple->value->uint8 == 0) {
        return;
      }
      if (!data->supports_third_party_hr) {
        data->supports_third_party_hr = true;
        health_service_set_heart_rate_sample_period(0 /* interval_s */);
        prv_update_scrollable_metrics(data);
      }
      data->current_bpm = new_tuple->value->uint8;
      break;
    }
    case SPORTS_CUSTOM_LABEL_KEY:
    {
      if (strncmp(new_tuple->value->cstring,
                  data->custom_label_string,
                  sizeof(data->custom_label_string)) != 0) {
        strncpy(data->custom_label_string,
                new_tuple->value->cstring,
                sizeof(data->custom_label_string));
        prv_update_scrollable_metrics(data);
      }
      break;
    }
    case SPORTS_CUSTOM_VALUE_KEY:
    {
      if (strncmp(new_tuple->value->cstring,
                  data->custom_value_string,
                  sizeof(data->custom_value_string)) != 0) {
        strncpy(data->custom_value_string,
                new_tuple->value->cstring,
                sizeof(data->custom_value_string));
        prv_update_scrollable_metrics(data);
      }
      break;
    }
    default:
      // Unknown key
      return;
  }
}

static bool prv_is_paused(void) {
  SportsAppData *data = app_state_get_user_data();
  return data->is_paused;
}

static bool prv_pause(bool should_be_paused) {
  SportsAppData *data = app_state_get_user_data();
  AppSync *s = &data->sync;
  const Tuple *tuple = app_sync_get(s, SPORTS_ACTIVITY_STATE_KEY);
  uint8_t new_state;

  switch (tuple->value->uint8) {
    case STATE_INIT_VALUE:
    case STATE_PAUSED_VALUE:
    case STATE_END_VALUE:
    default:
      new_state = STATE_RUNNING_VALUE;
      data->is_paused = true;
      break;
    case STATE_RUNNING_VALUE:
      new_state = STATE_PAUSED_VALUE;
      data->is_paused = false;
      break;
  }

  Tuplet values[] = {
    TupletInteger(SPORTS_ACTIVITY_STATE_KEY, new_state),
  };
  app_sync_set(s, values, 1);

  return true;
}

static void prv_update_data(void *unused) {
  // Nothing to do here.
}

static void prv_metric_to_string(WorkoutMetricType type, char *buffer, size_t buffer_size,
                                 void *i18n_owner, void *unused) {
  SportsAppData *data = app_state_get_user_data();

  switch (type) {
    case WorkoutMetricType_Hr:
    {
      snprintf(buffer, buffer_size, "%d", data->current_bpm);
      break;
    }
    case WorkoutMetricType_Speed:
    case WorkoutMetricType_Pace:
    {
      strncpy(buffer, data->pace_string, MIN(buffer_size, sizeof(data->pace_string)));
      break;
    }
    case WorkoutMetricType_Distance:
    {
      strncpy(buffer, data->distance_string, MIN(buffer_size, sizeof(data->distance_string)));
      break;
    }
    case WorkoutMetricType_Duration:
    {
      strncpy(buffer, data->duration_string, MIN(buffer_size, sizeof(data->duration_string)));
      break;
    }
    case WorkoutMetricType_Custom:
    {
      strncpy(buffer,
              data->custom_value_string,
              MIN(buffer_size, sizeof(data->custom_value_string)));
      break;
    }
    // Not supported by the sports API
    case WorkoutMetricType_Steps:
    case WorkoutMetricType_AvgPace:
    case WorkoutMetricType_None:
    case WorkoutMetricTypeCount:
      break;
  }
}

static int32_t prv_get_metric_value(WorkoutMetricType type, void *workout_data) {
  SportsAppData *data = app_state_get_user_data();

  switch (type) {
    case WorkoutMetricType_Hr:
      return data->current_bpm;
    default:
      return 0;
  }
}

static const char *prv_get_distance_string(const char *miles_string, const char *km_string) {
  SportsAppData *data = app_state_get_user_data();

  const Tuple *metric_tuple = app_sync_get(&data->sync, SPORTS_UNITS_KEY);
  uint8_t is_metric = 1;
  if (metric_tuple != NULL) {
    is_metric = MIN(metric_tuple->value->uint8, 1);
  }

  if (is_metric) {
    return km_string;
  } else {
    return miles_string;
  }
}

static char *prv_get_custom_metric_label_string(void) {
  SportsAppData *data = app_state_get_user_data();
  return data->custom_label_string;
}

static void prv_init(void) {
  SportsAppData *data = app_zalloc_check(sizeof(SportsAppData));
  app_state_set_user_data(data);

  app_message_open(114, 16);

  // Sync setup:
  const uint8_t is_metric = (uint8_t) false;
  const uint8_t is_pace = (uint8_t) true;
  const uint8_t state = STATE_INIT_VALUE;
  Tuplet initial_values[] = {
    TupletCString(SPORTS_DATA_KEY, "0:00"),
    TupletCString(SPORTS_DISTANCE_KEY, "0.0"),
    TupletCString(SPORTS_TIME_KEY, "00:00"),
    TupletInteger(SPORTS_UNITS_KEY, is_metric),
    TupletInteger(SPORTS_LABEL_KEY, is_pace),
    TupletInteger(SPORTS_ACTIVITY_STATE_KEY, state),
    TupletInteger(SPORTS_HRM_KEY, 0),
    TupletCString(SPORTS_CUSTOM_LABEL_KEY, ""),
    TupletCString(SPORTS_CUSTOM_VALUE_KEY, "")
  };
  app_sync_init(&data->sync, data->sync_buffer, sizeof(data->sync_buffer), initial_values,
                ARRAY_LENGTH(initial_values), prv_sync_tuple_changed_callback,
                prv_sync_error_callback, data);


  data->workout_controller = (WorkoutController) {
    .is_paused = prv_is_paused,
    .pause = prv_pause,
    .stop = NULL,
    .update_data = prv_update_data,
    .metric_to_string = prv_metric_to_string,
    .get_metric_value = prv_get_metric_value,
    .get_distance_string = prv_get_distance_string,
    .get_custom_metric_label_string = prv_get_custom_metric_label_string,
  };

  data->active_window = workout_active_create_tripple_layout(WorkoutMetricType_Duration,
                                                             WorkoutMetricType_Distance,
                                                             0,
                                                             NULL,
                                                             NULL,
                                                             &data->workout_controller);
  data->pace_speed_metric = DEFAULT_PACE_SPEED_METRIC;
  prv_update_scrollable_metrics(data);
  workout_active_window_push(data->active_window);

  // overall reduce the sniff-mode latency at the expense of some power...
  app_comm_set_sniff_interval(SNIFF_INTERVAL_REDUCED);

  health_service_set_heart_rate_sample_period(1 /* interval_s */);
  health_service_events_subscribe(prv_health_service_event_handler, data);

  activity_algorithm_enable_activity_tracking(false /* disable */);
}

static void prv_deinit(void) {
  SportsAppData *data = app_state_get_user_data();
  health_service_set_heart_rate_sample_period(0 /* interval_s */);
  app_sync_deinit(&data->sync);
  app_free(data);

  activity_algorithm_enable_activity_tracking(true /* enable */);
}

////////////////////
// App boilerplate

static void prv_main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}

const PebbleProcessMd *sports_app_get_info(void) {
  static const PebbleProcessMdSystem s_sports_app_info = {
    .common = {
      .main_func = &prv_main,
      .visibility = ProcessVisibilityShownOnCommunication,
      .uuid = {0x4d, 0xab, 0x81, 0xa6, 0xd2, 0xfc, 0x45, 0x8a,
               0x99, 0x2c, 0x7a, 0x1f, 0x3b, 0x96, 0xa9, 0x70},
    },
    .name = i18n_noop("Sports"),
  };
  return (const PebbleProcessMd*) &s_sports_app_info;
}
