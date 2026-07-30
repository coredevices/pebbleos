/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/activity/activity_insights.h"

#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/activity_private.h"
#include "pbl/services/activity/insights_settings.h"

#include "applib/event_service_client.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "resource/resource_ids.auto.h"
#include "resource/timeline_resource_ids.auto.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/system_task.h"
#include "pbl/services/activity/health_util.h"
#include "pbl/services/activity/hr_util.h"
#include "apps/system/workout/utils.h"
#include "pbl/services/notifications/notification_storage.h"
#include "pbl/services/settings/settings_file.h"
#include "pbl/services/timeline/health_layout.h"
#include "pbl/services/timeline/metricgroup.h"
#include "pbl/services/timeline/timeline.h"
#include "pbl/services/timeline/weather_layout.h"
#include "shell/prefs.h"
#include "shell/system_app_ids.auto.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "util/stats.h"
#include "pbl/util/string.h"
#include "util/time/time.h"
#include "util/units.h"

#include <stdio.h>

PBL_LOG_MODULE_DECLARE(service_activity, CONFIG_SERVICE_ACTIVITY_LOG_LEVEL);

#define INSIGHTS_LOG_DEBUG(fmt, args...) \
        PBL_LOG_D_DBG(LOG_DOMAIN_ACTIVITY_INSIGHTS, fmt, ## args)

#define SUBTITLE_BUFFER_LENGTH 18
#define TIME_BUFFER_LENGTH 9

#define NUM_COPY_VARIANTS 5
#define VARIANT_RANDOM (-1)

// The sleep summary notification only fires when the sleep exit lands within this local
// time-of-day window ([min, max)). The timeline pin is updated regardless.
#define SLEEP_SUMMARY_NOTIF_WAKE_MINUTE_MIN (4 * MINUTES_PER_HOUR)
#define SLEEP_SUMMARY_NOTIF_WAKE_MINUTE_MAX (18 * MINUTES_PER_HOUR)

typedef struct NotificationConfig {
  time_t notif_time;
  ActivitySession *session;
  ActivityInsightType insight_type;
  TimelineResourceId icon_id;
  const char *body;

  StringList *headings;
  StringList *values;

  struct {
    bool enabled;
    HealthCardType health_card_type;
  } open_app;

  struct {
    bool enabled;
    const Uuid *uuid;
  } open_pin;

  struct {
    bool enabled;
    ActivityInsightResponseType type;
    const char *title;
  } response;
} NotificationConfig;

typedef struct InsightCopyVariants {
  int num_variants;
  char *variants[NUM_COPY_VARIANTS];
} InsightCopyVariants;

// -----------------------------------------------------------------------------------------
// Globals
typedef struct InsightStateCommon {
  bool history_valid;        // True if history requirements were met for the associated reward
  time_t last_triggered_utc; // Last time reward was triggered, saved to flash
} InsightStateCommon;

static struct {
  InsightStateCommon common;
} s_sleep_reward_state;

static struct {
  InsightStateCommon common;
  ActivityScalarStore active_minutes;
} s_activity_reward_state;

// Cached insight settings
static ActivityInsightSettings s_sleep_reward_settings;
static ActivityInsightSettings s_sleep_summary_settings;
static ActivityInsightSettings s_activity_reward_settings;
static ActivityInsightSettings s_activity_summary_settings;
static ActivityInsightSettings s_activity_session_settings;

static PFSCallbackHandle s_pfs_cb_handle = NULL; // Required for handling settings file changes
static EventServiceInfo s_blobdb_event_info;     // Used to detect pin deletion events

// Timestamp and UUID of the last time we added a new summary pin - stored to flash to allow
// us to continue to update the pin across reboots
typedef struct PACKED SummaryPinLastState {
  time_t last_triggered_utc;
  Uuid uuid;
} SummaryPinLastState;

typedef struct ActivityPinState {
  Uuid uuid;
  bool removed;
  time_t next_update_time;
  ActivityScalarStore next_step_count;
} ActivityPinState;
static ActivityPinState s_activity_pin_state;

typedef struct SleepPinState {
  time_t last_triggered_utc;
  Uuid uuid;
  time_t first_enter_utc;
  int active_minutes;
  bool removed;
  bool notified;
  // Start of the sleep-day window this state belongs to. The summary notification fires at
  // most once per window, no matter how often the sleep bounds change within it.
  // Note: must stay the last field so older persisted states fail to restore cleanly.
  time_t window_utc;
} SleepPinState;
static SleepPinState s_sleep_pin_state;

typedef struct SessionPinState {
  time_t start_utc;
} SessionPinState;
static SessionPinState s_session_pin_state;

typedef struct NapPinState {
  time_t last_triggered_utc;
} NapPinState;
static NapPinState s_nap_pin_state;

// Sleep and activity metric stats
static ActivityInsightMetricHistoryStats s_sleep_stats;
static ActivityInsightMetricHistoryStats s_activity_stats;

// -----------------------------------------------------------------------------------------
// Reward notification configurations - notification attributes, settings keys, etc.
typedef struct RewardNotifConfig {
  InsightStateCommon *state;
  ActivityInsightType insight_type;
  ActivitySettingsKey settings_key;
  TimelineResourceId icon;
  const char *text_body;
  const char *text_positive_action;
  const char *text_neutral_action;
  const char *text_negative_action;
  const char *text_positive_response;
  const char *text_neutral_response;
  const char *text_negative_response;
  uint32_t icon_positive_response;
  uint32_t icon_neutral_response;
  uint32_t icon_negative_response;
} RewardNotifConfig;

static const RewardNotifConfig SLEEP_REWARD_NOTIF_CONFIG = {
  .state = &s_sleep_reward_state.common,
  .insight_type = ActivityInsightType_SleepReward,
  .settings_key = ActivitySettingsKeyInsightSleepRewardTime,
  .icon = TIMELINE_RESOURCE_SLEEP,
  .text_body = i18n_noop("How are you feeling? Have you noticed extra focus, better mood or "
                         "extra energy? You have been sleeping great this week! Keep it up!"),
  .text_positive_action = i18n_noop("I feel fabulous!"),
  .text_neutral_action = i18n_noop("About average"),
  .text_negative_action = i18n_noop("I'm still tired"),
  .text_positive_response = i18n_noop("Awesome!"),
  .text_neutral_response = i18n_noop("Keep it up!"),
  .text_negative_response = i18n_noop("We'll get there!"),
  .icon_positive_response = TIMELINE_RESOURCE_REWARD_GOOD,
  .icon_neutral_response = TIMELINE_RESOURCE_REWARD_AVERAGE,
  .icon_negative_response = TIMELINE_RESOURCE_REWARD_BAD,
};

static const RewardNotifConfig ACTIVITY_REWARD_NOTIF_CONFIG = {
  .state = &s_activity_reward_state.common,
  .insight_type = ActivityInsightType_ActivityReward,
  .settings_key = ActivitySettingsKeyInsightActivityRewardTime,
  .icon = TIMELINE_RESOURCE_ACTIVITY,
  .text_body = i18n_noop("Congratulations - you're having a super active day! Activity "
                         "makes you more focused and creative. How do you feel?"),
  .text_positive_action = i18n_noop("I feel great!"),
  .text_neutral_action = i18n_noop("About the same"),
  .text_negative_action = i18n_noop("Not feeling it"),
  .text_positive_response = i18n_noop("Awesome!"),
  .text_neutral_response = i18n_noop("Keep it up!"),
  .text_negative_response = i18n_noop("We'll get there!"),
  .icon_positive_response = TIMELINE_RESOURCE_REWARD_GOOD,
  .icon_neutral_response = TIMELINE_RESOURCE_REWARD_AVERAGE,
  .icon_negative_response = TIMELINE_RESOURCE_REWARD_BAD,
};

// -----------------------------------------------------------------------------------------
// Summary pin configurations

typedef struct SummaryPinPercentageConfig {
  const char *body;
  const char *detail_text; // Overrides common value (automatically localized)
} SummaryPinPercentageConfig;

typedef struct SummaryPinConfig {
  ActivityInsightSettings *insight_settings;
  const char *short_title;
  char *short_subtitle;
  char *detail_text;    // Note: this is not automatically localized
  HealthCardType health_card_type;
  TimelineResourceId icon; // Icon which is shown in the timeline list view
  SummaryPinPercentageConfig percent_config[PercentTierCount];
} SummaryPinConfig;

static char s_pin_subtitle_buffer[SUBTITLE_BUFFER_LENGTH] = "";
static const SummaryPinConfig ACTIVITY_SUMMARY_PIN_CONFIG = {
  .insight_settings = &s_activity_summary_settings,
  .short_title = i18n_noop("Activity Summary"),
  .short_subtitle = s_pin_subtitle_buffer,
  .health_card_type = HealthCardType_Activity,
  .icon = TIMELINE_RESOURCE_ACTIVITY,

  .percent_config = {
    { // PercentTier_AboveAverage
      .body = i18n_noop("Do you feel more energetic, sharper or optimistic? Being active helps!"),
      .detail_text = i18n_noop("GREAT DAY TODAY"),
    },
    { // PercentTier_OnAverage
      .body = i18n_noop("You're being consistent and that's important, keep at it!"),
      .detail_text = i18n_noop("CONSISTENT!"),
    },
    { // PercentTier_BelowAverage
      .body = i18n_noop("Resting is fine, but try to recover and step it up tomorrow!"),
      .detail_text = i18n_noop("NOT VERY ACTIVE"),
    },
    { // PercentTier_Fail
      .body = i18n_noop("Resting is fine, but try to recover and step it up tomorrow!"),
      .detail_text = i18n_noop("NOT VERY ACTIVE"),
    },
  }
};

static char s_sleep_period_buffer[SUBTITLE_BUFFER_LENGTH] = "";
static const SummaryPinConfig SLEEP_SUMMARY_PIN_CONFIG = {
  .insight_settings = &s_sleep_summary_settings,
  .short_title = i18n_noop("Sleep Summary"),
  .short_subtitle = s_pin_subtitle_buffer,
  .detail_text = s_sleep_period_buffer,
  .health_card_type = HealthCardType_Sleep,
  .icon = TIMELINE_RESOURCE_SLEEP,

  .percent_config = {
    { // PercentTier_AboveAverage
      .body = i18n_noop("You had a good night! Feel the energy 😃"),
    },
    { // PercentTier_OnAverage
      .body = i18n_noop("It's great that you're keeping a consistent sleep routine!"),
    },
    { // PercentTier_BelowAverage
      .body = i18n_noop("A good night's sleep goes a long way! Try to get more hours tonight."),
    },
    { // PercentTier_Fail
      .body = i18n_noop("A good night's sleep goes a long way! Try to get more hours tonight."),
    },
  }
};

static void prv_create_and_push_notification(const NotificationConfig *config);
static void prv_push_sleep_summary_notification(time_t notif_time, int32_t sleep_total_seconds,
                                                int32_t sleep_average_seconds, int variant);
static int32_t prv_get_step_count(void);

// ------------------------------------------------------------------------------------------------
// Helper functions for picking a variant from an InsightCopyVariants set
static const char *prv_get_variant(const InsightCopyVariants *set, int variant) {
  if (variant == VARIANT_RANDOM) {
    variant = rand() % set->num_variants;
    return set->variants[variant];
  } else if (variant < set->num_variants) {
    return set->variants[variant];
  } else {
    return NULL;
  }
}

// ------------------------------------------------------------------------------------------------
// Helper functions for saving insight state to settings file
static bool prv_restore_state(SettingsFile *file, ActivitySettingsKey key, void *val_out,
                              size_t val_out_len) {
  return settings_file_get(file, &key, sizeof(key), val_out, val_out_len);
}

// ------------------------------------------------------------------------------------------------
static bool prv_save_state(ActivitySettingsKey key, void *val, size_t val_len) {
  bool rv = false;

  SettingsFile *file = activity_private_settings_open();
  if (file) {
    rv = settings_file_set(file, &key, sizeof(key), val, val_len);
    activity_private_settings_close(file);
  }
  return rv;
}

// ------------------------------------------------------------------------------------------------
// Builds the base attribute list for insight notifications
static void prv_build_notification_attr_list(AttributeList *attr_list, const char *body,
                                             uint32_t icon, ActivityInsightType insight_type,
                                             ActivitySessionType activity_type) {
  attribute_list_add_resource_id(attr_list, AttributeIdIconTiny, icon);
  attribute_list_add_cstring(attr_list, AttributeIdBody, body);
  attribute_list_add_uint8(attr_list, AttributeIdBgColor, GColorOrangeARGB8);
  attribute_list_add_uint8(attr_list, AttributeIdHealthInsightType, insight_type);
  attribute_list_add_uint8(attr_list, AttributeIdHealthActivityType, activity_type);
}

// ------------------------------------------------------------------------------------------------
// Generates a new timeline item for a reward notification
static NOINLINE TimelineItem *prv_create_reward_notification(time_t notif_time,
    const RewardNotifConfig *notif_config) {
  AttributeList notif_attr_list = {0};
  prv_build_notification_attr_list(&notif_attr_list,
                                   i18n_get(notif_config->text_body, &notif_attr_list),
                                   notif_config->icon,
                                   notif_config->insight_type,
                                   ActivitySessionType_None);

  AttributeList positive_attr_list = {0};
  attribute_list_add_cstring(&positive_attr_list, AttributeIdTitle,
                             i18n_get(notif_config->text_positive_action, &notif_attr_list));
  attribute_list_add_cstring(&positive_attr_list, AttributeIdBody,
                             i18n_get(notif_config->text_positive_response, &notif_attr_list));
  attribute_list_add_uint32(&positive_attr_list, AttributeIdIconLarge,
                            notif_config->icon_positive_response);

  AttributeList neutral_attr_list = {0};
  attribute_list_add_cstring(&neutral_attr_list, AttributeIdTitle,
                             i18n_get(notif_config->text_neutral_action, &notif_attr_list));
  attribute_list_add_cstring(&neutral_attr_list, AttributeIdBody,
                             i18n_get(notif_config->text_neutral_response, &notif_attr_list));
  attribute_list_add_uint32(&neutral_attr_list, AttributeIdIconLarge,
                            notif_config->icon_neutral_response);

  AttributeList negative_attr_list = {0};
  attribute_list_add_cstring(&negative_attr_list, AttributeIdTitle,
                             i18n_get(notif_config->text_negative_action, &notif_attr_list));
  attribute_list_add_cstring(&negative_attr_list, AttributeIdBody,
                             i18n_get(notif_config->text_negative_response, &notif_attr_list));
  attribute_list_add_uint32(&negative_attr_list, AttributeIdIconLarge,
                            notif_config->icon_negative_response);

  const int num_actions = 3;
  TimelineItemActionGroup action_group = {
    .num_actions = num_actions,
    .actions = (TimelineItemAction[]) {
      {
        .id = ActivityInsightResponseTypePositive,
        .type = TimelineItemActionTypeInsightResponse,
        .attr_list = positive_attr_list,
      },
      {
        .id = ActivityInsightResponseTypeNeutral,
        .type = TimelineItemActionTypeInsightResponse,
        .attr_list = neutral_attr_list,
      },
      {
        .id = ActivityInsightResponseTypeNegative,
        .type = TimelineItemActionTypeInsightResponse,
        .attr_list = negative_attr_list,
      }
    },
  };

  // Note: it's fine if this returns null, since the parent functions will check for a null pointer
  TimelineItem *item = timeline_item_create_with_attributes(notif_time, 0,
                                                            TimelineItemTypeNotification,
                                                            LayoutIdNotification, &notif_attr_list,
                                                            &action_group);

  i18n_free_all(&notif_attr_list);
  attribute_list_destroy_list(&notif_attr_list);
  attribute_list_destroy_list(&positive_attr_list);
  attribute_list_destroy_list(&neutral_attr_list);
  attribute_list_destroy_list(&negative_attr_list);

  return item;
}

// ------------------------------------------------------------------------------------------------
// Sets the common header parameters, pushes the notification to the user and destroys the item
static void prv_push_notification(TimelineItem *item, const Uuid *parent_id) {
  if (item) {
    item->header.from_watch = true;
    item->header.parent_id = parent_id ? *parent_id : (Uuid)UUID_HEALTH_DATA_SOURCE;
    notifications_add_notification(item);
    timeline_item_destroy(item);
  }
}

// ------------------------------------------------------------------------------------------------
// Generates a new notification and pushes it to the notification window
static void prv_push_reward_notification(time_t notif_time, const RewardNotifConfig *notif_config) {
  TimelineItem *item = prv_create_reward_notification(notif_time, notif_config);
  prv_push_notification(item, NULL);
}

typedef struct ResponseItem {
  ActivityInsightResponseType type;
  const char *text;
  AttributeList attr_list;
} ResponseItem;

static void prv_set_open_app_action(AttributeList *action_attr_list, HealthCardType card_type,
                                    void *i18n_owner) {
  attribute_list_add_cstring(action_attr_list, AttributeIdTitle, i18n_get("Open App", i18n_owner));
  // Set the launch args to open the correct health app card
  HealthLaunchArgs launch_args = {
    .card_type = card_type,
  };
  attribute_list_add_uint32(action_attr_list, AttributeIdLaunchCode, launch_args.args);
}

// ------------------------------------------------------------------------------------------------
static NOINLINE TimelineItem *prv_create_pin_with_response_items(
    time_t pin_time_utc, time_t now_utc, uint32_t duration_m, LayoutId layout_id,
    AttributeList *pin_attr_list, HealthCardType health_card_type, int num_responses,
    ResponseItem *response_items) {
  AttributeList open_attr_list = {0};
  prv_set_open_app_action(&open_attr_list, health_card_type, pin_attr_list);

  AttributeList remove_attr_list = {0};
  attribute_list_add_cstring(&remove_attr_list, AttributeIdTitle, i18n_get("Remove",
                                                                           pin_attr_list));

  const int num_actions = 2 + num_responses;
  TimelineItemActionGroup action_group = {
    .num_actions = num_actions,
    // Malloc the actions in order to save stack space
    .actions = kernel_zalloc_check(sizeof(TimelineItemAction) * num_actions),
  };
  action_group.actions[0] = (TimelineItemAction) {
    .id = 0,
    .type = TimelineItemActionTypeOpenWatchApp,
    .attr_list = open_attr_list,
  };
  for (int i = 0; i < num_responses; i++) {
    ResponseItem *response_item = &response_items[i];
    attribute_list_add_cstring(&response_item->attr_list, AttributeIdTitle,
                               i18n_get(response_item->text, pin_attr_list));
    action_group.actions[i + 1] = (TimelineItemAction) {
      .id = response_items->type,
      .type = TimelineItemActionTypeInsightResponse,
      .attr_list = response_item->attr_list,
    };
  }
  action_group.actions[num_responses + 1] = (TimelineItemAction) {
    .id = 1,
    .type = TimelineItemActionTypeRemove,
    .attr_list = remove_attr_list,
  };

  // Note: it's fine if this returns null, since the parent functions will check for a null pointer
  TimelineItem *item = timeline_item_create_with_attributes(pin_time_utc, duration_m,
                                                            TimelineItemTypePin, layout_id,
                                                            pin_attr_list, &action_group);

  for (int i = 0; i < num_responses; i++) {
    ResponseItem *response_item = &response_items[i];
    attribute_list_destroy_list(&response_item->attr_list);
  }
  kernel_free(action_group.actions);
  attribute_list_destroy_list(&open_attr_list);
  attribute_list_destroy_list(&remove_attr_list);

  return item;
}

// ------------------------------------------------------------------------------------------------
static TimelineItem *prv_create_pin(time_t pin_time_utc, time_t now_utc, uint32_t duration_m,
                                    LayoutId layout_id, AttributeList *pin_attr_list,
                                    HealthCardType health_card_type) {
  return prv_create_pin_with_response_items(pin_time_utc, now_utc, duration_m, layout_id,
                                            pin_attr_list, health_card_type, 0, NULL);
}

// ------------------------------------------------------------------------------------------------
PercentTier prv_calc_percent_tier(const SummaryPinConfig *config, ActivityScalarStore cur_val,
                                  ActivityScalarStore average, int *percentage) {
  // Determine percentage of target
  *percentage = (average > 0) ? (((cur_val * 100) / (average)) - 100) : 0;

  if (*percentage < config->insight_settings->summary.fail_threshold) {
    return PercentTier_Fail;
  } else if (*percentage < config->insight_settings->summary.below_avg_threshold) {
    return PercentTier_BelowAverage;
  } else if (*percentage > config->insight_settings->summary.above_avg_threshold) {
    return PercentTier_AboveAverage;
  }

  return PercentTier_OnAverage;
}

// ------------------------------------------------------------------------------------------------
// Generates a new timeline item for a summary pin
static NOINLINE TimelineItem *prv_create_summary_pin(time_t pin_time_utc, time_t now_utc,
                                                     ActivityScalarStore cur_val,
                                                     ActivityScalarStore average,
                                                     const SummaryPinConfig *config) {
  AttributeList pin_attr_list = {0};
  attribute_list_add_cstring(&pin_attr_list, AttributeIdShortTitle,
                             i18n_get(config->short_title, &pin_attr_list));
  attribute_list_add_cstring(&pin_attr_list, AttributeIdShortSubtitle, config->short_subtitle);

  static struct {
    const char *avg_relation;
    uint8_t bg_color;
    TimelineResourceId card_icon;
  } s_tier_config[PercentTierCount] = {
    [PercentTier_Fail] = {
      .avg_relation = PBL_IF_RECT_ELSE(i18n_noop("BELOW AVG"), i18n_noop("Below avg")),
      .bg_color = GColorOrangeARGB8,
      .card_icon = TIMELINE_RESOURCE_ARROW_DOWN,
    },
    [PercentTier_BelowAverage] = {
      .avg_relation = PBL_IF_RECT_ELSE(i18n_noop("BELOW AVG"), i18n_noop("Below avg")),
      .bg_color = GColorOrangeARGB8,
      .card_icon = TIMELINE_RESOURCE_ARROW_DOWN,
    },
    [PercentTier_OnAverage] = {
      .avg_relation = PBL_IF_RECT_ELSE(i18n_noop("ON AVG"), i18n_noop("On avg")),
      .bg_color = GColorVividCeruleanARGB8,
      .card_icon = TIMELINE_RESOURCE_THUMBS_UP,
    },
    [PercentTier_AboveAverage] = {
      .avg_relation = PBL_IF_RECT_ELSE(i18n_noop("ABOVE AVG"), i18n_noop("Above avg")),
      .bg_color = GColorIslamicGreenARGB8,
      .card_icon = TIMELINE_RESOURCE_ARROW_UP,
    },
  };

  // Determine percentage of target
  int percentage;
  const PercentTier tier = prv_calc_percent_tier(config, cur_val, average, &percentage);

  attribute_list_add_uint8(&pin_attr_list, AttributeIdBgColor, s_tier_config[tier].bg_color);
  attribute_list_add_cstring(&pin_attr_list,
                             PBL_IF_RECT_ELSE(AttributeIdTitle, AttributeIdLocationName),
                             i18n_get(s_tier_config[tier].avg_relation, &pin_attr_list));

  // Select the correct layout config based on percentage
  const SummaryPinPercentageConfig *percent_config = &config->percent_config[tier];

  // Add the correct text as the LocationName attribute at the bottom of the layout
  const char *detail_text;
  if (percent_config->detail_text) {
    detail_text = i18n_get(percent_config->detail_text, &pin_attr_list);
  } else {
    detail_text = config->detail_text;
  }
  attribute_list_add_cstring(&pin_attr_list,
                             PBL_IF_RECT_ELSE(AttributeIdLocationName, AttributeIdTitle),
                             detail_text);

  attribute_list_add_cstring(&pin_attr_list, AttributeIdBody,
                             i18n_get(percent_config->body, &pin_attr_list));
  attribute_list_add_resource_id(&pin_attr_list, AttributeIdIconTiny, config->icon);
  attribute_list_add_resource_id(&pin_attr_list, AttributeIdIconSmall,
                                 s_tier_config[tier].card_icon);

  attribute_list_add_uint8(&pin_attr_list, AttributeIdDisplayTime, WeatherTimeType_None);
  attribute_list_add_uint32(&pin_attr_list, AttributeIdLastUpdated, now_utc);

  char *percentage_buf = kernel_malloc_check(SUBTITLE_BUFFER_LENGTH);
  if (percentage == 0) {
    strcpy(percentage_buf, "0%");
  } else {
    sniprintf(percentage_buf, SUBTITLE_BUFFER_LENGTH, "%+d%%", percentage);
  }
  attribute_list_add_cstring(&pin_attr_list, AttributeIdSubtitle, percentage_buf);

  TimelineItem *item = prv_create_pin(pin_time_utc, now_utc, 0, LayoutIdWeather, &pin_attr_list,
                                      config->health_card_type);

  kernel_free(percentage_buf);
  i18n_free_all(&pin_attr_list);
  attribute_list_destroy_list(&pin_attr_list);

  return item;
}

// ------------------------------------------------------------------------------------------------
// Inserts a new pin on the timeline if existing_uuid is UUID_INVALID, otherwise the pin is updated
// Returns true if it added a new pin, false if it updated
static bool prv_push_pin(TimelineItem *item, Uuid *existing_uuid) {
  bool rv = false;
  if (item) {
    item->header.from_watch = true;
    item->header.parent_id = (Uuid)UUID_HEALTH_DATA_SOURCE;

    if (!uuid_is_invalid(existing_uuid)) {
      item->header.id = *existing_uuid;
    } else {
      *existing_uuid = item->header.id;
      rv = true;
    }

    timeline_add(item);
    timeline_item_destroy(item);
  }

  return rv;
}

// ------------------------------------------------------------------------------------------------
static bool prv_push_summary_pin(time_t pin_time_utc, time_t now_utc, Uuid *existing_uuid,
                                 ActivityScalarStore cur_val, ActivityScalarStore average,
                                 const SummaryPinConfig *config) {
  TimelineItem *item = prv_create_summary_pin(pin_time_utc, now_utc, cur_val, average, config);
  return prv_push_pin(item, existing_uuid);
}

// ------------------------------------------------------------------------------------------------
// Pushes a new reward notification and saves the trigger time to flash
static void prv_push_reward(time_t now_utc, const RewardNotifConfig *notif_config) {
  prv_push_reward_notification(now_utc, notif_config);
  notif_config->state->last_triggered_utc = time_util_get_midnight_of(now_utc);

  // Save out the trigger time
  prv_save_state(notif_config->settings_key,
                 &notif_config->state->last_triggered_utc,
                 sizeof(notif_config->state->last_triggered_utc));

  INSIGHTS_LOG_DEBUG("Saved reward state: %ld", notif_config->state->last_triggered_utc);
}

// ------------------------------------------------------------------------------------------------
// Filter for calculating metric history stats (values <= 0 are considered invalid)
static bool prv_stats_filter(int index, int32_t value, void *context) {
  return (value > 0);
}

// ------------------------------------------------------------------------------------------------
// Calculates the mean and median of a metric over the entire history we have for it and counts the
// total and consecutive days of history
T_STATIC void prv_calculate_metric_history_stats(ActivityMetric metric,
                                                 ActivityInsightMetricHistoryStats *stats) {
  int32_t *history = kernel_malloc_check(sizeof(int32_t[ACTIVITY_HISTORY_DAYS]));
  activity_get_metric(metric, ACTIVITY_HISTORY_DAYS, history);

  const StatsBasicOp op =
      (StatsBasicOp_Average | StatsBasicOp_Count | StatsBasicOp_ConsecutiveFirst |
       StatsBasicOp_Median);
  struct {
    int32_t mean;
    int32_t count;
    int32_t first_streak;
    int32_t median;
  } result;

  // Note: we ignore history[0] since it's the current day
  stats_calculate_basic(op, &history[1], ACTIVITY_HISTORY_DAYS - 1, prv_stats_filter, NULL,
                        &result.mean);

  *stats = (ActivityInsightMetricHistoryStats) {
    .metric = metric,
    .mean = result.mean,
    .total_days = result.count,
    .consecutive_days = result.first_streak,
    .median = result.median,
  };

  kernel_free(history);

  INSIGHTS_LOG_DEBUG("Metric history stats - med: %"PRIu32" mean: %"PRIu32" tot: %"PRIu8
                     " cons: %"PRIu8, stats->median, stats->mean, stats->total_days,
                     stats->consecutive_days);
}

// -----------------------------------------------------------------------------------------------
// Validates history stats for a given metric against insight settings
static bool prv_validate_history_stats(const ActivityInsightMetricHistoryStats *stats,
                                       const ActivityInsightSettings *insight_settings) {
  // Make sure we have enough history
  if ((stats->total_days < insight_settings->reward.min_days_data) ||
      (stats->consecutive_days < insight_settings->reward.continuous_min_days_data)) {
    INSIGHTS_LOG_DEBUG("History validation failed - total/consecutive days didn't match: "
                       "%"PRIu8" %"PRIu8, stats->total_days, stats->consecutive_days);
    return false;
  }

  // We want to look at the x days before today (which is always index 0), so add 1
  uint32_t history_len = insight_settings->reward.target_qualifying_days + 1;
  if (history_len > ACTIVITY_HISTORY_DAYS) {
    PBL_LOG_ERR("Insight qualifying history length is too long: %"PRIu32,
            history_len);
    return false;
  }

  ActivityScalarStore target =
      ((uint32_t)(stats->median * insight_settings->reward.target_percent_of_median)) / 100;

  int32_t history[history_len];
  activity_get_metric(stats->metric, history_len, history);

  // Make sure enough days have been above the target
  // (start at 1 since we don't care about today's metric)
  for (uint32_t i = 1; i < history_len; ++i) {
    if (history[i] < (int32_t)target) {
      INSIGHTS_LOG_DEBUG("History validation failed - not above target on day %"PRIu32
                         ": %"PRIi32, i, history[i]);
      return false;
    }
  }

  return true;
}



// ------------------------------------------------------------------------------------------------
// This is called during init and midnight rollover in order to update our stats for the sleep
// and activity metrics to include the previous day's history
void activity_insights_recalculate_stats(void) {
  prv_calculate_metric_history_stats(ActivityMetricSleepTotalSeconds, &s_sleep_stats);
  prv_calculate_metric_history_stats(ActivityMetricStepCount, &s_activity_stats);

  // Determine if this history meets the criteria for showing an insight
  s_sleep_reward_state.common.history_valid =
      prv_validate_history_stats(&s_sleep_stats, &s_sleep_reward_settings);
  s_activity_reward_state.common.history_valid =
      prv_validate_history_stats(&s_activity_stats, &s_activity_reward_settings);

  s_activity_reward_state.active_minutes = 0;

  // Reset summary pin data
  s_activity_pin_state = (ActivityPinState) {
    .uuid = UUID_INVALID
  };
}

static ActivitySleepState prv_get_sleep_state(void) {
  int32_t sleep_state;
  activity_get_metric(ActivityMetricSleepState, 1, &sleep_state);
  return sleep_state;
}

// ------------------------------------------------------------------------------------------------
// Checks the common parameters for a given insight to see if it should be triggered
static bool prv_reward_check_common(const ActivityInsightSettings *insight_settings,
                                    const InsightStateCommon *insight_state,
                                    const ActivityInsightMetricHistoryStats *metric_stats,
                                    time_t now_utc) {
  // Make sure the reward is enabled
  if (!insight_settings->enabled) {
    return false;
  }

  // Make sure the previous nights met our criteria
  if (!insight_state->history_valid) {
    return false;
  }

  time_t time_next_trigger = insight_state->last_triggered_utc +
      insight_settings->reward.notif_min_interval_seconds;
  if (time_next_trigger > now_utc) {
    // Stop here if not enough time has passed to trigger this reward
    INSIGHTS_LOG_DEBUG("Not triggering activity reward - too soon to trigger");
    return false;
  }

  // Make sure we're not still sleeping
  if (prv_get_sleep_state() != ActivitySleepStateAwake) {
    INSIGHTS_LOG_DEBUG("Not triggering reward - asleep");
    return false;
  }

  // Finally, make sure the current metric value is over the target
  ActivityScalarStore target = ((uint32_t)(metric_stats->median *
      insight_settings->reward.target_percent_of_median)) / 100;

  int32_t cur_metric;
  activity_get_metric(metric_stats->metric, 1, &cur_metric);
  if (cur_metric < (int32_t)target) {
    INSIGHTS_LOG_DEBUG("Not triggering reward - not over target: %"PRIi32,
                       cur_metric);
    return false;
  }

  return true;
}

// ------------------------------------------------------------------------------------------------
static void prv_do_sleep_reward(time_t now_utc) {
  INSIGHTS_LOG_DEBUG("Checking sleep reward...");
  if (!prv_reward_check_common(&s_sleep_reward_settings, &s_sleep_reward_state.common,
                               &s_sleep_stats, now_utc)) {
    return;
  }

  // Make sure we've been awake long enough
  int32_t sleep_state_seconds;
  activity_get_metric(ActivityMetricSleepStateSeconds, 1, &sleep_state_seconds);
  if (sleep_state_seconds < s_sleep_reward_settings.reward.sleep.trigger_after_wakeup_seconds) {
    INSIGHTS_LOG_DEBUG("Not triggering sleep reward - haven't been awake long enough: %"PRId32,
                       sleep_state_seconds);
    return;
  }

  // All criteria have been met, show reward
  prv_push_reward(now_utc, &SLEEP_REWARD_NOTIF_CONFIG);
}

// -----------------------------------------------------------------------------------------
// Format a time given in seconds after midnight
static void prv_strcat_formatted_time(int32_t time_seconds, char *out_buf, size_t buf_length,
                                      const void *i18n_owner) {
  struct tm time = (struct tm) {
    .tm_hour = time_seconds / SECONDS_PER_HOUR,
    .tm_min = (time_seconds % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE
  };

  const char *format = clock_is_24h_style() ?
      i18n_get("%H:%M", i18n_owner) : i18n_get("%l:%M%p", i18n_owner);

  char time_str_buf[TIME_BUFFER_LENGTH];
  strftime(time_str_buf, TIME_BUFFER_LENGTH, format, &time);
  safe_strcat(out_buf, string_strip_leading_whitespace(time_str_buf), buf_length);
}

// -----------------------------------------------------------------------------------------
// Generates the sleep enter/exit time and total time strings
static void prv_generate_sleep_pin_strings(int32_t sleep_enter_seconds,
                                           int32_t sleep_exit_seconds,
                                           int32_t sleep_total_seconds) {
  SLEEP_SUMMARY_PIN_CONFIG.detail_text[0] = '\0';
  prv_strcat_formatted_time(sleep_enter_seconds, SLEEP_SUMMARY_PIN_CONFIG.detail_text,
                            SUBTITLE_BUFFER_LENGTH, &SLEEP_SUMMARY_PIN_CONFIG);

  safe_strcat(SLEEP_SUMMARY_PIN_CONFIG.detail_text, "-", SUBTITLE_BUFFER_LENGTH);

  prv_strcat_formatted_time(sleep_exit_seconds, SLEEP_SUMMARY_PIN_CONFIG.detail_text,
                            SUBTITLE_BUFFER_LENGTH, &SLEEP_SUMMARY_PIN_CONFIG);

  // Generate short subtitle text with the current step count
  int hours = sleep_total_seconds / SECONDS_PER_HOUR;
  int minutes = (sleep_total_seconds % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE;
  sniprintf(SLEEP_SUMMARY_PIN_CONFIG.short_subtitle, SUBTITLE_BUFFER_LENGTH,
            i18n_get("%uH %uM Sleep", &SLEEP_SUMMARY_PIN_CONFIG), hours, minutes);

  i18n_free_all(&SLEEP_SUMMARY_PIN_CONFIG);
}

// -----------------------------------------------------------------------------------------
static bool prv_push_sleep_summary_pin(time_t now_utc, time_t pin_time_utc,
                                       int32_t sleep_enter_seconds, int32_t sleep_exit_seconds,
                                       int32_t sleep_total_seconds,
                                       ActivityScalarStore sleep_average_seconds, Uuid *uuid) {

  prv_generate_sleep_pin_strings(sleep_enter_seconds, sleep_exit_seconds, sleep_total_seconds);

  // Insert or update the pin
  return prv_push_summary_pin(pin_time_utc, now_utc, uuid, sleep_total_seconds,
                              sleep_average_seconds, &SLEEP_SUMMARY_PIN_CONFIG);
}


// -----------------------------------------------------------------------------------------
static NOINLINE TimelineItem *prv_create_nap_pin(time_t now_utc, ActivitySession *session) {
  AttributeList pin_attr_list = {};

  attribute_list_add_resource_id(&pin_attr_list, AttributeIdIconPin, TIMELINE_RESOURCE_SLEEP);
  attribute_list_add_uint8(&pin_attr_list, AttributeIdHealthInsightType,
                           ActivityInsightType_ActivitySessionNap);
  attribute_list_add_uint8(&pin_attr_list, AttributeIdHealthActivityType,
                           ActivitySessionType_Nap);
  attribute_list_add_uint32(&pin_attr_list, AttributeIdTimestamp, session->start_utc);

  attribute_list_add_cstring(&pin_attr_list, AttributeIdShortTitle,
                             i18n_get("Nap Time", &pin_attr_list));

  // Fits the maximum strings "10H 30M of sleep", "10:00AM - 11:00PM" and i18n variants
  const size_t max_attr_length = 64;
  char *elapsed = kernel_zalloc_check(max_attr_length);
  char *short_subtitle = kernel_zalloc_check(max_attr_length);
  const uint32_t duration_s = session->length_min * SECONDS_PER_MINUTE;
  health_util_format_hours_and_minutes(elapsed, max_attr_length, duration_s, &pin_attr_list);
  const char *short_subtitle_fmt = i18n_get("%s of sleep", &pin_attr_list); /// "10H 30M of sleep"
  snprintf(short_subtitle, max_attr_length, short_subtitle_fmt, elapsed);
  attribute_list_add_cstring(&pin_attr_list, AttributeIdShortSubtitle, short_subtitle);
  attribute_list_add_cstring(&pin_attr_list, AttributeIdSubtitle, elapsed);

  const char *title_i18n = PBL_IF_RECT_ELSE(i18n_noop("YOU NAPPED"),
                                            i18n_noop("Of napping"));
  attribute_list_add_cstring(&pin_attr_list,
                             PBL_IF_RECT_ELSE(AttributeIdTitle, AttributeIdLocationName),
                             i18n_get(title_i18n, &pin_attr_list));

  char *start_time = kernel_zalloc_check(TIME_STRING_TIME_LENGTH);
  char *end_time = kernel_zalloc_check(TIME_STRING_TIME_LENGTH);
  char *time_range = kernel_zalloc_check(max_attr_length);
  const char *time_range_fmt = i18n_get("%s - %s", &pin_attr_list); /// "10:00AM - 11:00PM"
  clock_copy_time_string_timestamp(start_time, TIME_STRING_TIME_LENGTH, session->start_utc);
  clock_copy_time_string_timestamp(end_time, TIME_STRING_TIME_LENGTH,
                                   session->start_utc + duration_s);
  snprintf(time_range, max_attr_length, time_range_fmt, start_time, end_time);
  attribute_list_add_cstring(&pin_attr_list,
                             PBL_IF_RECT_ELSE(AttributeIdLocationName, AttributeIdTitle),
                             time_range);

  // Don't display the time in the title
  attribute_list_add_uint8(&pin_attr_list, AttributeIdDisplayTime, WeatherTimeType_None);
  attribute_list_add_uint32(&pin_attr_list, AttributeIdLastUpdated, now_utc);
  attribute_list_add_uint8(&pin_attr_list, AttributeIdBgColor, GColorSunsetOrangeARGB8);

  const int num_responses = 2;
  ResponseItem *response_items = kernel_zalloc_check(num_responses * sizeof(ResponseItem));
  response_items[0] = (ResponseItem) {
    .type = ActivityInsightResponseTypePositive,
    .text = i18n_noop("I feel great!"),
  };
  response_items[1] = (ResponseItem) {
    .type = ActivityInsightResponseTypeNegative,
    .text = i18n_noop("I need more"),
  };

  TimelineItem *item = prv_create_pin_with_response_items(
      session->start_utc, now_utc, session->length_min, LayoutIdWeather, &pin_attr_list,
      HealthCardType_Sleep, num_responses, response_items);

  kernel_free(response_items);
  kernel_free(time_range);
  kernel_free(end_time);
  kernel_free(start_time);
  kernel_free(short_subtitle);
  kernel_free(elapsed);
  i18n_free_all(&pin_attr_list);
  attribute_list_destroy_list(&pin_attr_list);

  return item;
}

// ------------------------------------------------------------------------------------------------
// Creates a notification to notify the user of the nap session
static void prv_push_nap_session_notification(time_t notif_time, ActivitySession *session,
                                              Uuid *pin_uuid) {
  const int hours = session->length_min / MINUTES_PER_HOUR;
  const int minutes = session->length_min % MINUTES_PER_HOUR;

  // Enough to fit the filled out format string below and i18n variants
  const int max_notif_length = 128;
  char *body = kernel_malloc_check(max_notif_length);
  snprintf(body, max_notif_length,
           i18n_get("Aren't naps great? You knocked out for %dH %dM!", body),
           hours, minutes);
  i18n_free_all(body);
  const NotificationConfig config = {
    .notif_time = notif_time,
    .session = session,
    .insight_type = ActivityInsightType_ActivitySessionNap,
    .icon_id = TIMELINE_RESOURCE_SLEEP,
    .body = body,
    .open_pin = {
      .enabled = true,
      .uuid = pin_uuid,
    },
    .response = {
      .enabled = true,
      .type = ActivityInsightResponseTypeMisclassified,
      .title = i18n_noop("I didn't nap!?"),
    },
  };
  prv_create_and_push_notification(&config);
  kernel_free(body);
}


// -----------------------------------------------------------------------------------------
static void prv_push_nap_session(time_t now_utc, ActivitySession *session) {
  Uuid pin_uuid = UUID_INVALID;
  TimelineItem *pin_item = prv_create_nap_pin(now_utc, session);
  if (prv_push_pin(pin_item, &pin_uuid) &&
      activity_prefs_sleep_insights_are_enabled()) {
    prv_push_nap_session_notification(now_utc, session, &pin_uuid);
  }
}

// -----------------------------------------------------------------------------------------
static void prv_do_sleep_notification(time_t now_utc, time_t sleep_exit_utc,
                                      int32_t sleep_total_seconds) {
  if (!activity_prefs_sleep_insights_are_enabled()) {
    return;
  }

  if (s_sleep_pin_state.notified) {
    INSIGHTS_LOG_DEBUG("Not notifying sleep pin - already notified");
    return;
  }

  // Only notify when the sleep exit lands at a plausible local wake-up time. Skipping does
  // not consume this window's notification, so a real morning wake-up still notifies.
  const int exit_minute_of_day = time_util_get_minute_of_day(sleep_exit_utc);
  if (exit_minute_of_day < SLEEP_SUMMARY_NOTIF_WAKE_MINUTE_MIN ||
      exit_minute_of_day >= SLEEP_SUMMARY_NOTIF_WAKE_MINUTE_MAX) {
    INSIGHTS_LOG_DEBUG("Not notifying sleep pin - exit outside wake window (%d)",
                       exit_minute_of_day);
    return;
  }

  // Notify about the pin after a certain amount of time
  const time_t since_exited = now_utc - sleep_exit_utc;
  if (since_exited < s_sleep_summary_settings.summary.sleep.trigger_notif_seconds) {
    INSIGHTS_LOG_DEBUG("Not notifying sleep pin - not trigger time yet (%ld)", since_exited);
    return;
  }

  // Notify only if they are above the minimum activity since the delay time
  const int trigger_active_minutes =
      s_sleep_summary_settings.summary.sleep.trigger_notif_active_minutes;
  if (s_sleep_pin_state.active_minutes < trigger_active_minutes) {
    INSIGHTS_LOG_DEBUG("Not notifying sleep pin - not active enough (%d < %d)",
                       s_sleep_pin_state.active_minutes, trigger_active_minutes);
    return;
  }

  s_sleep_pin_state.notified = true;
  prv_push_sleep_summary_notification(now_utc, sleep_total_seconds, s_sleep_stats.mean,
                                      VARIANT_RANDOM);

  prv_save_state(ActivitySettingsKeyInsightSleepSummaryState, &s_sleep_pin_state,
                 sizeof(s_sleep_pin_state));
}

// -----------------------------------------------------------------------------------------
static void prv_do_sleep_summary(time_t now_utc) {
  if (!s_sleep_summary_settings.enabled) {
    return;
  }

  // Don't bother adding a summary if we don't have any history for an average
  if (s_sleep_stats.total_days == 0) {
    INSIGHTS_LOG_DEBUG("Not adding sleep pin - no stats");
    return;
  }

  // Make sure we're not still sleeping
  int32_t sleep_state;
  activity_get_metric(ActivityMetricSleepState, 1, &sleep_state);
  if (sleep_state != ActivitySleepStateAwake) {
    INSIGHTS_LOG_DEBUG("Not adding sleep pin - still asleep");
    return;
  }

  // Get the sleep bounds for today and see if we actually have sleep data. The sleep bounds
  // do NOT include naps
  time_t sleep_enter_utc = 0;
  time_t sleep_exit_utc = 0;
  activity_sessions_prv_get_sleep_bounds_utc(now_utc, &sleep_enter_utc, &sleep_exit_utc);
  if (sleep_exit_utc <= sleep_enter_utc) {
    INSIGHTS_LOG_DEBUG("Not adding sleep pin - no sleep data for last night");
    return;
  }

  // The pin state is keyed to the sleep-day window: only a new window invalidates it.
  // Bounds changes within one window (merged sessions, evening extensions) update the pin
  // below without re-arming the notification.
  const time_t window_utc = activity_sessions_prv_get_sleep_window_start_utc(now_utc);
  if (window_utc != s_sleep_pin_state.window_utc
      || now_utc < s_sleep_pin_state.last_triggered_utc) {
    // Checking "now_utc < s_sleep_pin_state.last_triggered_utc" catches cases where
    // the activity_test integration test might have created a pin in the future (because it
    // mucks with the real time clock)
    INSIGHTS_LOG_DEBUG("Starting pin for new sleep window");
    s_sleep_pin_state = (SleepPinState) {
      .uuid = UUID_INVALID,
      .first_enter_utc = sleep_enter_utc,
      .window_utc = window_utc,
    };
  }

  if (s_sleep_pin_state.removed) {
    // If this pin was removed by the user, don't bother updating it.
    INSIGHTS_LOG_DEBUG("Pin was removed");
    return;
  }

  // Get metrics we need
  int32_t sleep_enter_seconds;
  activity_get_metric(ActivityMetricSleepEnterAtSeconds, 1, &sleep_enter_seconds);
  int32_t sleep_exit_seconds;
  activity_get_metric(ActivityMetricSleepExitAtSeconds, 1, &sleep_exit_seconds);
  int32_t sleep_total_seconds = 0;
  activity_get_metric(ActivityMetricSleepTotalSeconds, 1, &sleep_total_seconds);

  // If the bounds haven't changed since we last pushed the pin, send the notification for it
  // now if we haven't already.
  if (sleep_exit_utc <= s_sleep_pin_state.last_triggered_utc
      && sleep_enter_utc == s_sleep_pin_state.first_enter_utc) {
    // Notify about the sleep pin
    prv_do_sleep_notification(now_utc, sleep_exit_utc, sleep_total_seconds);
    INSIGHTS_LOG_DEBUG("Not adding sleep pin - already checked session %ld", sleep_exit_utc);
    return;
  }

  // Insert or update the pin. This intentionally leaves "notified" untouched: within one
  // sleep window the notification fires at most once, however often the bounds extend.
  INSIGHTS_LOG_DEBUG("Adding sleep pin");
  prv_push_sleep_summary_pin(now_utc, sleep_exit_utc, sleep_enter_seconds, sleep_exit_seconds,
                             sleep_total_seconds, s_sleep_stats.mean,
                             &s_sleep_pin_state.uuid);

  // Update sleep pin state
  s_sleep_pin_state.last_triggered_utc = sleep_exit_utc;
  s_sleep_pin_state.first_enter_utc = sleep_enter_utc;
  s_sleep_pin_state.active_minutes = 0;

  prv_save_state(ActivitySettingsKeyInsightSleepSummaryState, &s_sleep_pin_state,
                 sizeof(s_sleep_pin_state));
}

// ------------------------------------------------------------------------------------------------
void NOINLINE activity_insights_process_sleep_data(time_t now_utc) {
  // Check sleep insights
  if (activity_prefs_sleep_insights_are_enabled()) {
    prv_do_sleep_reward(now_utc);
  }

  prv_do_sleep_summary(now_utc);
}

// ------------------------------------------------------------------------------------------------
// Checks to see if we should trigger an activity reward
static NOINLINE void prv_do_activity_reward(time_t now_utc) {
  INSIGHTS_LOG_DEBUG("Checking activity reward...");
  if (!prv_reward_check_common(&s_activity_reward_settings, &s_activity_reward_state.common,
                               &s_activity_stats, now_utc)) {
    return;
  }

  // Make sure the user is currently active
  if (s_activity_reward_state.active_minutes <
      s_activity_reward_settings.reward.activity.trigger_active_minutes) {
    INSIGHTS_LOG_DEBUG("Not showing activity reward - have only been currently active for "
                       "%"PRIu32" minutes out of %"PRIu8, s_activity_reward_state.active_minutes,
                       s_activity_reward_settings.reward.activity.trigger_active_minutes);
    return;
  }

  // All criteria have been met, show reward
  prv_push_reward(now_utc, &ACTIVITY_REWARD_NOTIF_CONFIG);
}

// ------------------------------------------------------------------------------------------------
// Returns the step average corresponding to the current time of day
static ActivityScalarStore prv_cur_step_avg(time_t now_utc, int minute_of_day) {
  // Determine the current chunk
  ActivityMetricAverages *averages = kernel_malloc_check(sizeof(ActivityMetricAverages));
  activity_get_step_averages(time_util_get_day_in_week(now_utc), averages);

  // Sum up the averages
  const int minutes_per_step_avg = MINUTES_PER_DAY / ACTIVITY_NUM_METRIC_AVERAGES;
  int num_chunks = minute_of_day / minutes_per_step_avg;
  ActivityScalarStore total_steps_avg = 0;
  for (int i = 0; i < ACTIVITY_NUM_METRIC_AVERAGES && i < num_chunks; ++i) {
    if (averages->average[i] != ACTIVITY_METRIC_AVERAGES_UNKNOWN) {
      total_steps_avg += averages->average[i];
    }
  }

  kernel_free(averages);

  return total_steps_avg;
}

// ------------------------------------------------------------------------------------------------
// Creates a notification to notify the user of a new pin with a response action
static NOINLINE TimelineItem *prv_create_notification(const NotificationConfig *config) {
  AttributeList notif_attr_list = {};
  ActivitySession *session = config->session;
  prv_build_notification_attr_list(&notif_attr_list, config->body, config->icon_id,
                                   config->insight_type,
                                   session ? session->type : ActivitySessionType_None);

  if (session && session->start_utc) {
    attribute_list_add_uint32(&notif_attr_list, AttributeIdTimestamp, session->start_utc);
  }

  if (config->headings) {
    attribute_list_add_string_list(&notif_attr_list, AttributeIdHeadings, config->headings);
  }
  if (config->values) {
    attribute_list_add_string_list(&notif_attr_list, AttributeIdParagraphs, config->values);
  }

  AttributeList dismiss_action_attr_list = {};
  attribute_list_add_cstring(&dismiss_action_attr_list, AttributeIdTitle,
                             i18n_get("Dismiss", &notif_attr_list));

  AttributeList open_app_action_attr_list = {};
  AttributeList open_pin_action_attr_list = {};
  // Unfortunately, open app and pin both need the parent uuid, so they are mutually exclusive
  // Give open pin more precedence since the pin also links to the app
  if (config->open_pin.enabled) {
    attribute_list_add_cstring(&open_pin_action_attr_list, AttributeIdTitle,
                               i18n_get("Open Pin", &notif_attr_list));
  } else if (config->open_app.enabled) {
    prv_set_open_app_action(&open_app_action_attr_list, config->open_app.health_card_type,
                            &notif_attr_list);
  }

  AttributeList response_action_attr_list = {};
  if (config->response.enabled) {
    attribute_list_add_cstring(&response_action_attr_list, AttributeIdTitle,
                               i18n_get(config->response.title, &notif_attr_list));
  }

  const int max_num_actions = 4; // dismiss, open app, open pin, response
  int num_actions = 0;
  int action_id = 0;
  TimelineItemAction actions[max_num_actions];
  actions[num_actions++] = (TimelineItemAction) {
    .id = action_id++,
    .type = TimelineItemActionTypeDismiss,
    .attr_list = dismiss_action_attr_list,
  };
  if (config->open_pin.enabled) {
    actions[num_actions++] = (TimelineItemAction) {
      .id = action_id++,
      .type = TimelineItemActionTypeOpenPin,
      .attr_list = open_pin_action_attr_list,
    };
  } else if (config->open_app.enabled) {
    actions[num_actions++] = (TimelineItemAction) {
      .id = action_id++,
      .type = TimelineItemActionTypeOpenWatchApp,
      .attr_list = open_app_action_attr_list,
    };
  }
  if (config->response.enabled) {
    actions[num_actions++] = (TimelineItemAction) {
      .id = config->response.type,
      .type = TimelineItemActionTypeInsightResponse,
      .attr_list = response_action_attr_list,
    };
  }
  TimelineItemActionGroup action_group = {
    .num_actions = num_actions,
    .actions = actions,
  };

  // Note: it's fine if this returns null, since the parent functions will check for a null pointer
  TimelineItem *item = timeline_item_create_with_attributes(config->notif_time, 0,
                                                            TimelineItemTypeNotification,
                                                            LayoutIdNotification, &notif_attr_list,
                                                            &action_group);
  i18n_free_all(&notif_attr_list);
  attribute_list_destroy_list(&notif_attr_list);
  attribute_list_destroy_list(&dismiss_action_attr_list);
  attribute_list_destroy_list(&open_pin_action_attr_list);
  attribute_list_destroy_list(&response_action_attr_list);

  return item;
}

// ------------------------------------------------------------------------------------------------
// Creates a notification to notify the user of a new pin with a response action
static void prv_create_and_push_notification(const NotificationConfig *config) {
  TimelineItem *item = prv_create_notification(config);
  prv_push_notification(item, (config->open_pin.enabled && config->open_pin.uuid ?
                               config->open_pin.uuid : NULL));
}

// ------------------------------------------------------------------------------------------------
// Get the current step count metric
static int32_t prv_get_step_count(void) {
  int32_t steps;
  activity_get_metric(ActivityMetricStepCount, 1, &steps);
  return steps;
}

// ------------------------------------------------------------------------------------------------
// Creates a notification to notify the user of the activity summary
static void prv_push_activity_summary_notification(
    time_t notif_time, int32_t steps_total, int32_t steps_average, int variant) {
  static InsightCopyVariants s_tier_config[PercentTierCount] = {
    [PercentTier_AboveAverage] = {
      .num_variants = 5,
      .variants = {
        i18n_noop("Killer job! You've walked %d steps today which is %d%% above your typical. "
                  "Do it again tomorrow and you'll be on top of the world!"),
        i18n_noop("Nice moves! You've walked %d steps today which is %d%% above your typical. "
                  "Crush it again tomorrow 😀"),
        i18n_noop("Hey rockstar 🎤 You've walked %d steps today "
                  "which is %d%% above your typical. Nothing can stop you!"),
        i18n_noop("We can barely keep up! "
                  "You walked %d steps today which is %d%% above your typical. "
                  "You're on 🔥"),
        i18n_noop("You walked %d steps today which is %d%% above your typical. "
                  "You just outstepped YOURSELF. Mic drop."),
      },
    },
    [PercentTier_OnAverage] = {
      .num_variants = 4,
      .variants = {
        i18n_noop("You walked %d steps today; keep it up! "
                  "Being active is the key to feeling like a million bucks. "
                  "Try to beat your typical tomorrow."),
        i18n_noop("Good job! You walked %d steps today–do it again tomorrow."),
        i18n_noop("Someone's on the move 👊 You walked %d steps today; "
                  "keep doing what you're doing!"),
        i18n_noop("You keep moving, we'll keep counting! You've clocked in %d steps today. "
                  "Keep it rolling, hot stuff."),
      },
    },
    [PercentTier_BelowAverage] = {
      .num_variants = 4,
      .variants = {
        i18n_noop("You walked %d steps today which is %d%% below your typical. "
                  "Try to be more active tomorrow–you can do it!"),
        i18n_noop("You walked %d steps which is %d%% below your typical. "
                  "Being active makes you feel amaaaazing–try to get back on track tomorrow."),
        i18n_noop("You walked %d steps today which is %d%% below your typical. "
                  "Don't worry, tomorrow is just around the corner 😀"),
        i18n_noop("You walked %d steps which is %d%% below your typical, "
                  "but don't stress. You'll crush it tomorrow 😉"),
      },
    },
    [PercentTier_Fail] = {
      .num_variants = 3,
      .variants = {
        i18n_noop("You walked %d steps today. "
                  "Don't fret, you can get back on track in no time 😉"),
        i18n_noop("You walked %d steps today. Good news is the sky's the limit!"),
        i18n_noop("You walked %d steps today. "
                  "Try to take even more steps tomorrow–show us what you're made of!"),
      },
    },
  };

  int percentage;
  PercentTier tier = prv_calc_percent_tier(&ACTIVITY_SUMMARY_PIN_CONFIG,
                                           steps_total,
                                           steps_average, &percentage);
  const bool above_fail_threshold =
      steps_total >= ACTIVITY_SUMMARY_PIN_CONFIG.insight_settings->summary.activity.max_fail_steps;
  if ((tier == PercentTier_BelowAverage || tier == PercentTier_Fail) && above_fail_threshold) {
    // we don't want to show a negative insights if you've walked 10000 or more steps
    tier = PercentTier_OnAverage;
  }

  // Enough to fit any filled out format string above and i18n variants
  const int max_notif_length = 256;
  const char *fmt = prv_get_variant(&s_tier_config[tier], variant);
  if (fmt == NULL) {
    return;
  }

  char *body = kernel_malloc_check(max_notif_length);
  snprintf(body, max_notif_length, i18n_get(fmt, body), steps_total, ABS(percentage));
  i18n_free_all(body);

  const NotificationConfig config = {
    .notif_time = notif_time,
    .insight_type = ActivityInsightType_ActivitySummary,
    .icon_id = TIMELINE_RESOURCE_ACTIVITY,
    .body = body,
    .open_app = {
      .enabled = true,
      .health_card_type = HealthCardType_Activity,
    },
  };
  prv_create_and_push_notification(&config);
  kernel_free(body);
}


// ------------------------------------------------------------------------------------------------
// Creates a notification to notify the user of the sleep summary
static void prv_push_sleep_summary_notification(time_t notif_time, int32_t sleep_total_seconds,
                                                int32_t sleep_average_seconds, int variant) {
  static InsightCopyVariants s_tier_config[PercentTierCount] = {
    [PercentTier_AboveAverage] = {
      .num_variants = 5,
      .variants = {
        /// Sleep notification on wake up, slept above their typical sleep duration
        i18n_noop("Refreshed? You slept for %dH %dM which is %d%% above your typical. "
                  "Go tackle your day 😃."),
        i18n_noop("You caught some killer zzz’s! You slept for %dH %dM which is %d%% above "
                  "your typical. Keep it up."),
        i18n_noop("Rise and shine! You slept for %dH %dM which is %d%% above your typical. "
                  "Do it again tonight, sleep master."),
        i18n_noop("Mmmm...what a night. You slept for %dH %dM which is %d%% above your typical. "
                  "That's gotta feel good!"),
        i18n_noop("That's a lot of sheep you just counted! "
                  "You slept for %dH %dM which is %d%% above your typical. Boom shakalaka."),
      },
    },
    [PercentTier_OnAverage] = {
      .num_variants = 4,
      .variants = {
        /// Sleep notification on wake up, slept similar to their typical sleep duration
        i18n_noop("Good mornin’. You slept for %dH %dM. "
                  "Every good day begins with a solid night’s sleep..."
                  "kinda like that one 😉 Keep it up!"),
        i18n_noop("Good morning! You slept for %dH %dM. Consistency is key; "
                  "keep doing what you're doing 😃."),
        i18n_noop("You're rockin' the shut eye! You slept for %dH %dM. Make it a nightly ritual. "
                  "You deserve it."),
        i18n_noop("Feelin' good? You slept for %dH %dM. Nothing can stop you now 👊"),
      },
    },
    [PercentTier_BelowAverage] = {
      .num_variants = 4,
      .variants = {
        /// Sleep notification on wake up, slept below their typical sleep duration
        i18n_noop("Hey sleepy head. You slept for %dH %dM which "
                  "is %d%% below your typical. Try to get more tonight!"),
        i18n_noop("Groggy? You slept for %dH %dM which is %d%% below your typical. Sleep "
                  "is important for everything you do-try getting more shut eye tonight!"),
        i18n_noop("It's a new day! You slept for %dH %dM which is %d%% below your typical. "
                  "It's not your best, but there's always tonight."),
        i18n_noop("Goooood morning! You slept for %dH %dM which is %d%% below your typical. "
                  "Go crush your day and then get back in bed 😉"),
      },
    },
    [PercentTier_Fail] = {
      .num_variants = 3,
      .variants = {
        i18n_noop("You slept for %dH %dM which is %d%% below your typical. "
                  "Sleep is vital for all your great ideas–how 'bout getting more tonight?"),
        i18n_noop("You only slept for %dH %dM which is %d%% below your typical. "
                  "We know you're busy, but try getting more tonight. We believe in you 😉"),
        i18n_noop("You slept for %dH %dM which is %d%% below your typical. "
                  "We know stuff happens; take another crack at it tonight."),
      },
    },
  };

  const int hours = sleep_total_seconds / SECONDS_PER_HOUR;
  const int minutes = (sleep_total_seconds / SECONDS_PER_MINUTE) % MINUTES_PER_HOUR;
  int percentage;
  PercentTier tier = prv_calc_percent_tier(&SLEEP_SUMMARY_PIN_CONFIG , sleep_total_seconds,
                                                 sleep_average_seconds, &percentage);

  if ((tier == PercentTier_BelowAverage || tier == PercentTier_Fail) &&
      sleep_total_seconds / SECONDS_PER_MINUTE >=
      SLEEP_SUMMARY_PIN_CONFIG.insight_settings->summary.sleep.max_fail_minutes) {
    // we don't want to show a negative insights if you've slept 7 hours
    tier = PercentTier_OnAverage;
  }

  // Enough to fit any filled out format string above and i18n variants
  const int max_notif_length = 256;
  const char *fmt = prv_get_variant(&s_tier_config[tier], variant);
  if (fmt == NULL) {
    // invalid variant
    return;
  }

  char *body = kernel_malloc_check(max_notif_length);
  snprintf(body, max_notif_length, i18n_get(fmt, body), hours, minutes, ABS(percentage));
  i18n_free_all(body);

  const NotificationConfig config = {
    .notif_time = notif_time,
    .insight_type = ActivityInsightType_SleepSummary,
    .icon_id = TIMELINE_RESOURCE_SLEEP,
    .body = body,
    .open_app = {
      .enabled = true,
      .health_card_type = HealthCardType_Sleep,
    },
  };

  prv_create_and_push_notification(&config);
  kernel_free(body);
}

// ------------------------------------------------------------------------------------------------
static bool prv_push_activity_summary_pin(time_t now_utc, time_t pin_time_utc, int minute_of_day,
                                          ActivityScalarStore steps,
                                          ActivityScalarStore total_steps_avg,
                                          Uuid *uuid) {
  sniprintf(ACTIVITY_SUMMARY_PIN_CONFIG.short_subtitle, SUBTITLE_BUFFER_LENGTH,
            i18n_get("%u Steps", &ACTIVITY_SUMMARY_PIN_CONFIG), steps);

  i18n_free_all(&ACTIVITY_SUMMARY_PIN_CONFIG);

  return prv_push_summary_pin(pin_time_utc, now_utc, uuid, steps, total_steps_avg,
                              &ACTIVITY_SUMMARY_PIN_CONFIG);
}

// ------------------------------------------------------------------------------------------------
// Checks to see if we should add/update an activity summary pin
static NOINLINE void prv_do_activity_summary(time_t now_utc) {
  if (!s_activity_summary_settings.enabled) {
    return;
  }

  // Don't bother adding a summary if we don't have any history for an average
  if (s_activity_stats.total_days == 0) {
    return;
  }

  // Make sure it's not before the trigger time and the pin hasn't already been removed
  const int minute_of_day = time_util_get_minute_of_day(now_utc);
  if ((minute_of_day < s_activity_summary_settings.summary.activity.trigger_minute) ||
      s_activity_pin_state.removed) {
    INSIGHTS_LOG_DEBUG("Not adding activity pin - before trigger time (%d < %d) or removed (%d)",
                       minute_of_day, s_activity_summary_settings.summary.activity.trigger_minute,
                       s_activity_pin_state.removed);
    return;
  }

  // Make sure we actually have a step count
  const int32_t steps = prv_get_step_count();
  if (steps <= 0) {
    INSIGHTS_LOG_DEBUG("Not adding activity pin - no steps");
    return;
  }

  // Make sure we're overdue for an update (either time interval or change in steps)
  const time_t next_update_time = s_activity_pin_state.next_update_time;
  ActivityScalarStore next_step_count = s_activity_pin_state.next_step_count;
  if ((now_utc < next_update_time) && (steps < (int32_t)next_step_count)) {
    INSIGHTS_LOG_DEBUG("Not updating activity pin - less than next update time and next steps");
    return;
  }

  s_activity_pin_state.next_update_time =
      now_utc + s_activity_summary_settings.summary.activity.update_max_interval_seconds;
  s_activity_pin_state.next_step_count =
      steps + s_activity_summary_settings.summary.activity.update_threshold_steps;

  // Determine the average for today
  ActivityScalarStore total_steps_avg = prv_cur_step_avg(now_utc, minute_of_day);

  const time_t pin_time_utc = time_util_get_midnight_of(now_utc) +
          (s_activity_summary_settings.summary.activity.trigger_minute * SECONDS_PER_MINUTE);

  if (prv_push_activity_summary_pin(now_utc, pin_time_utc, minute_of_day, steps, total_steps_avg,
                                    &s_activity_pin_state.uuid)) {
    SummaryPinLastState activity_pin_last_state = {
      .uuid = s_activity_pin_state.uuid,
      .last_triggered_utc = now_utc,
    };
    prv_save_state(ActivitySettingsKeyInsightActivitySummaryState, &activity_pin_last_state,
                   sizeof(activity_pin_last_state));

    // Trigger a notification to go with the new pin (only if we're at the trigger time)
    if (activity_prefs_activity_insights_are_enabled() &&
        (minute_of_day == s_activity_summary_settings.summary.activity.trigger_minute)  &&
        s_activity_summary_settings.summary.activity.show_notification) {
      prv_push_activity_summary_notification(pin_time_utc, steps, total_steps_avg, VARIANT_RANDOM);
    }
  }
}

// ------------------------------------------------------------------------------------------------
static const char* prv_get_intro_str_for_activity(ActivitySession *session) {
  switch (session->type) {
    case ActivitySessionType_Walk: {
      static const InsightCopyVariants s_walking_intros = {
        .num_variants = 4,
        .variants = {
          i18n_noop("Didn’t that walk feel good?"),
          i18n_noop("Way to keep it active!"),
          i18n_noop("You got the moves!"),
          i18n_noop("Gettin' your step on?"),
        },
      };
      return prv_get_variant(&s_walking_intros, VARIANT_RANDOM);
    }

    case ActivitySessionType_Run: {
      static const InsightCopyVariants s_running_intros = {
        .num_variants = 5,
        .variants = {
          i18n_noop("Feelin' hot? Cause you're on 🔥"),
          i18n_noop("Hey lightning bolt, way to go!"),
          i18n_noop("You're a machine!"),
          i18n_noop("Hey speedster, we can barely keep up!"),
          i18n_noop("Way to show us what you're made of 👊"),
        },
      };
      return prv_get_variant(&s_running_intros, VARIANT_RANDOM);
    }

    case ActivitySessionType_Open: {
      static const InsightCopyVariants s_open_intros = {
        .num_variants = 5,
        .variants = {
          i18n_noop("Workin' up a sweat?"),
          i18n_noop("Well done 💪"),
          i18n_noop("Endorphin rush?"),
          i18n_noop("Can't stop, won't stop 👊"),
          i18n_noop("Keepin' that heart healthy! ❤"),
        },
      };
      return prv_get_variant(&s_open_intros, VARIANT_RANDOM);
    }

    default: {
      break;
    }
  }

  return "";
}

// ------------------------------------------------------------------------------------------------
static const char *prv_get_distance_unit(void *i18n_owner) {
  return health_util_get_distance_string(i18n_get("mi", i18n_owner), i18n_get("km", i18n_owner));
}

static void prv_add_metric_duration_info(StringList *headings, int headings_buf_size,
                                         StringList *values, int values_buf_size,
                                         ActivitySession *session) {
  const size_t duration_buffer_size = sizeof("00:00:00");
  char duration_str[duration_buffer_size];

  const int duration_s = session->length_min * SECONDS_PER_MINUTE;
  const int duration_m = ROUND(duration_s, SECONDS_PER_MINUTE);
  if (duration_m <= MINUTES_PER_HOUR) {
    snprintf(duration_str, duration_buffer_size, i18n_get("%d Min", headings), duration_m);
  } else {
    health_util_format_hours_and_minutes(duration_str, duration_buffer_size,
                                         duration_s, headings);
  }

  const char *activity_label = i18n_get(workout_utils_get_name_for_activity(session->type),
                                        headings);
  string_list_add_string(headings, headings_buf_size, activity_label, headings_buf_size);
  string_list_add_string(values, values_buf_size, duration_str, values_buf_size);
}

static void prv_add_avg_pace_metric_info(StringList *headings, int headings_buf_size,
                                         StringList *values, int values_buf_size,
                                         ActivitySession *session) {
  const int pace_buf_size = 16;
  char pace_str[pace_buf_size];
  const int pace_s = health_util_get_pace(session->length_min * SECONDS_PER_MINUTE,
                                          session->step_data.distance_meters);
  int offset = health_util_format_hours_minutes_seconds(pace_str, pace_buf_size, pace_s,
                                                        false, headings);

  snprintf(pace_str + offset, pace_buf_size - offset, " /%s", prv_get_distance_unit(headings));

  string_list_add_string(headings, headings_buf_size,
                         i18n_get("Avg Pace", headings), headings_buf_size);
  string_list_add_string(values, values_buf_size, pace_str, values_buf_size);
}

static void prv_add_distance_metric_info(StringList *headings, int headings_buf_size,
                                         StringList *values, int values_buf_size,
                                         ActivitySession *session) {
  const size_t distance_buf_size = 8;
  char distance_str[distance_buf_size];
  int offset = health_util_format_distance(distance_str, distance_buf_size,
                                           session->step_data.distance_meters);

  snprintf(distance_str + offset, distance_buf_size - offset,
           " %s", prv_get_distance_unit(headings));

  string_list_add_string(headings, headings_buf_size,
                         i18n_get("Distance", headings), headings_buf_size);
  string_list_add_string(values, values_buf_size, distance_str, values_buf_size);
}

static void prv_add_step_metric_info(StringList *headings, int headings_buf_size,
                                     StringList *values, int values_buf_size,
                                     ActivitySession *session) {
  const size_t step_buf_size = 8;
  char step_str[step_buf_size];
  snprintf(step_str, step_buf_size, "%d", session->step_data.steps);

  string_list_add_string(headings, headings_buf_size,
                         i18n_get("Steps", headings), headings_buf_size);
  string_list_add_string(values, values_buf_size, step_str, values_buf_size);
}

static void prv_add_active_calories_metric_info(StringList *headings, int headings_buf_size,
                                                StringList *values, int values_buf_size,
                                                ActivitySession *session) {
  const size_t calories_buf_size = 8;
  char calories_str[calories_buf_size];
  const int active_calories = session->step_data.active_kcalories;
  snprintf(calories_str, calories_buf_size, "%d", active_calories);

  string_list_add_string(headings, headings_buf_size,
                         i18n_get("Active Calories", headings), headings_buf_size);
  string_list_add_string(values, values_buf_size, calories_str, values_buf_size);
}

static void prv_add_hr_metric_info(StringList *headings, int headings_buf_size,
                                   StringList *values, int values_buf_size,
                                   int32_t avg_hr, int32_t *hr_zone_time_s) {
  const size_t hr_buf_size = 8;
  char hr_str[hr_buf_size];

  if (avg_hr) {
    snprintf(hr_str, hr_buf_size, "%"PRIi32"", avg_hr);
    string_list_add_string(headings, headings_buf_size,
                           i18n_get("Avg HR", headings), headings_buf_size);
    string_list_add_string(values, values_buf_size, hr_str, values_buf_size);
  }

  if (hr_zone_time_s) {
    const int zone_1_minutes = ROUND(hr_zone_time_s[HRZone_Zone1], SECONDS_PER_MINUTE);
    if (zone_1_minutes) {
      snprintf(hr_str, hr_buf_size, i18n_get("%d Min", headings), zone_1_minutes);
      string_list_add_string(headings, headings_buf_size,
                             i18n_get("Fat Burn", headings), headings_buf_size);
      string_list_add_string(values, values_buf_size, hr_str, values_buf_size);
    }
    const int zone_2_minutes = ROUND(hr_zone_time_s[HRZone_Zone2], SECONDS_PER_MINUTE);
    if (zone_2_minutes) {
      snprintf(hr_str, hr_buf_size, i18n_get("%d Min", headings), zone_2_minutes);
      string_list_add_string(headings, headings_buf_size,
                             i18n_get("Endurance", headings), headings_buf_size);
      string_list_add_string(values, values_buf_size, hr_str, values_buf_size);
    }
    const int zone_3_minutes = ROUND(hr_zone_time_s[HRZone_Zone3], SECONDS_PER_MINUTE);
    if (zone_3_minutes) {
      snprintf(hr_str, hr_buf_size, i18n_get("%d Min", headings), zone_3_minutes);
      string_list_add_string(headings, headings_buf_size,
                             i18n_get("Performance", headings), headings_buf_size);
      string_list_add_string(values, values_buf_size, hr_str, values_buf_size);
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Creates a notification to notify the user of the activity session
void activity_insights_push_activity_session_notification(time_t notif_time,
                                                          ActivitySession *session,
                                                          int32_t avg_hr,
                                                          int32_t *hr_zone_time_s) {
  if (session->length_min <= 0) {
    return;
  }

  ActivityInsightType type;
  TimelineResourceId icon;

  const int body_buf_size = 100;
  char *body = kernel_zalloc_check(body_buf_size);
  const char *intro_str = prv_get_intro_str_for_activity(session);
  snprintf(body, body_buf_size, "%s", i18n_get(intro_str, body));

  const int headings_buf_size = 128;
  StringList *headings = kernel_zalloc_check(headings_buf_size);

  const int values_buf_size = 128;
  StringList *values = kernel_zalloc_check(values_buf_size);


  if (session->type == ActivitySessionType_Run) {
    type = ActivityInsightType_ActivitySessionRun;
    icon = TIMELINE_RESOURCE_RUN;

    prv_add_metric_duration_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_avg_pace_metric_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_distance_metric_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_active_calories_metric_info(headings, headings_buf_size,
                                        values, values_buf_size, session);
    prv_add_hr_metric_info(headings, headings_buf_size, values, values_buf_size,
                           avg_hr, hr_zone_time_s);
  } else if (session->type == ActivitySessionType_Walk) {
    type = ActivityInsightType_ActivitySessionWalk;
    icon = TIMELINE_RESOURCE_ACTIVITY;

    prv_add_metric_duration_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_avg_pace_metric_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_distance_metric_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_step_metric_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_active_calories_metric_info(headings, headings_buf_size,
                                        values, values_buf_size, session);
    prv_add_hr_metric_info(headings, headings_buf_size, values, values_buf_size,
                           avg_hr, hr_zone_time_s);
  } else if (session->type == ActivitySessionType_Open) {
    type = ActivityInsightType_ActivitySessionOpen;
    icon = TIMELINE_RESOURCE_HEART;

    prv_add_metric_duration_info(headings, headings_buf_size, values, values_buf_size, session);
    prv_add_hr_metric_info(headings, headings_buf_size, values, values_buf_size,
                           avg_hr, hr_zone_time_s);
  } else {
    // Unsupported activity type
    goto cleanup;
  }


  const NotificationConfig config = {
    .notif_time = notif_time,
    .insight_type = type,
    .icon_id = icon,
    .body = body,
    .headings = headings,
    .values = values,
    .open_app = {
      .enabled = true,
      .health_card_type = HealthCardType_Activity,
    },
  };

  prv_create_and_push_notification(&config);

cleanup:
  i18n_free_all(body);
  i18n_free_all(headings);
  i18n_free_all(values);
  kernel_free(body);
  kernel_free(headings);
  kernel_free(values);
}

// ------------------------------------------------------------------------------------------------
static void prv_do_activity_session(time_t now_utc, ActivitySession *session) {
  if (!activity_prefs_activity_insights_are_enabled()) {
    return;
  }

  if (!s_activity_session_settings.enabled) {
    return;
  }

  if (s_session_pin_state.start_utc >= session->start_utc) {
    INSIGHTS_LOG_DEBUG("Not adding session pin - session too old");
    return;
  }

  if (now_utc - (session->start_utc + SECONDS_PER_MINUTE * session->length_min) <
      s_activity_session_settings.session.activity.trigger_cooldown_minutes * SECONDS_PER_MINUTE) {
    INSIGHTS_LOG_DEBUG("Not adding session pin - cooldown not yet elapsed");
    return;
  }

  if (prv_get_sleep_state() != ActivitySleepStateAwake) {
    INSIGHTS_LOG_DEBUG("Not adding session pin - asleep");
    return;
  }

  if (session->length_min < s_activity_session_settings.session.activity.trigger_elapsed_minutes) {
    INSIGHTS_LOG_DEBUG("Not adding session pin - not long enough (%"PRIu16" < %"PRIu16")",
                       session->length_min,
                       s_activity_session_settings.session.activity.trigger_elapsed_minutes);
    return;
  }

  if (session->manual) {
    // The workout service will handle the notifications for these
    return;
  }

  s_session_pin_state.start_utc = session->start_utc;

  prv_save_state(ActivitySettingsKeyInsightActivitySessionTime,
                 &s_session_pin_state.start_utc,
                 sizeof(s_session_pin_state.start_utc));

  if (s_activity_session_settings.session.show_notification) {
    activity_insights_push_activity_session_notification(now_utc, session, 0, NULL);
  }
}

// ------------------------------------------------------------------------------------------------
void prv_process_activity_sessions(time_t now_utc) {
  uint32_t num_sessions = ACTIVITY_MAX_ACTIVITY_SESSIONS_COUNT;
  ActivitySession *sessions = task_zalloc_check(num_sessions * sizeof(ActivitySession));

  if (activity_get_sessions(&num_sessions, sessions)) {
    for (unsigned int i = 0; i < num_sessions; i++) {
      if (sessions[i].ongoing) {
        // Don't process incomplete events
        continue;
      }
      switch (sessions[i].type) {
        case ActivitySessionType_Nap:
          // PBL-36355 Disable nap notifications
          // Nap notifications are disabled until we get better at detecting naps
          // Re-enable nap session unit tests when re-enabling nap session notifications
          break;
        case ActivitySessionType_Walk:
        case ActivitySessionType_Run:
          prv_do_activity_session(now_utc, &sessions[i]);
          break;
        default:
          break;
      }
    }
  }

  task_free(sessions);
}

// ------------------------------------------------------------------------------------------------
void NOINLINE activity_insights_process_minute_data(time_t now_utc) {
  // Update our active stats - needs to happen each iteration to ensure it's current
  // If we're above the 'active' threshold, increment the number of consecutive active minutes,
  // otherwise, reset to 0
  if (activity_metrics_prv_steps_per_minute() >
      s_activity_reward_settings.reward.activity.trigger_steps_per_minute) {
    s_activity_reward_state.active_minutes++;
  } else {
    s_activity_reward_state.active_minutes = 0;
  }

  if (activity_metrics_prv_steps_per_minute() >
      s_sleep_summary_settings.summary.sleep.trigger_notif_activity) {
    s_sleep_pin_state.active_minutes++;
  }

  // Check activity insights
  if (activity_prefs_activity_insights_are_enabled()) {
    prv_do_activity_reward(now_utc);
  }

  prv_do_activity_summary(now_utc);
  prv_process_activity_sessions(now_utc);
}

// ------------------------------------------------------------------------------------------------
static void prv_settings_file_changed_cb(void *data);

// Reloads the reward settings from flash and caches them
static void prv_reload_settings(void *not_used) {
  INSIGHTS_LOG_DEBUG("Reloading insights settings");

  if (s_pfs_cb_handle) {
    activity_insights_settings_unwatch(s_pfs_cb_handle);
  }

  if (!activity_insights_settings_read(ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_REWARD,
                                       &s_activity_reward_settings)) {
    s_activity_reward_settings.enabled = false; // worst-case, we disable the insight
  }

  if (!activity_insights_settings_read(ACTIVITY_INSIGHTS_SETTINGS_SLEEP_REWARD,
                                       &s_sleep_reward_settings)) {
    s_sleep_reward_settings.enabled = false; // worst-case, we disable the insight
  }

  if (!activity_insights_settings_read(ACTIVITY_INSIGHTS_SETTINGS_SLEEP_SUMMARY,
                                       &s_sleep_summary_settings)) {
    s_sleep_summary_settings.enabled = false; // worst-case, we disable the insight
  }

  if (!activity_insights_settings_read(ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SUMMARY,
                                       &s_activity_summary_settings)) {
    s_activity_summary_settings.enabled = false; // worst-case, we disable the insight
  }

  if (!activity_insights_settings_read(ACTIVITY_INSIGHTS_SETTINGS_ACTIVITY_SESSION,
                                       &s_activity_session_settings)) {
    s_activity_session_settings.enabled = false; // worst-case, we disable the insight
  }

  s_pfs_cb_handle = activity_insights_settings_watch(prv_settings_file_changed_cb);
}

static void prv_settings_file_changed_cb(void *data) {
  system_task_add_callback(prv_reload_settings, data);
}

static void prv_blobdb_event_handler(PebbleEvent *event, void *context) {
  PebbleBlobDBEvent *blobdb_event = &event->blob_db;
  if (blobdb_event->db_id != BlobDBIdPins) {
    // we only care about pins
    return;
  }

  BlobDBEventType type = blobdb_event->type;
  Uuid *id = (Uuid *)blobdb_event->key;
  if (type == BlobDBEventTypeDelete) {
    if (uuid_equal(id, &s_activity_pin_state.uuid)) {
      s_activity_pin_state.removed = true;
    } else if (uuid_equal(id, &s_sleep_pin_state.uuid)) {
      s_sleep_pin_state.removed = true;
    }
  }
}

// ------------------------------------------------------------------------------------------------
void activity_insights_init(time_t now_utc) {
  // Init insight settings file support
  activity_insights_settings_init();

  // Cache the settings so we don't hit flash every minute
  prv_reload_settings(NULL);

  // Subscribe to pin removal events
  s_blobdb_event_info = (EventServiceInfo) {
    .type = PEBBLE_BLOBDB_EVENT,
    .handler = prv_blobdb_event_handler,
  };
  event_service_client_subscribe(&s_blobdb_event_info);

  // Load in previous saved state
  // No need to lock mutex since this should only be called from activity_init
  SummaryPinLastState activity_pin_last_state;
  SettingsFile *file = activity_private_settings_open();
  if (file) {
    prv_restore_state(file, ActivitySettingsKeyInsightSleepRewardTime,
                      &s_sleep_reward_state.common.last_triggered_utc,
                      sizeof(s_sleep_reward_state.common.last_triggered_utc));
    prv_restore_state(file, ActivitySettingsKeyInsightActivityRewardTime,
                        &s_activity_reward_state.common.last_triggered_utc,
                        sizeof(s_activity_reward_state.common.last_triggered_utc));
    prv_restore_state(file, ActivitySettingsKeyInsightActivitySummaryState,
                      &activity_pin_last_state, sizeof(activity_pin_last_state));
    prv_restore_state(file, ActivitySettingsKeyInsightSleepSummaryState,
                      &s_sleep_pin_state, sizeof(s_sleep_pin_state));
    prv_restore_state(file, ActivitySettingsKeyInsightNapSessionTime,
                      &s_nap_pin_state.last_triggered_utc,
                      sizeof(s_nap_pin_state.last_triggered_utc));
    prv_restore_state(file, ActivitySettingsKeyInsightActivitySessionTime,
                      &s_session_pin_state.start_utc,
                      sizeof(s_session_pin_state.start_utc));
    activity_private_settings_close(file);
  }

  // A restored sleep pin state always has a valid window. If it doesn't (fresh boot, factory
  // reset or a state layout change), treat the in-progress window as already notified so the
  // next bounds recalculation can't re-notify for sleep the user already saw.
  if (s_sleep_pin_state.window_utc == 0) {
    s_sleep_pin_state.window_utc = activity_sessions_prv_get_sleep_window_start_utc(now_utc);
    s_sleep_pin_state.notified = true;
  }

  INSIGHTS_LOG_DEBUG("Last sleep reward state: %ld",
                     s_sleep_reward_state.common.last_triggered_utc);
  INSIGHTS_LOG_DEBUG("Last activity reward state: %ld",
                     s_activity_reward_state.common.last_triggered_utc);

  // Recalculate metric stats
  activity_insights_recalculate_stats();

  // If the pin we loaded was created on the previous day, we don't bother loading the UUID
  const time_t midnight_today = time_util_get_midnight_of(now_utc);
  if (time_util_get_midnight_of(activity_pin_last_state.last_triggered_utc) == midnight_today) {
    s_activity_pin_state.uuid = activity_pin_last_state.uuid;

    // Check if this pin has already been removed
    if (!timeline_exists(&s_activity_pin_state.uuid)) {
      s_sleep_pin_state.removed = true;
    }
  }
}

// ------------------------------------------------------------------------------------------------
// QA Testing functions
static void prv_test_push_summary_pins(void *unused) {
  time_t now_utc = rtc_get_time();
  int minute_of_day = (20 * MINUTES_PER_HOUR) + 30; // Activity pins only trigger after 8:30

  Uuid uuid_way_below = UUID_INVALID;
  Uuid uuid_below = UUID_INVALID;
  Uuid uuid = UUID_INVALID;
  Uuid uuid_above = UUID_INVALID;

  prv_push_activity_summary_pin(now_utc, now_utc, minute_of_day, 12345, 8000, &uuid_above);
  prv_push_activity_summary_pin(now_utc, now_utc, minute_of_day, 12345, 12345, &uuid);
  prv_push_activity_summary_pin(now_utc, now_utc, minute_of_day, 12345, 20000, &uuid_below);
  prv_push_activity_summary_pin(now_utc, now_utc, minute_of_day, 12345, 50000, &uuid_way_below);

  if (activity_prefs_activity_insights_are_enabled()) {
    for (int i = 0; i < NUM_COPY_VARIANTS; ++i) {
      prv_push_activity_summary_notification(now_utc, 1234, 10000, i);
      prv_push_activity_summary_notification(now_utc, 1234, 2000, i);
      prv_push_activity_summary_notification(now_utc, 1234, 1234, i);
      prv_push_activity_summary_notification(now_utc, 1234, 800, i);

      // Way below average, but more than 10,000 steps were taken so these should be neutral
      prv_push_activity_summary_notification(now_utc, 12345, 100000, i);
    }
  }

  time_t midnight = time_util_get_midnight_of(now_utc);
  int32_t enter_seconds = (23 * SECONDS_PER_HOUR); // 11 pm the day before
  int32_t exit_seconds = (7 * SECONDS_PER_HOUR);   // 7 am today
  int32_t total_seconds = (8 * SECONDS_PER_HOUR);
  int32_t deviate_seconds = (2 * SECONDS_PER_HOUR);
  time_t exit_utc = midnight + exit_seconds;
  uuid = UUID_INVALID;
  prv_push_sleep_summary_pin(now_utc, exit_utc, enter_seconds, exit_seconds,
                             total_seconds + deviate_seconds, total_seconds, &uuid);
  uuid = UUID_INVALID;
  prv_push_sleep_summary_pin(now_utc, exit_utc, enter_seconds, exit_seconds,
                             total_seconds, total_seconds, &uuid);
  uuid = UUID_INVALID;
  prv_push_sleep_summary_pin(now_utc, exit_utc, enter_seconds, exit_seconds,
                             total_seconds - deviate_seconds, total_seconds, &uuid);
  uuid = UUID_INVALID;
  prv_push_sleep_summary_pin(now_utc, exit_utc, enter_seconds, exit_seconds,
                             total_seconds - 3 * deviate_seconds, total_seconds, &uuid);
  if (activity_prefs_sleep_insights_are_enabled()) {
    for (int i = 0; i < NUM_COPY_VARIANTS; i++) {
      prv_push_sleep_summary_notification(now_utc, total_seconds + deviate_seconds,
                                          total_seconds, i);
      prv_push_sleep_summary_notification(now_utc, total_seconds, total_seconds, i);
      prv_push_sleep_summary_notification(now_utc, total_seconds - deviate_seconds,
                                          total_seconds, i);
      prv_push_sleep_summary_notification(now_utc, deviate_seconds,
                                          total_seconds, i);
    }
  }
}

static void prv_test_push_rewards(void *unused) {
  time_t now_utc = rtc_get_time();

  if (activity_prefs_sleep_insights_are_enabled()) {
    prv_push_reward(now_utc, &SLEEP_REWARD_NOTIF_CONFIG);
  }

  if (activity_prefs_activity_insights_are_enabled()) {
    prv_push_reward(now_utc, &ACTIVITY_REWARD_NOTIF_CONFIG);
  }
}

static void prv_test_push_walk_run_session(void *unused) {
  const time_t now_utc = rtc_get_time();
  ActivitySession walk_session = {
    .type = ActivitySessionType_Walk,
    .start_utc = now_utc - 30 * SECONDS_PER_MINUTE - 15 * SECONDS_PER_MINUTE,
    .length_min = 30,
    .step_data = {
      .steps = 2400,
      .active_kcalories = 150,
      .distance_meters = 2000,
    },
  };
  int32_t avg_walk_hr = 120;
  int32_t walk_hr_zone_time_s[HRZoneCount] = {10 * SECONDS_PER_MINUTE,
                                              15 * SECONDS_PER_MINUTE,
                                              10 * SECONDS_PER_MINUTE,
                                              0  * SECONDS_PER_MINUTE};
  activity_insights_push_activity_session_notification(now_utc, &walk_session,
                                                       avg_walk_hr, walk_hr_zone_time_s);

  ActivitySession run_session = {
    .type = ActivitySessionType_Run,
    .start_utc = now_utc - 30 * SECONDS_PER_MINUTE - 12 * SECONDS_PER_MINUTE,
    .length_min = 30,
    .step_data = {
      .steps = 4200,
      .active_kcalories = 300,
      .distance_meters = 4828,
    },
  };
  int32_t avg_run_hr = 150;
  int32_t run_hr_zone_time_s[HRZoneCount] = {5  * SECONDS_PER_MINUTE,
                                             10 * SECONDS_PER_MINUTE,
                                             10 * SECONDS_PER_MINUTE,
                                             15 * SECONDS_PER_MINUTE};
  activity_insights_push_activity_session_notification(now_utc, &run_session,
                                                       avg_run_hr, run_hr_zone_time_s);

  ActivitySession open_session = {
    .type = ActivitySessionType_Open,
    .start_utc = now_utc - 30 * SECONDS_PER_MINUTE - 12 * SECONDS_PER_MINUTE,
    .length_min = 30,
    .step_data = {
      .steps = 0,
      .active_kcalories = 200,
      .distance_meters = 0,
    },
  };
  int32_t avg_open_hr = 130;
  int32_t open_hr_zone_time_s[HRZoneCount] = {2  * SECONDS_PER_MINUTE,
                                              0  * SECONDS_PER_MINUTE,
                                              18 * SECONDS_PER_MINUTE,
                                              10 * SECONDS_PER_MINUTE};
  activity_insights_push_activity_session_notification(now_utc, &open_session,
                                                       avg_open_hr, open_hr_zone_time_s);
}

static void prv_test_push_nap_session(void *unused) {
  const time_t now_utc = rtc_get_time();
  const int length_min = MINUTES_PER_HOUR + MINUTES_PER_HOUR / 2;
  ActivitySession session = {
    .type = ActivitySessionType_Nap,
    .start_utc = now_utc - length_min * SECONDS_PER_MINUTE,
    .length_min = length_min,
  };
  prv_push_nap_session(now_utc, &session);
}

void activity_insights_test_push_summary_pins(void) {
  system_task_add_callback(prv_test_push_summary_pins, NULL);
}

void activity_insights_test_push_rewards(void) {
  system_task_add_callback(prv_test_push_rewards, NULL);
}

void activity_insights_test_push_walk_run_sessions(void) {
  system_task_add_callback(prv_test_push_walk_run_session, NULL);
}

void activity_insights_test_push_nap_session(void) {
  system_task_add_callback(prv_test_push_nap_session, NULL);
}
