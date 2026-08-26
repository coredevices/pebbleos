/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "hr_detail_card.h"
#include "detail_card.h"

#include "kernel/pbl_malloc.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "pbl/services/activity/health_util.h"
#include "shell/system_theme.h"

#include <stdio.h>

typedef struct HealthHrDetailCard {
  int16_t num_headings;
  HealthDetailHeading headings[MAX_NUM_HEADINGS];

  int16_t num_zones;
  HealthDetailZone zones[MAX_NUM_ZONES];
} HealthHrDetailCardData;

#define DEFAULT_MAX_PROGRESS (10 * SECONDS_PER_MINUTE)

static void prv_set_zone(HealthDetailZone *zone, int32_t minutes, int32_t *max_progress,
                         const size_t buffer_size, const char *zone_label, void *i18n_owner) {
  *zone = (HealthDetailZone) {
    .label = app_zalloc_check(buffer_size),
    .progress = minutes * SECONDS_PER_MINUTE,
    .fill_color = PBL_IF_COLOR_ELSE(GColorSunsetOrange, GColorDarkGray),
  };

  int pos = snprintf(zone->label, buffer_size, "%s ", i18n_get(zone_label, i18n_owner));
  if (zone->progress) {
    health_util_format_hours_and_minutes(zone->label + pos, buffer_size - pos,
                                         zone->progress, i18n_owner);
  }

  if (zone->progress > *max_progress) {
    *max_progress = zone->progress;
  }
}

static void prv_set_heading_value(char *buffer, const size_t buffer_size, const int32_t zone_time_s,
                                  void *i18n_owner) {
  if (zone_time_s == 0) {
    strncpy(buffer, EN_DASH, buffer_size);
    return;
  }

  health_util_format_hours_and_minutes(buffer, buffer_size, zone_time_s, i18n_owner);
}

Window *health_hr_detail_card_create(HealthData *health_data) {
  HealthHrDetailCardData *card_data = app_zalloc_check(sizeof(HealthHrDetailCardData));

  const int32_t zone1_minutes = health_data_hr_get_zone1_minutes(health_data);
  const int32_t zone2_minutes = health_data_hr_get_zone2_minutes(health_data);
  const int32_t zone3_minutes = health_data_hr_get_zone3_minutes(health_data);
  const int32_t zone_time_minutes = zone1_minutes + zone2_minutes + zone3_minutes;

  int32_t max_progress = DEFAULT_MAX_PROGRESS;

  const size_t buffer_size = 32;

  prv_set_zone(&card_data->zones[card_data->num_zones++], zone1_minutes, &max_progress, buffer_size,
               i18n_noop("Fat Burn"), card_data);

  prv_set_zone(&card_data->zones[card_data->num_zones++], zone2_minutes, &max_progress, buffer_size,
               i18n_noop("Endurance"), card_data);

  prv_set_zone(&card_data->zones[card_data->num_zones++], zone3_minutes, &max_progress, buffer_size,
               i18n_noop("Performance"), card_data);

  HealthDetailHeading *heading = &card_data->headings[card_data->num_headings++];

  *heading = (HealthDetailHeading) {
    /// Resting HR
    .primary_label = (char *)i18n_get("TIME IN ZONES", card_data),
    .primary_value = app_zalloc_check(buffer_size),
    .fill_color = PBL_IF_COLOR_ELSE(GColorDarkCandyAppleRed, GColorWhite),
    .outline_color = PBL_IF_COLOR_ELSE(GColorClear, GColorBlack),
  };

  prv_set_heading_value(heading->primary_value, buffer_size,
                        (zone_time_minutes * SECONDS_PER_MINUTE), card_data);

  const HealthDetailCardConfig config = {
    .num_headings = card_data->num_headings,
    .headings = card_data->headings,
    .weekly_max = max_progress,
    .bg_color = system_theme_get_bg_color(),
    .num_zones = card_data->num_zones,
    .zones = card_data->zones,
    .data = card_data,
  };

  return (Window *)health_detail_card_create(&config);
}

void health_hr_detail_card_destroy(Window *window) {
  HealthDetailCard *card = (HealthDetailCard *)window;
  HealthHrDetailCardData *card_data = card->data;
  for (int i = 0; i < card_data->num_headings; i++) {
    app_free(card_data->headings[i].primary_value);
  }
  for (int i = 0; i < card_data->num_zones; i++) {
    app_free(card_data->zones[i].label);
  }
  i18n_free_all(card_data);
  app_free(card_data);
  health_detail_card_destroy(card);
}
