/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/activity/health_util.h"

#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "shell/prefs.h"
#include "util/time/time.h"
#include "util/units.h"

#include <limits.h>
#include <stdio.h>

static void prv_convert_duration_to_hours_and_minutes(int duration_s, int *hours, int *minutes) {
  *hours = (duration_s / SECONDS_PER_HOUR) ?: INT_MIN;
  *minutes = ((duration_s % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE) ?: INT_MIN;
  if (*minutes == INT_MIN && *hours == INT_MIN) {
    *hours = 0;
  }
}

int health_util_format_hours_and_minutes(char *buffer, size_t buffer_size, int duration_s,
                                         void *i18n_owner) {
  int hours;
  int minutes;
  prv_convert_duration_to_hours_and_minutes(duration_s, &hours, &minutes);
  int pos = 0;
  if (hours != INT_MIN) {
    pos += snprintf(buffer + pos, buffer_size - pos, i18n_get("%dH", i18n_owner), hours);
    if (minutes != INT_MIN && pos < (int)buffer_size - 1) {
      buffer[pos++] = ' ';
    }
  }
  if (minutes != INT_MIN) {
    pos += snprintf(buffer + pos, buffer_size - pos, i18n_get("%dM", i18n_owner), minutes);
  }
  return pos;
}

int health_util_format_hours_minutes_seconds(char *buffer, size_t buffer_size, int duration_s,
                                             bool leading_zero, void *i18n_owner) {
  const int hours = duration_s / SECONDS_PER_HOUR;
  const int minutes = (duration_s % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE;
  const int seconds = (duration_s % SECONDS_PER_HOUR) % SECONDS_PER_MINUTE;
  if (hours > 0) {
    const char *fmt = leading_zero ? "%02d:%02d:%02d" : "%d:%02d:%02d";
    return snprintf(buffer, buffer_size, i18n_get(fmt, i18n_owner), hours, minutes, seconds);
  } else {
    const char *fmt = leading_zero ? "%02d:%02d" : "%d:%02d";
    return snprintf(buffer, buffer_size, i18n_get(fmt, i18n_owner), minutes, seconds);
  }
}

int health_util_format_minutes_and_seconds(char *buffer, size_t buffer_size, int duration_s,
                                           void *i18n_owner) {
  int minutes = duration_s / SECONDS_PER_MINUTE;
  int seconds = duration_s % SECONDS_PER_MINUTE;
  return snprintf(buffer, buffer_size, i18n_get("%d:%d", i18n_owner), minutes, seconds);
}

GTextNodeText *health_util_create_text_node(int buffer_size, GFont font, GColor color,
                                            GTextNodeContainer *container) {
  GTextNodeText *text_node = graphics_text_node_create_text(buffer_size);
  if (container) {
    graphics_text_node_container_add_child(container, &text_node->node);
  }
  text_node->font = font;
  text_node->color = color;
  return text_node;
}

GTextNodeText *health_util_create_text_node_with_text(const char *text, GFont font, GColor color,
                                                      GTextNodeContainer *container) {
  GTextNodeText *text_node = health_util_create_text_node(0, font, color, container);
  text_node->text = text;
  return text_node;
}

void health_util_duration_to_hours_and_minutes_text_node(int duration_s, void *i18n_owner,
                                                         GFont number_font, GFont units_font,
                                                         GColor color,
                                                         GTextNodeContainer *container) {
  int hours;
  int minutes;
  prv_convert_duration_to_hours_and_minutes(duration_s, &hours, &minutes);
  const int units_offset_y = fonts_get_font_height(number_font) - fonts_get_font_height(units_font);
  const int hours_and_minutes_buffer_size = sizeof("00");
  if (hours != INT_MIN) {
    GTextNodeText *hours_text_node = health_util_create_text_node(hours_and_minutes_buffer_size,
                                                                  number_font, color, container);
    snprintf((char *) hours_text_node->text, hours_and_minutes_buffer_size,
             i18n_get("%d", i18n_owner), hours);

    GTextNodeText *hours_units_text_node = health_util_create_text_node_with_text(
        i18n_get("h", i18n_owner), units_font, color, container);
    hours_units_text_node->node.offset.y = units_offset_y;
  }

  if (hours != INT_MIN && minutes != INT_MIN) {
    // add a space between the H and the number of minutes
    health_util_create_text_node_with_text(i18n_get(" ", i18n_owner), units_font, color, container);
  }

  if (minutes != INT_MIN) {
    GTextNodeText *minutes_text_node = health_util_create_text_node(hours_and_minutes_buffer_size,
                                                                    number_font, color, container);
    snprintf((char *) minutes_text_node->text, hours_and_minutes_buffer_size,
             i18n_get("%d", i18n_owner), minutes);

    GTextNodeText *minutes_units_text_node = health_util_create_text_node_with_text(
        i18n_get("min", i18n_owner), units_font, color, container);
    minutes_units_text_node->node.offset.y = units_offset_y;
  }
}

void health_util_convert_fraction_to_whole_and_decimal_part(int numerator, int denominator,
                                                            int* whole_part, int *decimal_part) {
  const int figure = ROUND(numerator * 100, denominator * 10);
  *whole_part = figure / 10;
  *decimal_part = figure % 10;
}

int health_util_format_whole_and_decimal(char *buffer, size_t buffer_size, int numerator,
                                         int denominator) {
  int converted_distance_whole_part = 0;
  int converted_distance_decimal_part = 0;
  health_util_convert_fraction_to_whole_and_decimal_part(numerator, denominator,
                                                         &converted_distance_whole_part,
                                                         &converted_distance_decimal_part);
  const char *fmt_i18n = i18n_noop("%d.%d");
  const int rv = snprintf(buffer, buffer_size, i18n_get(fmt_i18n, buffer),
                          converted_distance_whole_part, converted_distance_decimal_part);
  i18n_free(fmt_i18n, buffer);
  return rv;
}

int health_util_get_distance_factor(void) {
  switch (shell_prefs_get_units_distance()) {
    case UnitsDistance_Miles:
      return METERS_PER_MILE;
    case UnitsDistance_KM:
      return METERS_PER_KM;
    case UnitsDistanceCount:
      break;
  }
  return 1;
}

const char *health_util_get_distance_string(const char *miles_string, const char *km_string) {
  switch (shell_prefs_get_units_distance()) {
    case UnitsDistance_Miles:
      return miles_string;
    case UnitsDistance_KM:
      return km_string;
    case UnitsDistanceCount:
      break;
  }
  return "";
}

int health_util_format_distance(char *buffer, size_t buffer_size, uint32_t distance_m) {
  return health_util_format_whole_and_decimal(buffer, buffer_size, distance_m,
                                              health_util_get_distance_factor());
}

void health_util_convert_distance_to_whole_and_decimal_part(int distance_m, int *whole_part,
                                                            int *decimal_part) {
  const int conversion_factor = health_util_get_distance_factor();
  health_util_convert_fraction_to_whole_and_decimal_part(distance_m, conversion_factor,
                                                         whole_part, decimal_part);
}

time_t health_util_get_pace(int time_s, int distance_meter) {
  if (!distance_meter) {
    return 0;
  }

  return ROUND(time_s * health_util_get_distance_factor(), distance_meter);
}
