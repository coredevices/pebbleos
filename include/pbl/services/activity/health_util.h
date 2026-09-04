/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "apps/system/timeline/text_node.h"

#include <stddef.h>
#include <stdint.h>
#include <time.h>

//! The maximum number of text nodes needed in a text node container
#define MAX_TEXT_NODES 5

//! Extra 4 bytes is for i18n purposes
#define HEALTH_WHOLE_AND_DECIMAL_LENGTH (sizeof("00.0") + 4)

//! Format a duration in seconds to hours and minutes, e.g. "12H 59M"
//! If duration is less than an hour, the format of "59M" is used.
//! If duration is a multiple of an hour, the format of "12H" is used.
//! If duration is 0, the string "0H" is used.
//! @param[in,out] buffer the string buffer to write to
//! @param buffer_size the size of the string buffer
//! @param duration_s the duration is seconds
//! @param i18n_owner i18n owner that must be called with i18n_free_all some time after usage
//! @return snprintf-style number of bytes needed to be written not including the null terminator
int health_util_format_hours_and_minutes(char *buffer, size_t buffer_size, int duration_s,
                                         void *i18n_owner);

//! Create a text node and add it to the container and set the font and color
//! @param buffer_size the size of the string buffer
//! @param font GFont to be used for the text node
//! @param color GColor to be used fot the text node
//! @param container GTextNodeContainer that the text node will be added to
GTextNodeText *health_util_create_text_node(int buffer_size, GFont font, GColor color,
                                            GTextNodeContainer *container);

//! Create a text node with text and add it to the container and set the font and color
//! @param text the text string to be used for the text node
//! @param font GFont to be used for the text node
//! @param color GColor to be used fot the text node
//! @param container GTextNodeContainer that the text node will be added to
GTextNodeText *health_util_create_text_node_with_text(const char *text, GFont font, GColor color,
                                                      GTextNodeContainer *container);

//! Format a duration in seconds to hours, minutes and seconds, e.g. "1:15:32"
//! @param[in,out] buffer the string buffer to write to
//! @param buffer_size the size of the string buffer
//! @param duration_s the duration is seconds
//! @param leading_zero whether to include a leading zero in the formatted string
//! @param i18n_owner i18n owner that must be called with i18n_free_all some time after usage
//! @return snprintf-style number of bytes needed to be written not including the null terminator
int health_util_format_hours_minutes_seconds(char *buffer, size_t buffer_size, int duration_s,
                                             bool leading_zero, void *i18n_owner);

//! Format a duration in seconds to minutes and seconds, e.g. "5:32"
//! @param[in,out] buffer the string buffer to write to
//! @param buffer_size the size of the string buffer
//! @param duration_s the duration is seconds
//! @param i18n_owner i18n owner that must be called with i18n_free_all some time after usage
//! @return snprintf-style number of bytes needed to be written not including the null terminator
int health_util_format_minutes_and_seconds(char *buffer, size_t buffer_size, int duration_s,
                                           void *i18n_owner);

//! Format a duration in seconds to hours and minutes, e.g. "12H 59M", using text node
//! number_font will be used for the nodes with hours and minutes,
//! units_font will be used for the "H" and "M"
//! If duration is less than an hour, the format of "59M" is used.
//! If duration is a multiple of an hour, the format of "12H" is used.
//! If duration is 0, the string "0H" is used.
//! @param duration_s the duration is seconds
//! @param i18n_owner i18n owner that must be called with i18n_free_all some time after usage
//! @param number_font GFont to be used for the number text node
//! @param units_font GFont to be used for the units text node
//! @param color GColor to be used for the number and units text nodes
//! @param container GTextNodeContainer that will have the new number and units text nodes added to
void health_util_duration_to_hours_and_minutes_text_node(int duration_s, void *i18n_owner,
                                                         GFont number_font, GFont units_font,
                                                         GColor color,
                                                         GTextNodeContainer *container);

//! Convert a fraction into its whole and decimal parts
//! ex. 5/2 has a whole part of 2 and a decimal part of .5
//! @param numerator the numerator of the fraction
//! @param denominator the denominator of the fraction
//! @param[out] whole_part the whole part of the decimal representation
//! @param[out] decimal_part the decimal part of the decimal representation
void health_util_convert_fraction_to_whole_and_decimal_part(int numerator, int denominator,
                                                            int* whole_part, int *decimal_part);

//! Formats a fraction into its whole and decimal parts, e.g. "42.3"
//! @param[in,out] buffer the string buffer to write to
//! @param buffer_size the size of the string buffer
//! @param numerator the numerator of the fraction
//! @param denominator the denominator of the fraction
//! @return number of bytes written to buffer not including the null terminator
int health_util_format_whole_and_decimal(char *buffer, size_t buffer_size, int numerator,
                                         int denominator);

//! @return meters conversion factor for the user's distance pref
int health_util_get_distance_factor(void);

//! @return the pace from a distance in meters and a time in seconds
time_t health_util_get_pace(int time_s, int distance_meter);

//! Get the meters units string for the user's distance pref
//! @param miles_string the units string to use if the user's preference is miles
//! @param km_string the units string to use if the user's preference is kilometers
//! @return meters units string matching the user's distance pref
const char *health_util_get_distance_string(const char *miles_string, const char *km_string);

//! Formats distance in meters based on the user's units preference, e.g. "42.3"
//! @param[in,out] buffer the string buffer to write to
//! @param buffer_size the size of the string buffer
//! @param distance_m the distance in meters
//! @return number of bytes written to buffer not including the null terminator
int health_util_format_distance(char *buffer, size_t buffer_size, uint32_t distance_m);

//! Convert distance in meters its whole and decimal parts in the user's distance pref
//! @param distance_m the distance in meters
//! @param[out] whole_part the whole part of the converted decimal representation
//! @param[out] decimal_part the decimal part of the converted decimal representation
void health_util_convert_distance_to_whole_and_decimal_part(int distance_m, int *whole_part,
                                                            int *decimal_part);
