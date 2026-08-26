/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "board/board.h"

#include "util/time/time.h"
#include <stdbool.h>
#include <stdint.h>

//! @file alarm.h
//! Allows a user to set an alarm for a given time in the future. When this time arrives, a
//! PEBBLE_ALARM_CLOCK_EVENT event will be put.
//!
//! These alarm settings will be persisted across watch resets.

#define SMART_ALARM_RANGE_S (30 * SECONDS_PER_MINUTE)
#define SMART_ALARM_SNOOZE_DELAY_S (1 * SECONDS_PER_MINUTE)
#define SMART_ALARM_MAX_LIGHT_SLEEP_S (30 * SECONDS_PER_MINUTE)
#define SMART_ALARM_MAX_SMART_SNOOZE (SMART_ALARM_RANGE_S / SMART_ALARM_SNOOZE_DELAY_S)

#define ALARMS_APP_HIGHLIGHT_COLOR PBL_IF_COLOR_ELSE(GColorJaegerGreen, GColorBlack)

typedef int AlarmId; //! A unique ID that can be used to refer to each configured alarm.

#define ALARM_INVALID_ID (-1)

typedef enum AlarmKind {
  ALARM_KIND_EVERYDAY = 0, // Alarms of this type will happen each day
  ALARM_KIND_WEEKENDS,     // Alarms of this type will happen Monday - Friday
  ALARM_KIND_WEEKDAYS,     // Alarms of this type happen Saturaday and Sunday
  ALARM_KIND_JUST_ONCE,    // Alarms of this type will happen next time the specified time occurs
  ALARM_KIND_CUSTOM,       // Alarms of this type happen on specified days
} AlarmKind;

typedef enum AlarmType {
  AlarmType_Basic,
  AlarmType_Smart,
  AlarmTypeCount,
} AlarmType;

//! Built-in alarm tones, played on speaker hardware when sound is enabled.
typedef enum AlarmTone {
  AlarmTone_Reveille = 0,
  AlarmTone_Beacon,
  AlarmTone_Bell,
  AlarmTone_Chime,
} AlarmTone;

typedef struct AlarmInfo {
  int hour; //<! Range 0-23, where 0 is 12am
  int minute; //<! Range is 0-59
  AlarmKind kind; //<! The kind of recurrence the alarm will have
  //! A bool for each weekday (Sunday = index 0) enabled
  bool (*scheduled_days)[DAYS_PER_WEEK];
  bool enabled; //<! Whether the alarm to go off at the specified time
  bool is_smart; //<! Whether the alarm is a Smart Alarm
  bool sound_enabled; //<! Whether the alarm should play a tone on speaker hardware
  bool vibrate_enabled; //<! Whether the alarm should vibrate
  AlarmTone tone; //<! Selected tone for this alarm (used when sound_enabled is true)
  bool is_snoozed_next; //<! Whether only the next occurrence is snoozed/skipped
} AlarmInfo;

typedef void (*AlarmForEach)(AlarmId id, const AlarmInfo *info, void *context);

//! Creates an alarm
//! @param info The alarm configuration to be created with
AlarmId alarm_create(const AlarmInfo *info);

//! @param id The alarm that should be updated
//! @param hour Range 0-23, where 0 is 12am
//! @param minute Range is 0-59
void alarm_set_time(AlarmId id, int hour, int minute);

//! @param id The alarm that should be updated
//! @param kind The new kind of the alarm
//! @note The CUSTOM kind will be ignored
void alarm_set_kind(AlarmId id, AlarmKind kind);

//! @param id The alarm that should be updated
//! @param scheduled_days A bool for each weekday (Sunday = index 0) enabled
//! each weekday that is marked as true
void alarm_set_custom(AlarmId id, const bool scheduled_days[DAYS_PER_WEEK]);

//! @param id The alarm that should be updated
//! @param smart Whether the alarm is a smart alarm
void alarm_set_smart(AlarmId id, bool smart);

//! @param id The alarm that should be updated
//! @param enabled Whether the alarm should play a tone on speaker hardware
void alarm_set_sound_enabled(AlarmId id, bool enabled);

//! @param id The alarm that should be updated
//! @param enabled Whether the alarm should vibrate
void alarm_set_vibrate_enabled(AlarmId id, bool enabled);

//! @param id The alarm that should be updated
//! @param tone The tone to play when sound is enabled
void alarm_set_tone(AlarmId id, AlarmTone tone);

//! @param id The alarm that should be updated
//! @param snooze_next Whether only the next occurrence should be snoozed/skipped
void alarm_set_snoozed_next(AlarmId id, bool snooze_next);

//! @param id The alarm that is being queried
//! @return True if only the next occurrence is snoozed/skipped, False otherwise
bool alarm_get_snoozed_next(AlarmId id);

//! @param id The alarm to look up
//! @param info_out The AlarmInfo struct to fill in. The scheduled_days pointer is set to NULL;
//! callers needing per-weekday flags should use alarm_get_custom_days() instead.
//! @return True if the alarm exists, False otherwise
bool alarm_get_info(AlarmId id, AlarmInfo *info_out);

//! @return The id of the most recently fired alarm, or ALARM_INVALID_ID if no alarm has fired
//! since boot. Used by the alarm popup to look up the firing alarm's settings.
AlarmId alarm_get_most_recent_id(void);

//! @param id The alarm for which the scheduled_days array should be updated for
//! @param scheduled_days An empty bool array for each weekday (Sunday = index 0)
//! that is to be updated. Alarms will run on each weekday that is marked as true
//! @return True if the alarm exists, False otherwise
bool alarm_get_custom_days(AlarmId id, bool scheduled_days[DAYS_PER_WEEK]);

//! @param id The alarm that should be modified
//! @param enable Whether to enable the alarm
void alarm_set_enabled(AlarmId id, bool enable);

//! @param id The alarm that should be deleted
void alarm_delete(AlarmId id);

//! @param id The alarm that is being queried
//! @return True if the alarm exists and is not disabled, False otherwise
bool alarm_get_enabled(AlarmId id);

//! @param id The alarm that should be deleted
//! @param hour_out Hour which the alarm is scheduled for
//! @param minute_out Minute which the alarm is scheduled for
//! @return True if the alarm is scheduled, False if not (disabled / doesn't exist)
bool alarm_get_hours_minutes(AlarmId id, int *hour_out, int *minute_out);

//! @param id The alarm that is being queried
//! @param kind_out The type of alarm
//! @return True if the alarm exists, False otherwise
bool alarm_get_kind(AlarmId id, AlarmKind *kind_out);

//! @param next_alarm_time_out The time of the next enabled alarm
//! @return True if at least one alarm is enabled, False if no alarms are enabled.
bool alarm_get_next_enabled_alarm(time_t *next_alarm_time_out);

//! @return True if the next enabled alarm is a smart alarm, False if no alarms are enabled or the
//! next alarm is not smart.
bool alarm_is_next_enabled_alarm_smart(void);

//! @param id The alarm that is being queried
//! @param time_out The number of seconds until the alarm is scheduled to go off
//! @return True if the alarm is scheduled, False if not (disabled / doesn't exist)
bool alarm_get_time_until(AlarmId id, time_t *time_out);

//! Starts a snooze timer for the current snooze delay.
void alarm_set_snooze_alarm(void);

//! @return Snooze delay in minutes.
uint16_t alarm_get_snooze_delay(void);

//! Set the snooze delay for all alarms
//! @param delay_m snooze delay in minutes.
void alarm_set_snooze_delay(uint16_t delay_m);

//! Dismisses the most recently triggered alarm.
void alarm_dismiss_alarm(void);

//! Runs the callback for each alarm pairing
void alarm_for_each(AlarmForEach cb, void *context);

//! @return True if the max number of alarms hasn't been saved, False otherwise
bool alarm_can_schedule(void);

//! Call this when the clock time has changed. This will reschedule all the alarms so they'll go
//! off at the right time. This is required because the alarm subsystem registers timers that go
//! off at a number of seconds as opposed to an absolute time, and the number of seconds before
//! the timer goes off changes when the clock time changes.
void alarm_handle_clock_change(void);

void alarm_init(void);

//! Enable or disable alarms globally.
void alarm_service_enable_alarms(bool enable);

//! Get the string (e.g. "Weekends") for a given AlarmKind and all-caps specification
const char *alarm_get_string_for_kind(AlarmKind kind, bool all_caps);

//! For an alarm of type custom, retrieve a string representing the days that the alarm is set for
//! Example: 1 day: ("Mondays", "Tuesdays"), multiple days: ("Mon,Sat,Sun", "Tue,Thu")
//! @param [in] scheduled_days A bool for each weekday (Sunday = index 0) enabled
//! @param [out] alarm_day_text A character array that is to be updated with the days. It should
//! have a minimum of 28 bytes allocated
void alarm_get_string_for_custom(bool scheduled_days[DAYS_PER_WEEK], char *alarm_day_text);

//! Set the alarms app version opened
void alarm_prefs_set_alarms_app_opened(uint8_t version);

//! Get the alarms app version opened
uint8_t alarm_prefs_get_alarms_app_opened(void);
