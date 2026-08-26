/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "test_alarm_common.h"

// Stubs
#include "stubs_activity.h"
#include "stubs_blob_db_sync.h"
#include "stubs_blob_db_sync_util.h"
#include "stubs_clock.h"
#include "stubs_pbl_malloc.h"


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Counter variables
static int s_num_timer_register_calls = 0;
static int s_alarm_timer_timeout_ms = 0;
static int s_snooze_timer_timeout_ms = 0;
static int s_snooze_timer_id = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Fakes

time_t rtc_get_time(void) {
  return s_current_day + prv_hours_and_minutes_to_seconds(s_current_hour, s_current_minute);
}

void rtc_get_time_ms(time_t* out_seconds, uint16_t* out_ms) {
  *out_ms = 0;
  *out_seconds = rtc_get_time();
}

RtcTicks rtc_get_ticks(void) {
  return 0;
}

TimerID new_timer_create(void) {
  static TimerID s_next_timer_id = 0;
  s_snooze_timer_id = s_next_timer_id + 1;
  return ++s_next_timer_id;
}

bool new_timer_start(TimerID timer_id, uint32_t timeout_ms, NewTimerCallback cb, void *cb_data,
                     uint32_t flags) {
  s_num_timer_register_calls++;
  s_snooze_timer_timeout_ms = timeout_ms;
  return true;
}

bool new_timer_stop(TimerID timer_id) {
  s_snooze_timer_timeout_ms = 0;
  return true;
}

void new_timer_delete(TimerID timer_id) {
  s_snooze_timer_timeout_ms = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//! Setup

void test_alarm__initialize(void) {
  s_num_timer_register_calls = 0;
  s_num_timeline_adds = 0;
  s_num_timeline_removes = 0;
  s_num_alarm_events_put = 0;
  s_alarm_timer_timeout_ms = 0;
  s_snooze_timer_timeout_ms = 0;
  s_num_alarms_fired = 0;

  s_current_hour = 0;
  s_current_minute = 0;
  // Default to Thursday
  s_current_day = s_thursday;

  timeline_item_destroy(s_last_timeline_item_added);
  s_last_timeline_item_added = NULL;
  s_last_timeline_item_removed_uuid = (Uuid) {};

  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(false);

  cron_service_init();

  alarm_init();
  alarm_service_enable_alarms(true);
}

void test_alarm__cleanup(void) {
  cron_service_deinit();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Basic Store / Get Tests

void test_alarm__alarm_create_recurring_daily(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 5, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 5, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 6, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 6, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
}

void test_alarm__alarm_create_recurring_weekends(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 5, .minute = 14, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 5, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 6, .minute = 14, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 6, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
}

void test_alarm__alarm_create_recurring_weekdays(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 5, .minute = 14, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id, 5, 14, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  id = alarm_create(&(AlarmInfo) { .hour = 6, .minute = 14, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id, 6, 14, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
}

void test_alarm__alarm_create_just_once(void) {
  AlarmId id;
  // It's currently Thursday @ 00:00
  bool just_once_schedule_thursday[7] = {false, false, false, false, true, false, false};
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_JUST_ONCE });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_JUST_ONCE });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  id = alarm_create(&(AlarmInfo) { .hour = 5, .minute = 14, .kind = ALARM_KIND_JUST_ONCE });
  prv_assert_alarm_config(id, 5, 14, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  id = alarm_create(&(AlarmInfo) { .hour = 6, .minute = 14, .kind = ALARM_KIND_JUST_ONCE });
  prv_assert_alarm_config(id, 6, 14, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
}

void test_alarm__alarm_create_recurring_custom(void) {
  AlarmId id;
  bool custom_schedule1[7] = {true, false, true, false, false, true, true};
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &custom_schedule1 });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_CUSTOM, custom_schedule1);

  bool custom_schedule2[7] = {false, false, false, false, false, true, false};
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &custom_schedule2 });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_CUSTOM, custom_schedule2);

  bool custom_schedule3[7] = {true, true, true, true, true, true, true};
  id = alarm_create(&(AlarmInfo) { .hour = 5, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &custom_schedule3 });
  prv_assert_alarm_config(id, 5, 14, false, ALARM_KIND_CUSTOM, custom_schedule3);

  // FIXME:
  bool custom_schedule4[7] = {false, false, false, false, false, false, false};
  id = alarm_create(&(AlarmInfo) { .hour = 6, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &custom_schedule4 });
  prv_assert_alarm_config(id, 6, 14, false, ALARM_KIND_CUSTOM, custom_schedule4);
}

void test_alarm__alarm_set_time(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id1, 3, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  alarm_set_time(id1, 5, 6);
  prv_assert_alarm_config(id1, 5, 6, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);

  id2 = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id2, 4, 14, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  alarm_set_time(id2, 23, 56);
  prv_assert_alarm_config(id2, 23, 56, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);

  alarm_set_time(id1, 15, 16);
  prv_assert_alarm_config(id1, 15, 16, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);

  alarm_set_time(id2, 23, 46);
  prv_assert_alarm_config(id2, 23, 46, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
}

void test_alarm__alarm_set_recurring_daily(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  alarm_set_kind(id, ALARM_KIND_EVERYDAY);
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);

  bool custom_schedule1[7] = {true, false, true, false, false, true, true};
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &custom_schedule1 });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_CUSTOM, custom_schedule1);
  alarm_set_kind(id, ALARM_KIND_EVERYDAY);
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
}

void test_alarm__alarm_set_recurring_weekends(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  alarm_set_kind(id, ALARM_KIND_WEEKENDS);
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);

  bool custom_schedule1[7] = {true, false, true, false, false, true, true};
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &custom_schedule1 });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_CUSTOM, custom_schedule1);
  alarm_set_kind(id, ALARM_KIND_WEEKENDS);
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
}

void test_alarm__alarm_set_recurring_weekdays(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  alarm_set_kind(id, ALARM_KIND_WEEKDAYS);
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);

  bool custom_schedule1[7] = {true, false, true, false, false, true, true};
  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &custom_schedule1 });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_CUSTOM, custom_schedule1);
  alarm_set_kind(id, ALARM_KIND_WEEKDAYS);
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
}

void test_alarm__alarm_set_custom(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  bool custom_schedule1[7] = {true, false, true, false, false, true, true};
  alarm_set_custom(id, custom_schedule1);
  prv_assert_alarm_config(id, 3, 14, false, ALARM_KIND_CUSTOM, custom_schedule1);

  id = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  bool custom_schedule2[7] = {true, false, false, false, false, true, false};
  alarm_set_custom(id, custom_schedule2);
  prv_assert_alarm_config(id, 4, 14, false, ALARM_KIND_CUSTOM, custom_schedule2);
}

void test_alarm__alarm_get_custom_days(void) {
  AlarmId id1, id2;

  bool schedule_1[7] = {true, false, false, false, false, false, true};
  bool verify_schedule_1[7] = {false, false, false, false, false, false, false};
  id1 = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id1, 3, 14, false, ALARM_KIND_CUSTOM, schedule_1);
  alarm_get_custom_days(id1, verify_schedule_1);
  prv_assert_alarm_config(id1, 3, 14, false, ALARM_KIND_CUSTOM, verify_schedule_1);

  bool schedule_2[7] = {false, false, true, false, false, true, false};
  bool verify_schedule_2[7] = {false, false, false, false, false, false, false};
  id2 = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  alarm_set_custom(id2, schedule_2);
  prv_assert_alarm_config(id2, 4, 14, false, ALARM_KIND_CUSTOM, schedule_2);
  alarm_get_custom_days(id2, verify_schedule_2);
  prv_assert_alarm_config(id2, 4, 14, false, ALARM_KIND_CUSTOM, verify_schedule_2);
}

void test_alarm__alarm_get_string_for_custom(void) {
  bool schedule_1[7] = {true, false, false, false, false, false, true};
  char alarm_day_text_1[32] = {0};
  alarm_get_string_for_custom(schedule_1, alarm_day_text_1);
  cl_assert_equal_s(alarm_day_text_1, "Sat,Sun");

  bool schedule_2[7] = {true, true, true, true, true, true, true};
  char alarm_day_text_2[32] = {0};
  alarm_get_string_for_custom(schedule_2, alarm_day_text_2);
  cl_assert_equal_s(alarm_day_text_2, "Mon,Tue,Wed,Thu,Fri,Sat,Sun");

  bool schedule_3[7] = {false, true, false, false, false, false, false};
  char alarm_day_text_3[32] = {0};
  alarm_get_string_for_custom(schedule_3, alarm_day_text_3);
  cl_assert_equal_s(alarm_day_text_3, "Mondays");

  bool schedule_4[7] = {false, false, true, false, false, false, false};
  char alarm_day_text_4[32] = {0};
  alarm_get_string_for_custom(schedule_4, alarm_day_text_4);
  cl_assert_equal_s(alarm_day_text_4, "Tuesdays");

  bool schedule_5[7] = {false, false, false, true, false, false, false};
  char alarm_day_text_5[32] = {0};
  alarm_get_string_for_custom(schedule_5, alarm_day_text_5);
  cl_assert_equal_s(alarm_day_text_5, "Wednesdays");

  bool schedule_6[7] = {false, false, false, false, true, false, false};
  char alarm_day_text_6[32] = {0};
  alarm_get_string_for_custom(schedule_6, alarm_day_text_6);
  cl_assert_equal_s(alarm_day_text_6, "Thursdays");

  bool schedule_7[7] = {false, false, false, false, false, true, false};
  char alarm_day_text_7[32] = {0};
  alarm_get_string_for_custom(schedule_7, alarm_day_text_7);
  cl_assert_equal_s(alarm_day_text_7, "Fridays");

  bool schedule_8[7] = {false, false, false, false, false, false, true};
  char alarm_day_text_8[32] = {0};
  alarm_get_string_for_custom(schedule_8, alarm_day_text_8);
  cl_assert_equal_s(alarm_day_text_8, "Saturdays");

  bool schedule_9[7] = {true, false, false, false, false, false, false};
  char alarm_day_text_9[32] = {0};
  alarm_get_string_for_custom(schedule_9, alarm_day_text_9);
  cl_assert_equal_s(alarm_day_text_9, "Sundays");
}

void test_alarm__alarm_set_get_enabled(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  cl_assert_equal_i(s_num_timeline_adds, 3);
  cl_assert_equal_i(s_num_timeline_removes, 0);

  alarm_set_enabled(id1, false);
  prv_assert_alarm_config(id1, 3, 14, true, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  cl_assert_equal_i(s_num_timeline_adds, 3);
  cl_assert_equal_i(s_num_timeline_removes, 3);

  alarm_set_enabled(id1, true);
  prv_assert_alarm_config(id1, 3, 14, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  cl_assert_equal_i(s_num_timeline_adds, 6);
  cl_assert_equal_i(s_num_timeline_removes, 3);

  id2 = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  cl_assert_equal_i(s_num_timeline_adds, 9);
  cl_assert_equal_i(s_num_timeline_removes, 3);

  alarm_set_enabled(id2, false);
  prv_assert_alarm_config(id2, 4, 14, true, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  cl_assert_equal_i(s_num_timeline_adds, 12);
  cl_assert_equal_i(s_num_timeline_removes, 9);

  AlarmId invalid_id = 7;
  alarm_set_enabled(invalid_id, false);
  cl_assert_equal_i(s_num_timeline_adds, 12);
  cl_assert_equal_i(s_num_timeline_removes, 9);
}

void test_alarm__alarm_delete(void) {
  AlarmId id1, id2, id3, id4, id5, id6, id7;
  id1 = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  id2 = alarm_create(&(AlarmInfo) { .hour = 4, .minute = 14, .kind = ALARM_KIND_EVERYDAY });
  alarm_delete(id1);
  prv_assert_alarm_config_absent(id1);
  assert_alarm_pins_absent(id1);
  alarm_delete(id2);
  prv_assert_alarm_config_absent(id2);
  assert_alarm_pins_absent(id2);
  id3 = alarm_create(&(AlarmInfo) { .hour = 13, .minute = 13, .kind = ALARM_KIND_WEEKDAYS });
  id4 = alarm_create(&(AlarmInfo) { .hour = 14, .minute = 14, .kind = ALARM_KIND_WEEKDAYS });
  id5 = alarm_create(&(AlarmInfo) { .hour = 15, .minute = 15, .kind = ALARM_KIND_WEEKDAYS });
  id6 = alarm_create(&(AlarmInfo) { .hour = 16, .minute = 16, .kind = ALARM_KIND_WEEKDAYS });
  alarm_delete(id4);
  prv_assert_alarm_config_absent(id4);
  assert_alarm_pins_absent(id4);
  id7 = alarm_create(&(AlarmInfo) { .hour = 17, .minute = 17, .kind = ALARM_KIND_WEEKENDS });

  for (int i = 0; i < MAX_CONFIGURED_ALARMS; ++i) {
    alarm_delete(i);
    prv_assert_alarm_config_absent(i);
    assert_alarm_pins_absent(i);
  }

  alarm_delete(1);
}

void test_alarm__snooze_delay(void) {
  int delay;
  delay = alarm_get_snooze_delay();
  cl_assert_equal_i(delay, 10);
  alarm_set_snooze_delay(15);

  delay = alarm_get_snooze_delay();
  cl_assert_equal_i(delay, 15);
}

void test_alarm__set_snooze_alarm() {
  alarm_set_snooze_alarm();
  cl_assert_equal_i(s_snooze_timer_timeout_ms, 10 * 1000);
  cl_assert_equal_i(s_num_alarm_events_put, 0);
  s_current_minute = 10;
  cl_assert_equal_i(s_num_alarm_events_put, 1);
}

void test_alarm__get_string_for_kind(void) {
  bool all_caps = false;
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_EVERYDAY, all_caps), "Every Day");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_WEEKDAYS, all_caps), "Weekdays");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_WEEKENDS, all_caps), "Weekends");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_JUST_ONCE, all_caps), "Once");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_CUSTOM, all_caps), "Custom");

  all_caps = true;
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_EVERYDAY, all_caps), "EVERY DAY");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_WEEKDAYS, all_caps), "WEEKDAYS");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_WEEKENDS, all_caps), "WEEKENDS");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_JUST_ONCE, all_caps), "ONCE");
  cl_assert_equal_s(alarm_get_string_for_kind(ALARM_KIND_CUSTOM, all_caps), "CUSTOM");
}

void test_alarm__handle_clock_change(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 3, .minute = 14, .kind = ALARM_KIND_WEEKENDS });
  id2 = alarm_create(&(AlarmInfo) { .hour = 13, .minute = 14, .kind = ALARM_KIND_WEEKDAYS });

  s_current_hour = 12;
  s_current_minute = 14;
  alarm_handle_clock_change();

  s_current_hour = 13;
  s_current_minute = 14;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! Alarm Timeline Pin Tests

void test_alarm__pin_add(void) {
  const AlarmId dummy_alarm_id = 0;
  const AlarmKind alarm_kind = ALARM_KIND_WEEKENDS;
  Uuid added_pin_uuid;
  alarm_pin_add(s_monday, dummy_alarm_id, AlarmType_Basic, alarm_kind, &added_pin_uuid);
  cl_assert(uuid_equal(&added_pin_uuid, &s_last_timeline_item_added->header.id));

  AttributeList *pin_attr_list = &s_last_timeline_item_added->attr_list;

  const uint32_t pin_icon_tiny = attribute_get_uint32(pin_attr_list, AttributeIdIconTiny, 0);
  cl_assert_equal_i((int)pin_icon_tiny, TIMELINE_RESOURCE_ALARM_CLOCK);

  const char *pin_title = attribute_get_string(pin_attr_list, AttributeIdTitle, NULL);
  cl_assert_equal_s(pin_title, "Alarm");

  const char *pin_subtitle = attribute_get_string(pin_attr_list, AttributeIdSubtitle, NULL);
  cl_assert_equal_s(pin_subtitle, alarm_get_string_for_kind(alarm_kind, false /* all_caps */));

  const AlarmKind pin_alarm_kind = (AlarmKind)attribute_get_uint8(pin_attr_list,
                                                                  AttributeIdAlarmKind, 0);
  cl_assert_equal_i(pin_alarm_kind, alarm_kind);

  cl_assert_equal_i(s_last_timeline_item_added->action_group.num_actions, 1);

  const TimelineItemAction *alarm_action = s_last_timeline_item_added->action_group.actions;
  cl_assert_equal_i(alarm_action->id, dummy_alarm_id);
  cl_assert_equal_i(alarm_action->type, TimelineItemActionTypeOpenWatchApp);

  const AttributeList *action_attr_list = &alarm_action->attr_list;

  const char *action_title = attribute_get_string(action_attr_list, AttributeIdTitle, NULL);
  cl_assert_equal_s(action_title, "Edit");
}

void test_alarm__pin_remove(void) {
  const AlarmId dummy_alarm_id = 0;
  Uuid pin_uuid;
  alarm_pin_add(s_monday, dummy_alarm_id, AlarmType_Basic, ALARM_KIND_WEEKENDS, &pin_uuid);
  alarm_pin_remove(&pin_uuid);
  cl_assert(uuid_equal(&pin_uuid, &s_last_timeline_item_removed_uuid));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//! More Advanced Tests

void test_alarm__recurring_daily_alarm_timeout_ahead(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  cl_assert_equal_i(s_num_timeline_adds , 3);

  s_current_hour = 10;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
}

void test_alarm__recurring_daily_alarm_timeout_behind(void) {
  AlarmId id;
  s_current_hour = 11;
  s_current_minute = 30;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  cl_assert_equal_i(s_num_timeline_adds , 3);

  // Alarm set for tomorrow, so add 24 hours.
  s_current_hour = 10 + 24;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
}

void test_alarm__recurring_daily_alarm(void) {
  time_t next_alarm_time = 0;
  alarm_get_next_enabled_alarm(&next_alarm_time);
  cl_assert_equal_i(next_alarm_time, 0);

  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  cl_assert_equal_i(s_num_timeline_adds, 3);

  alarm_get_next_enabled_alarm(&next_alarm_time);
  cl_assert_equal_i(next_alarm_time,
                    s_current_day + 10 * SECONDS_PER_HOUR + 30 * SECONDS_PER_MINUTE);

  id2 = alarm_create(&(AlarmInfo) { .hour = 11, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id2, 11, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);
  cl_assert_equal_i(s_num_timeline_adds, 6);
  cl_assert_equal_i(s_num_timeline_removes, 0);

  alarm_get_next_enabled_alarm(&next_alarm_time);
  cl_assert_equal_i(next_alarm_time,
                    s_current_day + 10 * SECONDS_PER_HOUR + 30 * SECONDS_PER_MINUTE);

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
  cl_assert_equal_i(s_num_alarm_events_put, 1);
  cl_assert_equal_i(s_num_timeline_adds, 13);
  cl_assert_equal_i(s_num_timeline_removes, 6);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);
  cl_assert_equal_i(s_num_alarm_events_put, 2);
  cl_assert_equal_i(s_num_timeline_adds, 20);
  cl_assert_equal_i(s_num_timeline_removes, 12);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_friday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 3);
  cl_assert_equal_i(s_num_alarm_events_put, 3);
  cl_assert_equal_i(s_num_timeline_adds, 27);
  cl_assert_equal_i(s_num_timeline_removes, 18);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 4);
  cl_assert_equal_i(s_num_alarm_events_put, 4);
  cl_assert_equal_i(s_num_timeline_adds, 34);
  cl_assert_equal_i(s_num_timeline_removes, 24);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_saturday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 5);
  cl_assert_equal_i(s_num_alarm_events_put, 5);
  cl_assert_equal_i(s_num_timeline_adds, 41);
  cl_assert_equal_i(s_num_timeline_removes, 30);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 6);
  cl_assert_equal_i(s_num_alarm_events_put, 6);
  cl_assert_equal_i(s_num_timeline_adds, 48);
  cl_assert_equal_i(s_num_timeline_removes, 36);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_sunday; // Make sure the wday can wrap properly
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 7);
  cl_assert_equal_i(s_num_alarm_events_put, 7);
  cl_assert_equal_i(s_num_timeline_adds, 55);
  cl_assert_equal_i(s_num_timeline_removes, 42);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());
}

void test_alarm__recurring_weekends_alarm_timeout_ahead(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);

  // It's currently Thursday @ 00:00. The alarm shouldn't go off for over 2 days + 10:30
  s_current_hour = 10;
  s_current_minute = 29;
  s_current_day = s_saturday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);
  // Only 1 pin should be added (for Saturday)
  cl_assert_equal_i(s_num_timeline_adds, 1);

  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
  cl_assert_equal_i(s_num_timeline_adds, 3);
}

void test_alarm__recurring_weekends_alarm_timeout_behind(void) {
  AlarmId id;
  s_current_hour = 11;
  s_current_minute = 30;
  s_current_day = s_saturday;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  // Only 1 pin should be added (for Sunday)
  cl_assert_equal_i(s_num_timeline_adds, 1);
}

void test_alarm__recurring_weekends_alarm(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  // Only 1 pin should be added (for Saturday)
  cl_assert_equal_i(s_num_timeline_adds, 1);

  id2 = alarm_create(&(AlarmInfo) { .hour = 11, .minute = 30, .kind = ALARM_KIND_WEEKENDS });
  prv_assert_alarm_config(id2, 11, 30, false, ALARM_KIND_WEEKENDS, s_weekend_schedule);
  cl_assert_equal_i(s_num_timeline_adds, 2);

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_saturday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
  cl_assert_equal_i(s_num_alarm_events_put, 1);
  cl_assert_equal_i(s_num_timeline_adds, 6);
  cl_assert_equal_i(s_num_timeline_removes, 2);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);
  cl_assert_equal_i(s_num_alarm_events_put, 2);
  cl_assert_equal_i(s_num_timeline_adds, 9);
  cl_assert_equal_i(s_num_timeline_removes, 5);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_sunday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 3);
  cl_assert_equal_i(s_num_alarm_events_put, 3);
  cl_assert_equal_i(s_num_timeline_adds, 11);
  cl_assert_equal_i(s_num_timeline_removes, 7);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again, but not until Saturday
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 4);
  cl_assert_equal_i(s_num_alarm_events_put, 4);
  cl_assert_equal_i(s_num_timeline_adds, 12);
  cl_assert_equal_i(s_num_timeline_removes, 8);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());
}

void test_alarm__recurring_weekday_alarm_timeout_ahead(void) {
  AlarmId id;
  s_current_day = s_saturday;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  // Only 1 pin should be added (for Monday)
  cl_assert_equal_i(s_num_timeline_adds, 1);
}

void test_alarm__recurring_weekday_alarm_timeout_behind(void) {
  AlarmId id;
  s_current_hour = 11;
  s_current_minute = 30;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  // Only 1 pin should be added (for Friday)
  cl_assert_equal_i(s_num_timeline_adds, 1);
}

void test_alarm__recurring_weekday_alarm(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  // 2 pins should be added (for Thursday / Friday)
  cl_assert_equal_i(s_num_timeline_adds, 2);

  id2 = alarm_create(&(AlarmInfo) { .hour = 11, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id2, 11, 30, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  cl_assert_equal_i(s_num_timeline_adds, 4);

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
  cl_assert_equal_i(s_num_alarm_events_put, 1);
  cl_assert_equal_i(s_num_timeline_adds, 8);
  cl_assert_equal_i(s_num_timeline_removes, 4);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);
  cl_assert_equal_i(s_num_alarm_events_put, 2);
  cl_assert_equal_i(s_num_timeline_adds, 11);
  cl_assert_equal_i(s_num_timeline_removes, 7);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_friday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 3);
  cl_assert_equal_i(s_num_alarm_events_put, 3);
  cl_assert_equal_i(s_num_timeline_adds, 14);
  cl_assert_equal_i(s_num_timeline_removes, 9);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again, but not until Monday
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 4);
  cl_assert_equal_i(s_num_alarm_events_put, 4);
  cl_assert_equal_i(s_num_timeline_adds, 17);
  cl_assert_equal_i(s_num_timeline_removes, 11);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());
}

void test_alarm__just_once_alarm_timeout_ahead(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_JUST_ONCE });
  // It's currently Thursday @ 00:00
  bool just_once_schedule_thursday[7] = {false, false, false, false, true, false, false};
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  // Only 1 pin should be added
  cl_assert_equal_i(s_num_timeline_adds, 1);
}

void test_alarm__just_once_alarm_timeout_behind(void) {
  AlarmId id;
  s_current_hour = 11;
  s_current_minute = 30;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_JUST_ONCE });
  bool just_once_schedule_friday[7] = {false, false, false, false, false, true, false};
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_JUST_ONCE, just_once_schedule_friday);
  // Only 1 pin should be added
  cl_assert_equal_i(s_num_timeline_adds, 1);
}

void test_alarm__just_once_alarm(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_JUST_ONCE });
  // It's currently Thursday @ 00:00
  bool just_once_schedule_thursday[7] = {false, false, false, false, true, false, false};
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  // Only 1 pin should be added
  cl_assert_equal_i(s_num_timeline_adds, 1);

  id2 = alarm_create(&(AlarmInfo) { .hour = 11, .minute = 30, .kind = ALARM_KIND_JUST_ONCE });
  prv_assert_alarm_config(id2, 11, 30, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  // Only 1 pin should be added
  cl_assert_equal_i(s_num_timeline_adds, 2);

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
  prv_assert_alarm_config(id1, 10, 30, true, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  cl_assert_equal_i(s_num_alarm_events_put, 1);
  cl_assert_equal_i(s_num_timeline_adds, 4);
  cl_assert_equal_i(s_num_timeline_removes, 2);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. No alarms should be up
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);
  prv_assert_alarm_config(id2, 11, 30, true, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);
  cl_assert_equal_i(s_num_alarm_events_put, 2);
  cl_assert_equal_i(s_num_timeline_adds, 5);
  cl_assert_equal_i(s_num_timeline_removes, 3);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());
}

void test_alarm__custom_alarm_everyday(void) {
  AlarmId id1;

  bool schedule_1[7] = {true, true, true, true, true, true, true};
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_CUSTOM, schedule_1);

  // It's currently Thursday @ 00:00.

  s_current_day = s_friday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);

  s_current_day = s_saturday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);

  s_current_day = s_sunday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 3);

  s_current_day = s_monday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 4);

  s_current_day = s_tuesday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 5);

  s_current_day = s_wednesday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 6);
}

void test_alarm__custom_alarm_weekends_and_weekday(void) {
  AlarmId id1;

  bool schedule_1[7] = {true, false, false, true, false, false, true};
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_CUSTOM, schedule_1);

  // It's currently Thursday @ 00:00.

  s_current_day = s_friday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);

  s_current_day = s_saturday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);

  s_current_day = s_sunday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);

  s_current_day = s_monday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);

  s_current_day = s_tuesday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);

  s_current_day = s_wednesday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);
}

void test_alarm__custom_alarm_partial_weekdays(void) {
  AlarmId id1;

  bool schedule_1[7] = {false, true, true, true, true, false, false};
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_CUSTOM, schedule_1);

  // It's currently Thursday @ 00:00.

  s_current_day = s_friday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);

  s_current_day = s_saturday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);

  s_current_day = s_sunday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);

  s_current_day = s_monday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);

  s_current_day = s_tuesday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);

  s_current_day = s_wednesday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 3);
}

void test_alarm__custom_alarm_weekends(void) {
  AlarmId id1, id2;

  bool schedule_1[7] = {true, false, false, false, false, false, true};
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_CUSTOM, schedule_1);

  // Only 1 pin should be added (for Saturday)
  cl_assert_equal_i(s_num_timeline_adds, 1);

  id2 = alarm_create(&(AlarmInfo) { .hour = 11, .minute = 30, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id2, 11, 30, false, ALARM_KIND_CUSTOM, schedule_1);
  cl_assert_equal_i(s_num_timeline_adds, 2);

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_saturday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
  cl_assert_equal_i(s_num_alarm_events_put, 1);
  cl_assert_equal_i(s_num_timeline_adds, 6);
  cl_assert_equal_i(s_num_timeline_removes, 2);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);
  cl_assert_equal_i(s_num_alarm_events_put, 2);
  cl_assert_equal_i(s_num_timeline_adds, 9);
  cl_assert_equal_i(s_num_timeline_removes, 5);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // First alarm goes off. Second one should be up
  s_current_hour = 10;
  s_current_minute = 30;
  s_current_day = s_sunday;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 3);
  cl_assert_equal_i(s_num_alarm_events_put, 3);
  cl_assert_equal_i(s_num_timeline_adds, 11);
  cl_assert_equal_i(s_num_timeline_removes, 7);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());

  // Second alarm goes off. First one should be up again, but not until Saturday
  s_current_hour = 11;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 4);
  cl_assert_equal_i(s_num_alarm_events_put, 4);
  cl_assert_equal_i(s_num_timeline_adds, 12);
  cl_assert_equal_i(s_num_timeline_removes, 8);
  cl_assert_equal_i(s_last_timeline_item_added->header.timestamp, rtc_get_time());
}

void test_alarm__custom_alarm_no_alarm(void) {
  AlarmId id1;

  bool schedule_1[7] = {false, false, false, false, false, false, false};
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_CUSTOM, schedule_1);

  cl_assert_equal_i(s_alarm_timer_timeout_ms, 0);
}

void test_alarm__custom_alarm_multiple(void) {
  AlarmId id1, id2;

  // Alarm set for Tuesday and Saturday
  bool schedule_1[7] = {false, false, true, false, false, false, true};
  id1 = alarm_create(&(AlarmInfo) { .hour = 1, .minute = 30, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_1 });
  prv_assert_alarm_config(id1, 1, 30, false, ALARM_KIND_CUSTOM, schedule_1);
  // It's currently Thursday @ 00:00, 2 days + 1:30 till next alarm
  /* cl_assert_equal_i(s_alarm_timer_timeout_ms, */
  /*                   (SECONDS_PER_DAY * 2 + prv_hours_and_minutes_to_seconds(1, 30)) * 1000); */

  s_current_day = s_sunday;
  s_current_hour = 12;
  s_current_minute = 15;
  cron_service_wakeup();
  bool schedule_2[7] = {false, true, false, false, false, false, false};
  id2 = alarm_create(&(AlarmInfo) { .hour = 13, .minute = 15, .kind = ALARM_KIND_CUSTOM, .scheduled_days = &schedule_2 });
  prv_assert_alarm_config(id2, 13, 15, false, ALARM_KIND_CUSTOM, schedule_2);
  cl_assert_equal_i(s_num_alarms_fired, 1);

  s_current_day = s_tuesday;
  s_current_hour = 1;
  s_current_minute = 0;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 2);
}

void test_alarm__disable_upcoming_alarm(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);

  id2 = alarm_create(&(AlarmInfo) { .hour = 11, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id2, 11, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);

  // Disable the 10:30 alarm
  alarm_set_enabled(id1, false);

  // The 10:30 alarm should not have gone off
  s_current_hour = 11;
  s_current_minute = 0;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);

  // Disable the 11:30 alarm
  alarm_set_enabled(id2, false);

  // The 11:30 alarm should not go off either
  s_current_hour = 12;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);

  // Enable the 11:30 alarm - now it should go off
  s_current_hour = 11;
  alarm_set_enabled(id2, true);
  s_current_hour = 12;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
}

void test_alarm__delete_upcoming_alarm(void) {
  AlarmId id1, id2;
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id1, 10, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);

  id2 = alarm_create(&(AlarmInfo) { .hour = 11, .minute = 30, .kind = ALARM_KIND_EVERYDAY });
  prv_assert_alarm_config(id2, 11, 30, false, ALARM_KIND_EVERYDAY, s_every_day_schedule);

  // Delete the 10:30 alarm
  alarm_delete(id1);

  // The 10:30 alarm should not go off
  s_current_hour = 11;
  s_current_minute = 0;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);

  // Delete the 11:30 alarm
  alarm_delete(id2);

  // The 11:30 alarm should not go off either
  s_current_hour = 12;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);
}

void test_alarm__alarm_type_change_updates_timeout(void) {
  AlarmId id;
  id = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  prv_assert_alarm_config(id, 10, 30, false, ALARM_KIND_WEEKDAYS, s_weekday_schedule);
  // 2 pins should be added (for Thursday / Friday)
  cl_assert_equal_i(s_num_timeline_adds, 2);

  alarm_set_kind(id, ALARM_KIND_WEEKENDS);

  // Alarm should not go off on Thursday anymore
  s_current_hour = 10;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 0);

  // Alarm should go off on the weekend
  s_current_day = s_saturday;
  s_current_hour = 10;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
}

void test_alarm__alarm_get_next_enabled_alarm(void) {
  // Declarations
  time_t next_alarm = 0;
  time_t time_until_alarm = 0;
  bool at_least_one_alarm_enabled = false;
  bool alarm_is_scheduled = false;
  AlarmId id1, id2;

  // No alarms scheduled
  at_least_one_alarm_enabled = alarm_get_next_enabled_alarm(&next_alarm);
  cl_assert_equal_b(at_least_one_alarm_enabled, false);
  cl_assert_equal_i(next_alarm, 0);

  // Schedule an alarm, it becomes the next alarm
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  at_least_one_alarm_enabled = alarm_get_next_enabled_alarm(&next_alarm);
  cl_assert_equal_b(at_least_one_alarm_enabled, true);
  alarm_is_scheduled = alarm_get_time_until(id1, &time_until_alarm);
  cl_assert_equal_b(alarm_is_scheduled, true);
  cl_assert_equal_i(next_alarm, rtc_get_time() + time_until_alarm);

  // Schedule another alarm before the previous alarm, it becomes the next alarm
  id2 = alarm_create(&(AlarmInfo) { .hour = 9, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  at_least_one_alarm_enabled = alarm_get_next_enabled_alarm(&next_alarm);
  cl_assert_equal_b(at_least_one_alarm_enabled, true);
  alarm_is_scheduled = alarm_get_time_until(id2, &time_until_alarm);
  cl_assert_equal_b(alarm_is_scheduled, true);
  cl_assert_equal_i(next_alarm, rtc_get_time() + time_until_alarm);

  // Disable both alarms, now there is no next alarm
  alarm_set_enabled(id1, false);
  alarm_set_enabled(id2, false);
  at_least_one_alarm_enabled = alarm_get_next_enabled_alarm(&next_alarm);
  cl_assert_equal_b(at_least_one_alarm_enabled, false);
}

void test_alarm__alarm_is_next_enabled_alarm_smart(void) {
  // Declarations
  AlarmId id1;
  AlarmId id2;
  AlarmId id3;

  // No alarms scheduled, our function should return false
  cl_assert_equal_b(alarm_is_next_enabled_alarm_smart(), false);

  // Schedule a basic (non-smart) alarm, our function should return false
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  cl_assert_equal_b(alarm_is_next_enabled_alarm_smart(), false);

  // Schedule a smart alarm before the basic alarm, our function should return true
  id2 = alarm_create(&(AlarmInfo) { .hour = 9, .minute = 30, .is_smart = true });
  cl_assert_equal_b(alarm_is_next_enabled_alarm_smart(), true);

  // Schedule another basic alarm before the smart alarm, our function should return false again
  id3 = alarm_create(&(AlarmInfo) { .hour = 8, .minute = 30, .kind = ALARM_KIND_WEEKDAYS });
  cl_assert_equal_b(alarm_is_next_enabled_alarm_smart(), false);

  // Disable all three alarms, now there is no next alarm and so our function should return false
  alarm_set_enabled(id1, false);
  alarm_set_enabled(id2, false);
  alarm_set_enabled(id3, false);
  cl_assert_equal_b(alarm_is_next_enabled_alarm_smart(), false);
}

void test_alarm__skip_two_alarms(void) {
  AlarmId id1, id2;
  bool just_once_schedule_thursday[7] = {false, false, false, false, true, false, false};
  id1 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 10, .kind = ALARM_KIND_JUST_ONCE });
  prv_assert_alarm_config(id1, 10, 10, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);

  id2 = alarm_create(&(AlarmInfo) { .hour = 10, .minute = 20, .kind = ALARM_KIND_JUST_ONCE });
  prv_assert_alarm_config(id2, 10, 20, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);

  // Skip ahead of both alarms
  // One of the alarms should go off
  s_current_hour = 10;
  s_current_minute = 30;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);

  // The other alarm should not go off
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//! Alarms missed while the watch was down

//! Restarts the alarm service the way a reboot would, leaving the settings file intact.
static void prv_simulate_reboot(void) {
  cron_service_deinit();
  cron_service_init();
  alarm_init();
  alarm_service_enable_alarms(true);
}

void test_alarm__missed_just_once_alarm_rearmed_for_next_day(void) {
  // It's currently Thursday @ 00:00
  AlarmId id = alarm_create(&(AlarmInfo) { .hour = 6, .minute = 0, .kind = ALARM_KIND_JUST_ONCE });
  bool just_once_schedule_thursday[7] = {false, false, false, false, true, false, false};
  prv_assert_alarm_config(id, 6, 0, false, ALARM_KIND_JUST_ONCE, just_once_schedule_thursday);

  // The watch was down over 06:00 and comes back up too late to catch the alarm up. The alarm
  // must not stay armed for Thursday next week.
  s_current_hour = 8;
  prv_simulate_reboot();

  cl_assert_equal_i(s_num_alarms_fired, 0);
  bool just_once_schedule_friday[7] = {false, false, false, false, false, true, false};
  prv_assert_alarm_config(id, 6, 0, false, ALARM_KIND_JUST_ONCE, just_once_schedule_friday);

  s_current_day = s_friday;
  s_current_hour = 6;
  s_current_minute = 0;
  cron_service_wakeup();
  cl_assert_equal_i(s_num_alarms_fired, 1);
}

// TODO:
// - Test disable while snoozing
// - Test delete while snoozing

