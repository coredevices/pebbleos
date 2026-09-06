/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/notifications/alerts_preferences.h"
#include "pbl/services/notifications/ancs/ancs_filtering.h"
#include "pbl/services/blob_db/ios_notif_pref_db.h"
#include "pbl/services/timeline/attributes_actions.h"


// Stubs
////////////////////////////////////////////////////////////////
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"

// Fakes
////////////////////////////////////////////////////////////////

static time_t s_now;
time_t rtc_get_time(void) {
  return s_now;
}

RtcTicks rtc_get_ticks(void) {
  return 0;
}

bool s_performed_store = false;
AttributeList *s_expected_attributes = NULL;
TimelineItemActionGroup *s_expected_actions = NULL;

static uint8_t app_id_data[] = {
  0x00,                // id
  0x04, 0x00,          // length
  'P', 'h', 'i', 'l',  // Value
};
static ANCSAttribute *s_app_id_attr = (ANCSAttribute *)&app_id_data;

static uint8_t display_name_data[] = {
  0x00,                               // id
  0x07, 0x00,                         // length
  'A', 'w', 'e', 's', 'o', 'm', 'e',  // Value
};
static ANCSAttribute *s_display_name_attr = (ANCSAttribute *)&display_name_data;

static uint8_t title_data[] = {
  0x01,                                                                 // id
  0x0E, 0x00,                                                           // length
  'A', 'p', 'p', 'l', 'e', ' ', 'P', 'a', 'y', ' ', '=', ' ', ':', '('  // Value
};
static ANCSAttribute *s_title_attr = (ANCSAttribute *)&title_data;

static struct {
  StringList list;
  char data[1];
} s_empty_filtering_rules = {
  .list = {
    .serialized_byte_length = 1,
  },
  .data = { 0 },
};

status_t ios_notif_pref_db_store_prefs(const uint8_t *app_id, int length, AttributeList *attr_list,
                                       TimelineItemActionGroup *action_group) {
  s_performed_store = true;
  if (s_expected_attributes) {
    const int buf_size = 256;
    uint8_t expected_buf[buf_size];
    uint8_t actual_buf[buf_size];
    attributes_actions_serialize_payload(s_expected_attributes, s_expected_actions,
                                         expected_buf, buf_size);
    attributes_actions_serialize_payload(attr_list, action_group,
                                         actual_buf, buf_size);
    cl_assert_equal_m(expected_buf, actual_buf,
        attributes_actions_get_serialized_payload_size(s_expected_attributes, s_expected_actions));
  }
  return S_SUCCESS;
}

void ios_notif_pref_db_free_prefs(iOSNotifPrefs *prefs) {
  return;
}

static void prv_compare_notif_prefs(iOSNotifPrefs *prefs1, iOSNotifPrefs *prefs2) {
  const int buf_size = 256;
  uint8_t buf1[buf_size];
  uint8_t buf2[buf_size];
  attributes_actions_serialize_payload(&prefs1->attr_list, &prefs1->action_group, buf1, buf_size);
  attributes_actions_serialize_payload(&prefs2->attr_list, &prefs2->action_group, buf2, buf_size);
  cl_assert_equal_m(buf1, buf2, attributes_actions_get_serialized_payload_size(&prefs2->attr_list, &prefs2->action_group));
}

void test_ancs_filtering__initialize(void) {
  s_now = 1;
  s_performed_store = false;
  s_expected_attributes = NULL;
  s_expected_actions = NULL;
}

void test_ancs_filtering__cleanup(void) {
}

void test_ancs_filtering__record_app_no_action_needed(void) {
  // We have some existing prefs which includes all the defaults
  // We don't need to insert anything
  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 7,
      .attributes = (Attribute[]) {
        { .id = AttributeIdTitle, .cstring = "Title" },
        { .id = AttributeIdBody, .cstring = "Body" },
        { .id = AttributeIdAppName, .cstring = "Awesome" },
        { .id = AttributeIdLastUpdated, .uint32 = s_now },
        { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_Always },
        { .id = AttributeIdMuteExpiration, .uint32 = 0 },
        { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
      },
    },
  };
  iOSNotifPrefs *existing_prefs = &prefs;

  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, s_display_name_attr, s_title_attr);
  cl_assert(!s_performed_store);
}

void test_ancs_filtering__record_app_no_prefs_yet(void) {
  // No existing prefs yet, we should insert all the defaults
  iOSNotifPrefs *existing_prefs = NULL;

  AttributeList attr_list = {
    .num_attributes = 5,
    .attributes = (Attribute[]) {
      { .id = AttributeIdAppName, .cstring = "Awesome" },
      { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_None },
      { .id = AttributeIdMuteExpiration, .uint32 = 0 },
      { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
      { .id = AttributeIdLastUpdated, .uint32 = s_now },
    }
  };
  s_expected_attributes = &attr_list;

  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, s_display_name_attr, s_title_attr);
  cl_assert(s_performed_store);

  // Make sure the our existing prefs got updated
  iOSNotifPrefs expected_prefs = {
    .attr_list = attr_list,
  };
  prv_compare_notif_prefs(existing_prefs, &expected_prefs);
}

void test_ancs_filtering__record_app_existing_mute(void) {
  // We already have a mute attribute and 2 unrelated attributes, make sure mute doesn't change
  // and the remaining default attributes are added
  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 3,
      .attributes = (Attribute[]) {
        { .id = AttributeIdTitle, .cstring = "Title" },
        { .id = AttributeIdBody, .cstring = "Body" },
        { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_Always },
      },
    },
  };
  iOSNotifPrefs *existing_prefs = &prefs;

  AttributeList expected_attributes = {
    .num_attributes = 7,
    .attributes = (Attribute[]) {
      { .id = AttributeIdTitle, .cstring = "Title" },
      { .id = AttributeIdBody, .cstring = "Body" },
      { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_Always },
      { .id = AttributeIdAppName, .cstring = "Awesome" },
      { .id = AttributeIdMuteExpiration, .uint32 = 0 },
      { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
      { .id = AttributeIdLastUpdated, .uint32 = s_now },
    }
  };
  s_expected_attributes = &expected_attributes;

  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, s_display_name_attr, s_title_attr);
  cl_assert(s_performed_store);

  // Make sure the our existing prefs got updated
  iOSNotifPrefs expected_prefs = {
    .attr_list = expected_attributes,
  };
  prv_compare_notif_prefs(existing_prefs, &expected_prefs);
}

void test_ancs_filtering__record_app_existing_display_name(void) {
  // We got a new app name, make sure it gets updated
  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdAppName, .cstring = "Phil was here" },
      },
    },
  };
  iOSNotifPrefs *existing_prefs = &prefs;

  AttributeList expected_attributes = {
    .num_attributes = 5,
    .attributes = (Attribute[]) {
      { .id = AttributeIdAppName, .cstring = "Awesome" },
      { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_None },
      { .id = AttributeIdMuteExpiration, .uint32 = 0 },
      { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
      { .id = AttributeIdLastUpdated, .uint32 = s_now },
    }
  };
  s_expected_attributes = &expected_attributes;

  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, s_display_name_attr, s_title_attr);
  cl_assert(s_performed_store);

  // Make sure the our existing prefs got updated
  iOSNotifPrefs expected_prefs = {
    .attr_list = expected_attributes,
  };
  prv_compare_notif_prefs(existing_prefs, &expected_prefs);
}

void test_ancs_filtering__record_app_update_timestamp(void) {
  // We already have all the default attributes, but it has been a while since we got our last
  // notification from this source
  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 5,
      .attributes = (Attribute[]) {
        { .id = AttributeIdAppName, .cstring = "Awesome" },
        { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_None },
        { .id = AttributeIdMuteExpiration, .uint32 = 0 },
        { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
        { .id = AttributeIdLastUpdated, .uint32 = s_now },
      },
    },
  };
  iOSNotifPrefs *existing_prefs = &prefs;

  s_now += SECONDS_PER_DAY - 1;
  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, s_display_name_attr, s_title_attr);
  cl_assert(!s_performed_store);

  s_now += 2;
  AttributeList expected_attributes = {
    .num_attributes = 5,
    .attributes = (Attribute[]) {
      { .id = AttributeIdAppName, .cstring = "Awesome" },
      { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_None },
      { .id = AttributeIdMuteExpiration, .uint32 = 0 },
      { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
      { .id = AttributeIdLastUpdated, .uint32 = s_now },
    }
  };
  s_expected_attributes = &expected_attributes;

  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, s_display_name_attr, s_title_attr);
  cl_assert(s_performed_store);

  // Make sure the our existing prefs got updated
  iOSNotifPrefs expected_prefs = {
    .attr_list = expected_attributes,
  };
  prv_compare_notif_prefs(existing_prefs, &expected_prefs);
}

void test_ancs_filtering__should_ignore_because_muted(void) {
  iOSNotifPrefs mute_always = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdMuteDayOfWeek, .uint8 = 0x7F },
      }
    }
  };

  iOSNotifPrefs mute_weekends = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdMuteDayOfWeek, .uint8 = 0x41 },
      }
    }
  };

  iOSNotifPrefs mute_weekdays = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdMuteDayOfWeek, .uint8 = 0x3E },
      }
    }
  };

  s_now = 1451606400; // Friday Jan 1, 2016
  cl_assert(ancs_filtering_is_muted(&mute_always));
  cl_assert(!ancs_filtering_is_muted(&mute_weekends));
  cl_assert(ancs_filtering_is_muted(&mute_weekdays));

  s_now += SECONDS_PER_DAY; // Saturday Jan 2, 2016
  cl_assert(ancs_filtering_is_muted(&mute_always));
  cl_assert(ancs_filtering_is_muted(&mute_weekends));
  cl_assert(!ancs_filtering_is_muted(&mute_weekdays));

  s_now += SECONDS_PER_DAY; // Sunday Jan 3, 2016
  cl_assert(ancs_filtering_is_muted(&mute_always));
  cl_assert(ancs_filtering_is_muted(&mute_weekends));
  cl_assert(!ancs_filtering_is_muted(&mute_weekdays));

  s_now += SECONDS_PER_DAY; // Monday Jan 4, 2016
  cl_assert(ancs_filtering_is_muted(&mute_always));
  cl_assert(!ancs_filtering_is_muted(&mute_weekends));
  cl_assert(ancs_filtering_is_muted(&mute_weekdays));
}

void test_ancs_filtering__record_app_existing_mute_expiration(void) {
  // We already have a mute expiration attribute, make sure it doesn't change
  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 4,
      .attributes = (Attribute[]) {
        { .id = AttributeIdTitle, .cstring = "Title" },
        { .id = AttributeIdBody, .cstring = "Body" },
        { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_None },
        { .id = AttributeIdMuteExpiration, .uint32 = 12345678 },
      },
    },
  };
  iOSNotifPrefs *existing_prefs = &prefs;

  AttributeList expected_attributes = {
    .num_attributes = 7,
    .attributes = (Attribute[]) {
      { .id = AttributeIdTitle, .cstring = "Title" },
      { .id = AttributeIdBody, .cstring = "Body" },
      { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_None },
      { .id = AttributeIdMuteExpiration, .uint32 = 12345678 },
      { .id = AttributeIdAppName, .cstring = "Awesome" },
      { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
      { .id = AttributeIdLastUpdated, .uint32 = s_now },
    }
  };
  s_expected_attributes = &expected_attributes;

  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, s_display_name_attr, s_title_attr);
  cl_assert(s_performed_store);

  // Make sure the our existing prefs got updated
  iOSNotifPrefs expected_prefs = {
    .attr_list = expected_attributes,
  };
  prv_compare_notif_prefs(existing_prefs, &expected_prefs);
}

void test_ancs_filtering__should_ignore_because_timed_muted(void) {
  iOSNotifPrefs muted = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdMuteExpiration, .uint32 = 2000 },
      }
    }
  };

  s_now = 1000;
  cl_assert(ancs_filtering_is_muted(&muted));

  s_now = 2000;
  cl_assert(!ancs_filtering_is_muted(&muted));

  s_now = 3000;
  cl_assert(!ancs_filtering_is_muted(&muted));
}

void test_ancs_filtering__record_app_no_display_name(void) {
  iOSNotifPrefs *existing_prefs = NULL;

  // No display name so we expect the app name to be the title
  AttributeList expected_attributes = {
    .num_attributes = 5,
    .attributes = (Attribute[]) {
      { .id = AttributeIdAppName, .cstring = "Apple Pay = :(" },
      { .id = AttributeIdMuteDayOfWeek, .uint8 = MuteBitfield_None },
      { .id = AttributeIdMuteExpiration, .uint32 = 0 },
      { .id = AttributeIdNotificationFilteringRules, .string_list = &s_empty_filtering_rules.list },
      { .id = AttributeIdLastUpdated, .uint32 = s_now },
    }
  };
  s_expected_attributes = &expected_attributes;

  ancs_filtering_record_app(&existing_prefs, s_app_id_attr, NULL, s_title_attr);
  cl_assert(s_performed_store);

  // Make sure the our existing prefs got updated
  iOSNotifPrefs expected_prefs = {
    .attr_list = expected_attributes,
  };
  prv_compare_notif_prefs(existing_prefs, &expected_prefs);
}

void test_ancs_filtering__matches_text_rule_body_case_insensitive(void) {
  static uint8_t body_data[] = {
    0x03,                         // id
    0x12, 0x00,                   // length
    'M', 'e', 'e', 't', 'i', 'n', 'g', ' ', 'i', 's', ' ', 'S', 'P', 'A', 'M', ' ', 'n', 'o'
  };
  ANCSAttribute *body_attr = (ANCSAttribute *)&body_data;

  struct {
    StringList list;
    char data[9];
  } filtering_rules = {
    .list = {
      .serialized_byte_length = 9,
    },
    // count=1, type=text, field=body, case=insensitive, pattern="spam\0"
    .data = { 0x01, 0x00, 0x02, 0x00, 's', 'p', 'a', 'm', '\0' },
  };

  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdNotificationFilteringRules, .string_list = &filtering_rules.list },
      },
    },
  };

  cl_assert(ancs_filtering_matches_rules(&prefs, s_title_attr, NULL, body_attr));
}

void test_ancs_filtering__matches_text_rule_title_case_sensitive(void) {
  struct {
    StringList list;
    char data[9];
  } filtering_rules = {
    .list = {
      .serialized_byte_length = 9,
    },
    // count=1, type=text, field=title, case=sensitive, pattern="Apple\0"
    .data = { 0x01, 0x00, 0x01, 0x01, 'A', 'p', 'p', 'l', 'e' },
  };
  filtering_rules.data[8] = '\0';

  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdNotificationFilteringRules, .string_list = &filtering_rules.list },
      },
    },
  };

  cl_assert(ancs_filtering_matches_rules(&prefs, s_title_attr, NULL, NULL));
}

void test_ancs_filtering__does_not_match_regex_rule(void) {
  struct {
    StringList list;
    char data[9];
  } filtering_rules = {
    .list = {
      .serialized_byte_length = 9,
    },
    // count=1, type=regex, field=any, case=insensitive, pattern=".*\0"
    .data = { 0x01, 0x01, 0x00, 0x00, '.', '*', '\0', 0x00, 0x00 },
  };
  filtering_rules.list.serialized_byte_length = 7;

  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdNotificationFilteringRules, .string_list = &filtering_rules.list },
      },
    },
  };

  cl_assert(!ancs_filtering_matches_rules(&prefs, s_title_attr, NULL, NULL));
}

void test_ancs_filtering__matches_text_rule_title_via_subtitle(void) {
  static uint8_t subtitle_data[] = {
    0x02,                                                            // id
    0x0B, 0x00,                                                      // length
    'S', 'o', 'm', 'e', ' ', 'S', 'e', 'r', 'v', 'e', 'r'           // Value
  };
  ANCSAttribute *subtitle_attr = (ANCSAttribute *)&subtitle_data;

  struct {
    StringList list;
    char data[11];
  } filtering_rules = {
    .list = {
      .serialized_byte_length = 11,
    },
    // count=1, type=text, field=title, case=insensitive, pattern="server\0"
    .data = { 0x01, 0x00, 0x01, 0x00, 's', 'e', 'r', 'v', 'e', 'r', '\0' },
  };

  iOSNotifPrefs prefs = {
    .attr_list = {
      .num_attributes = 1,
      .attributes = (Attribute[]) {
        { .id = AttributeIdNotificationFilteringRules, .string_list = &filtering_rules.list },
      },
    },
  };

  // "server" is in the subtitle, not the title: a title rule must match it.
  cl_assert(ancs_filtering_matches_rules(&prefs, s_title_attr, subtitle_attr, NULL));
  // A body rule must not consider the subtitle.
  filtering_rules.data[2] = 0x02;
  cl_assert(!ancs_filtering_matches_rules(&prefs, s_title_attr, subtitle_attr, NULL));
}
