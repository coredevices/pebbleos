/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/timeline/attribute.h"
#include "pbl/util/size.h"

#include <stdint.h>

// Stubs
////////////////////////////////////////////////////////////////
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"

// Setup
////////////////////////////////////////////////////////////////

static Attribute action1_attributes[] = {
  {.id = AttributeIdTitle, .cstring = "Dismiss"},
};

static Attribute action2_attributes[] = {
  {.id = AttributeIdTitle, .cstring = "Like"},
  {.id = AttributeIdAncsAction, .int8 = 1}
};

static Attribute attributes[] = {
    {.id = AttributeIdTitle, .cstring = "Test Notification"},
    {.id = AttributeIdSubtitle, .cstring = "Subtitle"},
    {.id = AttributeIdBody, .cstring = "This is a test notification. "
        "Look at it and behold the awesome."},
};

void test_attribute__initialize(void) {
}

void test_attribute__cleanup(void) {
}

// Tests
////////////////////////////////////////////////////////////////

void test_attribute__uint32_list(void) {
  AttributeList attr_list = {};
  uint8_t metric_buffer[Uint32ListSize(3)];
  Uint32List *metric_values = (Uint32List *)metric_buffer;
  metric_values->num_values = 3;
  metric_values->values[0] = 100;
  metric_values->values[1] = 200;
  metric_values->values[2] = 300;
  attribute_list_add_uint32_list(&attr_list, AttributeIdMetricIcons, metric_values);

  Uint32List *other = attribute_get_uint32_list(&attr_list, AttributeIdMetricIcons);
  for (int i = 0; i < metric_values->num_values; i++) {
    cl_assert_equal_i(metric_values->values[i], other->values[i]);
  }

  const size_t serialized_size = attribute_list_get_serialized_size(&attr_list);
  cl_assert_equal_i(serialized_size, 19);
  uint8_t serialized_buffer[serialized_size];
  attribute_list_serialize(&attr_list, serialized_buffer, &serialized_buffer[serialized_size]);

  const size_t buffer_size = attribute_list_get_string_buffer_size(&attr_list);
  uint8_t deserialized_buffer[buffer_size];

  AttributeList attr_list_out = {};
  attribute_list_init_list(attr_list.num_attributes, &attr_list_out);
  const uint8_t *buffer = (uint8_t *)deserialized_buffer;
  const uint8_t *cursor = serialized_buffer;
  attribute_deserialize_list((char **)&buffer, (char *)&deserialized_buffer[buffer_size],
                             &cursor, &serialized_buffer[serialized_size],
                             attr_list_out);
  other = attribute_get_uint32_list(&attr_list_out, AttributeIdMetricIcons);
  for (int i = 0; i < metric_values->num_values; i++) {
    cl_assert_equal_i(metric_values->values[i], other->values[i]);
  }
}

static void prv_check_attribute_list_serialize(AttributeList *attr_list_to_serialize,
                                        const uint8_t *expected_attr_list_serialized,
                                        size_t expected_attr_list_serialized_size) {
  uint8_t buffer[expected_attr_list_serialized_size];
  const size_t size = attribute_list_serialize(attr_list_to_serialize, buffer,
                                               buffer + expected_attr_list_serialized_size);
  cl_assert_equal_i(size, expected_attr_list_serialized_size);
  cl_assert_equal_m(expected_attr_list_serialized, buffer, expected_attr_list_serialized_size);
}

void test_attribute__serialize_attr_list(void) {
  AttributeList attr_list1 = {
    .num_attributes = ARRAY_LENGTH(action1_attributes),
    .attributes = action1_attributes
  };
  AttributeList attr_list2 = {
      .num_attributes = ARRAY_LENGTH(action2_attributes),
      .attributes = action2_attributes
  };
  AttributeList attr_list3 = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes
  };

  static uint8_t attr_list1_serialized[] = {
    // Action Attributes
    0x01,                     // Attribute ID - Title
    0x07, 0x00,               // Attribute Length
    // Attribute text:
    'D', 'i', 's', 'm', 'i', 's', 's',
  };

  static uint8_t attr_list2_serialized[] = {
    0x01,                     // Attribute 1 ID - Title
    0x04, 0x00,               // Attribute 1 Length
    // Attribute text:
    'L', 'i', 'k', 'e',
    0x07,                     // Attribute 2 ID - ANCS UID
    0x01, 0x00,               // Attribute 2 Length
    // Attribute text: "Test"
    0x01
  };

  static uint8_t attr_list3_serialized[] = {
    // Attribute 1
    0x01,                     // Attribute ID - Title
    0x11, 0x00,               // Attribute Length
    // Attribute text: "Test Notification"
    0x54, 0x65, 0x73, 0x74, 0x20, 0x4e, 0x6f, 0x74,  0x69, 0x66, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6f,
    0x6e,

    // Attribute 2
    0x02,                     // Attribute ID - Subtitle
    0x08, 0x00,               // Attribute Length
    // Attribute text: "Subtitle"
    'S', 'u', 'b', 't', 'i', 't', 'l', 'e',

    // Attribute 3
    0x03,                     // Attribute ID - Body
    0x3f, 0x00,               // Attribute Length
    // Attribute text: "This is a test notification. Look at it and behold the awesome."
    0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20,  0x61, 0x20, 0x74, 0x65, 0x73, 0x74, 0x20, 0x6e,
    0x6f, 0x74, 0x69, 0x66, 0x69, 0x63, 0x61, 0x74,  0x69, 0x6f, 0x6e, 0x2e, 0x20, 0x4c, 0x6f, 0x6f,
    0x6b, 0x20, 0x61, 0x74, 0x20, 0x69, 0x74, 0x20,  0x61, 0x6e, 0x64, 0x20, 0x62, 0x65, 0x68, 0x6f,
    0x6c, 0x64, 0x20, 0x74, 0x68, 0x65, 0x20, 0x61,  0x77, 0x65, 0x73, 0x6f, 0x6d, 0x65, 0x2e,
  };

  prv_check_attribute_list_serialize(&attr_list1, attr_list1_serialized,
                                     sizeof(attr_list1_serialized));
  prv_check_attribute_list_serialize(&attr_list2, attr_list2_serialized,
                                     sizeof(attr_list2_serialized));
  prv_check_attribute_list_serialize(&attr_list3, attr_list3_serialized,
                                     sizeof(attr_list3_serialized));
}

void test_attribute__attributes_add_to_list(void) {
  static const uint32_t value_uint32 = 123123423;
  static const uint8_t value_uint8 = 17;

  AttributeList list = {0};

  attribute_list_add_cstring(&list, AttributeIdTitle, "Title1");
  cl_assert_equal_s(attribute_get_string(&list, AttributeIdTitle, ""), "Title1");
  cl_assert_equal_i(list.num_attributes, 1);

  attribute_list_add_cstring(&list, AttributeIdSubtitle, "Subtitle");
  cl_assert_equal_s(attribute_get_string(&list, AttributeIdSubtitle, ""), "Subtitle");
  cl_assert_equal_s(attribute_get_string(&list, AttributeIdTitle, ""), "Title1");
  cl_assert_equal_i(list.num_attributes, 2);

  attribute_list_add_cstring(&list, AttributeIdTitle, "Title2");
  cl_assert_equal_s(attribute_get_string(&list, AttributeIdTitle, ""), "Title2");
  cl_assert_equal_s(attribute_get_string(&list, AttributeIdSubtitle, ""), "Subtitle");
  cl_assert_equal_i(list.num_attributes, 2);

  attribute_list_add_uint32(&list, AttributeIdLastUpdated, value_uint32);
  attribute_list_add_uint8(&list, AttributeIdBgColor, value_uint8);
  cl_assert_equal_i(value_uint32, attribute_get_uint32(&list, AttributeIdLastUpdated, 0));
  cl_assert_equal_i(value_uint8, attribute_get_uint8(&list, AttributeIdBgColor, 0));
  cl_assert_equal_i(list.num_attributes, 4);

  attribute_list_destroy_list(&list);
}

void test_attribute__attribute_list_copy(void) {
  AttributeList list = {0};
  attribute_list_add_cstring(&list, AttributeIdTitle, "Title");
  attribute_list_add_cstring(&list, AttributeIdSubtitle, "Subtitle");
  attribute_list_add_cstring(&list, AttributeIdBody, "Body");

  // title + subtitle + body + 3 * (attributeid + pointer)
  size_t size_list = attribute_list_get_buffer_size(&list);
  cl_assert_equal_i(size_list, (5 + 1) + (8 + 1) + (4 + 1) + 3 * sizeof(Attribute));
  uint8_t *buffer = kernel_malloc_check(size_list);
  uint8_t *buffer_orig = buffer;
  AttributeList list2 = {0};
  cl_assert(attribute_list_copy(&list2, &list, buffer, buffer + size_list));
  // check that we haven't modified buffer
  cl_assert(buffer == buffer_orig);
  cl_assert_equal_s(attribute_get_string(&list2, AttributeIdTitle, ""), "Title");
  cl_assert_equal_s(attribute_get_string(&list2, AttributeIdSubtitle, ""), "Subtitle");
  cl_assert_equal_s(attribute_get_string(&list2, AttributeIdBody, ""), "Body");

  // check that the pointers have moved
  cl_assert(attribute_get_string(&list2, AttributeIdTitle, "") !=
    attribute_get_string(&list, AttributeIdTitle, ""));
  cl_assert(attribute_get_string(&list2, AttributeIdSubtitle, "") !=
    attribute_get_string(&list, AttributeIdSubtitle, ""));
  cl_assert(attribute_get_string(&list2, AttributeIdBody, "") !=
    attribute_get_string(&list, AttributeIdBody, ""));
  attribute_list_destroy_list(&list);
  kernel_free(buffer);
}

static void prv_check_app_glance_subtitle_in_attribute_list_deserializes(
    const uint8_t *serialized_attribute_list_to_deserialize,
    size_t serialized_attribute_list_to_deserialize_size, uint8_t num_attributes,
    const char *expected_app_glance_subtitle_after_deserializing) {

  // Get the buffer size needed for the attributes we're going to deserialize
  // We don't have a value to check this against but we implicitly check it because if it's
  // incorrect then the overall deserialization will fail
  const uint8_t *end = serialized_attribute_list_to_deserialize +
                           serialized_attribute_list_to_deserialize_size;
  const uint8_t *buffer_size_cursor = serialized_attribute_list_to_deserialize;
  const int32_t buffer_size =
      attribute_get_buffer_size_for_serialized_attributes(num_attributes, &buffer_size_cursor, end);

  // Allocate buffers both for the Attribute structs as well as the data they'll hold
  Attribute attribute_buffer[num_attributes];
  char attribute_data_buffer[buffer_size];

  // Setup the arguments for the `attribute_deserialize_list` function
  char *attribute_data_buffer_pointer = attribute_data_buffer;
  AttributeList deserialization_result_attribute_list = (AttributeList) {
      .num_attributes = num_attributes,
      .attributes = attribute_buffer,
  };
  const uint8_t *deserialization_cursor = serialized_attribute_list_to_deserialize;

  // Check that the deserialization completes successfully
  cl_assert_equal_b(attribute_deserialize_list(&attribute_data_buffer_pointer,
                                               attribute_data_buffer + buffer_size,
                                               &deserialization_cursor, end,
                                               deserialization_result_attribute_list),
                    true);
  // Check that the app glance subtitle string we deserialized matches the string we expect
  cl_assert_equal_s(attribute_get_string(&deserialization_result_attribute_list,
                                         AttributeIdSubtitleTemplateString, NULL),
                    expected_app_glance_subtitle_after_deserializing);
}

void test_attribute__app_glance_subtitle_in_attribute_list(void) {
  Attribute app_glance_subtitle_attributes[] = {
    {
      .id = AttributeIdSubtitleTemplateString,
      .cstring = "Your app at a glance!"
    },
  };
  AttributeList app_glance_subtitle_attribute_list = {
      .num_attributes = ARRAY_LENGTH(app_glance_subtitle_attributes),
      .attributes = app_glance_subtitle_attributes,
  };
  const uint8_t app_glance_subtitle_attribute_list_serialized[] = {
      0x2F,                     // Attribute ID - App Glance Subtitle
      0x15, 0x00,               // Attribute Length
      // Attribute text:
      'Y', 'o', 'u', 'r', ' ', 'a', 'p', 'p', ' ', 'a', 't', ' ', 'a', ' ',
      'g', 'l', 'a', 'n', 'c', 'e', '!',
  };
  const size_t app_glance_subtitle_attribute_list_serialized_size =
      sizeof(app_glance_subtitle_attribute_list_serialized);

  // Check that serializing the AttributeList matches the serialized byte array above
  prv_check_attribute_list_serialize(&app_glance_subtitle_attribute_list,
                                     app_glance_subtitle_attribute_list_serialized,
                                     app_glance_subtitle_attribute_list_serialized_size);

  // Now let's check that deserializing the serialized byte array above results in the same
  // attributes as the AttributeList above...

  // It's assumed we know the number of attributes in the serialized list, so just copy it from
  // the AttributeList we hope to recreate
  const uint8_t num_attributes = app_glance_subtitle_attribute_list.num_attributes;

  prv_check_app_glance_subtitle_in_attribute_list_deserializes(
      app_glance_subtitle_attribute_list_serialized,
      app_glance_subtitle_attribute_list_serialized_size, num_attributes,
      attribute_get_string(&app_glance_subtitle_attribute_list, AttributeIdSubtitleTemplateString,
                           NULL));
}

void test_attribute__too_long_app_glance_subtitle_in_attribute_list(void) {
  Attribute app_glance_subtitle_attributes[] = {
      {
          .id = AttributeIdSubtitleTemplateString,
          .cstring = "This is a really really really really really really really really really "
                     "really really really really really really really really really really "
                     "long subtitle!"
      },
  };
  // Check that we're actually using a string longer than the max app glance subtitle length
  cl_assert(
      strlen(app_glance_subtitle_attributes->cstring) > ATTRIBUTE_APP_GLANCE_SUBTITLE_MAX_LEN);

  AttributeList app_glance_subtitle_attribute_list = {
      .num_attributes = ARRAY_LENGTH(app_glance_subtitle_attributes),
      .attributes = app_glance_subtitle_attributes,
  };
  const uint8_t app_glance_subtitle_attribute_list_serialized[] = {
      0x2F,                     // Attribute ID - App Glance Subtitle
      0x9D, 0x00,               // Attribute Length
      // Attribute text:
      'T', 'h', 'i', 's', ' ', 'i', 's', ' ', 'a', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'r', 'e', 'a', 'l', 'l', 'y', ' ',
      'l', 'o', 'n', 'g', ' ', 's', 'u', 'b', 't', 'i', 't', 'l', 'e', '!',
  };
  const size_t app_glance_subtitle_attribute_list_serialized_size =
      sizeof(app_glance_subtitle_attribute_list_serialized);

  // Check that serializing the AttributeList matches the serialized byte array above
  // Note that serializing an app glance subtitle that is too long doesn't have any effect; we only
  // respect the max length when deserializing it!
  prv_check_attribute_list_serialize(&app_glance_subtitle_attribute_list,
                                     app_glance_subtitle_attribute_list_serialized,
                                     app_glance_subtitle_attribute_list_serialized_size);

  // Now let's check that deserializing the serialized byte array above results in a truncated
  // version of the original string because it's longer than the max length

  // It's assumed we know the number of attributes in the serialized list, so just copy it from
  // the AttributeList we hope to recreate
  const uint8_t num_attributes = app_glance_subtitle_attribute_list.num_attributes;

  prv_check_app_glance_subtitle_in_attribute_list_deserializes(
      app_glance_subtitle_attribute_list_serialized,
      app_glance_subtitle_attribute_list_serialized_size, num_attributes,
      "This is a really really really really really really really really really really really "
      "really really really really really really really really long su");
}

void test_attribute__unknown_attribute_id_does_not_overflow(void) {
  // has_attribute is indexed by attribute id, so an id this firmware doesn't know must not be
  // written into it.
  static const uint8_t serialized[] = {
    NumAttributeIds + 3,
    0x01, 0x00,
    1,
  };
  bool has_attribute[NumAttributeIds] = {0};
  cl_assert_equal_b(attribute_check_serialized_list(serialized, serialized + sizeof(serialized), 1,
                                                    has_attribute),
                    true);
  for (int i = 0; i < NumAttributeIds; i++) {
    cl_assert_equal_b(has_attribute[i], false);
  }
}
