/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "comm/ble/kernel_le_client/ancs/ancs.h"
#include "comm/ble/kernel_le_client/ancs/ancs_types.h"
#include "comm/ble/kernel_le_client/ancs/ancs_util.h"

#include "util/buffer.h"

#include "clar.h"

#include <string.h>


// Stubs
///////////////////////////////////////////////////////////

#include "stubs_analytics.h"
#include "stubs_ble.h"
#include "stubs_bt_stack.h"
#include "stubs_logging.h"
#include "stubs_serial.h"
#include "stubs_passert.h"
#include "stubs_mutex.h"
#include "stubs_system_reset.h"
#include "stubs_task_watchdog.h"
#include "stubs_pbl_malloc.h"

// Test data
///////////////////////////////////////////////////////////

#include "ancs_test_data.h"

// Tests
///////////////////////////////////////////////////////////

void test_ancs_util__should_parse_complete_notif_attr_dict(void) {
  bool error = false;
  cl_assert(ancs_util_is_complete_notif_attr_response(s_complete_dict, sizeof(s_complete_dict), &error));
  cl_assert(!error);
}

void test_ancs_util__should_identify_missing_attribute(void) {
  bool error = false;
  cl_assert(!ancs_util_is_complete_notif_attr_response(s_missing_last_attribute, sizeof(s_missing_last_attribute),
        &error));
  cl_assert(!error);
}

void test_ancs_util__should_identify_invalid_attr_length(void) {
  bool error = false;
  cl_assert(!ancs_util_is_complete_notif_attr_response(s_invalid_attribute_length, sizeof(s_invalid_attribute_length),
        &error));
  cl_assert(error);
}

void test_ancs_util__should_parse_incomplete_last_attribute(void) {
  bool error = false;
  cl_assert(!ancs_util_is_complete_notif_attr_response(s_chunked_dict_part_one, sizeof(s_chunked_dict_part_one),
        &error));
  cl_assert(!error);
}

void test_ancs_util__should_not_parse_malformed_notif_attr_dict(void) {
  bool error = false;
  cl_assert(!ancs_util_is_complete_notif_attr_response(s_chunked_dict_part_two, sizeof(s_chunked_dict_part_two),
        &error));
  cl_assert(error);
}

void test_ancs_util__should_extract_dict_from_buffer(void) {
  static const char* expected_message = "This is a very complicated case, Maude. You know, a lotta ins, lotta outs, lotta what-have-you's. And, uh, lotta strands to keep in my head, man. Lotta strands in old Duder's head. Luckily I'm adherin";

  int bytes_written;
  Buffer* b = buffer_create(500);
  bytes_written = buffer_add(b, s_chunked_dict_part_one, sizeof(s_chunked_dict_part_one));
  cl_assert_equal_i(bytes_written, sizeof(s_chunked_dict_part_one));

  bytes_written = buffer_add(b, s_chunked_dict_part_two, sizeof(s_chunked_dict_part_two));
  cl_assert_equal_i(bytes_written, sizeof(s_chunked_dict_part_two));

  bool error = false;
  cl_assert(ancs_util_is_complete_notif_attr_response(b->data, b->bytes_written, &error));
  cl_assert(!error);

  ANCSAttribute* attr_ptrs[NUM_FETCHED_NOTIF_ATTRIBUTES];

  // Only pass the attribute data to ancs_util_get_attr_ptrs:
  const GetNotificationAttributesMsg *msg =
  (const GetNotificationAttributesMsg*) b->data;
  const size_t attributes_length = b->bytes_written - sizeof(*msg);
  cl_assert(ancs_util_get_attr_ptrs(msg->attributes_data, attributes_length, s_fetched_notif_attributes,
        NUM_FETCHED_NOTIF_ATTRIBUTES, attr_ptrs, &error));
  cl_assert(!error);

  ANCSAttribute* message = (ANCSAttribute*) attr_ptrs[3];
  cl_assert(strncmp((char*) message->value, expected_message, message->length) == 0);

  free(b);
}


void test_ancs_util__should_detect_duplicate_message(void) {
#if(0)
  // Extract the dict
  static const char* expected_message = "This is a very complicated case, Maude. You know, a lotta ins, lotta outs, lotta what-have-you's. And, uh, lotta strands to keep in my head, man. Lotta strands in old Duder's head. Luckily I'm adherin";

  ancs_util_reset_notification_cache();

  Buffer* b = buffer_create(500);
  cl_assert_equal_i(buffer_add(b, s_chunked_dict_part_one, sizeof(s_chunked_dict_part_one)), sizeof(s_chunked_dict_part_one));
  cl_assert_equal_i(buffer_add(b, s_chunked_dict_part_two, sizeof(s_chunked_dict_part_two)), sizeof(s_chunked_dict_part_two));

  bool error = false;
  cl_assert(ancs_util_is_complete_notif_attr_response(b->data, b->bytes_written, &error));
  cl_assert(!error);

  ANCSAttribute *attr_ptrs[NUM_FETCHED_NOTIF_ATTRIBUTES];

  const GetNotificationAttributesMsg *msg =
  (const GetNotificationAttributesMsg*) b->data;
  const size_t attributes_length = b->bytes_written - sizeof(*msg);
  cl_assert(ancs_util_get_attr_ptrs(msg->attributes_data, attributes_length, s_fetched_notif_attributes,
        NUM_FETCHED_NOTIF_ATTRIBUTES, attr_ptrs, &error));
  cl_assert(!error);

  ANCSAttribute* message = (ANCSAttribute*) attr_ptrs[3];
  cl_assert(strncmp((char*) message->value, expected_message, message->length) == 0);

  // Ensure the dupe is detected
  bool was_added_to_cache = ancs_util_cache_notification(
      attr_ptrs[FetchedAttributeIndexAppID],
      attr_ptrs[FetchedAttributeIndexTitle],
      attr_ptrs[FetchedAttributeIndexSubtitle],
      attr_ptrs[FetchedAttributeIndexMessage],
      attr_ptrs[FetchedAttributeIndexDate]
  );

  cl_assert(was_added_to_cache);

  was_added_to_cache = ancs_util_cache_notification(
      attr_ptrs[FetchedAttributeIndexAppID],
      attr_ptrs[FetchedAttributeIndexTitle],
      attr_ptrs[FetchedAttributeIndexSubtitle],
      attr_ptrs[FetchedAttributeIndexMessage],
      attr_ptrs[FetchedAttributeIndexDate]
  );

  cl_assert(was_added_to_cache == false);

  // Ensure the dupe is no longer detected
  ancs_util_reset_notification_cache();
  was_added_to_cache = ancs_util_cache_notification(
      attr_ptrs[FetchedAttributeIndexAppID],
      attr_ptrs[FetchedAttributeIndexTitle],
      attr_ptrs[FetchedAttributeIndexSubtitle],
      attr_ptrs[FetchedAttributeIndexMessage],
      attr_ptrs[FetchedAttributeIndexDate]
  );

  cl_assert(was_added_to_cache);

  free(b);

#endif
}
