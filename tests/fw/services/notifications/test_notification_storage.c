/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/notifications/notification_storage.h"
#include "pbl/services/notifications/notification_storage_private.h"

#include "flash_region/flash_region.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/util/size.h"

#include "clar.h"

#include "stdbool.h"

// Stubs
////////////////////////////////////
#include "fake_spi_flash.h"
#include "fake_rtc.h"
#include "stubs_analytics.h"
#include "stubs_hexdump.h"
#include "stubs_layout_layer.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_prompt.h"
#include "stubs_rand_ptr.h"
#include "stubs_serial.h"
#include "stubs_sleep.h"
#include "stubs_task_watchdog.h"

#define TEST_START FLASH_REGION_FILE_TEST_SPACE_BEGIN
#define TEST_SIZE (FLASH_REGION_FILE_TEST_SPACE_END - \
    FLASH_REGION_FILE_TEST_SPACE_BEGIN)

extern void notification_storage_reset(void);

typedef void (*SystemTaskEventCallback)(void *data);

static Attribute action1_attributes[] = {
  {.id = AttributeIdTitle, .cstring = "Dismiss"},
};

static Attribute action2_attributes[] = {
  {.id = AttributeIdTitle, .cstring = "Archive"},
};

static StringList string_list = {
    .serialized_byte_length = 3,
    .data = "A\0B",
};

static Attribute action3_attributes[] = {
  {.id = AttributeIdAncsAction, .int8 = 1},
  {.id = AttributeIdCannedResponses, .string_list = &string_list,},
};

static TimelineItemAction actions[] = {
  {
    .id = 0,
    .type = TimelineItemActionTypeResponse,
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(action1_attributes),
      .attributes = action1_attributes
    }
  },
  {
    .id = 1, .type = TimelineItemActionTypeResponse,
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(action2_attributes),
      .attributes = action2_attributes
    }
  },
  {
    .id = 2,
    .type = TimelineItemActionTypeResponse,
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(action3_attributes),
      .attributes = action3_attributes
    }
  },
};

static Attribute attributes[] = {
    {.id = AttributeIdTitle, .cstring = "Sender"},
    {.id = AttributeIdBody, .cstring = "Message"},
    {.id = AttributeIdSubtitle, .cstring = "Subject"},
};

bool system_task_add_callback(SystemTaskEventCallback cb, void *data) {
  return true;
}

PebblePhoneCaller* phone_call_util_create_caller(const char *number, const char *name) {
  return NULL;
}

// Setup
////////////////////////////////////

void test_notification_storage__initialize(void) {
  fake_spi_flash_init(0, 0x1000000);
  pfs_init(false);
  pfs_format(false /* write erase headers */);
  notification_storage_reset();
}

void test_notification_storage__cleanup(void) {
}

static void compare_attr_list(AttributeList a, AttributeList b) {
  cl_assert_equal_i(a.num_attributes, b.num_attributes);
  for (int i = 0; i < a.num_attributes; i++) {
    cl_assert_equal_i(a.attributes[i].id, b.attributes[i].id);
    switch (a.attributes[i].id) {
      case AttributeIdTitle:
      case AttributeIdSubtitle:
      case AttributeIdBody:
        cl_assert_equal_s(a.attributes[i].cstring, b.attributes[i].cstring);
        break;
      case AttributeIdAncsAction:
        cl_assert_equal_i(a.attributes[i].int8, b.attributes[i].int8);
        break;
      case AttributeIdCannedResponses: {
        StringList *list_a = a.attributes[i].string_list;
        StringList *list_b = b.attributes[i].string_list;
        cl_assert_equal_i(list_a->serialized_byte_length, list_b->serialized_byte_length);
        cl_assert_equal_i(
            string_list_count(list_a),
            string_list_count(list_b)
        );
        uint32_t count = string_list_count(list_a);
        for (uint32_t idx = 0; i<count; i++) {
          cl_assert_equal_s(
              string_list_get_at(list_a, idx),
              string_list_get_at(list_b, idx)
          );
        }
        break;
      }

      default:
        cl_assert(false);
        break;
    }
  }
}

static void compare_notifications(TimelineItem *a, TimelineItem *b) {
  cl_assert(uuid_equal(&a->header.id, &b->header.id));
  cl_assert_equal_i(a->header.ancs_uid, b->header.ancs_uid);
  cl_assert_equal_i(a->header.status, b->header.status);
  cl_assert_equal_i(a->header.timestamp, b->header.timestamp);
  cl_assert_equal_i(a->header.layout, b->header.layout);
  compare_attr_list(a->attr_list, b->attr_list);
  cl_assert_equal_i(a->action_group.num_actions, b->action_group.num_actions);
  for (int i = 0; i < a->action_group.num_actions; i++) {
    cl_assert_equal_i(a->action_group.actions[i].id, b->action_group.actions[i].id);
    cl_assert_equal_i(a->action_group.actions[i].type, b->action_group.actions[i].type);
    compare_attr_list(a->action_group.actions[i].attr_list, b->action_group.actions[i].attr_list);
  }
}

typedef struct {
  Uuid older_id;
  Uuid recent_id;
  uint8_t header_count;
  uint8_t item_count;
} NotificationItemsIteratorContext;

static bool prv_items_iterator_callback(void *data, const CommonTimelineItemHeader *header,
                                        const TimelineItem *item) {
  NotificationItemsIteratorContext *context = data;
  if (item) {
    context->item_count++;
    cl_assert(uuid_equal(&item->header.id, &context->recent_id));
    cl_assert_equal_s(attribute_get_string(&item->attr_list, AttributeIdTitle, NULL), "Sender");
  } else {
    context->header_count++;
    cl_assert(uuid_equal(&header->id, &context->older_id));
  }
  return true;
}

typedef struct {
  Uuid expected_ids[2];
  uint8_t item_count;
} RecoveringNotificationItemsIteratorContext;

static bool prv_recovering_items_iterator_callback(void *data,
                                                   const CommonTimelineItemHeader *header,
                                                   const TimelineItem *item) {
  RecoveringNotificationItemsIteratorContext *context = data;
  cl_assert(item);
  cl_assert(context->item_count < ARRAY_LENGTH(context->expected_ids));
  cl_assert(uuid_equal(&item->header.id, &context->expected_ids[context->item_count]));
  context->item_count++;
  return true;
}

// Tests
////////////////////////////////////
void test_notification_storage__basic(void) {
  Uuid id = {0x6b, 0xf6, 0x21, 0x5b, 0xc9, 0x7f, 0x40, 0x9e,
             0x8c, 0x31, 0x4f, 0x55, 0x65, 0x72, 0x22, 0xb4};
  TimelineItem e = {
    .header = {
      .id = id,
      .status = 0,
      .layout = LayoutIdGeneric,
      .type = TimelineItemTypeNotification,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  notification_storage_store(&e);

  TimelineItem r;
  cl_assert(notification_storage_get(&id, &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  cl_assert(notification_storage_get(&id, &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  Uuid invalid_uuid;
  uuid_generate(&invalid_uuid);
  cl_assert_equal_b(notification_storage_get(&invalid_uuid, &r), false);
}

void test_notification_storage__iterate_items_after_skips_old_and_deleted_payloads(void) {
  TimelineItem older = {
      .header =
          {
              .id = UuidMake(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
              .timestamp = 100,
              .type = TimelineItemTypeNotification,
              .layout = LayoutIdGeneric,
          },
      .attr_list =
          {
              .num_attributes = ARRAY_LENGTH(attributes),
              .attributes = attributes,
          },
  };
  TimelineItem recent = older;
  recent.header.id = UuidMake(2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  recent.header.timestamp = 200;
  TimelineItem deleted = older;
  deleted.header.id = UuidMake(3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  deleted.header.timestamp = 300;

  notification_storage_store(&older);
  notification_storage_store(&recent);
  notification_storage_store(&deleted);
  notification_storage_remove(&deleted.header.id);

  NotificationItemsIteratorContext context = {
      .older_id = older.header.id,
      .recent_id = recent.header.id,
  };
  notification_storage_iterate_items_after(200, prv_items_iterator_callback, &context);

  cl_assert_equal_i(context.header_count, 1);
  cl_assert_equal_i(context.item_count, 1);
}

void test_notification_storage__iterate_items_after_skips_corrupt_record(void) {
  TimelineItem first = {
      .header =
          {
              .id = UuidMake(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
              .timestamp = 100,
              .type = TimelineItemTypeNotification,
              .layout = LayoutIdGeneric,
          },
      .attr_list =
          {
              .num_attributes = ARRAY_LENGTH(attributes),
              .attributes = attributes,
          },
  };
  TimelineItem corrupt = first;
  corrupt.header.id = UuidMake(2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  corrupt.header.timestamp = 200;
  corrupt.header.status = 0xC0;
  TimelineItem last = first;
  last.header.id = UuidMake(3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  last.header.timestamp = 300;

  notification_storage_store(&first);
  notification_storage_store(&corrupt);
  notification_storage_store(&last);

  RecoveringNotificationItemsIteratorContext context = {
      .expected_ids = {first.header.id, last.header.id},
  };
  notification_storage_iterate_items_after(0, prv_recovering_items_iterator_callback, &context);

  cl_assert_equal_i(context.item_count, ARRAY_LENGTH(context.expected_ids));
}

void test_notification_storage__multiple(void) {
  Uuid i1 ;
  uuid_generate(&i1);
  TimelineItem e1 = {
    .header = {
      .id = i1,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  Uuid i2;
  uuid_generate(&i2);
  TimelineItem e2 = {
    .header = {
      .id = i2,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda6,
    },
    .attr_list = {
      .num_attributes = 2,
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = 1,
      .actions = &actions[2],
    }
  };

  Uuid i3;
  uuid_generate(&i3);
  TimelineItem e3 = {
    .header = {
      .id = i3,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda7,
    },
    .attr_list = {
      .num_attributes = 1,
      .attributes = &attributes[2],
    },
    .action_group = {
      .num_actions = 2,
      .actions = actions,
    }
  };

  notification_storage_store(&e1);

  TimelineItem r;
  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&e1, &r);
  free(r.allocated_buffer);

  notification_storage_store(&e2);
  notification_storage_store(&e3);

  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&e1, &r);
  free(r.allocated_buffer);

  cl_assert(notification_storage_get(&i2, &r));
  compare_notifications(&e2, &r);
  free(r.allocated_buffer);

  cl_assert(notification_storage_get(&i3, &r));
  compare_notifications(&e3, &r);
  free(r.allocated_buffer);

  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&e1, &r);
  free(r.allocated_buffer);
}

void test_notification_storage__remove_single(void) {
  Uuid i;
  uuid_generate(&i);
  TimelineItem e = {
    .header = {
      .id = i,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  notification_storage_store(&e);

  TimelineItem r;
  cl_assert(notification_storage_get(&i, &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  notification_storage_remove(&i);
  cl_assert_equal_b(notification_storage_get(&i, &r), false);
}

void test_notification_storage__set_actioned_flag(void) {
  Uuid i;
  uuid_generate(&i);
  TimelineItem e = {
    .header = {
      .id = i,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  notification_storage_store(&e);

  TimelineItem r;
  cl_assert(notification_storage_get(&i, &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  notification_storage_set_status(&i, TimelineItemStatusActioned);
  cl_assert(notification_storage_get(&i, &r));
  cl_assert(uuid_equal(&e.header.id, &r.header.id));
  cl_assert_equal_i(e.header.ancs_uid, r.header.ancs_uid);
  cl_assert_equal_i(TimelineItemStatusActioned, r.header.status);
  cl_assert_equal_i(e.header.timestamp, r.header.timestamp);
  cl_assert_equal_i(e.header.layout, r.header.layout);
  compare_attr_list(e.attr_list, r.attr_list);
}

void test_notification_storage__remove_multiple_first(void) {
 Uuid i1;
 uuid_generate(&i1);
  TimelineItem e1 = {
    .header = {
      .id = i1,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  Uuid i2;
  uuid_generate(&i2);
  TimelineItem e2 = {
    .header = {
      .id = i2,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda6,
    },
    .attr_list = {
      .num_attributes = 2,
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = 1,
      .actions = &actions[2],
    }
  };

  notification_storage_store(&e1);
  notification_storage_store(&e2);

  notification_storage_remove(&i1);

  TimelineItem r;
  cl_assert_equal_b(notification_storage_get(&i1, &r), false);

  cl_assert(notification_storage_get(&i2, &r));
  compare_notifications(&e2, &r);
  free(r.allocated_buffer);
}

void test_notification_storage__remove_add(void) {
  Uuid i1;
  uuid_generate(&i1);
  TimelineItem e1 = {
    .header = {
      .id = i1,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  Uuid i2;
  uuid_generate(&i2);
  TimelineItem e2 = {
    .header = {
      .id = i2,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda6,
    },
    .attr_list = {
      .num_attributes = 2,
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = 1,
      .actions = &actions[2],
    }
  };

  Uuid i3;
  uuid_generate(&i3);
  TimelineItem e3 = {
    .header = {
      .id = i3,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda7,
    },
    .attr_list = {
      .num_attributes = 1,
      .attributes = &attributes[2],
    },
    .action_group = {
      .num_actions = 2,
      .actions = actions,
    }
  };

  notification_storage_store(&e1);
  notification_storage_store(&e2);

  notification_storage_remove(&i2);

  TimelineItem r;
  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&e1, &r);
  free(r.allocated_buffer);

  cl_assert_equal_b(notification_storage_get(&i2, &r), false);

  notification_storage_store(&e3);

  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&e1, &r);
  free(r.allocated_buffer);

  cl_assert_equal_b(notification_storage_get(&i2, &r), false);

  cl_assert(notification_storage_get(&i3, &r));
  compare_notifications(&e3, &r);
  free(r.allocated_buffer);

  e2.header.timestamp = e3.header.timestamp + 1;
  notification_storage_store(&e2);

  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&e1, &r);
  free(r.allocated_buffer);

  cl_assert(notification_storage_get(&i2, &r));
  compare_notifications(&e2, &r);
  free(r.allocated_buffer);

  cl_assert(notification_storage_get(&i3, &r));
  compare_notifications(&e3, &r);
  free(r.allocated_buffer);
}

void test_notification_storage__remove_add_compress(void) {
  time_t timestamp = 0x10000000;
  TimelineItem e = {
    .header = {
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };


  const size_t notif_size = sizeof(SerializedTimelineItemHeader) +
      timeline_item_get_serialized_payload_size(&e);
  const size_t file_size = NOTIFICATION_STORAGE_FILE_SIZE;
  const size_t count = file_size / notif_size;
  Uuid uuids[count];

  int i;
  for (i = 0; i < count; i++) {
    uuid_generate(&uuids[i]);
    e.header.id = uuids[i];
    e.header.timestamp = timestamp + i;
    notification_storage_store(&e);
  }
  TimelineItem r;
  cl_assert(notification_storage_get(&uuids[0], &r));
  e.header.id = uuids[0];
  e.header.timestamp = timestamp;
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  cl_assert(notification_storage_get(&uuids[i - 1], &r));
  e.header.id = uuids[i - 1];
  e.header.timestamp = timestamp + i - 1;
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  // Storage is now full and none have been removed.
  // As more space is needed, blocks of 4k are freed up
  Uuid uuid;
  uuid_generate(&uuid);
  e.header.id = uuid;
  e.header.timestamp = timestamp + i;
  notification_storage_store(&e);
  cl_assert(notification_storage_get(&uuid, &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  const size_t block_size = file_size / 4;
  int erase_count = block_size / notif_size + (((block_size % notif_size) > 0) ? 1 : 0);
  int j;
  for (j = 0; j < erase_count; j++) {
    e.header.id = uuids[j];
    e.header.timestamp = timestamp + j;
    cl_assert_equal_b(notification_storage_get(&uuids[j], &r), false);
  }
  e.header.id = uuids[j];
  e.header.timestamp = timestamp + j;
  cl_assert(notification_storage_get(&uuids[j], &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  // Fill up storage again
  for (++i; i < count + erase_count; i++) {
    e.header.id = uuids[i % count];
    e.header.timestamp = timestamp + i;
    notification_storage_store(&e);
    cl_assert(notification_storage_get(&uuids[i % count], &r));
    compare_notifications(&e, &r);
    free(r.allocated_buffer);
  }

  //Check that no notifications have been deleted
  e.header.id = uuids[j];
  e.header.timestamp = timestamp + j;
  cl_assert(notification_storage_get(&uuids[j], &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  //Free up enough space for one notification by removing one
  notification_storage_remove(&uuids[i/2]);
  e.header.id = uuids[i/2];
  e.header.timestamp = timestamp + i/2;
  cl_assert_equal_b(notification_storage_get(&uuids[i/2], &r), false);

  //Add another notification. Compression should take place without freeing up a 4k block
  e.header.id = uuids[i];
  e.header.timestamp = timestamp + i;
  notification_storage_store(&e);
  cl_assert(notification_storage_get(&uuids[i], &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  //Ensure that compression does not remove old notifications
  e.header.id = uuids[j];
  e.header.timestamp = timestamp + j;
  cl_assert(notification_storage_get(&uuids[j], &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  //Add another notification. Compression should free up another 4k block
  e.header.id = uuids[i];
  e.header.timestamp = timestamp + i;
  notification_storage_store(&e);
  cl_assert(notification_storage_get(&uuids[i], &r));
  compare_notifications(&e, &r);
  free(r.allocated_buffer);

  // Check that an expected number of notifications were removed
  erase_count += j;
  for (; j < erase_count; j++) {
    e.header.id = uuids[j];
    e.header.timestamp = timestamp + j;
    cl_assert_equal_b(notification_storage_get(&uuids[j], &r), false);
  }
}

void test_notification_storage__find_ancs_id(void) {
  Uuid i1;
  uuid_generate(&i1);
  TimelineItem e1 = {
    .header = {
      .id = i1,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  Uuid i2;
  uuid_generate(&i2);
  TimelineItem e2 = {
    .header = {
      .id = i2,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 1,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda6,
    },
    .attr_list = {
      .num_attributes = 2,
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = 1,
      .actions = &actions[2],
    }
  };

  Uuid i3;
  uuid_generate(&i3);
  TimelineItem e3 = {
    .header = {
      .id = i3,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 84,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda7,
    },
    .attr_list = {
      .num_attributes = 1,
      .attributes = &attributes[2],
    },
    .action_group = {
      .num_actions = 2,
      .actions = actions,
    }
  };

  notification_storage_store(&e1);
  notification_storage_store(&e2);
  notification_storage_store(&e3);

  Uuid u;
  cl_assert(notification_storage_find_ancs_notification_id(0, &u));
  cl_assert(uuid_equal(&u, &e1.header.id));
  cl_assert(notification_storage_find_ancs_notification_id(1, &u));
  cl_assert(uuid_equal(&u, &e2.header.id));
  cl_assert(notification_storage_find_ancs_notification_id(84, &u));
  cl_assert(uuid_equal(&u, &e3.header.id));

  // Add a new notification with the same ANCS UID as an existing notification to make sure we
  // return the new notification instead of the old one
  TimelineItem e4 = e3;
  uuid_generate(&e4.header.id);
  notification_storage_store(&e4);
  cl_assert(notification_storage_find_ancs_notification_id(84, &u));
  cl_assert(uuid_equal(&u, &e4.header.id));
}

void test_notification_storage__find_by_timestamp(void) {
  Uuid i1;
  uuid_generate(&i1);
  TimelineItem e1 = {
    .header = {
      .id = i1,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  Uuid i2;
  uuid_generate(&i2);
  TimelineItem e2 = {
    .header = {
      .id = i2,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 1,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda6,
    },
    .attr_list = {
      .num_attributes = 2,
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = 1,
      .actions = &actions[2],
    }
  };

  Uuid i3;
  uuid_generate(&i3);
  TimelineItem e3 = {
    .header = {
      .id = i3,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 84,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda7,
    },
    .attr_list = {
      .num_attributes = 1,
      .attributes = &attributes[2],
    },
    .action_group = {
      .num_actions = 2,
      .actions = actions,
    }
  };

  notification_storage_store(&e1);
  notification_storage_store(&e2);
  notification_storage_store(&e3);

  TimelineItem test = e1;
  test.header.id = UUID_INVALID;
  test.header.ancs_uid = 51;

  CommonTimelineItemHeader h;
  cl_assert(notification_storage_find_ancs_notification_by_timestamp(&test, &h));
  cl_assert_equal_m(&e1.header, &h, sizeof(CommonTimelineItemHeader));

  test.action_group.num_actions = 2;
  cl_assert_equal_b(notification_storage_find_ancs_notification_by_timestamp(&test, &h),
                    false);

  test = e2;
  test.header.id = UUID_INVALID;
  cl_assert(notification_storage_find_ancs_notification_by_timestamp(&test, &h));
  cl_assert_equal_m(&e2.header, &h, sizeof(CommonTimelineItemHeader));

  test = e3;
  test.header.id = UUID_INVALID;
  cl_assert(notification_storage_find_ancs_notification_by_timestamp(&test, &h));
  cl_assert_equal_m(&e3.header, &h, sizeof(CommonTimelineItemHeader));

  notification_storage_remove(&i2);
  test = e2;
  test.header.id = UUID_INVALID;
  cl_assert_equal_b(notification_storage_find_ancs_notification_by_timestamp(&test, &h),
                    false);

  test = e3;
  test.header.id = UUID_INVALID;
  cl_assert(notification_storage_find_ancs_notification_by_timestamp(&test, &h));
  cl_assert_equal_m(&e3.header, &h, sizeof(CommonTimelineItemHeader));
}

void test_notification_storage__should_detect_corruption(void) {
  Uuid i1;
  uuid_generate(&i1);
  TimelineItem e1 = {
    .header = {
      .id = i1,
      .type = TimelineItemTypeNotification,
      .status = TimelineItemStatusRead,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  notification_storage_store(&e1);
  TimelineItem r;
  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&r, &e1);

  TimelineItem e2 = e1;
  Uuid i2;
  uuid_generate(&i2);
  e2.header.id = i2;
  e2.header.status = 0xC0;
  notification_storage_store(&e2);
  cl_assert_equal_b(notification_storage_get(&i2, &r), false);

  TimelineItem e3 = e1;
  Uuid i3;
  uuid_generate(&i3);
  e3.header.id = i3;
  e3.header.type = TimelineItemTypeOutOfRange;
  notification_storage_store(&e3);
  cl_assert_equal_b(notification_storage_get(&i3, &r), false);

  Uuid i4;
  uuid_generate(&i4);
  TimelineItem e4 = e1;
  e4.header.id = i4;
  e4.header.layout = NumLayoutIds;
  notification_storage_store(&e4);
  cl_assert_equal_b(notification_storage_get(&i4, &r), false);
}

static void prv_rewrite_bump_timestamp_callback(TimelineItem *notification,
    SerializedTimelineItemHeader *header, void *data) {
  int *count = data;
  (*count)++;
  header->common.timestamp += 42;
  notification->header.timestamp = header->common.timestamp;
}

void test_notification_storage__rewrite_skips_deleted(void) {
  Uuid i1;
  uuid_generate(&i1);
  TimelineItem e1 = {
    .header = {
      .id = i1,
      .type = TimelineItemTypeNotification,
      .status = 0,
      .ancs_uid = 0,
      .layout = LayoutIdGeneric,
      .timestamp = 0x53f0dda5,
    },
    .attr_list = {
      .num_attributes = ARRAY_LENGTH(attributes),
      .attributes = attributes,
    },
    .action_group = {
      .num_actions = ARRAY_LENGTH(actions),
      .actions = actions,
    }
  };

  TimelineItem e2 = e1;
  Uuid i2;
  uuid_generate(&i2);
  e2.header.id = i2;

  TimelineItem e3 = e1;
  Uuid i3;
  uuid_generate(&i3);
  e3.header.id = i3;

  notification_storage_store(&e1);
  notification_storage_store(&e2);
  notification_storage_store(&e3);

  // Delete the middle notification; the rewrite must skip it and still visit e3
  notification_storage_remove(&i2);

  int count = 0;
  notification_storage_rewrite(prv_rewrite_bump_timestamp_callback, &count);
  cl_assert_equal_i(count, 2);

  TimelineItem r;
  e1.header.timestamp += 42;
  cl_assert(notification_storage_get(&i1, &r));
  compare_notifications(&e1, &r);
  free(r.allocated_buffer);

  cl_assert_equal_b(notification_storage_get(&i2, &r), false);

  e3.header.timestamp += 42;
  cl_assert(notification_storage_get(&i3, &r));
  compare_notifications(&e3, &r);
  free(r.allocated_buffer);

  // Storage must still accept and retrieve new notifications after a rewrite
  Uuid i4;
  uuid_generate(&i4);
  TimelineItem e4 = e1;
  e4.header.id = i4;
  notification_storage_store(&e4);
  cl_assert(notification_storage_get(&i4, &r));
  compare_notifications(&e4, &r);
  free(r.allocated_buffer);
}
