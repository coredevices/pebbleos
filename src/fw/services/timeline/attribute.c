/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/attribute.h"

#include "pbl/services/timeline/attribute_private.h"

#include "system/passert.h"
#include "kernel/pbl_malloc.h"
#include <pbl/logging/logging.h>
#include "pbl/util/math.h"

PBL_LOG_MODULE_DECLARE(service_timeline, CONFIG_SERVICE_TIMELINE_LOG_LEVEL);

#define MAX_LENGTH_TITLE (64)
#define MAX_LENGTH_SUBTITLE (64)
#define MAX_LENGTH_BODY (512)
#define MAX_LENGTH_ANCS_ACTION (1)
#define MAX_LENGTH_CANNED_RESPONSES (512)

static const uint16_t MAX_ATTRIBUTE_LENGTHS[] =
    {MAX_LENGTH_TITLE, MAX_LENGTH_SUBTITLE, MAX_LENGTH_BODY};

typedef enum {
  AttributeTypeUnknown,
  AttributeTypeString,
  AttributeTypeUint8,
  AttributeTypeUint16,
  AttributeTypeUint32,
  AttributeTypeInt8,
  AttributeTypeInt16,
  AttributeTypeInt32,
  AttributeTypeStringList,
  AttributeTypeResourceId,
  AttributeTypeUint32List,
} AttributeType;

static AttributeType prv_attribute_type(AttributeId id) {
  switch (id) {
    case AttributeIdTitle:
    case AttributeIdSubtitle:
    case AttributeIdBody:
    case AttributeIdShortTitle:
    case AttributeIdShortSubtitle:
    case AttributeIdLocationName:
    case AttributeIdSender:
    case AttributeIdRankAway:
    case AttributeIdRankHome:
    case AttributeIdNameAway:
    case AttributeIdNameHome:
    case AttributeIdRecordAway:
    case AttributeIdRecordHome:
    case AttributeIdScoreAway:
    case AttributeIdScoreHome:
    case AttributeIdBroadcaster:
    case AttributeIdAppName:
    case AttributeIdiOSAppIdentifier:
    case AttributeIdAddress:
    case AttributeIdAuthCode:
    case AttributeIdSubtitleTemplateString:
      return AttributeTypeString;
    case AttributeIdAncsAction:
    case AttributeIdSportsGameState:
    case AttributeIdPrimaryColor:
    case AttributeIdSecondaryColor:
    case AttributeIdBgColor:
    case AttributeIdDisplayRecurring:
    case AttributeIdEmojiSupported:
    case AttributeIdHealthInsightType:
    case AttributeIdDisplayTime:
    case AttributeIdMuteDayOfWeek:
    case AttributeIdHealthActivityType:
    case AttributeIdAlarmKind:
    case AttributeIdImageAspectRatio:
      return AttributeTypeUint8;
    case AttributeIdIconTiny:
    case AttributeIdIconSmall:
    case AttributeIdIconLarge:
    case AttributeIdIconPin:
    case AttributeIdIcon:
      return AttributeTypeResourceId;
    case AttributeIdLastUpdated:
    case AttributeIdLaunchCode:
    case AttributeIdAncsId:
    case AttributeIdTimestamp:
    case AttributeIdMuteExpiration:
      return AttributeTypeUint32;
    case AttributeIdCannedResponses:
    case AttributeIdHeadings:
    case AttributeIdParagraphs:
    case AttributeIdMetricNames:
    case AttributeIdMetricValues:
    case AttributeIdNotificationFilteringRules:
      return AttributeTypeStringList;
    case AttributeIdMetricIcons:
    case AttributeIdVibrationPattern:
      return AttributeTypeUint32List;
    default:
      return AttributeTypeUnknown;
  }
}

static bool prv_deserialize_attribute(char **buffer, char *const buf_end, const uint8_t **cursor,
                                      const uint8_t *payload_end, Attribute *notif_attr) {
  SerializedAttributeHeader *attribute = (SerializedAttributeHeader *)*cursor;

  *cursor += sizeof(SerializedAttributeHeader);
  if ((*cursor + attribute->length) > payload_end) {
    return false;
  }

  notif_attr->id = attribute->id;
  switch (prv_attribute_type(attribute->id)) {
    case AttributeTypeString: {
      notif_attr->cstring = *buffer;
      uint16_t attr_length;
      if (attribute->id <= AttributeIdBody) {
        // Valid attribute IDs start at 1 but MAX_ATTRIBUTE_LENGTHS indexes starts at 0, so -1
        attr_length = MIN(MAX_ATTRIBUTE_LENGTHS[attribute->id - 1], attribute->length);
      } else if (attribute->id == AttributeIdSubtitleTemplateString) {
        attr_length = MIN((uint16_t)ATTRIBUTE_APP_GLANCE_SUBTITLE_MAX_LEN, attribute->length);
      } else {
        attr_length = attribute->length;
      }
      *buffer += attr_length + 1;
      PBL_ASSERTN(*buffer <= buf_end);
      memcpy(notif_attr->cstring, *cursor, attr_length);
      notif_attr->cstring[attr_length] = '\0';
      break;
    }
    case AttributeTypeUint8:
      if (attribute->length != sizeof(uint8_t)) {
        return false;
      }
      notif_attr->uint8 = **cursor;
      break;
    case AttributeTypeUint32:
      if (attribute->length != sizeof(uint32_t)) {
        return false;
      }
      notif_attr->uint32 = *(uint32_t*)*cursor;
      break;
    case AttributeTypeResourceId:
      if (attribute->length != sizeof(uint32_t)) {
        return false;
      }
      notif_attr->uint32 = *(uint32_t*)*cursor;
      break;
    case AttributeTypeStringList: {
      notif_attr->string_list = (StringList *)*buffer;
      notif_attr->string_list->serialized_byte_length = MIN((uint16_t)MAX_LENGTH_CANNED_RESPONSES,
          attribute->length);
      uint16_t data_length = notif_attr->string_list->serialized_byte_length +
          (uint16_t)sizeof(char); // terminator after last string
      *buffer += sizeof(StringList) + data_length;
      PBL_ASSERTN(*buffer <= buf_end);
      memcpy(notif_attr->string_list->data, *cursor, data_length - 1);
      notif_attr->string_list->data[data_length - 1] = '\0';
      break;
    }
    case AttributeTypeUint32List: {
      notif_attr->uint32_list = (Uint32List *)*buffer;
      *buffer += attribute->length;
      PBL_ASSERTN(*buffer <= buf_end);
      memcpy(notif_attr->uint32_list, *cursor, attribute->length);
      break;
    }

    default:
      return false;
      break;
  }
  *cursor += attribute->length;
  return true;
}

static int32_t prv_get_buffer_size_for_serialized_attribute(const uint8_t **cursor,
    const uint8_t *end) {
  SerializedAttributeHeader *attribute = (SerializedAttributeHeader *)*cursor;
  *cursor += sizeof(SerializedAttributeHeader);
  if ((*cursor + attribute->length) > end) {
    return -1;
  }
  int32_t string_alloc_size = 0;
  switch (prv_attribute_type(attribute->id)) {
    case AttributeTypeString:
      if (attribute->id <= AttributeIdBody) {
      string_alloc_size +=
          MIN(MAX_ATTRIBUTE_LENGTHS[attribute->id - 1], attribute->length) + 1;
      } else if (attribute->id == AttributeIdSubtitleTemplateString) {
        string_alloc_size += MIN(ATTRIBUTE_APP_GLANCE_SUBTITLE_MAX_LEN, attribute->length) + 1;
      } else {
        string_alloc_size += attribute->length + 1;
      }
      break;
    case AttributeTypeStringList:
      string_alloc_size += sizeof(StringList)
          + MIN(MAX_LENGTH_CANNED_RESPONSES, attribute->length) + 1;
      break;
    case AttributeTypeUint32List:
      string_alloc_size += attribute->length;
      break;
    default:
      // Attribute will be stored in place, not in string buffer
      break;
  }
  *cursor += attribute->length;
  return string_alloc_size;
}

Attribute *prv_add_attribute(AttributeList *list, AttributeId id) {
  Attribute *attribute_found = attribute_find(list, id);
  if (attribute_found) {
    return attribute_found;
  }

  uint8_t attribute_idx = list->num_attributes;
  list->num_attributes++;
  list->attributes = kernel_realloc(list->attributes, list->num_attributes * sizeof(Attribute));
  list->attributes[attribute_idx].id = id;
  return &list->attributes[attribute_idx];
}

int32_t attribute_get_buffer_size_for_serialized_attributes(uint8_t num_attributes,
    const uint8_t **cursor, const uint8_t *end) {
  int32_t size = 0;
  for (unsigned int i = 0; i < num_attributes; i++) {
    int32_t result = prv_get_buffer_size_for_serialized_attribute(cursor, end);
    if (result < 0) {
      return result;
    }
    size += result;
  }
  return size;
}

size_t attribute_list_get_serialized_size(const AttributeList *attr_list) {
  if (!attr_list) {
    return 0;
  }

  size_t size = 0;
  size += (attr_list->num_attributes * sizeof(SerializedAttributeHeader));
  for (int i = 0; i < attr_list->num_attributes; i++) {
    switch (prv_attribute_type(attr_list->attributes[i].id)) {
        case AttributeTypeString:
          size += strlen(attr_list->attributes[i].cstring);
          break;
        case AttributeTypeResourceId:
        case AttributeTypeUint32:
          size += sizeof(attr_list->attributes[i].uint32);
          break;
        case AttributeTypeUint8:
          size += sizeof(attr_list->attributes[i].uint8);
          break;
        case AttributeTypeStringList:
          size += attr_list->attributes[i].string_list->serialized_byte_length;
          break;
        case AttributeTypeUint32List:
          size += Uint32ListSize(attr_list->attributes[i].uint32_list->num_values);
          break;
        default:
          break;
    }
  }
  return size;
}

size_t attribute_list_serialize(const AttributeList *attr_list, uint8_t *buffer, uint8_t *buf_end) {

  PBL_ASSERTN(attr_list != NULL);
  PBL_ASSERTN(buffer != NULL);
  PBL_ASSERTN(buf_end != NULL);

  uint8_t *buf_start = buffer;

  for (int i = 0; i < attr_list->num_attributes; i++) {
    SerializedAttributeHeader *attribute = (SerializedAttributeHeader *)buffer;
    buffer += sizeof(SerializedAttributeHeader);
    PBL_ASSERTN(buffer <= buf_end);
    attribute->id = attr_list->attributes[i].id;
    switch (prv_attribute_type(attr_list->attributes[i].id)) {
        case AttributeTypeString:
          attribute->length = strlen(attr_list->attributes[i].cstring);
          memcpy(buffer, attr_list->attributes[i].cstring, attribute->length);
          break;
        case AttributeTypeUint32:
        case AttributeTypeResourceId:
          attribute->length = sizeof(uint32_t);
          memcpy(buffer, &attr_list->attributes[i].uint32, attribute->length);
          break;
        case AttributeTypeUint8:
          attribute->length = sizeof(uint8_t);
          memcpy(buffer, &attr_list->attributes[i].uint8, attribute->length);
          break;

        case AttributeTypeStringList:
          attribute->length = attr_list->attributes[i].string_list->serialized_byte_length;
          memcpy(buffer, attr_list->attributes[i].string_list->data, attribute->length);
          break;
        case AttributeTypeUint32List:
          attribute->length = Uint32ListSize(attr_list->attributes[i].uint32_list->num_values);
          memcpy(buffer, attr_list->attributes[i].uint32_list, attribute->length);
          break;
        default:
          attribute->length = 0;
          break;
    }
    buffer += attribute->length;
    PBL_ASSERTN(buffer <= buf_end);
  }
  return buffer - buf_start;
}

bool attribute_deserialize_list(char **buffer, char *const buf_end,
    const uint8_t **cursor, const uint8_t *payload_end, AttributeList attr_list) {

  for (int i = 0; i < attr_list.num_attributes; i++) {
    if (!prv_deserialize_attribute(buffer, buf_end, cursor, payload_end,
                                   &attr_list.attributes[i])) {
      PBL_LOG_WRN("Encountered unknown attribute");
      break;
    }
  }
  return true;
}

static size_t prv_get_attribute_length(const Attribute *attr) {
  switch (prv_attribute_type(attr->id)) {
    case AttributeTypeString:
      return strlen(attr->cstring) + 1; // +1 for null char
    case AttributeTypeStringList:
      return sizeof(StringList) + attr->string_list->serialized_byte_length + 1; // +1 for null char
    case AttributeTypeUint32List:
      return Uint32ListSize(attr->uint32_list->num_values);
    default:
      // The rest of the types fit within the Attribute struct
      return 0;
  }
}

static bool prv_deep_copy_attribute(Attribute *dest, const Attribute *src, uint8_t **buffer,
                                    uint8_t *const buffer_end) {
  const size_t attribute_length = prv_get_attribute_length(src);
  if (*buffer + attribute_length > buffer_end) {
    return false;
  }

  switch (prv_attribute_type(src->id)) {
    case AttributeTypeString: {
      dest->cstring = (char *)*buffer;
      strncpy(dest->cstring, src->cstring, attribute_length);
      break;
    }
    case AttributeTypeStringList: {
      dest->string_list = (StringList *)*buffer;
      dest->string_list->serialized_byte_length = src->string_list->serialized_byte_length;
      memcpy(dest->string_list->data, src->string_list->data,
             src->string_list->serialized_byte_length);
      dest->string_list->data[src->string_list->serialized_byte_length] = '\0';
      break;
    }
    case AttributeTypeUint32List: {
      dest->uint32_list = (Uint32List *)*buffer;
      memcpy(dest->uint32_list, src->uint32_list, attribute_length);
      break;
    }
    default: {
      // don't need to deep copy non-strings
      break;
    }
  }

  *buffer += attribute_length;
  return true;
}

bool attribute_copy(Attribute *dest, const Attribute *src, uint8_t **buffer,
                    uint8_t *const buffer_end) {
  // shallow copy the attribute
  memcpy(dest, src, sizeof(Attribute));

  // deep copy strings into the buffer
  return prv_deep_copy_attribute(dest, src, buffer, buffer_end);
}

bool attribute_list_copy(AttributeList *out, const AttributeList *in, uint8_t *buffer,
                         uint8_t *const buffer_end) {
  out->num_attributes = in->num_attributes;
  uint8_t *write_ptr = buffer;
  out->attributes = (Attribute *)write_ptr;
  // shallow copy the attributes
  for (int i = 0; i < in->num_attributes; i++) {
    if (write_ptr + sizeof(Attribute) > buffer_end) {
      return false;
    }
    memcpy(&out->attributes[i], &in->attributes[i], sizeof(Attribute));
    write_ptr += sizeof(Attribute);
  }

  for (int i = 0; i < in->num_attributes; i++) {
    bool r = prv_deep_copy_attribute(&out->attributes[i], &in->attributes[i],
                                     &write_ptr, buffer_end);
    if (!r) {
      return false;
    }
  }

  return true;
}

size_t attribute_list_get_buffer_size(const AttributeList *list) {
  return (sizeof(Attribute) * list->num_attributes) + attribute_list_get_string_buffer_size(list);
}

size_t attribute_list_get_string_buffer_size(const AttributeList *list) {
  size_t size = 0;
  for (int i = 0; i < list->num_attributes; i++) {
    size += prv_get_attribute_length(&list->attributes[i]);
  }
  return size;
}

void attribute_list_add_cstring(AttributeList *list, AttributeId id, const char *cstring) {
  if (prv_attribute_type(id) != AttributeTypeString) {
    PBL_LOG_WRN("Adding attribute with type cstring for non-cstring attribute");
  }
  prv_add_attribute(list, id)->cstring = (char*) cstring;
}

void attribute_list_add_uint32(AttributeList *list, AttributeId id, uint32_t uint32) {
  if (prv_attribute_type(id) != AttributeTypeUint32) {
    PBL_LOG_WRN("Adding attribute with type uint32 for non-uint32_t attribute");
  }
  prv_add_attribute(list, id)->uint32 = uint32;
}

void attribute_list_add_resource_id(AttributeList *list, AttributeId id,
                                    uint32_t resource_id) {
  if (prv_attribute_type(id) != AttributeTypeResourceId) {
    PBL_LOG_WRN("Adding attribute with type ResourceId for non-ResourceId " \
            "attribute");
  }
  prv_add_attribute(list, id)->uint32 = resource_id;
}

void attribute_list_add_uint8(AttributeList *list, AttributeId id, uint8_t uint8) {
  if (prv_attribute_type(id) != AttributeTypeUint8) {
    PBL_LOG_WRN("Adding attribute with type uint8 for non-uint8_t attribute");
  }
  prv_add_attribute(list, id)->uint8 = uint8;
}

void attribute_list_add_string_list(AttributeList *list, AttributeId id, StringList *string_list) {
  PBL_ASSERTN(prv_attribute_type(id) == AttributeTypeStringList);
  prv_add_attribute(list, id)->string_list = string_list;
}

void attribute_list_add_uint32_list(AttributeList *list, AttributeId id, Uint32List *uint32_list) {
  PBL_ASSERTN(prv_attribute_type(id) == AttributeTypeUint32List);
  prv_add_attribute(list, id)->uint32_list = uint32_list;
}

void attribute_list_add_attribute(AttributeList *list, const Attribute *new_attribute) {
  Attribute *attribute = prv_add_attribute(list, new_attribute->id);
  *attribute = *new_attribute;
}

void attribute_list_init_list(uint8_t num_attributes, AttributeList *list_out) {
  *list_out = (AttributeList) {
    .num_attributes = num_attributes,
    .attributes = kernel_zalloc_check(num_attributes * sizeof(Attribute))
  };
}

void attribute_list_destroy_list(AttributeList *list) {
  PBL_ASSERTN(list != NULL);
  kernel_free(list->attributes);
}

bool attribute_check_serialized_list(const uint8_t *cursor, const uint8_t *val_end,
    uint8_t num_attributes, bool has_attribute[]) {
  for (int i = 0; i < num_attributes; i++) {
    SerializedAttributeHeader *attrib_hdr = (SerializedAttributeHeader *)cursor;
    cursor += sizeof(SerializedAttributeHeader);
    switch (prv_attribute_type(attrib_hdr->id)) {
      case AttributeTypeString:
        // variable-length
        break;
      case AttributeTypeUint8:
        if (attrib_hdr->length != sizeof(uint8_t)) {
          return false;
        }
        break;
      case AttributeTypeResourceId:
      case AttributeTypeUint32:
        if (attrib_hdr->length != sizeof(uint32_t)) {
          return false;
        }
        break;
      case AttributeTypeStringList:
        break;
      default:
        break;
    }
    cursor += attrib_hdr->length;
    if (cursor > val_end) {
      return false;
    } else if (attrib_hdr->id < NumAttributeIds) {
      // has_attribute is indexed by id, so a phone that knows an id this firmware doesn't must not
      // reach the write.
      has_attribute[attrib_hdr->id] = true;
    }
  }
  return true;
}

void attribute_init_string(Attribute *attribute, char *buffer, AttributeId attribute_id) {
  PBL_ASSERTN(attribute != NULL);
  PBL_ASSERTN(buffer != NULL);

  attribute->cstring = buffer;
  attribute->id = attribute_id;
}

Attribute *attribute_find(const AttributeList *attr_list, AttributeId id) {
  if (!attr_list) {
    return NULL;
  }
  if (id == AttributeIdUnused) {
    return NULL;
  }

  for (int i = 0; i < attr_list->num_attributes; i++) {
    if (attr_list->attributes[i].id == id) {
      return &attr_list->attributes[i];
    }
  }
  return NULL;
}

const char *attribute_get_string(const AttributeList *attr_list, AttributeId id,
                                 char *default_value) {
  PBL_ASSERTN(attr_list != NULL);
  if (id == AttributeIdUnused) {
    return default_value;
  }

  PBL_ASSERTN(prv_attribute_type(id) == AttributeTypeString);
  Attribute *attribute = attribute_find(attr_list, id);
  return attribute ? attribute->cstring : default_value;
}

StringList *attribute_get_string_list(
    const AttributeList *attr_list, AttributeId id) {
  PBL_ASSERTN(attr_list != NULL);

  Attribute *attribute = attribute_find(attr_list, id);
  return attribute ? attribute->string_list : NULL;
}

uint8_t attribute_get_uint8(const AttributeList *attr_list,
    AttributeId id, uint8_t default_value) {

  PBL_ASSERTN(attr_list != NULL);

//  HB TODO: test the type of id!
  Attribute *attribute = attribute_find(attr_list, id);
  return attribute ? attribute->uint8 : default_value;
}

uint32_t attribute_get_uint32(const AttributeList *attr_list,
    AttributeId id, uint32_t default_value) {

  PBL_ASSERTN(attr_list != NULL);

//  HB TODO: test the type of id!
  Attribute *attribute = attribute_find(attr_list, id);
  return attribute ? attribute->uint32 : default_value;
}

Uint32List *attribute_get_uint32_list(const AttributeList *attr_list, AttributeId id) {
  PBL_ASSERTN(attr_list != NULL);
  PBL_ASSERTN(prv_attribute_type(id) == AttributeTypeUint32List);
  Attribute *attribute = attribute_find(attr_list, id);
  return attribute ? attribute->uint32_list : NULL;
}
