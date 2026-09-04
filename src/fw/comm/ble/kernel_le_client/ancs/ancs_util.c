/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "ancs_util.h"
#include "ancs_types.h"


#include "syscall/syscall.h"

#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"

#include <stdint.h>


bool ancs_util_is_complete_notif_attr_response(const uint8_t* data, const size_t length, bool* out_error) {
  PBL_ASSERTN(out_error);
  const size_t header_len = sizeof(GetNotificationAttributesMsg);
  if (length > header_len) {
    ANCSAttribute *attributes[NUM_FETCHED_NOTIF_ATTRIBUTES] = {};
    bool complete = ancs_util_get_attr_ptrs(data + header_len, length - header_len,
                                            s_fetched_notif_attributes,
                                            ARRAY_LENGTH(s_fetched_notif_attributes),
                                            attributes, out_error);

    return complete;
  }
  return false;
}

bool ancs_util_is_complete_app_attr_dict(const uint8_t* data, size_t length, bool* out_error) {
  PBL_ASSERTN(out_error);

  // Search for end of the App ID before checking that all attributes are present.
  while (length > 0) {
    length--;
    if (*data++ == 0) {
      break;
    }
  }
  if (length == 0) {
    *out_error = false;
    return false;
  }
  return ancs_util_get_attr_ptrs(data, length,
                                 s_fetched_app_attributes,
                                 ARRAY_LENGTH(s_fetched_app_attributes),
                                 NULL, out_error);
}

bool ancs_util_get_attr_ptrs(const uint8_t* data, const size_t length, const FetchedAttribute* attr_list,
    const int num_attrs, ANCSAttribute *out_attr_ptrs[], bool* out_error) {

  PBL_ASSERTN(out_error);
  *out_error = false;

  const uint8_t* iter = data;
  if (length < sizeof(ANCSAttribute)) {
    PBL_LOG_ERR("ANCS data length is too small. Length: %d, sizeof(ANCSAttribute): %d",
                (int)length, (int)sizeof(ANCSAttribute));
    *out_error = true;
    return false;
  }

  bool attrs_found[num_attrs];
  memset(attrs_found, 0, sizeof(attrs_found));

  bool extracted_complete_attribute = false;
  // Iterate over the contents of the buffer
  while ((iter + sizeof(ANCSAttribute)) <= (data + length)) {
    ANCSAttribute* attr = (ANCSAttribute*) iter;
    const uint8_t* next_iter = (uint8_t*) attr->value + attr->length;

    // Match this attribute with its entry in the FetchedNotifAttribute list
    bool is_found = false;
    for (int i = 0; i < num_attrs; ++i) {
      is_found = (attr->id == attr_list[i].id);
      if (is_found) {
        // Check that attribute length is valid
        bool attr_length_invalid = (attr_list[i].max_length != 0) && (attr->length > attr_list[i].max_length);
        if (attr_length_invalid) {
          PBL_LOG_ERR("Length of ANCS attribute %d is invalid: length: %d, max_length: %d",
                      attr->id, attr->length, attr_list[i].max_length);
          *out_error = true;
          return false;
        }
        attrs_found[i] = true;
        if (out_attr_ptrs) {
          out_attr_ptrs[i] = (ANCSAttribute *) attr;
        }

        break;
      }
    }

    if (!is_found) {
      // The attribute was unexpected, the dictionary is malformed
      PBL_LOG_ERR("Unexpected ANCS attribute. ID = %d. The dictionary is malformed", attr->id);
      *out_error = true;
      return false;
    }

    extracted_complete_attribute = ((uint8_t*)attr->value + attr->length <= data + length);
    iter = next_iter;
  }

  // The dictionary was well-formed, all the attributes found so far are ones
  // that were in the FetchedNotifAttribute list
  // Check if there are any outstanding attributes that have not been found
  for (int i = 0; i < num_attrs; ++i) {

    const bool optional = (attr_list[i].flags & FetchedAttributeFlagOptional);
    if (optional) {
      continue;
    }

    if (!attrs_found[i]) {
      return false;
    }
  }

  return extracted_complete_attribute;
}
