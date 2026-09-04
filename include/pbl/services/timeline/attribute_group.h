/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/timeline/item.h"

#include <stdint.h>
#include <stdbool.h>

//! You probably don't want to be using the APIs in this file unless you are doing
//! something similar to attributes_actions.h and attributes_addresses.h

//! This file consolidates some code that deals with data which looks like:
//! AttributeList
//! ActionGroup / AddressList (referred to as "group data")

typedef enum {
  AttributeGroupType_Action,
  AttributeGroupType_Address,
} AttributeGroupType;


//! Documentation for these functions and their params can be found in the wrapping files

bool attribute_group_parse_serial_data(AttributeGroupType type,
                                       uint8_t num_attributes,
                                       uint8_t num_group_type_elements,
                                       const uint8_t *data,
                                       size_t size,
                                       size_t *string_alloc_size_out,
                                       uint8_t *attributes_per_group_type_element_out);

size_t attribute_group_get_required_buffer_size(AttributeGroupType type,
                                                uint8_t num_attributes,
                                                uint8_t num_group_type_elements,
                                                const uint8_t *attributes_per_group_type_element,
                                                size_t required_size_for_strings);

void attribute_group_init(AttributeGroupType type,
                          AttributeList *attr_list,
                          void *group_ptr,
                          uint8_t **buffer,
                          uint8_t num_attributes,
                          uint8_t num_group_type_elements,
                          const uint8_t *attributes_per_group_type_element);

bool attribute_group_deserialize(AttributeGroupType type,
                                 AttributeList *attr_list,
                                 void *group_ptr,
                                 uint8_t *buffer,
                                 uint8_t *buf_end,
                                 const uint8_t *payload,
                                 size_t payload_size);

size_t attribute_group_get_serialized_payload_size(AttributeGroupType type,
                                                   AttributeList *attr_list,
                                                   void *group_ptr);

size_t attribute_group_serialize_payload(AttributeGroupType type,
                                         AttributeList *attr_list,
                                         void *group_ptr,
                                         uint8_t *buffer,
                                         size_t buffer_size);
