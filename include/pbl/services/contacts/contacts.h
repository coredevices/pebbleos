/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "attributes_address.h"

#include "pbl/util/uuid.h"

typedef struct {
  Uuid id;
  uint32_t flags;
  AttributeList attr_list;
  AddressList addr_list;
} Contact;

//! Lookup a contact given its uuid. Will return NULL if no contact is found.
//! The contact must be freed with contacts_free_contact().
Contact* contacts_get_contact_by_uuid(const Uuid *uuid);

//! Frees a contact
void contacts_free_contact(Contact *contact);
