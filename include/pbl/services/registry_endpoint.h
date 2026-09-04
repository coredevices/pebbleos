/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

typedef enum {
  RegistryEndpointIdSystem = 5000,
  RegistryEndpointIdFactory = 5001,
} RegistryEndpointId;

void registry_endpoint_callback(CommSession *session, const uint8_t* data, unsigned int length_bytes);

void factory_registry_endpoint_callback(CommSession *session, const uint8_t* data, unsigned int length_bytes);
