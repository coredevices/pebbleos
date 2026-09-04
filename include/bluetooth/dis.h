/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/attributes.h"

// The reason the headers that define these lengths aren't included is because this header
// is included by the various number of bt_driver implementations. They don't know what "mfg"
// is, etc.
// NOTE: These sizes are asserted in a .c file to be in sync with the FW
#define MODEL_NUMBER_LEN  (10) // MFG_HW_VERSION_SIZE + 1
#define MANUFACTURER_LEN  (18) // sizeof("Pebble Technology")
#define SERIAL_NUMBER_LEN (13) // MFG_SERIAL_NUMBER_SIZE + 1
#define FW_REVISION_LEN   (32) // FW_METADATA_VERSION_TAG_BYTES)
#define SW_REVISION_LEN   (8)  // Fmt: xx.xxx\0

typedef struct PACKED DisInfo {
  char model_number[MODEL_NUMBER_LEN];
  char manufacturer[MANUFACTURER_LEN];
  char serial_number[SERIAL_NUMBER_LEN];
  char fw_revision[FW_REVISION_LEN];
  char sw_revision[SW_REVISION_LEN];
} DisInfo;
