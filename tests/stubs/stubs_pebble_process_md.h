/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "process_management/pebble_process_md.h"
#include "util/attributes.h"

Version WEAK process_metadata_get_sdk_version(const PebbleProcessMd *md) {
  return (Version) { PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR, PROCESS_INFO_CURRENT_SDK_VERSION_MINOR };
}

ProcessAppSDKType WEAK process_metadata_get_app_sdk_type(const PebbleProcessMd *md) {
  return 0;
}

int WEAK process_metadata_get_code_bank_num(const PebbleProcessMd *md) {
  return 0;
}
