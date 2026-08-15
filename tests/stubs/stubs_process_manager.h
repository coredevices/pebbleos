/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/platform.h"
#include "process_management/process_manager.h"
#include "stubs_compiled_with_legacy2_sdk.h"
#include "pbl/util/attributes.h"
#include "stubs_compiled_with_legacy2_sdk.h"

const PebbleProcessMd *WEAK sys_process_manager_get_current_process_md(void) {
  return NULL;
}

ProcessAppSDKType WEAK sys_process_manager_get_current_process_sdk_type(void) {
  return ProcessAppSDKType_System;
}

bool WEAK sys_process_manager_get_current_process_uuid(Uuid *uuid_out) {
  return false;
}

bool WEAK process_manager_compiled_with_legacy3_sdk(void) {
  return false;
}

PlatformType WEAK process_manager_current_platform(void) {
  return PBL_PLATFORM_TYPE_CURRENT;
}

void WEAK process_manager_put_kill_process_event(PebbleTask task, bool gracefully) {
  return;
}

const void* WEAK process_manager_get_current_process_args(void) {
  return NULL;
}

bool WEAK process_manager_send_event_to_process(PebbleTask task, PebbleEvent *e) {
  return true;
}

void WEAK process_manager_send_callback_event_to_process(PebbleTask task,
                                                         void (*callback)(void *data),
                                                         void *data) {
  callback(data);
}

