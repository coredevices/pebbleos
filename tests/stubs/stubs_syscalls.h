/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "syscall/syscall.h"
#include "pbl/util/attributes.h"

struct tm *WEAK sys_localtime_r(const time_t *timep, struct tm *result) {
  return localtime_r(timep, result);
}

ResAppNum WEAK sys_get_current_resource_num(void) {
  return 0;
}

size_t WEAK sys_resource_size(ResAppNum app_num, uint32_t handle) {
  return resource_size(app_num, handle);
}

size_t WEAK sys_resource_load_range(ResAppNum app_num, uint32_t id, uint32_t start_bytes,
                                    uint8_t *buffer, size_t num_bytes) {
  return resource_load_byte_range_system(app_num, id, start_bytes, buffer, num_bytes);
}

bool WEAK sys_resource_bytes_are_readonly(void *bytes) {
  return false;
}

const uint8_t *WEAK sys_resource_read_only_bytes(ResAppNum app_num, uint32_t resource_id,
                                                 size_t *num_bytes_out) {
  return NULL;
}

uint32_t WEAK sys_resource_get_and_cache(ResAppNum app_num, uint32_t resource_id) {
  return resource_id;
}

bool WEAK sys_resource_is_valid(ResAppNum app_num, uint32_t resource_id) {
  return resource_is_valid(app_num, resource_id);
}

void WEAK sys_font_reload_font(FontInfo *fontinfo) {
  return;
}

GFont WEAK sys_font_get_system_font(const char *key) {
  return NULL;
}

bool WEAK sys_accel_manager_data_unsubscribe(AccelManagerState *state) {
  return false;
}

AppInstallId WEAK sys_process_manager_get_current_process_id(void) {
  return (AppInstallId)(-1);
}

void WEAK sys_get_app_uuid(Uuid *uuid) {}

bool WEAK action_bar_layer_is_on_right(void) {
  return true;
}
