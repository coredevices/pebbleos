/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/applib_malloc.auto.h"
#include "applib_resource_private.h"
#include "board/board.h"
#include "process_state/app_state/app_state.h"
#include "resource/resource_storage_builtin.h"
#include "resource/resource_storage_flash.h"
#include "syscall/syscall.h"

ResHandle applib_resource_get_handle(uint32_t resource_id) {
  if (sys_resource_is_valid(sys_get_current_resource_num(), resource_id)) {
    return (ResHandle)resource_id;
  }

  return 0;
}

size_t applib_resource_size(ResHandle h) {
  return sys_resource_size(sys_get_current_resource_num(), (uint32_t)h);
}

size_t applib_resource_load(ResHandle h, uint8_t *buffer, size_t max_length) {
  return sys_resource_load_range(sys_get_current_resource_num(),
                                 (uint32_t)h, 0, buffer, max_length);
}

size_t applib_resource_load_byte_range(
    ResHandle h, uint32_t start_offset, uint8_t *buffer, size_t num_bytes) {
  return sys_resource_load_range(sys_get_current_resource_num(),
                                 (uint32_t)h, start_offset, buffer, num_bytes);
}

void *applib_resource_mmap_or_load(ResAppNum app_num, uint32_t resource_id,
                                   size_t offset, size_t num_bytes, bool used_aligned) {
  if (num_bytes == 0) {
    return NULL;
  }

  const uint8_t *mapped_data = (app_num == SYSTEM_APP) ?
                               sys_resource_read_only_bytes(SYSTEM_APP, resource_id, NULL) : NULL;

  uint8_t *result = NULL;

  if (mapped_data) {
    applib_resource_track_mmapped(mapped_data);
    result = (uint8_t *)(mapped_data + offset);
  } else {
    // TODO: PBL-40010 clean this up
    // we are wasting 7 bytes here so that clients of this API have the chance to
    // align the data
    result = applib_malloc(num_bytes + (used_aligned ? 7 : 0));
    if (!result || sys_resource_load_range(app_num, resource_id, offset,
                                           result, num_bytes) != num_bytes) {
      applib_free(result);
      return NULL;
    }
  }

  return result;
}

void applib_resource_munmap_or_free(void *bytes) {
  if (!applib_resource_munmap(bytes)) {
    applib_free(bytes);
  }
}


// Mmapped resources are direct pointers into never-freed flash: nothing to
// refcount or release, callers must just not free() them.
static bool prv_resource_is_mmapped(const void *bytes) {
  return resource_storage_builtin_bytes_are_readonly(bytes) ||
         resource_storage_flash_bytes_are_readonly(bytes);
}

bool applib_resource_track_mmapped(const void *bytes) {
  return prv_resource_is_mmapped(bytes);
}

bool applib_resource_is_mmapped(const void *bytes) {
  return prv_resource_is_mmapped(bytes);
}

bool applib_resource_munmap(const void *bytes) {
  return prv_resource_is_mmapped(bytes);
}