/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/process_management/app_storage.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <pbl/drivers/flash.h>
#include "process_management/pebble_process_info.h"
#include "resource/resource_storage.h"
#include "pbl/services/filesystem/pfs.h"
#include "pbl/services/filesystem/app_file.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/build_id.h"

PBL_LOG_MODULE_DECLARE(service_process_management, CONFIG_SERVICE_PROCESS_MANAGEMENT_LOG_LEVEL);

// 64k. Note that both tintin and snowy apps have a maximum size of 64k enforced by the SDK, even
// though there isn't enough memory for load more than 24k in practice on tintin.
static const uint32_t APP_MAX_SIZE = 0x10000;

bool app_storage_get_process_load_size(const PebbleProcessInfo *info,
                                       size_t *load_size_out) {
  if (info->num_reloc_entries > (SIZE_MAX / sizeof(uint32_t))) {
    PBL_LOG_WRN("App relocation table size overflows: entries=%"PRIu32,
                info->num_reloc_entries);
    return false;
  }

  const size_t reloc_size = info->num_reloc_entries * sizeof(uint32_t);
  if (info->load_size > (SIZE_MAX - reloc_size)) {
    PBL_LOG_WRN("App load size overflows: load=%"PRIu16" reloc=%zu",
                info->load_size, reloc_size);
    return false;
  }

  *load_size_out = info->load_size + reloc_size;
  return true;
}

AppStorageGetAppInfoResult app_storage_get_process_info(PebbleProcessInfo* app_info,
  uint8_t *build_id_out, AppInstallId app_id, PebbleTask task_type) {

  char process_name[APP_FILENAME_MAX_LENGTH];
  app_storage_get_file_name(process_name, sizeof(process_name), app_id, task_type);
  int fd;
  if ((fd = pfs_open(process_name, OP_FLAG_READ, 0, 0)) < S_SUCCESS) {
    return (GET_APP_INFO_COULD_NOT_READ_FORMAT);
  }
  if (pfs_read(fd, (uint8_t *)app_info, sizeof(PebbleProcessInfo)) != sizeof(PebbleProcessInfo)) {
    pfs_close(fd);
    return (GET_APP_INFO_COULD_NOT_READ_FORMAT);
  }
  if (build_id_out) {
    const uint8_t padding_size = sizeof(PebbleProcessInfo) % 4;
    // The note.gnu.build-id section seems to have a hard-coded word-alignment requirement...
    uint8_t note_buffer[BUILD_ID_TOTAL_EXPECTED_LEN + padding_size];
    const ElfExternalNote *note = (const ElfExternalNote *) (note_buffer + padding_size);
    int result = pfs_read(fd, note_buffer, sizeof(note_buffer));
    if ((result == (int) sizeof(note_buffer)) &&
        build_id_contains_gnu_build_id(note)) {
      memcpy(build_id_out, note->data + note->name_length, BUILD_ID_EXPECTED_LEN);
    } else {
      memset(build_id_out, 0, BUILD_ID_EXPECTED_LEN);
    }
  }
  pfs_close(fd);

  if (strncmp("PBLAPP", app_info->header, sizeof(app_info->header)) != 0) {
    // there isn't a valid app in the bank
    return GET_APP_INFO_COULD_NOT_READ_FORMAT;
  }

  const bool is_sdk_compatible =
      (app_info->sdk_version.major == PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR &&
       app_info->sdk_version.minor <= PROCESS_INFO_CURRENT_SDK_VERSION_MINOR);

  if (is_sdk_compatible == false) {
    PBL_LOG_WRN("App requires support for SDK version (%u.%u), we only support version (%u.%u).",
            app_info->sdk_version.major, app_info->sdk_version.minor,
            PROCESS_INFO_CURRENT_SDK_VERSION_MAJOR, PROCESS_INFO_CURRENT_SDK_VERSION_MINOR);

    // The app's is built with an SDK that is incompatible with the running fw
    return GET_APP_INFO_INCOMPATIBLE_SDK;
  }

  if (app_info->virtual_size > APP_MAX_SIZE) {
    PBL_LOG_WRN("App size (%u) larger than bank size; invalid app.", app_info->virtual_size);
    // The app's metadata indicates an app larger than the maximum bank size
    return GET_APP_INFO_APP_TOO_LARGE;
  }

  return GET_APP_INFO_SUCCESS;
}

void app_storage_delete_app(AppInstallId id) {
  PBL_ASSERTN(id > 0);
  char process_name[APP_FILENAME_MAX_LENGTH];

  // remove worker
  app_storage_get_file_name(process_name, sizeof(process_name), id, PebbleTask_Worker);
  pfs_remove(process_name);
  // remove app too
  app_storage_get_file_name(process_name, sizeof(process_name), id, PebbleTask_App);
  pfs_remove(process_name);
  // remove resources
  resource_storage_clear(id);
}

bool app_storage_app_exists(AppInstallId id) {
  PBL_ASSERTN(id > 0);
  char process_name[APP_FILENAME_MAX_LENGTH];

  // check app binary first
  app_storage_get_file_name(process_name, sizeof(process_name), id, PebbleTask_App);
  int fd = pfs_open(process_name, OP_FLAG_READ, 0, 0);
  if (fd < 0) {
    return false;
  }
  pfs_close(fd);

  // now check resource bank
  return resource_storage_check((ResAppNum)id, 0, NULL);
}

void app_storage_get_file_name(char *name, size_t buf_length, AppInstallId app_id,
                               PebbleTask task) {
  const char *task_str = (task == PebbleTask_App) ? APP_FILE_NAME_SUFFIX
                                                  : WORKER_FILE_NAME_SUFFIX;
  size_t task_str_len =
      (task == PebbleTask_App) ? strlen(APP_FILE_NAME_SUFFIX)
                               : strlen(WORKER_FILE_NAME_SUFFIX);
  app_file_name_make(name, buf_length, app_id, task_str, task_str_len);
}
