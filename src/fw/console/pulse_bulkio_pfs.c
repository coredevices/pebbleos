/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pulse_bulkio_domain_handler.h"

#include "pbl/services/filesystem/pfs.h"
#include "system/passert.h"
#include "system/status_codes.h"
#include "pbl/util/attributes.h"
#include "pbl/util/math.h"

#include <stdint.h>
#include <string.h>

typedef struct PACKED PFSStatResp {
  uint8_t flags;
  uint32_t size;
} PFSStatResp;

typedef struct PACKED PFSOpenOptions {
  uint8_t op_flags;
  uint8_t filetype;
  uint32_t start_size;
  char filename[0];
} PFSOpenOptions;

#if !defined(CONFIG_RECOVERY_FW)
static int prv_open_file(void *packet_data, size_t length) {
  PFSOpenOptions *options = packet_data;

  size_t filename_length = MIN(FILE_MAX_NAME_LEN, length - sizeof(*options));

  char filename[filename_length+1];
  strncpy(filename, options->filename, filename_length);
  filename[filename_length] = '\0';

  int fd = pfs_open(filename, options->op_flags, options->filetype, options->start_size);
  return fd;
}

static int prv_fd_from_context(void *context) {
  return (uintptr_t)context;
}

static int pfs_domain_read(uint8_t *buf, uint32_t address, uint32_t length, void *context) {
  int fd = prv_fd_from_context(context);
  pfs_seek(fd, address, FSeekSet);
  return pfs_read(fd, buf, length);
}

static int pfs_domain_write(uint8_t *buf, uint32_t address, uint32_t length, void *context) {
  int fd = prv_fd_from_context(context);
  pfs_seek(fd, address, FSeekSet);
  return pfs_write(fd, buf, length);
}

static int pfs_domain_stat(uint8_t *resp, size_t resp_max_len, void *context) {
  int fd = prv_fd_from_context(context);

  PFSStatResp *stat_resp = (PFSStatResp*)resp;
  *stat_resp = (PFSStatResp) {
    .flags = 0,
    .size = (uint32_t)pfs_get_file_size(fd)
  };

  return sizeof(PFSStatResp);
}

static status_t pfs_domain_erase(uint8_t *packet_data, size_t length, uint8_t cookie) {
  char filename[FILE_MAX_NAME_LEN + 1];
  strncpy(filename, (char*)packet_data, FILE_MAX_NAME_LEN);

  length = MIN(FILE_MAX_NAME_LEN, length);
  filename[length] = '\0';

  return pfs_remove(filename);
}

static status_t pfs_domain_open(uint8_t *packet_data, size_t length, void **resp) {
  int fd = prv_open_file(packet_data, length);
  if (fd < 0) {
    return fd;
  }

  *resp = (void*)(uintptr_t)fd;
  return S_SUCCESS;
}

static status_t pfs_domain_close(void *context) {
  int fd = prv_fd_from_context(context);
  return pfs_close(fd);
}

PulseBulkIODomainHandler pulse_bulkio_domain_pfs = {
  .id = PulseBulkIODomainType_PFS,
  .open_proc = pfs_domain_open,
  .read_proc = pfs_domain_read,
  .write_proc = pfs_domain_write,
  .close_proc = pfs_domain_close,
  .stat_proc = pfs_domain_stat,
  .erase_proc = pfs_domain_erase
};
#endif
