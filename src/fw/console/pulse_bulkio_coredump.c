/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pulse_bulkio_domain_handler.h"

#include <pbl/drivers/flash.h>
#include "kernel/core_dump.h"
#include "kernel/core_dump_private.h"
#include "system/status_codes.h"
#include "pbl/util/attributes.h"

#include <stdint.h>

typedef struct PACKED CoredumpStatResp {
  uint8_t flags;
  uint8_t unread;
  uint32_t size;
} CoredumpStatResp;

static uint32_t prv_get_coredump_index(void *packet_data) {
  return *(uint32_t*)packet_data;
}

static int coredump_domain_read(uint8_t *buf, uint32_t address, uint32_t length,
                                     void *context) {
  uint32_t index = (uintptr_t)context;
  uint32_t core_base_addr = core_dump_get_slot_address(index) + sizeof(CoreDumpFlashRegionHeader);
  flash_read_bytes(buf, core_base_addr + address, length);
  return length;
}

static int coredump_domain_write(uint8_t *buf, uint32_t address, uint32_t length,
                                      void *context) {
  return E_INVALID_OPERATION;
}

static int coredump_domain_stat(uint8_t *resp, size_t resp_max_len, void *context) {
  uint32_t index = (uintptr_t)context;
  uint32_t addr = core_dump_get_slot_address(index);

  CoredumpStatResp *stat_resp = (CoredumpStatResp*)resp;
  *stat_resp = (CoredumpStatResp) {
    .flags = 0,
    .unread = core_dump_is_unread_available(addr),
    // Size of 0 indicates no core dump available
    .size = 0
  };

  if (stat_resp->unread == 1) {
    status_t ret = core_dump_size(addr, &stat_resp->size);

    if (FAILED(ret)) {
      return ret;
    }
  }

  return sizeof(CoredumpStatResp);
}

static status_t coredump_domain_erase(uint8_t *packet_data, size_t length, uint8_t cookie) {
  uint32_t index = prv_get_coredump_index(packet_data);
  uint32_t addr = core_dump_get_slot_address(index);
  core_dump_mark_read(addr);
  return S_SUCCESS;
}

static status_t coredump_domain_open(uint8_t *packet_data, size_t length, void **resp) {
  *resp = (void*)(uintptr_t)prv_get_coredump_index(packet_data);
  return S_SUCCESS;
}

static status_t coredump_domain_close(void *context) {
  return S_SUCCESS;
}

PulseBulkIODomainHandler pulse_bulkio_domain_coredump = {
  .id = PulseBulkIODomainType_Coredump,
  .open_proc = coredump_domain_open,
  .close_proc = coredump_domain_close,
  .read_proc = coredump_domain_read,
  .write_proc = coredump_domain_write,
  .stat_proc = coredump_domain_stat,
  .erase_proc = coredump_domain_erase
};
