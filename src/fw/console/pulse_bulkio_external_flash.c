/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pulse_bulkio_domain_handler.h"

#include <pbl/drivers/flash.h>
#include "flash_region/flash_region.h"
#include "kernel/pbl_malloc.h"
#include "system/status_codes.h"
#include "pbl/util/attributes.h"

#include <stdint.h>

typedef struct PACKED ExternalFlashEraseOptions {
  uint32_t address;
  uint32_t length;
} ExternalFlashEraseOptions;

typedef struct ExternalFlashEraseState {
  uint32_t address;
  uint32_t length;
  unsigned int next_sector;
  uint8_t cookie;
} ExternalFlashEraseState;

static int external_flash_domain_read(uint8_t *buf, uint32_t address, uint32_t length,
                                      void *context) {
  flash_read_bytes(buf, address, length);
  return length;
}

static int external_flash_domain_write(uint8_t *buf, uint32_t address, uint32_t length,
                                      void *context) {
  flash_write_bytes(buf, address, length);
  return length;
}

static int external_flash_domain_stat(uint8_t *resp, size_t resp_max_len, void *context) {
  return E_INVALID_OPERATION;
}

static void prv_erase_sector(void *context, status_t result) {
  ExternalFlashEraseState *state = context;

  const unsigned int sectors_to_erase = (
      state->length + SECTOR_SIZE_BYTES - 1) / SECTOR_SIZE_BYTES;

  if (FAILED(result)) {
    pulse_bulkio_erase_message_send(PulseBulkIODomainType_ExternalFlash, result, state->cookie);
    kernel_free(state);
  } else if (state->next_sector < sectors_to_erase) {
    unsigned int sector_addr = state->address + state->next_sector * SECTOR_SIZE_BYTES;
    state->next_sector += 1;
    pulse_bulkio_erase_message_send(PulseBulkIODomainType_ExternalFlash, S_TRUE, state->cookie);
    flash_erase_sector(sector_addr, prv_erase_sector, state);
  } else {
    pulse_bulkio_erase_message_send(PulseBulkIODomainType_ExternalFlash, S_SUCCESS, state->cookie);
    kernel_free(state);
  }
}

static status_t external_flash_domain_erase(uint8_t *packet_data, size_t length, uint8_t cookie) {
  if (length != sizeof(ExternalFlashEraseOptions)) {
    return E_INVALID_ARGUMENT;
  }

  ExternalFlashEraseOptions *options = (ExternalFlashEraseOptions*)packet_data;

  ExternalFlashEraseState *state = kernel_malloc(sizeof(ExternalFlashEraseState));
  *state = (ExternalFlashEraseState) {
    .address = options->address,
    .length = options->length,
    .next_sector = 0,
    .cookie = cookie
  };

  prv_erase_sector(state, 0);

  // Return a non-zero code to indicate the erase is still in process
  return S_TRUE;
}

static status_t external_flash_domain_open(uint8_t *packet_data, size_t length, void **resp) {
  return S_SUCCESS;
}

static status_t external_flash_domain_close(void *context) {
  return S_SUCCESS;
}

PulseBulkIODomainHandler pulse_bulkio_domain_external_flash = {
  .id = PulseBulkIODomainType_ExternalFlash,
  .open_proc = external_flash_domain_open,
  .close_proc = external_flash_domain_close,
  .read_proc = external_flash_domain_read,
  .write_proc = external_flash_domain_write,
  .stat_proc = external_flash_domain_stat,
  .erase_proc = external_flash_domain_erase
};
