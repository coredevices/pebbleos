/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "board/board.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/qspi_flash.h"
#include "drivers/flash/qspi_flash_part_definitions.h"
#include "flash_region/flash_region.h"
#include "kernel/pbl_malloc.h"
#include "system/passert.h"
#include "system/status_codes.h"
#include "util/math.h"

static bool prv_blank_check_poll(uint32_t addr, bool is_subsector) {
  const uint32_t size_bytes = is_subsector ? SUBSECTOR_SIZE_BYTES : SECTOR_SIZE_BYTES;
  const uint32_t BUF_SIZE_BYTES = 128;
  const uint32_t BUF_SIZE_WORDS = BUF_SIZE_BYTES / sizeof(uint32_t);
  const uint32_t FLASH_RESET_WORD_VALUE = 0xFFFFFFFF;
  uint32_t buffer[BUF_SIZE_WORDS];
  for (uint32_t offset = 0; offset < size_bytes; offset += BUF_SIZE_BYTES) {
    flash_impl_read_sync(buffer, addr + offset, BUF_SIZE_BYTES);
    for (uint32_t i = 0; i < BUF_SIZE_WORDS; ++i) {
      if (buffer[i] != FLASH_RESET_WORD_VALUE) {
        return false;
      }
    }
  }
  return true;
}

static int prv_erase_nor(QSPIFlash *dev, uint32_t addr, uint32_t size) {
  FLASH_HandleTypeDef *hflash;
  uint32_t taddr, remain;
  int res;
  __disable_irq();

  hflash = &dev->qspi->state->ctx.handle;

  if ((addr < hflash->base) || (addr > (hflash->base + hflash->size))) return 1;

  taddr = addr - hflash->base;
  remain = size;

  if ((taddr & (SUBSECTOR_SIZE_BYTES - 1)) != 0) return -1;
  if ((remain & (SUBSECTOR_SIZE_BYTES - 1)) != 0) return -2;

  while (remain > 0) {
    res = HAL_QSPIEX_SECT_ERASE(hflash, taddr);
    if (res != 0) return 1;

    remain -= SUBSECTOR_SIZE_BYTES;
    taddr += SUBSECTOR_SIZE_BYTES;
  }

  __enable_irq();
  return 0;
}

static int prv_write_nor(QSPIFlash *dev, uint32_t addr, uint8_t *buf, uint32_t size) {
  FLASH_HandleTypeDef *hflash;
  uint32_t taddr, start, remain, fill;
  uint8_t *tbuf;
  int res;
  uint8_t *local_buf = NULL;

  hflash = &dev->qspi->state->ctx.handle;

  if ((addr < hflash->base) || (addr > (hflash->base + hflash->size))) return 0;

  if (IS_SAME_FLASH_ADDR(buf, addr) || IS_SPI_NONDMA_RAM_ADDR(buf) ||
      (IS_DMA_ACCROSS_1M_BOUNDARY((uint32_t)buf, size))) {
    local_buf = kernel_malloc_check(size);
    memcpy(local_buf, buf, size);
    tbuf = local_buf;
  } else {
    tbuf = buf;
  }

  __disable_irq();

  taddr = addr - hflash->base;
  remain = size;

  start = taddr & (PAGE_SIZE_BYTES - 1);
  // start address not page aligned
  if (start > 0) {
    fill = PAGE_SIZE_BYTES - start;
    if (fill > size) {
      fill = size;
    }

    res = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, fill);
    if ((uint32_t)res != fill) {
      size = 0;
      goto exit;
    }

    taddr += fill;
    tbuf += fill;
    remain -= fill;
  }

  while (remain > 0) {
    fill = remain > PAGE_SIZE_BYTES ? PAGE_SIZE_BYTES : remain;
    res = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, fill);
    if ((uint32_t)res != fill) {
      size = 0;
      goto exit;
    }

    taddr += fill;
    tbuf += fill;
    remain -= fill;
  }

exit:
  __enable_irq();

  if (local_buf) {
    kernel_free(local_buf);
  }

  return size;
}

bool qspi_flash_check_whoami(QSPIFlash *dev) { return true; }

status_t qspi_flash_write_protection_enable(QSPIFlash *dev) { return S_NO_ACTION_REQUIRED; }

status_t qspi_flash_lock_sector(QSPIFlash *dev, uint32_t addr) { return S_SUCCESS; }

status_t qspi_flash_unlock_all(QSPIFlash *dev) { return S_SUCCESS; }

void qspi_flash_init(QSPIFlash *dev, QSPIFlashPart *part, bool coredump_mode) {
  HAL_StatusTypeDef res;

  (void)coredump_mode;

  dev->qspi->state->ctx.dual_mode = 1;

  res = HAL_FLASH_Init(&dev->qspi->state->ctx, (qspi_configure_t *)&dev->qspi->cfg,
                       &dev->qspi->state->hdma, (struct dma_config *)&dev->qspi->dma,
                       dev->qspi->clk_div);
  PBL_ASSERT(res == HAL_OK, "HAL_FLASH_Init failed");
}

status_t qspi_flash_is_erase_complete(QSPIFlash *dev) {
  // A call to the HAL_QSPIEX_SECT_ERASE interface will always return success after the call
  return S_SUCCESS;
}

status_t qspi_flash_erase_begin(QSPIFlash *dev, uint32_t addr, bool is_subsector) {
  if (prv_erase_nor(dev, addr, is_subsector ? SUBSECTOR_SIZE_BYTES : SECTOR_SIZE_BYTES) != 0) {
    return E_ERROR;
  }

  return S_SUCCESS;
}

status_t qspi_flash_erase_suspend(QSPIFlash *dev, uint32_t addr) {
  // Everything will be blocked during the erase process, so nothing will happen if you call this
  // function.
  return S_SUCCESS;
}

void qspi_flash_erase_resume(QSPIFlash *dev, uint32_t addr) {
  // Everything will be blocked during the erase process, so nothing will happen if you call this
  // function.
}

void qspi_flash_read_blocking(QSPIFlash *dev, uint32_t addr, void *buffer, uint32_t length) {
  PBL_ASSERT(length > 0, "flash_impl_read_sync() called with 0 bytes to read");
  memcpy(buffer, (void *)(addr), length);
}

int qspi_flash_write_page_begin(QSPIFlash *dev, const void *buffer, uint32_t addr,
                                uint32_t length) {
  return prv_write_nor(dev, addr, (uint8_t *)buffer, length);
}

status_t qspi_flash_get_write_status(QSPIFlash *dev) {
  // It will be done in HAL_QSPIEX_WRITE_PAGE, so it must return success
  return S_SUCCESS;
}

void qspi_flash_set_lower_power_mode(QSPIFlash *dev, bool active) {}

status_t qspi_flash_blank_check(QSPIFlash *dev, uint32_t addr, bool is_subsector) {
  return prv_blank_check_poll(addr, is_subsector);
}

status_t flash_impl_set_nvram_erase_status(bool is_subsector, FlashAddress addr) {
  return S_SUCCESS;
}

status_t flash_impl_clear_nvram_erase_status(void) { return S_SUCCESS; }

status_t flash_impl_get_nvram_erase_status(bool *is_subsector, FlashAddress *addr) {
  return S_FALSE;
}