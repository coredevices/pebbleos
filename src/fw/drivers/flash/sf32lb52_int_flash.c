#include "board/board.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/qspi_flash.h"
#include "drivers/flash/qspi_flash_part_definitions.h"
#include "flash_region/flash_region.h"
#include "system/passert.h"
#include "system/status_codes.h"
#include "util/math.h"

#include "board/board.h"

static QSPI_FLASH_CTX_T spi_flash_handle;
static DMA_HandleTypeDef spi_flash_dma_handle;

#define FLASH2_IRQHandler DMAC1_CH2_IRQHandler
#define FLASH2_DMA_INSTANCE DMA1_Channel2
#define FLASH2_DMA_REQUEST DMA_REQUEST_1
#define FLASH2_DMA_IRQ DMAC1_CH2_IRQn

#define FLASH2_CONFIG           \
  {                             \
      .Instance = FLASH2,       \
      .line = 2,                \
      .base = FLASH2_BASE_ADDR, \
      .msize = 4,               \
      .SpiMode = 0,             \
  }

#define FLASH2_DMA_CONFIG              \
  {                                    \
      .dma_irq_prio = 0,               \
      .Instance = FLASH2_DMA_INSTANCE, \
      .dma_irq = FLASH2_DMA_IRQ,       \
      .request = FLASH2_DMA_REQUEST,   \
  }

bool flash_check_whoami(void) { return true; }

FlashAddress flash_impl_get_sector_base_address(FlashAddress addr) {
  return (addr & SECTOR_ADDR_MASK);
}

FlashAddress flash_impl_get_subsector_base_address(FlashAddress addr) {
  return (addr & SUBSECTOR_ADDR_MASK);
}

void flash_impl_enable_write_protection(void) {}

status_t flash_impl_write_protect(FlashAddress start_sector, FlashAddress end_sector) {
  return S_SUCCESS;
}

status_t flash_impl_unprotect(void) { return S_SUCCESS; }

status_t flash_impl_init(bool coredump_mode) {
  (void)coredump_mode;
  qspi_configure_t flash_cfg = FLASH2_CONFIG;
  struct dma_config flash_dma = FLASH2_DMA_CONFIG;

  flash_cfg.line = HAL_FLASH_QMODE;
  HAL_Delay_us(0);
  spi_flash_handle.dual_mode = 1;
  spi_flash_handle.flash_mode = 0;  // nor
  HAL_StatusTypeDef res = HAL_FLASH_Init(&spi_flash_handle, &flash_cfg, &spi_flash_dma_handle,
                                         &flash_dma, 5);

  if (res != HAL_OK) {
    return E_ERROR;
  }
  return S_SUCCESS;
}

status_t flash_impl_get_erase_status(void) {
  // A call to the HAL_QSPIEX_SECT_ERASE interface will always return success after the call
  return S_SUCCESS;
}

static int prv_erase_nor(uint32_t addr, uint32_t size) {
  FLASH_HandleTypeDef *hflash;
  uint32_t taddr, remain;
  int res;
  __disable_irq();

  hflash = &spi_flash_handle.handle;

  if ((addr < hflash->base) || (addr > (hflash->base + hflash->size))) return 1;

  taddr = addr - hflash->base;
  remain = size;

  if ((taddr & (QSPI_NOR_SECT_SIZE - 1)) != 0) return -1;
  if ((remain & (QSPI_NOR_SECT_SIZE - 1)) != 0) return -2;

  while (remain > 0) {
    res = HAL_QSPIEX_SECT_ERASE(hflash, taddr);
    if (res != 0) return 1;

    remain -= QSPI_NOR_SECT_SIZE;
    taddr += QSPI_NOR_SECT_SIZE;
  }

  __enable_irq();
  return 0;
}

static int prv_write_nor(uint32_t addr, uint8_t *buf, uint32_t size) {
  FLASH_HandleTypeDef *hflash;
  uint32_t taddr, start, remain, fill;
  uint8_t *tbuf;
  int res;
  uint8_t *local_buf = NULL;

  hflash = &spi_flash_handle.handle;

  if ((addr < hflash->base) || (addr > (hflash->base + hflash->size))) return 0;

  if (IS_SAME_FLASH_ADDR(buf, addr) || IS_SPI_NONDMA_RAM_ADDR(buf)
     || (IS_DMA_ACCROSS_1M_BOUNDARY((uint32_t)buf, size)))
  {
    local_buf = (uint8_t *)malloc(size);
    PBL_ASSERT(local_buf != NULL, "malloc failed");
    memcpy(local_buf, buf, size);
    tbuf = local_buf;
  }
  else
  {
    tbuf = buf;
  }

  __disable_irq();

  taddr = addr - hflash->base;
  remain = size;

  start = taddr & (QSPI_NOR_PAGE_SIZE - 1);
  if (start > 0)  // start address not page aligned
  {
    fill = QSPI_NOR_PAGE_SIZE - start;  // remained size in one page
    if (fill > size)                    // not over one page
    {
      fill = size;
    }
    res = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, fill);

    if ((uint32_t)res != fill) 
    {
      size = 0;
      goto __EXIT;
    }
    taddr += fill;
    tbuf += fill;
    remain -= fill;
  }
  while (remain > 0) {
    fill = remain > QSPI_NOR_PAGE_SIZE ? QSPI_NOR_PAGE_SIZE : remain;
    res = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, fill);
    if ((uint32_t)res != fill)
    {
      size = 0;
      goto __EXIT;
    }

    taddr += fill;
    tbuf += fill;
    remain -= fill;
  }


__EXIT:  
  __enable_irq();

  if (local_buf)
  {
    free(local_buf);
  }

  return size;
}

status_t flash_impl_erase_subsector_begin(FlashAddress subsector_addr) {
  if (prv_erase_nor(subsector_addr, SUBSECTOR_SIZE_BYTES) != 0) {
    return E_ERROR;
  }
  return S_SUCCESS;
}
status_t flash_impl_erase_sector_begin(FlashAddress sector_addr) {
  if (prv_erase_nor(sector_addr, SECTOR_SIZE_BYTES) != 0) {
    return E_ERROR;
  }
  return S_SUCCESS;
}

status_t flash_impl_erase_suspend(FlashAddress sector_addr) {
  // Everything will be blocked during the erase process, so nothing will happen if you call this
  // function.
  return S_SUCCESS;
}

status_t flash_impl_erase_resume(FlashAddress sector_addr) {
  // Everything will be blocked during the erase process, so nothing will happen if you call this
  // function.
  return S_SUCCESS;
}

status_t flash_impl_read_sync(void *buffer_ptr, FlashAddress start_addr, size_t buffer_size) {
  PBL_ASSERT(buffer_size > 0, "flash_impl_read_sync() called with 0 bytes to read");
  memcpy(buffer_ptr, (void *)(start_addr), buffer_size);
  return S_SUCCESS;
}

int flash_impl_write_page_begin(const void *buffer, const FlashAddress start_addr, size_t len) {
  return prv_write_nor(start_addr, (uint8_t *)buffer, len);
}

status_t flash_impl_get_write_status(void) {
  // It will be done in HAL_QSPIEX_WRITE_PAGE, so it must return success
  return S_SUCCESS;
}

status_t flash_impl_enter_low_power_mode(void) { return S_SUCCESS; }
status_t flash_impl_exit_low_power_mode(void) { return S_SUCCESS; }

status_t flash_impl_set_burst_mode(bool burst_mode) {
  // NYI
  return S_SUCCESS;
}

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

status_t flash_impl_blank_check_sector(FlashAddress addr) {
  return prv_blank_check_poll(addr, false /* !is_subsector */);
}
status_t flash_impl_blank_check_subsector(FlashAddress addr) {
  return prv_blank_check_poll(addr, true /* !is_subsector */);
}

uint32_t flash_impl_get_typical_sector_erase_duration_ms(void) { return 150; }

uint32_t flash_impl_get_typical_subsector_erase_duration_ms(void) { return 50; }