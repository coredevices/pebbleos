/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdint.h>

#include "board/board.h"
#include <pbl/drivers/qspi.h>
#include <pbl/drivers/flash/qspi_flash.h>
#include "flash_region/flash_region.h"
#include "kernel/util/delay.h"
#include "kernel/util/sleep.h"
#include "pbl/soc/nrf/sleep.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"

#include <hal/nrf_qspi.h>
#include <nrfx.h>

#include "FreeRTOS.h"
#include "semphr.h"

// NOTE: This driver does not cover anomaly 244, which may cause data corruption
// if HF clock source is switching between HFXO and HFINT (e.g. by BLE). This
// issue has not been observed at operating speeds <= 8MHz, therefore, no
// workaround is implemented here. A warning log will be emitted if driver is
// initialized at higher frequencies.

// Asynchronous read/write can actually add overhead for small sizes due to context
// switching and semaphore handling, so we define a minimum size for async ops.
#define MIN_RW_ASYNC_SIZE 256U

// Obtain security register index (zero-indexed) from address
// Bits 15-12 of the address correspond to the security register index (one-indexed)
#define SEC_ADDR_TO_IDX(addr) (((addr) >> 12U) - 1U)

// Minimum address to enable 4-byte addressing
#define ADDR_4BYTE_THRESHOLD 0x1000000UL

static uint8_t __attribute__((aligned(4))) s_bounce_buf[32];

void QSPI_IRQHandler(void) {
  BaseType_t woken = pdFALSE;

  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

  xSemaphoreGiveFromISR(QSPI_FLASH->qspi->state->sem, &woken);
  portYIELD_FROM_ISR(woken);
}

// -----------------------------------------------------------------------------
// Internal helpers

// Workaround for nRF52 Anomaly 215:
//
// Symptoms:
// CPU halts.
//
// Conditions:
// Init and start QSPI, use XIP, then write to or read any QSPI register with an
// offset above 0x600.
//
// Consequences:
// CPU halts.
//
// Workaround:
// Trigger QSPI TASKS_ACTIVATE after XIP is used and wait for QSPI EVENTS_READY
// before accessing any QSPI register with an offset above 0x600.
static void prv_workaround_215_apply(void) {
  nrf_qspi_pins_t pins;
  nrf_qspi_pins_t disconnected_pins = {
      .sck_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .csn_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io0_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io1_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io2_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io3_pin = NRF_QSPI_PIN_NOT_CONNECTED,
  };

  // Disconnect pins to not wait for response from external memory
  nrf_qspi_pins_get(NRF_QSPI, &pins);
  nrf_qspi_pins_set(NRF_QSPI, &disconnected_pins);

  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);

  while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
  }

  // Restore previous pins
  nrf_qspi_pins_set(NRF_QSPI, &pins);
}

static void prv_cinstr_write_read(QSPIFlash *dev, uint8_t instr, const void *data, void *buf,
                                  size_t len) {
  nrf_qspi_cinstr_conf_t conf = {
      .opcode = instr,
      .length = len + 1U,
      .io2_level = true,
      .io3_level = true,
  };

  PBL_ASSERTN(len <= 8U);

  nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  // Accessing registers with offset > 0x600 below, requires workaround for Anomaly 215
  prv_workaround_215_apply();
  if (data != NULL) {
    nrf_qspi_cinstrdata_set(NRF_QSPI, conf.length, data);
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_cinstr_transfer_start(NRF_QSPI, &conf);
  while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

  if (buf != NULL) {
    nrf_qspi_cinstrdata_get(NRF_QSPI, conf.length, buf);
  }
}

static inline void prv_cinstr(QSPIFlash *dev, uint8_t instr) {
  prv_cinstr_write_read(dev, instr, NULL, NULL, 0U);
}

static inline void prv_cinstr_read(QSPIFlash *dev, uint8_t instr, void *buf, size_t len) {
  prv_cinstr_write_read(dev, instr, NULL, buf, len);
}

static inline void prv_cinstr_write(QSPIFlash *dev, uint8_t instr, const void *data, size_t len) {
  prv_cinstr_write_read(dev, instr, data, NULL, len);
}

static void prv_read(QSPIFlash *dev, void *buf, size_t len, uint32_t addr) {
  if ((len <= MIN_RW_ASYNC_SIZE) || dev->state->coredump_mode) {
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  } else {
    nrf_qspi_int_enable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    soc_nrf_sleep_full_block();
  }

  nrf_qspi_read_buffer_set(NRF_QSPI, buf, len, addr);
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_READSTART);

  if ((len <= MIN_RW_ASYNC_SIZE) || dev->state->coredump_mode) {
    while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
    }
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  } else {
    xSemaphoreTake(dev->qspi->state->sem, portMAX_DELAY);
    soc_nrf_sleep_full_release();
  }
}

static void prv_write(QSPIFlash *dev, const void *buf, size_t len, uint32_t addr) {
  if ((len <= MIN_RW_ASYNC_SIZE) || dev->state->coredump_mode) {
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  } else {
    nrf_qspi_int_enable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    soc_nrf_sleep_full_block();
  }

  nrf_qspi_write_buffer_set(NRF_QSPI, buf, len, addr);
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_WRITESTART);

  if ((len <= MIN_RW_ASYNC_SIZE) || dev->state->coredump_mode) {
    while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
    }
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  } else {
    xSemaphoreTake(dev->qspi->state->sem, portMAX_DELAY);
    soc_nrf_sleep_full_release();
  }
}

static inline void prv_write_enable(QSPIFlash *dev) {
  QSPIFlashPart *part = dev->state->part;

  prv_cinstr(dev, part->instructions.write_enable);
}

static inline void prv_read_sr1(QSPIFlash *dev, uint8_t *sr1) {
  QSPIFlashPart *part = dev->state->part;

  prv_cinstr_read(dev, part->instructions.rdsr1, sr1, 1U);
}

static inline void prv_read_sr2(QSPIFlash *dev, uint8_t *sr2) {
  QSPIFlashPart *part = dev->state->part;

  prv_cinstr_read(dev, part->instructions.rdsr2, sr2, 1U);
}

static inline void prv_write_sr(QSPIFlash *dev, const uint8_t *sr, size_t len) {
  QSPIFlashPart *part = dev->state->part;

  prv_cinstr_write(dev, part->instructions.wrsr, sr, len);
}

static inline void prv_write_sr2(QSPIFlash *dev, uint8_t sr2) {
  QSPIFlashPart *part = dev->state->part;

  prv_cinstr_write(dev, part->instructions.wrsr2, &sr2, 1U);
}

static inline bool prv_wip_check(QSPIFlash *dev) {
  QSPIFlashPart *part = dev->state->part;
  uint8_t sr1;

  prv_read_sr1(dev, &sr1);

  return (sr1 & part->status_bit_masks.busy) != 0U;
}

static void prv_configure_qe(QSPIFlash *dev) {
  QSPIFlashPart *part = dev->state->part;
  uint8_t sr[2];

  // Check first if read/write mode requires QE to be set
  if (!(dev->read_mode == QSPI_FLASH_READ_READ2IO || dev->read_mode == QSPI_FLASH_READ_READ4O ||
        dev->read_mode == QSPI_FLASH_READ_READ4IO || dev->write_mode == QSPI_FLASH_WRITE_PP4O ||
        dev->write_mode == QSPI_FLASH_WRITE_PP4IO)) {
    return;
  }

  // Check if QE is needed
  if (part->qer_type == JESD216_DW15_QER_NONE) {
    return;
  }

  // Enable QE bit
  switch (part->qer_type) {
    case JESD216_DW15_QER_S1B6:
      prv_read_sr1(dev, &sr[0]);
      sr[0] |= (1U << 6U);
      prv_write_sr(dev, sr, 1U);
      break;
    case JESD216_DW15_QER_S2B1v1:
    case JESD216_DW15_QER_S2B1v4:
    case JESD216_DW15_QER_S2B1v5:
      // Writing SR2 requires writing SR1 as well
      prv_read_sr1(dev, &sr[0]);
      prv_read_sr2(dev, &sr[1]);
      sr[1] |= (1U << 1U);
      prv_write_sr(dev, sr, 2U);
      break;
    case JESD216_DW15_QER_S2B1v6:
      // We can write SR2 without writing SR1
      prv_read_sr2(dev, &sr[1]);
      sr[1] |= (1U << 1U);
      prv_write_sr2(dev, sr[1]);
      break;
    default:
      PBL_ASSERTN(false);
  }
}

status_t prv_qspi_security_register_check(QSPIFlash *dev, uint32_t addr) {
  QSPIFlashPart *part = dev->state->part;
  bool addr_valid = false;

  for (uint8_t i = 0U; i < part->sec_registers.num_sec_regs; ++i) {
    if (addr >= part->sec_registers.sec_regs[i] &&
        addr < part->sec_registers.sec_regs[i] + part->sec_registers.sec_reg_size) {
      addr_valid = true;
      break;
    }
  }

  if (!addr_valid) {
    return E_INVALID_ARGUMENT;
  }

  return S_SUCCESS;
}

// -----------------------------------------------------------------------------
// QSPI interface

void qspi_flash_init(QSPIFlash *dev, QSPIFlashPart *part, bool coredump_mode) {
  nrf_qspi_pins_t conf_pins;
  nrf_qspi_prot_conf_t conf_prot;
  nrf_qspi_phy_conf_t conf_phy;

  dev->state->part = part;
  dev->state->coredump_mode = coredump_mode;

  if (dev->qspi->state->initialized) {
    return;
  }

  // QSPI clock is 32MHz, we have dividers from 1 to 16
  PBL_ASSERTN(dev->qspi->clk_freq_hz <= 32000000UL && dev->qspi->clk_freq_hz >= 200000UL);
  if (dev->qspi->clk_freq_hz > 8000000UL) {
    PBL_LOG_WRN(
        "QSPI initialized at %lu Hz, which may cause data corruption if HF clock source switches "
        "(anomaly 244)",
        dev->qspi->clk_freq_hz);
  }

  // QSPI pins
  conf_pins.sck_pin = dev->qspi->clk_gpio;
  conf_pins.csn_pin = dev->qspi->cs_gpio;
  conf_pins.io0_pin = dev->qspi->data_gpio[0];
  conf_pins.io1_pin = dev->qspi->data_gpio[1];
  conf_pins.io2_pin = dev->qspi->data_gpio[2];
  conf_pins.io3_pin = dev->qspi->data_gpio[3];

  nrf_qspi_pins_set(NRF_QSPI, &conf_pins);

  // QSPI protocol configuration
  switch (dev->read_mode) {
    case QSPI_FLASH_READ_READ2O:
      conf_prot.readoc = NRF_QSPI_READOC_READ2O;
      break;
    case QSPI_FLASH_READ_READ2IO:
      conf_prot.readoc = NRF_QSPI_READOC_READ2IO;
      break;
    case QSPI_FLASH_READ_READ4O:
      conf_prot.readoc = NRF_QSPI_READOC_READ4O;
      break;
    case QSPI_FLASH_READ_READ4IO:
      conf_prot.readoc = NRF_QSPI_READOC_READ4IO;
      break;
    default:
      conf_prot.readoc = NRF_QSPI_READOC_FASTREAD;
      break;
  }

  switch (dev->write_mode) {
    case QSPI_FLASH_WRITE_PP2O:
      conf_prot.writeoc = NRF_QSPI_WRITEOC_PP2O;
      break;
    case QSPI_FLASH_WRITE_PP4O:
      conf_prot.writeoc = NRF_QSPI_WRITEOC_PP4O;
      break;
    case QSPI_FLASH_WRITE_PP4IO:
      conf_prot.writeoc = NRF_QSPI_WRITEOC_PP4IO;
      break;
    default:
      conf_prot.writeoc = NRF_QSPI_WRITEOC_PP;
      break;
  }

  if (part->size > ADDR_4BYTE_THRESHOLD) {
    conf_prot.addrmode = NRF_QSPI_ADDRMODE_32BIT;
  } else {
    conf_prot.addrmode = NRF_QSPI_ADDRMODE_24BIT;
  }

  conf_prot.dpmconfig = false;

  nrf_qspi_ifconfig0_set(NRF_QSPI, &conf_prot);

  // QSPI PHY configuration
  conf_phy.sck_delay = 5U;
  conf_phy.dpmen = false;
  conf_phy.spi_mode = NRF_QSPI_MODE_0;
  conf_phy.sck_freq = (32000000UL / dev->qspi->clk_freq_hz) - 1U;

  nrf_qspi_ifconfig1_set(NRF_QSPI, &conf_phy);

  // Enable QSPI peripheral
  nrf_qspi_enable(NRF_QSPI);

  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);
  while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

  // Reset the flash to stop any program's or erase in progress from before reboot
  prv_cinstr(dev, part->instructions.reset_enable);
  prv_cinstr(dev, part->instructions.reset);

  psleep(part->reset_latency_ms);

  // Enable 4-byte addressing if needed
  if (conf_prot.addrmode == NRF_QSPI_ADDRMODE_32BIT) {
    prv_cinstr(dev, part->instructions.en4b);
  }

  // Configure QE if needed
  prv_configure_qe(dev);

  NVIC_SetPriority(QSPI_IRQn, 5);
  NVIC_EnableIRQ(QSPI_IRQn);

  dev->qspi->state->sem = xSemaphoreCreateBinary();
  dev->qspi->state->initialized = true;
}

bool qspi_flash_check_whoami(QSPIFlash *dev) {
  QSPIFlashPart *part = dev->state->part;
  uint32_t val;

  prv_cinstr_read(dev, part->instructions.qspi_id, &val, 3U);

  return val == part->qspi_id_value;
}

bool qspi_flash_is_in_coredump_mode(QSPIFlash *dev) {
  return dev->state->coredump_mode;
}

status_t qspi_flash_erase_begin(QSPIFlash *dev, uint32_t addr, bool is_subsector) {
  nrf_qspi_erase_len_t len;

  // address must be word aligned
  if ((addr & 0x3U) != 0U) {
    return E_INVALID_ARGUMENT;
  }

  if (is_subsector) {
    len = NRF_QSPI_ERASE_LEN_4KB;
  } else {
    len = NRF_QSPI_ERASE_LEN_64KB;
  }

  prv_write_enable(dev);

  nrf_qspi_erase_ptr_set(NRF_QSPI, addr, len);
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ERASESTART);
  while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

  soc_nrf_sleep_full_block();

  return S_SUCCESS;
}

status_t qspi_flash_erase_suspend(QSPIFlash *dev, uint32_t addr) {
  QSPIFlashPart *part = dev->state->part;

  if (!prv_wip_check(dev)) {
    return S_NO_ACTION_REQUIRED;
  }

  prv_cinstr(dev, part->instructions.erase_suspend);

  if (part->suspend_to_read_latency_us) {
    delay_us(part->suspend_to_read_latency_us);
  }

  return S_SUCCESS;
}

void qspi_flash_erase_resume(QSPIFlash *dev, uint32_t addr) {
  QSPIFlashPart *part = dev->state->part;

  prv_cinstr(dev, part->instructions.erase_resume);
}

status_t qspi_flash_is_erase_complete(QSPIFlash *dev) {
  QSPIFlashPart *part = dev->state->part;
  uint8_t sr2;

  // erase ongoing
  if (prv_wip_check(dev)) {
    return E_BUSY;
  }

  // erase suspended
  prv_read_sr2(dev, &sr2);
  if ((sr2 & part->flag_status_bit_masks.erase_suspend) != 0U) {
    return E_AGAIN;
  }

  soc_nrf_sleep_full_release();

  return S_SUCCESS;
}

void qspi_flash_read_blocking(QSPIFlash *dev, uint32_t addr, void *buffer, uint32_t length) {
  uint8_t __attribute__((aligned(4))) b_buf[4];
  uint8_t buf_pre;
  uint8_t buf_suf;
  uint32_t buf_mid;

  buf_pre = (4U - (uint8_t)((uint32_t)buffer % 4U)) % 4U;
  if (buf_pre > length) {
    buf_pre = length;
  }

  buf_suf = (uint8_t)((length - buf_pre) % 4U);
  buf_mid = length - buf_pre - buf_suf;

  if (buf_pre != 0U) {
    prv_read(dev, b_buf, 4U, addr);

    memcpy(buffer, b_buf, buf_pre);

    addr += buf_pre;
    buffer = ((uint8_t *)buffer) + buf_pre;
  }

  if (buf_mid != 0U) {
    prv_read(dev, buffer, buf_mid, addr);

    addr += buf_mid;
    buffer = ((uint8_t *)buffer) + buf_mid;
  }

  if (buf_suf != 0U) {
    prv_read(dev, b_buf, 4U, addr);

    memcpy(buffer, b_buf, buf_suf);
  }
}

int qspi_flash_write_page_begin(QSPIFlash *dev, const void *buffer, uint32_t addr,
                                uint32_t length) {
  uint8_t __attribute__((aligned(4))) b_buf[4];
  uint8_t buf_pre;
  uint8_t buf_suf;
  uint32_t buf_mid;

  // we can write from start address up to the end of the page
  length = MIN(length, PAGE_SIZE_BYTES - (addr % PAGE_SIZE_BYTES));

  // bounce data to RAM if not in RAM
  if (!nrfx_is_in_ram(buffer)) {
    length = MIN(length, sizeof(s_bounce_buf));
    memcpy(s_bounce_buf, buffer, length);
    buffer = s_bounce_buf;
  }

  // buffer needs to be word aligned, if it is not, split the write into chunks
  // (prefix/middle/suffix) with word-aligned buffers
  buf_pre = (4U - (uint8_t)((uint32_t)buffer % 4U)) % 4U;
  if (buf_pre > length) {
    buf_pre = length;
  }

  buf_suf = (uint8_t)((length - buf_pre) % 4U);
  buf_mid = length - buf_pre - buf_suf;

  prv_write_enable(dev);

  if (buf_pre != 0U) {
    memset(&b_buf[buf_pre], 0xff, sizeof(b_buf) - buf_pre);
    memcpy(b_buf, buffer, buf_pre);

    prv_write(dev, b_buf, 4U, addr);

    addr += buf_pre;
    buffer = ((uint8_t *)buffer) + buf_pre;
  }

  if (buf_mid != 0U) {
    while (prv_wip_check(dev)) {
    }

    prv_write(dev, buffer, buf_mid, addr);

    addr += buf_mid;
    buffer = ((uint8_t *)buffer) + buf_mid;
  }

  if (buf_suf != 0U) {
    while (prv_wip_check(dev)) {
    }

    memset(&b_buf[buf_suf], 0xff, 4U - buf_suf);
    memcpy(b_buf, buffer, buf_suf);

    prv_write(dev, b_buf, 4U, addr);
  }

  return length;
}

status_t qspi_flash_get_write_status(QSPIFlash *dev) {
  return prv_wip_check(dev) ? E_BUSY : S_SUCCESS;
}

void qspi_flash_set_lower_power_mode(QSPIFlash *dev, bool active) {
  QSPIFlashPart *part = dev->state->part;

  if (active) {
    prv_cinstr(dev, part->instructions.enter_low_power);
    if (part->standby_to_low_power_latency_us) {
      delay_us(part->standby_to_low_power_latency_us);
    }

    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_DEACTIVATE);
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

    nrf_qspi_disable(NRF_QSPI);
  } else {
    nrf_qspi_enable(NRF_QSPI);

    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);
    while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
    }
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

    prv_cinstr(dev, part->instructions.exit_low_power);
    if (part->low_power_to_standby_latency_us) {
      delay_us(part->low_power_to_standby_latency_us);
    }
  }
}

status_t qspi_flash_blank_check(QSPIFlash *dev, uint32_t addr, bool is_subsector) {
  const uint32_t size_bytes = is_subsector ? SUBSECTOR_SIZE_BYTES : SECTOR_SIZE_BYTES;
  const uint32_t BUF_SIZE_BYTES = 128;
  const uint32_t BUF_SIZE_WORDS = BUF_SIZE_BYTES / sizeof(uint32_t);
  uint32_t buffer[BUF_SIZE_WORDS];

  for (uint32_t offset = 0U; offset < size_bytes; offset += BUF_SIZE_BYTES) {
    qspi_flash_read_blocking(dev, addr + offset, buffer, BUF_SIZE_BYTES);
    for (uint32_t i = 0U; i < BUF_SIZE_WORDS; ++i) {
      if (buffer[i] != 0xFFFFFFFFUL) {
        return S_FALSE;
      }
    }
  }

  return S_TRUE;
}

status_t qspi_flash_read_security_register(QSPIFlash *dev, uint32_t addr, uint8_t *val) {
  QSPIFlashPart *part = dev->state->part;
  status_t ret;
  uint8_t out[6];
  uint8_t in[6];
  uint8_t len;

  ret = prv_qspi_security_register_check(dev, addr);
  if (ret != S_SUCCESS) {
    return ret;
  }

  if (part->size > ADDR_4BYTE_THRESHOLD) {
    len = 6;
    out[0] = (addr >> 24U);
    out[1] = (addr >> 16U) & 0xFFU;
    out[2] = (addr >> 8U) & 0xFFU;
    out[3] = addr & 0xFFU;
  } else {
    len = 5;
    out[0] = (addr >> 16U) & 0xFFU;
    out[1] = (addr >> 8U) & 0xFFU;
    out[2] = addr & 0xFFU;
  }

  prv_cinstr_write_read(dev, part->instructions.read_sec, out, in, len);

  if (part->size > ADDR_4BYTE_THRESHOLD) {
    *val = in[5];
  } else {
    *val = in[4];
  }

  return 0;
}

status_t qspi_flash_security_register_is_locked(QSPIFlash *dev, uint32_t addr, bool *locked) {
  QSPIFlashPart *part = dev->state->part;
  uint8_t sr2;
  status_t res;

  res = prv_qspi_security_register_check(dev, addr);
  if (res != S_SUCCESS) {
    return res;
  }

  prv_cinstr_read(dev, part->instructions.rdsr2, &sr2, 1);

  *locked = !!(sr2 & ((1U << SEC_ADDR_TO_IDX(addr)) << 3U));

  return 0;
}

status_t qspi_flash_erase_security_register(QSPIFlash *dev, uint32_t addr) {
  QSPIFlashPart *part = dev->state->part;
  status_t ret;
  uint8_t out[4];
  uint8_t len;

  ret = prv_qspi_security_register_check(dev, addr);
  if (ret != S_SUCCESS) {
    return ret;
  }

  if (part->size > ADDR_4BYTE_THRESHOLD) {
    len = 4U;
    out[0] = (addr >> 24U);
    out[1] = (addr >> 16U) & 0xFFU;
    out[2] = (addr >> 8U) & 0xFFU;
    out[3] = addr & 0xFFU;
  } else {
    len = 3U;
    out[0] = (addr >> 16U) & 0xFFU;
    out[1] = (addr >> 8U) & 0xFFU;
    out[2] = addr & 0xFFU;
  }

  prv_write_enable(dev);

  prv_cinstr_write(dev, part->instructions.erase_sec, out, len);

  while (prv_wip_check(dev)) {
  }

  return 0;
}

status_t qspi_flash_write_security_register(QSPIFlash *dev, uint32_t addr, uint8_t val) {
  QSPIFlashPart *part = dev->state->part;
  status_t ret;
  uint8_t out[5];
  uint8_t len;

  ret = prv_qspi_security_register_check(dev, addr);
  if (ret != S_SUCCESS) {
    return ret;
  }

  if (part->size > ADDR_4BYTE_THRESHOLD) {
    len = 5U;
    out[0] = (addr >> 24U);
    out[1] = (addr >> 16U) & 0xFFU;
    out[2] = (addr >> 8U) & 0xFFU;
    out[3] = addr & 0xFFU;
    out[4] = val;
  } else {
    len = 4U;
    out[0] = (addr >> 16U) & 0xFFU;
    out[1] = (addr >> 8U) & 0xFFU;
    out[2] = addr & 0xFFU;
    out[3] = val;
  }

  prv_write_enable(dev);

  prv_cinstr_write(dev, part->instructions.program_sec, out, len);

  while (prv_wip_check(dev)) {
  }

  return 0;
}

const FlashSecurityRegisters *qspi_flash_security_registers_info(QSPIFlash *dev) {
  QSPIFlashPart *part = dev->state->part;

  return &part->sec_registers;
}

#ifdef CONFIG_RECOVERY_FW
status_t qspi_flash_lock_security_register(QSPIFlash *dev, uint32_t addr) {
  uint8_t sr[2];
  status_t res;

  res = prv_qspi_security_register_check(dev, addr);
  if (res != S_SUCCESS) {
    return res;
  }

  prv_read_sr1(dev, &sr[0]);
  prv_read_sr2(dev, &sr[1]);

  sr[1] |= (1U << SEC_ADDR_TO_IDX(addr)) << 3U;

  prv_write_sr(dev, sr, 2U);

  return 0;
}
#endif  // CONFIG_RECOVERY_FW

status_t qspi_flash_write_protection_enable(QSPIFlash *dev) {
  return S_NO_ACTION_REQUIRED;
}

status_t qspi_flash_lock_sector(QSPIFlash *dev, uint32_t addr) {
  return S_SUCCESS;
}

status_t qspi_flash_unlock_all(QSPIFlash *dev) {
  return S_SUCCESS;
}