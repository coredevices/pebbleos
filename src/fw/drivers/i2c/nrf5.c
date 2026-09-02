/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/i2c/hal.h>
#include <pbl/drivers/i2c/definitions.h>
#include <pbl/drivers/i2c/nrf5.h>

#include "system/passert.h"

#include "FreeRTOS.h"
#include "pbl/kernel/sem.h"

#include <nrfx.h>

#include <string.h>

#define I2C_IRQ_PRIORITY (0xc)
#define I2C_NORMAL_MODE_CLOCK_SPEED_MAX   (100000)
#define I2C_READ_WRITE_BIT    (0x01)

// Register address + payload buffered for single-transfer register writes.
// Register writes are small; a larger payload fails the transfer.
#define I2C_REG_WRITE_BUF_SIZE 16
static uint8_t s_reg_write_buf[NRFX_TWIM_ENABLED_COUNT][I2C_REG_WRITE_BUF_SIZE];

static void prv_twim_evt_handler(nrfx_twim_evt_t const *evt, void *ctx) {
  I2CBus *bus = (I2CBus *) ctx;
  bool success = evt->type == NRFX_TWIM_EVT_DONE;
  I2CTransferEvent event = success ? I2CTransferEvent_TransferComplete : I2CTransferEvent_Error;
  bool should_csw = i2c_handle_transfer_event(bus, event);
  portEND_SWITCHING_ISR(should_csw);
}

static void prv_twim_init(I2CBus *bus) {
  nrfx_twim_config_t config = NRFX_TWIM_DEFAULT_CONFIG(
    bus->scl_gpio.gpio_pin, bus->sda_gpio.gpio_pin);
  config.frequency = bus->hal->frequency;
  config.hold_bus_uninit = true;
  
  nrfx_err_t err = nrfx_twim_init(&bus->hal->twim, &config, prv_twim_evt_handler, (void *)bus);
  PBL_ASSERTN(err == NRFX_SUCCESS);
}

void i2c_hal_init(I2CBus *bus) {
  prv_twim_init(bus); 
  nrfx_twim_uninit(&bus->hal->twim);
}

void i2c_hal_enable(I2CBus *bus) {
  prv_twim_init(bus); 
  nrfx_twim_enable(&bus->hal->twim);
}

void i2c_hal_disable(I2CBus *bus) {
  nrfx_twim_disable(&bus->hal->twim);
  nrfx_twim_uninit(&bus->hal->twim);
}

bool i2c_hal_is_busy(I2CBus *bus) {
  return nrfx_twim_is_busy(&bus->hal->twim);
}

void i2c_hal_abort_transfer(I2CBus *bus) {
  nrfx_twim_disable(&bus->hal->twim);
  nrfx_twim_enable(&bus->hal->twim);
}

void i2c_hal_init_transfer(I2CBus *bus) {
}

void i2c_hal_start_transfer(I2CBus *bus) {
  nrfx_twim_xfer_desc_t desc;
  I2CTransfer *transfer = &bus->state->transfer;

  desc.address = transfer->device_address >> 1;
  if (transfer->type == I2CTransferType_SendRegisterAddress) {
    if (transfer->direction == I2CTransferDirection_Read) {
      desc.type = NRFX_TWIM_XFER_TXRX;
      desc.primary_length = 1;
      desc.p_primary_buf = &transfer->register_address;
      desc.secondary_length = transfer->size;
      desc.p_secondary_buf = transfer->data;
    } else {
      // Register write: send the address and data as one contiguous transfer.
      // The two-buffer write (TXTX) sets up its data phase mid-transfer and can
      // silently drop it depending on the calling context, so never use it.
      if (transfer->size + 1U > I2C_REG_WRITE_BUF_SIZE) {
        // Payload does not fit the combined-write buffer; fail the transfer.
        bus->state->transfer_event = I2CTransferEvent_Error;
        pbl_sem_give(&bus->state->event_semaphore);
        return;
      }
      uint8_t *wbuf = s_reg_write_buf[bus->hal->twim.drv_inst_idx];
      wbuf[0] = transfer->register_address;
      memcpy(&wbuf[1], transfer->data, transfer->size);
      desc.type = NRFX_TWIM_XFER_TX;
      desc.primary_length = transfer->size + 1;
      desc.p_primary_buf = wbuf;
      desc.secondary_length = 0;
    }
  } else {
    if (transfer->direction == I2CTransferDirection_Read) {
      desc.type = NRFX_TWIM_XFER_RX;
    } else {
      desc.type = NRFX_TWIM_XFER_TX;
    }
    desc.primary_length = transfer->size;
    desc.p_primary_buf = transfer->data;
    desc.secondary_length = 0;
  }
  
  nrfx_err_t rv = nrfx_twim_xfer(&bus->hal->twim, &desc, 0);
  PBL_ASSERTN(rv == NRFX_SUCCESS);
}
