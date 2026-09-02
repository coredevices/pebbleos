/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/kernel/sem.h"
#include <pbl/drivers/rtc.h>
#include "pbl/kernel/mutex.h"

#include "portmacro.h"

#include <stdint.h>

typedef enum I2CTransferEvent {
  I2CTransferEvent_Timeout,
  I2CTransferEvent_TransferComplete,
  I2CTransferEvent_NackReceived,
  I2CTransferEvent_Error,
} I2CTransferEvent;

typedef enum {
  I2CTransferDirection_Read,
  I2CTransferDirection_Write
} I2CTransferDirection;

typedef enum {
  // Send a register address, followed by a repeat start for reads
  I2CTransferType_SendRegisterAddress,

  // Do not send a register address; used for block writes/reads
  I2CTransferType_NoRegisterAddress
} I2CTransferType;

typedef enum I2CTransferState {
  I2CTransferState_WriteAddressTx,
  I2CTransferState_WriteRegAddress,
  I2CTransferState_RepeatStart,
  I2CTransferState_WriteAddressRx,
  I2CTransferState_WaitForData,
  I2CTransferState_ReadData,
  I2CTransferState_WriteData,
  I2CTransferState_EndWrite,
  I2CTransferState_Complete,
} I2CTransferState;

typedef struct I2CTransfer {
  I2CTransferState state;
  uint16_t device_address;
  I2CTransferDirection direction;
  I2CTransferType type;
  uint8_t register_address;
  uint32_t size;
  uint32_t idx;
  uint8_t *data;
} I2CTransfer;

typedef struct I2CBusState {
  I2CTransfer transfer;
  I2CTransferEvent transfer_event;
  int transfer_nack_count;
  RtcTicks transfer_start_ticks;
  int user_count;
  struct pbl_sem event_semaphore;
  struct pbl_mutex bus_mutex;
} I2CBusState;

struct I2CBus {
  I2CBusState *const state;
  const struct I2CBusHal *const hal;
#ifdef CONFIG_SOC_NRF52
  AfConfig scl_gpio;  ///< Alternate Function configuration for SCL pin
  AfConfig sda_gpio;  ///< Alternate Function configuration for SDA pin
#endif
  const char *name;  //! Device ID for logging purposes
};

struct I2CSlavePort {
  const I2CBus *bus;
  uint16_t address;
};

//! Initialize the I2C driver.
void i2c_init(I2CBus *bus);

//! Transfer event handler implemented in i2c.c and called by HAL implementation
portBASE_TYPE i2c_handle_transfer_event(I2CBus *device, I2CTransferEvent event);
