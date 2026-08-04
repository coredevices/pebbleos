/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/i2c.h>
#include <pbl/drivers/i2c/definitions.h>
#include <pbl/drivers/i2c/hal.h>

#include "board/board.h"
#include "debug/power_tracking.h"
#include "pbl/services/analytics/analytics.h"
#include <pbl/drivers/gpio.h>
#include <pbl/drivers/rtc.h>
#include "FreeRTOS.h"
#include "kernel/pbl_malloc.h"
#include "pbl/os/tick.h"
#include "kernel/util/sleep.h"
#include "pbl/os/mutex.h"
#include "semphr.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/size.h"

#ifdef CONFIG_PMIC
#include <pbl/drivers/pmic.h>
#endif

#include <inttypes.h>

PBL_LOG_MODULE_DEFINE(driver_i2c, CONFIG_DRIVER_I2C_LOG_LEVEL);

#define I2C_ERROR_TIMEOUT_MS  (1000)
#define I2C_TIMEOUT_ATTEMPTS_MAX (2 * 1000 * 1000)

// MFI NACKs while busy. We delay ~1ms between retries so this is approximately a 1000ms timeout.
// The longest operation of the MFi chip is "start signature generation", which seems to take
// 223-224 NACKs, but sometimes for unknown reasons it can take much longer.
#define I2C_NACK_COUNT_MAX    (1000)

#define Read I2CTransferDirection_Read
#define Write I2CTransferDirection_Write
#define SendRegisterAddress I2CTransferType_SendRegisterAddress
#define NoRegisterAddress I2CTransferType_NoRegisterAddress

/*----------------SEMAPHORE/LOCKING FUNCTIONS--------------------------*/

static bool prv_semaphore_take(I2CBusState *bus) {
  return (xSemaphoreTake(bus->event_semaphore, 0) == pdPASS);
}

static bool prv_semaphore_wait(I2CBusState *bus) {
  TickType_t timeout_ticks = milliseconds_to_ticks(I2C_ERROR_TIMEOUT_MS);
  return (xSemaphoreTake(bus->event_semaphore, timeout_ticks) == pdPASS);
}

static void prv_semaphore_give(I2CBusState *bus) {
  // If this fails, something is very wrong
  (void)xSemaphoreGive(bus->event_semaphore);
}

static portBASE_TYPE prv_semaphore_give_from_isr(I2CBusState *bus) {
  portBASE_TYPE should_context_switch = pdFALSE;
  (void)xSemaphoreGiveFromISR(bus->event_semaphore,  &should_context_switch);
  return should_context_switch;
}

/*-------------------BUS/PIN CONFIG FUNCTIONS--------------------------*/
// FIXME: These rail control functions should be moved to board-specific implementations
// https://pebbletechnology.atlassian.net/browse/PBL-32232

#ifdef CONFIG_PMIC
void i2c_rail_ctl_pmic(I2CBus *bus, bool enable) {
  set_ldo3_power_state(enable);
}
#endif

//! Configure the bus pins, enable the peripheral clock and initialize the I2C peripheral.
//! Always lock the bus and peripheral config access before enabling it
static void prv_bus_enable(I2CBus *bus) {
  i2c_hal_enable(bus);
}

//! De-initialize and gate the clock to the peripheral
//! Power down rail if the bus supports that and no devices are using it
//! Always lock the bus and peripheral config access before disabling it
static void prv_bus_disable(I2CBus *bus) {
  i2c_hal_disable(bus);
}

//! Perform a soft reset of the bus
//! Always lock the bus before reset
static void prv_bus_reset(I2CBus *bus) {
  prv_bus_disable(bus);
  prv_bus_enable(bus);
}

/*---------------INIT/USE/RELEASE/RESET FUNCTIONS----------------------*/

void i2c_init(I2CBus *bus) {
  PBL_ASSERTN(bus);

  *bus->state = (I2CBusState) {
    .event_semaphore = xSemaphoreCreateBinary(),
    .bus_mutex = mutex_create(),
  };

  // Must give token before one can be taken without blocking
  xSemaphoreGive(bus->state->event_semaphore);

  i2c_hal_init(bus);
}

void i2c_use(I2CSlavePort *slave) {
  PBL_ASSERTN(slave);
  mutex_lock(slave->bus->state->bus_mutex);

  if (slave->bus->state->user_count == 0) {
    prv_bus_enable(slave->bus);
  }
  slave->bus->state->user_count++;

  mutex_unlock(slave->bus->state->bus_mutex);
}

void i2c_release(I2CSlavePort *slave) {
  PBL_ASSERTN(slave);
  mutex_lock(slave->bus->state->bus_mutex);

  if (slave->bus->state->user_count == 0) {
    PBL_LOG_ERR("Attempted release of disabled bus %s", slave->bus->name);
    mutex_unlock(slave->bus->state->bus_mutex);
    return;
  }

  slave->bus->state->user_count--;
  if (slave->bus->state->user_count == 0) {
    prv_bus_disable(slave->bus);
  }

  mutex_unlock(slave->bus->state->bus_mutex);
}

void i2c_reset(I2CSlavePort *slave) {
  PBL_ASSERTN(slave);

  // Take control of bus; only one task may use bus at a time
  mutex_lock(slave->bus->state->bus_mutex);

  if (slave->bus->state->user_count == 0) {
    PBL_LOG_ERR("Attempted reset of disabled bus %s when still in use by "
        "another bus", slave->bus->name);
    mutex_unlock(slave->bus->state->bus_mutex);
    return;
  }

  PBL_LOG_WRN("Resetting I2C bus %s", slave->bus->name);

  // decrement user count for reset so that if this user is the only user, the
  // bus will be powered down during the reset
  slave->bus->state->user_count--;

  // Reset and reconfigure bus and pins
  prv_bus_reset(slave->bus);

  // Restore user count
  slave->bus->state->user_count++;

  mutex_unlock(slave->bus->state->bus_mutex);
}

bool i2c_bitbang_recovery(I2CSlavePort *slave) {
  PBL_LOG_ERR("I2C bitbang recovery not supported on this platform");
  return false;
}

/*--------------------DATA TRANSFER FUNCTIONS--------------------------*/

//! Wait a short amount of time for busy bit to clear
static bool prv_wait_for_not_busy(I2CBus *bus) {
  static const int WAIT_DELAY = 10; // milliseconds

  if (i2c_hal_is_busy(bus)) {
    psleep(WAIT_DELAY);
    if (i2c_hal_is_busy(bus)) {
      PBL_LOG_ERR("Timed out waiting for bus %s to become non-busy", bus->name);
      return false;
    }
  }

  return true;
}

//! Set up and start a transfer to a bus, wait for it to finish and clean up after the transfer
//! has completed
//! Caller must hold bus mutex
static bool prv_do_transfer_locked(I2CBus *bus, I2CTransferDirection direction, uint16_t device_address,
                                   uint8_t register_address, uint32_t size, uint8_t *data,
                                   I2CTransferType type) {
  if (bus->state->user_count == 0) {
    PBL_LOG_ERR("Attempted access to disabled bus %s", bus->name);
    return false;
  }

  // If bus is busy (it shouldn't be as this function waits for the bus to report a non-idle state
  // before exiting) reset the bus and wait for it to become not-busy
  // Exit if bus remains busy. User module should reset the I2C module at this point
  if (i2c_hal_is_busy(bus)) {
    prv_bus_reset(bus);

    if (!prv_wait_for_not_busy(bus)) {
      // Bus did not recover after reset
      PBL_LOG_ERR("I2C bus did not recover after reset (%s)", bus->name);
      return false;
    }
  }

  // Take binary semaphore so that next take will block
  PBL_ASSERT(prv_semaphore_take(bus->state), "Could not acquire semaphore token");

  // Set up transfer
  bus->state->transfer = (I2CTransfer) {
    .device_address = device_address,
    .register_address = register_address,
    .direction = direction,
    .type = type,
    .size = size,
    .idx = 0,
    .data = data,
  };

  i2c_hal_init_transfer(bus);

  bus->state->transfer_nack_count = 0;
  bus->state->transfer_start_ticks = rtc_get_ticks();

  bool result = false;
  bool complete = false;
  do {
    i2c_hal_start_transfer(bus);

    // Wait on semaphore until it is released by interrupt or a timeout occurs
    if (prv_semaphore_wait(bus->state)) {
      if ((bus->state->transfer_event == I2CTransferEvent_TransferComplete) ||
          (bus->state->transfer_event == I2CTransferEvent_Error)) {
        if (bus->state->transfer_event == I2CTransferEvent_Error) {
          PBL_LOG_ERR("I2C Error on bus %s", bus->name);
          PBL_ANALYTICS_ADD(i2c_transfer_error_count, 1);
        }
        complete = true;
        result = (bus->state->transfer_event == I2CTransferEvent_TransferComplete);
      } else if (bus->state->transfer_nack_count < I2C_NACK_COUNT_MAX) {
        // NACK received after start condition sent: the MFI chip NACKs start conditions whilst it
        // is busy
        // Retry start condition after a short delay.
        // A NACK count is incremented for each NACK received, so that legitimate NACK
        // errors cause the transfer to be aborted (after the NACK count max has been reached).

        bus->state->transfer_nack_count++;

        // Wait 1-2ms:
        psleep(2);

      } else {
        // Too many NACKs received, abort transfer
        i2c_hal_abort_transfer(bus);
        complete = true;
        PBL_LOG_ERR("I2C Error: too many NACKs received on bus %s", bus->name);
        break;
      }

    } else {
      // Timeout, abort transfer
      i2c_hal_abort_transfer(bus);
      complete = true;
      PBL_LOG_ERR("Transfer timed out on bus %s", bus->name);
      PBL_ANALYTICS_ADD(i2c_transfer_error_count, 1);
      break;
    }
  } while (!complete);

  // Return semaphore token so another transfer can be started
  prv_semaphore_give(bus->state);

  // Wait for bus to to clear the busy flag before a new transfer starts
  // Theoretically a transfer could complete successfully, but the busy flag never clears,
  // which would cause the next transfer to fail
  if (!prv_wait_for_not_busy(bus)) {
    // Reset I2C bus if busy flag does not clear
    prv_bus_reset(bus);
  }

  return result;
}

//! Wrapper that manages locking for prv_do_transfer_locked
static bool prv_do_transfer(I2CBus *bus, I2CTransferDirection direction, uint16_t device_address,
                            uint8_t register_address, uint32_t size, uint8_t *data,
                            I2CTransferType type) {
  mutex_lock(bus->state->bus_mutex);

  bool result = prv_do_transfer_locked(bus, direction, device_address, register_address, size,
                                       data, type);

  mutex_unlock(bus->state->bus_mutex);

  return result;
}

bool i2c_read_register(I2CSlavePort *slave, uint8_t register_address, uint8_t *result) {
  return i2c_read_register_block(slave, register_address, 1, result);
}

bool i2c_read_register_block(I2CSlavePort *slave,  uint8_t register_address_start,
                             uint32_t read_size, uint8_t* result_buffer) {
  PBL_ASSERTN(slave);
  PBL_ASSERTN(result_buffer);
  // Do transfer locks the bus
  bool result = prv_do_transfer(slave->bus, Read, slave->address, register_address_start, read_size,
                                result_buffer, SendRegisterAddress);

  if (!result) {
    PBL_LOG_ERR("Read failed on bus %s", slave->bus->name);
  }

  return result;
}

bool i2c_read_block(I2CSlavePort *slave, uint32_t read_size, uint8_t* result_buffer) {
  PBL_ASSERTN(slave);
  PBL_ASSERTN(result_buffer);

  bool result = prv_do_transfer(slave->bus, Read, slave->address, 0, read_size, result_buffer,
                            NoRegisterAddress);

  if (!result) {
    PBL_LOG_ERR("Block read failed on bus %s", slave->bus->name);
  }

  return result;
}

bool i2c_write_register(I2CSlavePort *slave, uint8_t register_address, uint8_t value) {
  return i2c_write_register_block(slave, register_address, 1, &value);
}

bool i2c_write_register_block(I2CSlavePort *slave, uint8_t register_address_start,
                              uint32_t write_size, const uint8_t* buffer) {
  PBL_ASSERTN(slave);
  PBL_ASSERTN(buffer);
  // Do transfer locks the bus
  bool result = prv_do_transfer(slave->bus, Write, slave->address, register_address_start,
                                write_size, (uint8_t*)buffer, SendRegisterAddress);

  if (!result) {
    PBL_LOG_ERR("Write failed on bus %s", slave->bus->name);
  }

  return result;
}

bool i2c_write_block(I2CSlavePort *slave, uint32_t write_size, const uint8_t* buffer) {
  PBL_ASSERTN(slave);
  PBL_ASSERTN(buffer);

  // Do transfer locks the bus
  bool result = prv_do_transfer(slave->bus, Write, slave->address, 0, write_size, (uint8_t*)buffer,
                                NoRegisterAddress);

  if (!result) {
    PBL_LOG_ERR("Block write failed on bus %s", slave->bus->name);
  }

  return result;
}

bool i2c_write_read_block(I2CSlavePort *slave, uint32_t write_size, const uint8_t* write_buffer,
                          uint32_t read_size, uint8_t* read_buffer) {
  PBL_ASSERTN(slave);
  PBL_ASSERTN(write_buffer);
  PBL_ASSERTN(read_buffer);

  I2CBus *bus = slave->bus;

  // Take control of bus; only one task may use bus at a time
  mutex_lock(bus->state->bus_mutex);

  // Perform write transfer
  bool result = prv_do_transfer_locked(bus, Write, slave->address, 0, write_size,
                                       (uint8_t*)write_buffer, NoRegisterAddress);

  // Only proceed with read if write succeeded
  if (result) {
    result = prv_do_transfer_locked(bus, Read, slave->address, 0, read_size,
                                    read_buffer, NoRegisterAddress);
  }

  mutex_unlock(bus->state->bus_mutex);

  if (!result) {
    PBL_LOG_ERR("Write-read block failed on bus %s", bus->name);
  }

  return result;
}

/*----------------------HAL INTERFACE--------------------------------*/

portBASE_TYPE i2c_handle_transfer_event(I2CBus *bus, I2CTransferEvent event) {
  bus->state->transfer_event = event;
  return prv_semaphore_give_from_isr(bus->state);
}
