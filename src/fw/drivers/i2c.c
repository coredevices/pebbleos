/*
 * Copyright 2024 Google LLC
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

#include "i2c.h"
#include "i2c_definitions.h"
#include "i2c_hal.h"

#include "board/board.h"
#include "debug/power_tracking.h"
#include "drivers/gpio.h"
#include "drivers/periph_config.h"
#include "drivers/rtc.h"
#include "FreeRTOS.h"
#include "kernel/pbl_malloc.h"
#include "os/tick.h"
#include "kernel/util/sleep.h"
#include "kernel/util/stop.h"
#include "os/mutex.h"
#include "semphr.h"
#include "services/common/analytics/analytics.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/size.h"

#if CAPABILITY_HAS_PMIC
#include "drivers/pmic.h"
#endif

#include <inttypes.h>

#define STM32F2_COMPATIBLE
#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#define NRF5_COMPATIBLE
#define SF32LB_COMPATIBLE
#include <mcu.h>

#define I2C_ERROR_TIMEOUT_MS  (1000)
#define I2C_TIMEOUT_ATTEMPTS_MAX (2 * 1000 * 1000)

// MFI NACKs while busy. We delay ~1ms between retries so this is approximately a 1000ms timeout.
// The longest operation of the MFi chip is "start signature generation", which seems to take
// 223-224 NACKs, but sometimes for unknown reasons it can take much longer.
#define I2C_NACK_COUNT_MAX    (1000)

typedef enum {
  Read,
  Write
} TransferDirection;

typedef enum {
  SendRegisterAddress,      // Send a register address, followed by a repeat start for reads
  NoRegisterAddress         // Do not send a register address
} TransferType;

static uint32_t s_max_transfer_duration_ticks;

static void prv_analytics_track_i2c_error(void);

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

#if CAPABILITY_HAS_PMIC
void i2c_rail_ctl_pmic(I2CBus *bus, bool enable) {
  set_ldo3_power_state(enable);
}
#endif

void i2c_rail_ctl_pin(I2CBus *bus, bool enable) {
  gpio_output_set(&bus->rail_gpio, enable);
}

static void prv_rail_ctl(I2CBus *bus, bool enable) {
  bus->rail_ctl_fn(bus, enable);
  if (enable) {
    // FIXME: The power tracking data is going to be bogus for any board with more than one bus
    // with controllable power.
    // https://pebbletechnology.atlassian.net/browse/PBL-32232 should address this
    power_tracking_start(PowerSystem2v5Reg);
    // wait for the bus supply to stabilize and the peripherals to start up.
    // the MFI chip requires its reset pin to be stable for at least 10ms from startup.
    psleep(20);
  } else {
    power_tracking_stop(PowerSystem2v5Reg);
  }
}

//! Power down I2C bus power supply
//! Always lock bus and peripheral config access before use
static void prv_bus_rail_power_down(I2CBus *bus) {
  if (!bus->rail_ctl_fn) {
    return;
  }
  prv_rail_ctl(bus, false);

#if MICRO_FAMILY_NRF5
  i2c_hal_pins_set_gpio(bus);
#endif

  // Drain through pull-ups
  OutputConfig out_scl = {
    .gpio = bus->scl_gpio.gpio,
    .gpio_pin = bus->scl_gpio.gpio_pin,
    .active_high = true
  };
  gpio_output_init(&out_scl, GPIO_PuPd_NOPULL, GPIO_Speed_2MHz);
  gpio_output_set(&out_scl, false);

  OutputConfig out_sda = {
    .gpio = bus->sda_gpio.gpio,
    .gpio_pin = bus->sda_gpio.gpio_pin,
    .active_high = true
  };
  gpio_output_init(&out_sda, GPIO_PuPd_NOPULL, GPIO_Speed_2MHz);
  gpio_output_set(&out_sda, false);

  bus->state->last_rail_stop_ticks = rtc_get_ticks();
}

//! Configure bus pins for use by I2C peripheral
//! Lock bus and peripheral config access before configuring pins
static void prv_bus_pins_cfg_i2c(I2CBus *bus) {
#if MICRO_FAMILY_NRF5
  i2c_hal_pins_set_i2c(bus);
#else
  gpio_af_init(&bus->scl_gpio, GPIO_OType_OD, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  gpio_af_init(&bus->sda_gpio, GPIO_OType_OD, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
#endif
}

static void prv_bus_pins_cfg_input(I2CBus *bus) {
#if MICRO_FAMILY_NRF5
  i2c_hal_pins_set_gpio(bus);
#endif

  InputConfig in_scl = {
    .gpio = bus->scl_gpio.gpio,
    .gpio_pin = bus->scl_gpio.gpio_pin,
  };
  gpio_input_init(&in_scl);

  InputConfig in_sda = {
    .gpio = bus->sda_gpio.gpio,
    .gpio_pin = bus->sda_gpio.gpio_pin,
  };
  gpio_input_init(&in_sda);
}

//! Power up I2C bus power supply
//! Always lock bus and peripheral config access before use
static void prv_bus_rail_power_up(I2CBus *bus) {
  if (!bus->rail_ctl_fn) {
    return;
  }

  static const uint32_t MIN_STOP_TIME_MS = 10;

  // check that at least enough time has elapsed since the last turn-off
  RtcTicks time_stopped_ms =
      ((rtc_get_ticks() - bus->state->last_rail_stop_ticks) * RTC_TICKS_HZ) / 1000;
  I2C_DEBUG("Bus %s rail start after a delay of %"PRIu32"ms", bus->name,
                  (uint32_t)time_stopped_ms);

  if (time_stopped_ms < MIN_STOP_TIME_MS) {
    I2C_DEBUG("Waiting %"PRIu32"ms before enabling I2C bus %s rail.",
              (uint32_t)(MIN_STOP_TIME_MS - time_stopped_ms), bus->name);
    psleep(MIN_STOP_TIME_MS - time_stopped_ms);
  }

  prv_bus_pins_cfg_input(bus);

  prv_rail_ctl(bus, true);
}

//! Configure the bus pins, enable the peripheral clock and initialize the I2C peripheral.
//! Always lock the bus and peripheral config access before enabling it
static void prv_bus_enable(I2CBus *bus) {
  // Don't power up rail if the bus is already in use (enable can be called to reset bus)
  if (bus->state->user_count ==  0) {
    prv_bus_rail_power_up(bus);
  }

  prv_bus_pins_cfg_i2c(bus);

  i2c_hal_enable(bus);
}

//! De-initialize and gate the clock to the peripheral
//! Power down rail if the bus supports that and no devices are using it
//! Always lock the bus and peripheral config access before disabling it
static void prv_bus_disable(I2CBus *bus) {
  i2c_hal_disable(bus);

  // Do not de-power rail if there are still devices using bus (just reset peripheral and pin
  // configuration during a bus reset)
  if (bus->state->user_count == 0) {
    prv_bus_rail_power_down(bus);
  } else {
    prv_bus_pins_cfg_input(bus);
  }
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

  if (bus->rail_gpio.gpio) {
    gpio_output_init(&bus->rail_gpio, GPIO_OType_PP, GPIO_Speed_2MHz);
  }
  prv_bus_rail_power_down(bus);
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
    PBL_LOG(LOG_LEVEL_ERROR, "Attempted release of disabled bus %s", slave->bus->name);
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
    PBL_LOG(LOG_LEVEL_ERROR, "Attempted reset of disabled bus %s when still in use by "
        "another bus", slave->bus->name);
    mutex_unlock(slave->bus->state->bus_mutex);
    return;
  }

  PBL_LOG(LOG_LEVEL_WARNING, "Resetting I2C bus %s", slave->bus->name);

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
  PBL_ASSERTN(slave);

  static const int MAX_TOGGLE_COUNT = 10;
  static const int TOGGLE_DELAY = 10;

  // Protect access to bus
  mutex_lock(slave->bus->state->bus_mutex);

  if (slave->bus->state->user_count == 0) {
    PBL_LOG(LOG_LEVEL_ERROR, "Attempted bitbang recovery on disabled bus %s", slave->bus->name);
    mutex_unlock(slave->bus->state->bus_mutex);
    return false;
  }

#if MICRO_FAMILY_NRF5
  i2c_hal_pins_set_gpio(slave->bus);
#endif

  InputConfig in_sda = {
    .gpio = slave->bus->sda_gpio.gpio,
    .gpio_pin = slave->bus->sda_gpio.gpio_pin,
  };
  gpio_input_init(&in_sda);

  OutputConfig out_scl = {
    .gpio = slave->bus->scl_gpio.gpio,
    .gpio_pin = slave->bus->scl_gpio.gpio_pin,
    .active_high = true
  };
  gpio_output_init(&out_scl, GPIO_PuPd_NOPULL, GPIO_Speed_2MHz);
  gpio_output_set(&out_scl, true);

  bool recovered = false;
  for (int i = 0; i < MAX_TOGGLE_COUNT; ++i) {
    gpio_output_set(&out_scl, false);
    psleep(TOGGLE_DELAY);
    gpio_output_set(&out_scl, true);
    psleep(TOGGLE_DELAY);

    if (gpio_input_read(&in_sda)) {
      recovered = true;
      break;
    }
  }
  if (recovered) {
    PBL_LOG(LOG_LEVEL_DEBUG, "I2C Bus %s recovered", slave->bus->name);
  } else {
    PBL_LOG(LOG_LEVEL_ERROR, "I2C Bus %s still hung after bitbang reset", slave->bus->name);
  }

  prv_bus_pins_cfg_i2c(slave->bus);
  prv_bus_reset(slave->bus);

  mutex_unlock(slave->bus->state->bus_mutex);

  return recovered;
}

/*--------------------DATA TRANSFER FUNCTIONS--------------------------*/

//! Wait a short amount of time for busy bit to clear
static bool prv_wait_for_not_busy(I2CBus *bus) {
  static const int WAIT_DELAY = 10; // milliseconds

  if (i2c_hal_is_busy(bus)) {
    psleep(WAIT_DELAY);
    if (i2c_hal_is_busy(bus)) {
      PBL_LOG(LOG_LEVEL_ERROR, "Timed out waiting for bus %s to become non-busy", bus->name);
      return false;
    }
  }

  return true;
}

//! Set up and start a transfer to a bus, wait for it to finish and clean up after the transfer
//! has completed
static bool prv_do_transfer(I2CBus *bus, TransferDirection direction, uint16_t device_address,
                            uint8_t register_address, uint32_t size, uint8_t *data,
                            TransferType type) {
  // Take control of bus; only one task may use bus at a time
  mutex_lock(bus->state->bus_mutex);

  if (bus->state->user_count == 0) {
    PBL_LOG(LOG_LEVEL_ERROR, "Attempted access to disabled bus %s", bus->name);
    mutex_unlock(bus->state->bus_mutex);
    return false;
  }

  // Disable stop mode while the I2C transfer is in progress - stop mode disables I2C peripheral
  stop_mode_disable(bus->stop_mode_inhibitor);

  // If bus is busy (it shouldn't be as this function waits for the bus to report a non-idle state
  // before exiting) reset the bus and wait for it to become not-busy
  // Exit if bus remains busy. User module should reset the I2C module at this point
  if (i2c_hal_is_busy(bus)) {
    prv_bus_reset(bus);

    if (!prv_wait_for_not_busy(bus)) {
      // Bus did not recover after reset
      stop_mode_enable(bus->stop_mode_inhibitor);
      mutex_unlock(bus->state->bus_mutex);
      PBL_LOG(LOG_LEVEL_ERROR, "I2C bus did not recover after reset (%s)", bus->name);
      prv_analytics_track_i2c_error();
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
        // Track the max transfer duration so we can keep tabs on the MFi chip's nacking behavior
        const uint32_t duration_ticks = rtc_get_ticks() - bus->state->transfer_start_ticks;
        if (duration_ticks > s_max_transfer_duration_ticks) {
          s_max_transfer_duration_ticks = duration_ticks;
        }

        if (bus->state->transfer_event == I2CTransferEvent_Error) {
          PBL_LOG(LOG_LEVEL_ERROR, "I2C Error on bus %s", bus->name);
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
        PBL_LOG(LOG_LEVEL_ERROR, "I2C Error: too many NACKs received on bus %s", bus->name);
        break;
      }

    } else {
      // Timeout, abort transfer
      i2c_hal_abort_transfer(bus);
      complete = true;
      PBL_LOG(LOG_LEVEL_ERROR, "Transfer timed out on bus %s", bus->name);
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

  stop_mode_enable(bus->stop_mode_inhibitor);

  mutex_unlock(bus->state->bus_mutex);

  if (!result) {
    prv_analytics_track_i2c_error();
  }
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
    PBL_LOG(LOG_LEVEL_ERROR, "Read failed on bus %s", slave->bus->name);
  }

  return result;
}

bool i2c_read_block(I2CSlavePort *slave, uint32_t read_size, uint8_t* result_buffer) {
  PBL_ASSERTN(slave);
  PBL_ASSERTN(result_buffer);

  bool result = prv_do_transfer(slave->bus, Read, slave->address, 0, read_size, result_buffer,
                            NoRegisterAddress);

  if (!result) {
    PBL_LOG(LOG_LEVEL_ERROR, "Block read failed on bus %s", slave->bus->name);
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
    PBL_LOG(LOG_LEVEL_ERROR, "Write failed on bus %s", slave->bus->name);
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
    PBL_LOG(LOG_LEVEL_ERROR, "Block write failed on bus %s", slave->bus->name);
  }

  return result;
}

/*----------------------HAL INTERFACE--------------------------------*/

portBASE_TYPE i2c_handle_transfer_event(I2CBus *bus, I2CTransferEvent event) {
  bus->state->transfer_event = event;
  return prv_semaphore_give_from_isr(bus->state);
}

/*------------------------ANALYTICS----------------------------------*/

static void prv_analytics_track_i2c_error(void) {
  analytics_inc(ANALYTICS_DEVICE_METRIC_I2C_ERROR_COUNT, AnalyticsClient_System);
}

void analytics_external_collect_i2c_stats(void) {
  analytics_set(ANALYTICS_DEVICE_METRIC_I2C_MAX_TRANSFER_DURATION_TICKS,
                s_max_transfer_duration_ticks, AnalyticsClient_System);
  s_max_transfer_duration_ticks = 0;
}

/*------------------------COMMAND FUNCTIONS--------------------------*/
// FIXME: Move to board-specific implementations
// https://pebbletechnology.atlassian.net/browse/PBL-32232
#if PLATFORM_TINTIN
void command_power_2v5(char *arg) {
  // Intentionally ignore the s_running_count and make it so!
  // This is intended for low level electrical test only
  if (!strcmp("on", arg)) {
    prv_bus_rail_power_up(I2C_MFI->bus);
  } else {
    prv_bus_rail_power_down(I2C_MFI->bus);
  }
}
#endif
