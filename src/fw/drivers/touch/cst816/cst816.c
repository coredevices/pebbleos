/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include <pbl/drivers/exti.h>
#include <pbl/drivers/gpio.h>
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/rtc.h>
#include <pbl/drivers/touch/touch_sensor.h>
#include "kernel/events.h"
#include "kernel/util/sleep.h"
#include "pbl/os/tick.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/touch/touch.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/math.h"

#include "cst816_fw.h"

PBL_LOG_MODULE_DEFINE(driver_touch_cst816, CONFIG_DRIVER_TOUCH_LOG_LEVEL);

#define CST816_RESET_CYCLE_TIME       10  /* ms */
#define CST816_POR_DELAY_TIME         110 /* ms */
#define CST816_REG_WR_DELAY_TIME      2   /* ms */ 
#define CST816_FW_CHECKSUM_CAL_TIME   500 /* ms */ 

#define CST816_POWER_MODE_REG         0xE5
#define CST816_POWER_MODE_SLEEP       0x03
#define CST816_CHIP_ID_REG            0xA7
#define CST816_FW_VERSION_REG         0xA9
#define CST816_TOUCH_DATA_REG         0x02
#define CST816_TOUCH_DATA_SIZE        5
#define CST816_GESTURE_ID             0x01
#define CST816_GESTURE_NONE           0x00
#define CST816_GESTURE_RIGHT          0x01
#define CST816_GESTURE_LEFT           0x02
#define CST816_GESTURE_DOWN           0x03
#define CST816_GESTURE_UP             0x04
#define CST816_GESTURE_CLICK          0x05
#define CST816_GESTURE_DOUBLE_CLICK   0x0B
#define CST816_GESTURE_LONG_PRESS     0x0C

#define CST816_BOOT_MODE_REG          0xA001
#define CST816_BOOT_MODE_CMD          0xAB
#define CST816_BOOT_FLAG_REG          0xA003
#define CST816_BOOT_FLAG_VAL          0xC1
#define CST816_FW_START_ADDR_REG      0xA014
#define CST816_FW_PAGE_REG            0xA018
#define CST816_FW_PAGE_SIZE           512
#define CST816_FW_PAGE_DONE           0xA004
#define CST816_FW_PAGE_STATE          0xA005
#define CST816_BOOT_EXIT_REG          0xA006
#define CST816_BOOT_EXIT_VAL          0xEE
#define CST816_FW_PAGE_READY          0x55
#define CST816_FW_WR_TIME             100 /* ms */
#define CST816_FW_CHECKSUM_REG        0xA008
#define CST816_FW_VER_INFO_INDEX      (-11)

/* Workaround: the CST816 occasionally wedges and stops asserting its INT line.
 * If no touch activity is seen between two watchdog checks, hard-reset it. */
#define CST816_WATCHDOG_PERIOD_MIN    30

/* The chip stays awake for 2s after a wake; an interrupt seen >=2s after the
 * previous one therefore marks a fresh sleep->awake transition. */
#define CST816_WAKE_SPACING_MS        2000

static bool s_callback_scheduled = false;
static bool s_enabled = false;
static bool s_reset_scheduled = false;
static bool s_activity_since_check = false;
static RtcTicks s_last_irq_ticks = 0;
static PebbleMutex *s_i2c_lock;

static void prv_exti_cb(bool *should_context_switch);
static void cst816_hw_reset(void);
static void prv_watchdog_cb(void *data);

static RegularTimerInfo s_watchdog_timer = {
  .cb = prv_watchdog_cb,
};

static bool prv_read_data(uint16_t register_address, uint8_t *result, uint16_t size, bool is_work_mode) {
  mutex_lock(s_i2c_lock);
  I2CSlavePort* port = CST816->i2c;
  uint8_t addr_size = 1;
  if(!is_work_mode) {
    port = CST816->i2c_boot;
    addr_size = 2;
  }
  i2c_use(port);
  uint8_t regad[2] = { register_address >> 8, register_address & 0xFF };
  bool rv = i2c_write_block(port, addr_size, is_work_mode?regad+1:regad);
  if (rv) {
    rv = i2c_read_block(port, size, result);
  }
  i2c_release(port);
  mutex_unlock(s_i2c_lock);
  return rv;
}

static bool prv_write_data(uint16_t register_address, const uint8_t *datum, uint16_t size, bool is_work_mode) {
  mutex_lock(s_i2c_lock);
  I2CSlavePort* port = CST816->i2c;
  uint8_t addr_size = 1;
  if(!is_work_mode) {
    port = CST816->i2c_boot;
    addr_size = 2;
  }
  i2c_use(port);
  uint8_t data[size + sizeof(register_address)];
  data[0] = register_address >> 8;
  data[1] = register_address & 0xFF;
  memcpy(data+sizeof(register_address), datum, size);
  bool rv = i2c_write_block(port, size+addr_size, is_work_mode?data+1:data);
  i2c_release(port);
  mutex_unlock(s_i2c_lock);
  return rv;
}

static bool cst816_enter_bootmode(void) {
#if RESET_PIN_CTRLBY_NPM1300
  NPM1300_OPS.gpio_set(Npm1300_Gpio2, 0);
  psleep(CST816_RESET_CYCLE_TIME);
  NPM1300_OPS.gpio_set(Npm1300_Gpio2, 1);
  psleep(CST816_RESET_CYCLE_TIME);
#else
  gpio_output_set(&CST816->reset, true);
  psleep(CST816_RESET_CYCLE_TIME);
  gpio_output_set(&CST816->reset, false);
  psleep(CST816_RESET_CYCLE_TIME);
#endif

  uint8_t retry_cnt = 10;
  while (retry_cnt--) {
    uint8_t cmd = CST816_BOOT_MODE_CMD;
    bool rv = prv_write_data(CST816_BOOT_MODE_REG, &cmd, 1, 0);
    psleep(CST816_REG_WR_DELAY_TIME);
    rv &= prv_read_data(CST816_BOOT_FLAG_REG, &cmd, 1, 0);
    psleep(CST816_REG_WR_DELAY_TIME);

    if (cmd == CST816_BOOT_FLAG_VAL) {
      return true;
    }
  }

  return false;
}

static uint16_t cst816_read_checksum(void)
{
  uint8_t cmd = 0;
  bool rv = prv_write_data(CST816_BOOT_FLAG_REG, &cmd, 1, 0);
  psleep(CST816_FW_CHECKSUM_CAL_TIME);

  uint8_t data[2];
  rv &= prv_read_data(CST816_FW_CHECKSUM_REG, data, 2, 0);
  PBL_ASSERT(rv, "get checksum error");
  uint16_t checksum = (((uint16_t)(data[1] & 0xFF)) << 8) | data[0];

  return checksum;
}

static bool cst816_fw_update(void) {
  if (sizeof(app_bin) > 10) {
    uint16_t start_addr = (((uint16_t)(app_bin[1] & 0xFF)) << 8) | app_bin[0];
    uint16_t length = (((uint16_t)(app_bin[3] & 0xFF)) << 8) | app_bin[2];
    uint16_t checksum = (((uint16_t)(app_bin[5] & 0xFF)) << 8) | app_bin[4];
    uint16_t fw_offset = 6;
    
    while (length) {
      PBL_LOG_DBG("fw start_addr:%d length:%d", start_addr, length);
      uint8_t addr[2] = {start_addr&0xff, start_addr>>8};
      bool rv = prv_write_data(CST816_FW_START_ADDR_REG, addr, 2, 0);
      psleep(CST816_REG_WR_DELAY_TIME);
      if(!prv_write_data(CST816_FW_PAGE_REG, app_bin+fw_offset,
                        length>=CST816_FW_PAGE_SIZE?CST816_FW_PAGE_SIZE:length , 0)) {
        PBL_LOG_ERR("cst816 update fw error by iic");
        return false;
      }
      psleep(CST816_REG_WR_DELAY_TIME);
      uint8_t cmd = 0xEE;
      rv = prv_write_data(CST816_FW_PAGE_DONE, &cmd, 1, 0);
      psleep(CST816_FW_WR_TIME);
      for (int t=0;; t++) {
        if(t > 50) {
          PBL_LOG_ERR("cst816 update fw error by writing timeout");
          return false;
        }
        psleep(CST816_RESET_CYCLE_TIME);
        uint8_t ready;
        rv = prv_read_data(CST816_FW_PAGE_STATE, &ready, 1, 0);
        if (rv && ready == CST816_FW_PAGE_READY) {
          break;
        }
      }
      fw_offset += CST816_FW_PAGE_SIZE;
      start_addr += CST816_FW_PAGE_SIZE;
      length -= length>=CST816_FW_PAGE_SIZE?CST816_FW_PAGE_SIZE:length;
    }

    uint16_t checksum_read = cst816_read_checksum();
    if (checksum_read == checksum) {
      uint8_t boot_exit_cmd = CST816_BOOT_EXIT_VAL;
      bool rv = prv_write_data(CST816_BOOT_EXIT_REG, &boot_exit_cmd, 1, 0);
      if (!rv) {
        PBL_LOG_ERR("exit boot failed");
        return false;
      }

      PBL_LOG_INFO("Updated firmware to version 0x%02X (0x%04X)",
                   app_bin[sizeof(app_bin) + CST816_FW_VER_INFO_INDEX], checksum_read);

      cst816_hw_reset();
      return true;
    }
    PBL_LOG_ERR("cst816 update fw error by checksum:%x read:%x", checksum, checksum_read);
  }

  return false;
}

static void cst816_hw_reset(void) {
#ifdef RESET_PIN_CTRLBY_NPM1300
  NPM1300_OPS.gpio_set(Npm1300_Gpio2, 0);
  psleep(CST816_RESET_CYCLE_TIME);
  NPM1300_OPS.gpio_set(Npm1300_Gpio2, 1);
  psleep(CST816_POR_DELAY_TIME);
#else
  gpio_output_set(&CST816->reset, true);
  psleep(CST816_RESET_CYCLE_TIME);
  gpio_output_set(&CST816->reset, false);
  psleep(CST816_POR_DELAY_TIME);
#endif
}

void touch_sensor_init(void) {
  uint8_t chip_id;
  uint8_t fw_version;
  bool rv;

  s_i2c_lock = mutex_create();

#ifndef RESET_PIN_CTRLBY_NPM1300
  gpio_output_init(&CST816->reset, GPIO_OType_PP);
#endif

  cst816_hw_reset();

  rv = prv_read_data(CST816_CHIP_ID_REG, &chip_id, 1, 1);
  if (!rv) {
    PBL_LOG_ERR("Could not read CST816 chip ID");
  } else {
    rv = prv_read_data(CST816_FW_VERSION_REG, &fw_version, 1, 1);
    if (!rv) {
      PBL_LOG_ERR("Could not read CST816 firmware version");
    } else {
      PBL_LOG_DBG("CST816 firmware: 0x%02X", fw_version);
    }
  }

  uint8_t target_ver = app_bin[sizeof(app_bin) + CST816_FW_VER_INFO_INDEX];

  // A chip stranded in boot mode by an interrupted update stops answering at
  // the work-mode address; only a reflash brings it back, so attempt the
  // update even when the probe above failed.
  if (!rv || target_ver != fw_version) {
    if (!cst816_enter_bootmode()) {
      PBL_LOG_ERR("Could not enter CST816 boot mode");
      return;
    }
    rv = cst816_fw_update();
    if (!rv) {
      return;
    }
  }

  // initialize exti
  exti_configure_pin(CST816->int_exti, ExtiTrigger_Falling, prv_exti_cb);

  touch_sensor_set_enabled(false);
}

static void prv_process_pending_messages(void* context) {
  bool rv;
  s_callback_scheduled = false;

  // Any interrupt means the chip is alive; pet the idle watchdog.
  s_activity_since_check = true;

  // Count interrupts spaced >=2s apart as sleep->awake transitions.
  RtcTicks now = rtc_get_ticks();
  if (now - s_last_irq_ticks >= milliseconds_to_ticks(CST816_WAKE_SPACING_MS)) {
    PBL_ANALYTICS_ADD(touch_driver_wake_cnt, 1);
  }
  s_last_irq_ticks = now;

  uint8_t id;
  rv = prv_read_data(CST816_GESTURE_ID, &id, 1, 1);
  if (!rv) {
    PBL_LOG_ERR("Failed to read gesture ID, trying to recover");
    touch_handle_update(TouchState_FingerUp, 0, 0);
    exti_disable(CST816->int_exti);
    touch_sensor_set_enabled(true);
    return;
  }

  uint8_t data[CST816_TOUCH_DATA_SIZE] = {0};
  rv = prv_read_data(CST816_TOUCH_DATA_REG, data, CST816_TOUCH_DATA_SIZE, 1);
  if (!rv) {
    PBL_LOG_ERR("Failed to read touch data, trying to recover");
    touch_handle_update(TouchState_FingerUp, 0, 0);
    exti_disable(CST816->int_exti);
    touch_sensor_set_enabled(true);
    return;
  }

  uint8_t press = data[0] & 0x0F;
  GPoint point = {
    .x = (((uint16_t)(data[1] & 0x0F)) << 8) | data[2],
    .y = (((uint16_t)(data[3] & 0X0F)) << 8) | data[4],
  };

  if (CST816->invert_x_axis) {
    point.x = CST816->max_x - point.x;
  }

  if (CST816->invert_y_axis) {
    point.y = CST816->max_y - point.y;
  }

  switch (id) {
    case CST816_GESTURE_CLICK:
      touch_handle_gesture(TouchGesture_Tap, point.x, point.y);
      break;
    case CST816_GESTURE_DOUBLE_CLICK:
      touch_handle_gesture(TouchGesture_DoubleTap, point.x, point.y);
      break;
    default:
      break;
  }

  if (press == 0x01) {
    touch_handle_update(TouchState_FingerDown, point.x, point.y);
  } else {
    touch_handle_update(TouchState_FingerUp, point.x, point.y);
  }
}

static void prv_exti_cb(bool *should_context_switch) {
  if (s_callback_scheduled) {
    return;
  }

  system_task_add_callback_from_isr(prv_process_pending_messages, NULL, should_context_switch);
  s_callback_scheduled = true;
}

// Runs on the system task: the actual recovery reset (cst816_hw_reset() sleeps
// ~120ms, so it must not run on the regular-timer task).
static void prv_idle_reset_worker(void *context) {
  s_reset_scheduled = false;
  if (!s_enabled) {
    return;
  }

  exti_disable(CST816->int_exti);
  cst816_hw_reset();
  s_callback_scheduled = false;
  s_activity_since_check = true;
  exti_enable(CST816->int_exti);
}

// Runs on the regular-timer (NewTimers) task; keep it cheap, just offload.
static void prv_watchdog_cb(void *data) {
  if (!s_enabled || s_reset_scheduled) {
    return;
  }

  if (s_activity_since_check) {
    s_activity_since_check = false;
    return;
  }

  s_reset_scheduled = true;
  system_task_add_callback(prv_idle_reset_worker, NULL);
}

void touch_sensor_set_enabled(bool enabled) {
  cst816_hw_reset();

  if (enabled) {
    exti_enable(CST816->int_exti);
    s_enabled = true;
    s_activity_since_check = true;
    if (!regular_timer_is_scheduled(&s_watchdog_timer)) {
      regular_timer_add_multiminute_callback(&s_watchdog_timer, CST816_WATCHDOG_PERIOD_MIN);
    }
  } else {
    s_enabled = false;
    if (regular_timer_is_scheduled(&s_watchdog_timer)) {
      regular_timer_remove_callback(&s_watchdog_timer);
    }
    uint8_t data = CST816_POWER_MODE_SLEEP;
    prv_write_data(CST816_POWER_MODE_REG, &data, 1, 1);
    exti_disable(CST816->int_exti);
  }
}
