/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>

#include "debug/power_tracking.h"

#include "board/board.h"

#include "console/dbgserial.h"
#include "console/dbgserial_input.h"
#include "console/pulse.h"

#include <pbl/drivers/rtc.h>
#include <pbl/drivers/flash.h>
#include <pbl/drivers/debounced_button.h>

#include <pbl/drivers/accel.h>
#include <pbl/drivers/ambient_light.h>
#include <pbl/drivers/backlight.h>
#include <pbl/drivers/battery.h>
#include <pbl/drivers/display/display.h>
#include <pbl/drivers/hrm.h>
#include <pbl/drivers/mag.h>
#include <pbl/drivers/mic.h>
#include <pbl/drivers/otp.h>
#include <pbl/drivers/pmic.h>
#include <pbl/drivers/pressure.h>
#include <pbl/drivers/task_watchdog.h>
#include <pbl/drivers/temperature.h>
#include <pbl/drivers/touch/touch_sensor.h>
#include <pbl/drivers/vibe.h>
#include <pbl/drivers/voltage_monitor.h>
#include <pbl/drivers/watchdog.h>
#include <pbl/drivers/sf32lb52/rc10k.h>

#include "resource/resource.h"
#include "resource/system_resource.h"

#include "kernel/util/task_init.h"
#include "kernel/events.h"
#include "kernel/kernel_heap.h"
#include "kernel/fault_handling.h"
#include "kernel/memory_layout.h"
#include "logging/pulse_logging.h"
#include "pbl/services/services.h"
#include "pbl/services/boot_splash.h"
#include "pbl/services/clock.h"
#include "pbl/services/compositor/compositor.h"
#include "pbl/services/regular_timer.h"
#include "pbl/services/system_task.h"
#include "pbl/services/new_timer/new_timer_service.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/prf_update.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/kernel_applib_state.h"
#include "kernel/util/delay.h"
#include "util/mbuf.h"
#include "system/firmware_storage.h"
#include "system/version.h"

#include "kernel/event_loop.h"

#include "console/serial_console.h"
#include "system/bootbits.h"
#include <pbl/logging/logging.h>

#include "debug/debug.h"

#include "pbl/kernel/sched.h"
#include "pbl/kernel/thread.h"

#include "mfg/mfg_info.h"
#include "mfg/mfg_serials.h"

#include <bluetooth/init.h>

void soc_early_init(void);

static TimerID s_lowpower_timer = TIMER_INVALID_ID;
#ifndef CONFIG_MFG
static TimerID s_uptime_timer = TIMER_INVALID_ID;
#endif
static void main_task(void *parameter);

static void print_splash_screen(void)
{

#if defined(CONFIG_MFG)
  PBL_LOG_ALWAYS("PebbleOS - MANUFACTURING MODE");
#elif defined(CONFIG_RECOVERY_FW)
  PBL_LOG_ALWAYS("PebbleOS - RECOVERY MODE");
#else
  PBL_LOG_ALWAYS("PebbleOS");
#endif
  PBL_LOG_ALWAYS("%s%s",
          TINTIN_METADATA.version_tag,
          (TINTIN_METADATA.is_dual_slot && !TINTIN_METADATA.is_recovery_firmware) ?
            (TINTIN_METADATA.is_slot_0 ? " (slot0)" : " (slot1)") :
            "");
  PBL_LOG_ALWAYS("(c) 2013-2026 The PebbleOS contributors");
  PBL_LOG_ALWAYS(" ");
}

int main(void) {
  soc_early_init();

  extern void * __ISR_VECTOR_TABLE__;  // Defined in linker script
  SCB->VTOR = (uint32_t)&__ISR_VECTOR_TABLE__;

  NVIC_SetPriorityGrouping(3); // 4 bits for group priority; 0 bits for subpriority

  enable_fault_handlers();

  kernel_heap_init();

  mbuf_init();
  delay_init();
  dbgserial_init();
  pulse_early_init();
  print_splash_screen();

  rtc_init();

#ifdef CONFIG_RECOVERY_FW
  boot_bit_clear(BOOT_BIT_RECOVERY_START_IN_PROGRESS);
#endif

  extern uint32_t __kernel_main_stack_start__[];
  extern uint32_t __kernel_main_stack_size__[];
  extern uint32_t __stack_guard_size__[];
  struct pbl_thread_attr attr = {
    .name = "KernelMain",
    .entry = main_task,
    .prio = PBL_PRIO_IDLE + 3,
    .privileged = true,
    .stack = (void *)((uintptr_t)__kernel_main_stack_start__ + (uintptr_t)__stack_guard_size__),
    .stack_size = (uintptr_t)__kernel_main_stack_size__ - (uintptr_t)__stack_guard_size__,
  };

  pebble_task_create(PebbleTask_KernelMain, &attr);

  pbl_kernel_start();
}

static void watchdog_timer_callback(void* data) {
  task_watchdog_bit_set(PebbleTask_NewTimers);
}

static void register_system_timers(void) {
  static RegularTimerInfo watchdog_timer = { .list_node = { 0, 0 }, .cb = watchdog_timer_callback };
  regular_timer_add_seconds_callback(&watchdog_timer);
}

static void init_drivers(void) {
  board_init();

  // The dbgserial input support requires timer support, so it is initialized here, much later
  // than the core dbgserial_init().
  dbgserial_input_init();

  serial_console_init();

#ifdef HAS_DRIVER_VOLTAGE_MONITOR
  voltage_monitor_init();
#endif

  battery_init();
  vibe_init();

#ifdef CONFIG_PMIC
  pmic_init();
#endif

  flash_init();
  flash_sleep_when_idle(true);
  flash_enable_write_protection();
  flash_prf_set_protection(true);

  uint8_t vibe_cali = mfg_info_get_vibe_cali();
  if (vibe_cali != MFG_INFO_VIBE_CALI_INVALID) {
    vibe_apply_calibration(vibe_cali);
  }

#ifdef CONFIG_MIC
  mic_init(MIC);
#endif

#ifdef CONFIG_TOUCH
  touch_sensor_init();
#endif

  accel_init();
#ifdef CONFIG_MAG
  mag_init();
#endif
#ifdef CONFIG_PRESSURE
  pressure_init();
#endif

  backlight_init();
  ambient_light_init();

  temperature_init();

  rtc_init_timers();
  rtc_alarm_init();

  power_tracking_init();
}

static void clear_reset_loop_detection_bits(void) {
  boot_bit_clear(BOOT_BIT_RESET_LOOP_DETECT_ONE);
  boot_bit_clear(BOOT_BIT_RESET_LOOP_DETECT_TWO);
  boot_bit_clear(BOOT_BIT_RESET_LOOP_DETECT_THREE);
}

#ifndef CONFIG_MFG
static void uptime_callback(void* data) {
  PBL_LOG_VERBOSE("Uptime reached 15 minutes, set stable bit.");
  new_timer_delete(s_uptime_timer);
  boot_bit_set(BOOT_BIT_FW_STABLE);
}
#endif

static void prv_low_power_debug_config_callback(void* data) {
  new_timer_delete(s_lowpower_timer);
}

static NOINLINE void prv_main_task_init(void) {
  // The Snowy bootloader does not clear the watchdog flag itself. Clear the
  // flag ourselves so that a future safe reset does not look like a watchdog
  // reset to the bootloader.
  static McuRebootReason s_mcu_reboot_reason;
  s_mcu_reboot_reason = watchdog_clear_reset_flag();

#ifdef CONFIG_PULSE_EVERYWHERE
  pulse_init();
  pulse_logging_init();
#endif

  pebble_task_configure_idle_task();

  task_init();

  memory_layout_setup_mpu();

  board_early_init();

  boot_splash_start();

  kernel_applib_init();

  system_task_init();

  events_init();

  new_timer_service_init();
  regular_timer_init();

  // Initialize the task watchdog and immediately pause it for 30 seconds to
  // give us time to initialize everything without worrying about task watchdog
  // from firing if we block other tasks.
  task_watchdog_init();
  task_watchdog_pause(30);

  pbl_analytics_init();
  register_system_timers();
  system_task_timer_init();

  init_drivers();

  clock_init();

#if defined(CONFIG_IS_BIGBOARD)
  // Program a random S/N into the Bigboard in case it's not been done yet:
  mfg_write_bigboard_serial_number();
#endif

#if defined(CONFIG_MFG)
  mfg_info_update_constant_data();
#endif

  debug_init(s_mcu_reboot_reason);

  services_early_init();

  debug_print_last_launched_app();

  // Do this early before things can screw ith it.
  check_prf_update();

#if defined(CONFIG_PBLBOOT) && defined(CONFIG_RECOVERY_FW) && !defined(CONFIG_MFG)
  // Invalidate slot0/1 when booting PRF, so we force main firmware re-install
  firmware_storage_invalidate_firmware_slot(0);
  firmware_storage_invalidate_firmware_slot(1);
#endif

  // When there are new system resources waiting to be installed, this call
  // will actually install them:
  resource_init();

  system_resource_init();

#ifdef CONFIG_HRM
  hrm_init(HRM);
#endif

  // The display has to be initialized before bluetooth because on Snowy the
  // display FPGA shares the 32 kHz clock signal with bluetooth. If the FPGA is
  // not programmed when we attempt to initialize bluetooth, it prevents the
  // clock from reaching the bluetooth module and initialization fails.
  display_init();

  // Stop boot splash before initializing compositor
  boot_splash_stop();
  // Can't use the compositor framebuffer until the compositor is initialized
  compositor_init();
  kernel_ui_init();

  bt_driver_init();

  services_init();

  // The RTC needs be calibrated after the mfg registry service has been initialized so we can
  // load the measured frequency.
#if defined(CONFIG_SOC_SF32LB52) && !defined(SF32LB52_USE_LXT)
  rc10k_init();
#endif
  rtc_calibrate_frequency(mfg_info_get_rtc_freq());

  clear_reset_loop_detection_bits();

  task_watchdog_mask_set(PebbleTask_KernelMain);

  // Leave the board with stop and sleep mode debugging enabled for at least 10
  // seconds to give OpenOCD time to start and still able to connect when it is
  // ready to flash in the new image via JTAG
  s_lowpower_timer = new_timer_create();
  new_timer_start(s_lowpower_timer,
                  10 * 1000, prv_low_power_debug_config_callback, NULL, 0 /*flags*/);

#ifndef CONFIG_MFG
  s_uptime_timer = new_timer_create();
  new_timer_start(s_uptime_timer, 15 * 60 * 1000, uptime_callback, NULL, 0 /*flags*/);
#else
  boot_bit_set(BOOT_BIT_FW_STABLE);
#endif

  // Initialize button driver at the last moment to prevent "system on" button press from
  // entering the kernel event queue.
  debounced_button_init();

  task_watchdog_resume();
}

static void main_task(void *parameter) {
  prv_main_task_init();
  launcher_main_loop();
}
