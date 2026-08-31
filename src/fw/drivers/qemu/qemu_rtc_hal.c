/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/rtc.h>

#include "board/board.h"
#include <pbl/logging/logging.h>

#include <stdio.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

// RTC MMIO register offsets (must match QEMU pebble-rtc device)
#define RTC_TIME_LO    0x00  // Unix timestamp low 32 bits (r/w)
#define RTC_TIME_HI    0x04  // Unix timestamp high 32 bits (r)
#define RTC_ALARM      0x08
#define RTC_CTRL       0x0C
#define RTC_TICKS_REG  0x10  // Monotonic 1000Hz tick counter (r)
#define RTC_BACKUP_BASE 0x40

// CTRL bits
#define CTRL_ALARM_IE  (1 << 0)

// STATUS bits
#define STATUS_ALARM_PENDING (1 << 0)

// Number of backup registers
#define NUM_BACKUP_REGS 16

// Backup register access macros
#define RTC_BACKUP_REG(n) (QEMU_RTC_BASE + RTC_BACKUP_BASE + ((n) * 4))

// RTC_WriteBackupRegister / RTC_ReadBackupRegister compatibility
// These are used by the common RTC code (rtc/common.c) for timezone storage.
void RTC_WriteBackupRegister(uint32_t reg_id, uint32_t value) {
  if (reg_id < NUM_BACKUP_REGS) {
    REG32(RTC_BACKUP_REG(reg_id)) = value;
  }
}

uint32_t RTC_ReadBackupRegister(uint32_t reg_id) {
  if (reg_id < NUM_BACKUP_REGS) {
    return REG32(RTC_BACKUP_REG(reg_id));
  }
  return 0;
}

void rtc_init(void) {
  // Clear alarm IRQ pending and disable alarm
  REG32(QEMU_RTC_BASE + RTC_CTRL) = CTRL_ALARM_IE;  // w1c the IRQ bit
  REG32(QEMU_RTC_BASE + RTC_CTRL) = 0;
}

void rtc_calibrate_frequency(uint32_t frequency) {
  // No calibration needed for QEMU
  (void)frequency;
}

void rtc_init_timers(void) {
  // No additional timers needed for QEMU RTC
}

void rtc_set_time(time_t time) {
  REG32(QEMU_RTC_BASE + RTC_TIME_LO) = (uint32_t)time;
}

time_t rtc_get_time(void) {
  return (time_t)REG32(QEMU_RTC_BASE + RTC_TIME_LO);
}

void rtc_set_time_tm(struct tm *time_tm) {
  time_t t = mktime(time_tm);
  rtc_set_time(t);
}

void rtc_get_time_tm(struct tm *time_tm) {
  time_t t = rtc_get_time();
  localtime_r(&t, time_tm);
}

void rtc_get_time_ms(time_t *out_seconds, uint16_t *out_ms) {
  *out_seconds = rtc_get_time();
  // Use ticks to compute sub-second portion
  uint32_t ticks = REG32(QEMU_RTC_BASE + RTC_TICKS_REG);
  *out_ms = (uint16_t)(ticks % 1000);
}

bool rtc_sanitize_struct_tm(struct tm *t) {
  // Remember tm_year is years since 1900.
  if (t->tm_year < 100) {
    t->tm_year = 100;
    return true;
  } else if (t->tm_year > 137) {
    t->tm_year = 137;
    return true;
  }
  return false;
}

bool rtc_sanitize_time_t(time_t *t) {
  struct tm time_struct;
  gmtime_r(t, &time_struct);

  const bool result = rtc_sanitize_struct_tm(&time_struct);
  *t = mktime(&time_struct);

  return result;
}

RtcTicks rtc_get_ticks(void) {
  // Read 32-bit tick counter from QEMU (1000Hz)
  return (RtcTicks)REG32(QEMU_RTC_BASE + RTC_TICKS_REG);
}

void rtc_alarm_init(void) {
  // Enable alarm interrupt support
}

void rtc_alarm_set(RtcTicks num_ticks) {
  RtcTicks current = rtc_get_ticks();
  REG32(QEMU_RTC_BASE + RTC_ALARM) = (uint32_t)(current + num_ticks);
  REG32(QEMU_RTC_BASE + RTC_CTRL) |= CTRL_ALARM_IE;
}

RtcTicks rtc_alarm_get_elapsed_ticks(void) {
  return 0;
}

bool rtc_alarm_is_initialized(void) {
  return true;
}

// Timezone uses backup registers 11-15 to avoid conflicts with bootbits (0-10)
#define TZ_BACKUP_BASE 11

void rtc_set_timezone(TimezoneInfo *tzinfo) {
  uint32_t *raw = (uint32_t *)tzinfo;
  _Static_assert(sizeof(TimezoneInfo) <= 5 * sizeof(uint32_t),
      "RTC Set Timezone invalid data size");

  RTC_WriteBackupRegister(TZ_BACKUP_BASE + 0, raw[0]);
  RTC_WriteBackupRegister(TZ_BACKUP_BASE + 1, raw[1]);
  RTC_WriteBackupRegister(TZ_BACKUP_BASE + 2, raw[2]);
  RTC_WriteBackupRegister(TZ_BACKUP_BASE + 3, raw[3]);
  RTC_WriteBackupRegister(TZ_BACKUP_BASE + 4, raw[4]);
}

void rtc_get_timezone(TimezoneInfo *tzinfo) {
  uint32_t *raw = (uint32_t *)tzinfo;

  raw[0] = RTC_ReadBackupRegister(TZ_BACKUP_BASE + 0);
  raw[1] = RTC_ReadBackupRegister(TZ_BACKUP_BASE + 1);
  raw[2] = RTC_ReadBackupRegister(TZ_BACKUP_BASE + 2);
  raw[3] = RTC_ReadBackupRegister(TZ_BACKUP_BASE + 3);
  raw[4] = RTC_ReadBackupRegister(TZ_BACKUP_BASE + 4);
}

uint16_t rtc_get_timezone_id(void) {
  // timezone_id is stored in the upper 16 bits of register 1 (matching common.c layout)
  return ((RTC_ReadBackupRegister(TZ_BACKUP_BASE + 1) >> 16) & 0xFFFF);
}

bool rtc_is_timezone_set(void) {
  // True if the timezone abbreviation has been set
  return (RTC_ReadBackupRegister(TZ_BACKUP_BASE + 0) != 0);
}

const char *rtc_get_time_string(char *buffer) {
  return time_t_to_string(buffer, rtc_get_time());
}

const char *time_t_to_string(char *buffer, time_t t) {
  struct tm time;
  localtime_r(&t, &time);
  strftime(buffer, TIME_STRING_BUFFER_SIZE, "%c", &time);
  return buffer;
}

void rtc_timezone_clear(void) {
  for (int i = 0; i < 5; i++) {
    RTC_WriteBackupRegister(TZ_BACKUP_BASE + i, 0);
  }
}
