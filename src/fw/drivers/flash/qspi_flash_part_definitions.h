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

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum JESD216Dw15QerType {
  JESD216_DW15_QER_NONE = 0,
  JESD216_DW15_QER_S2B1v1 = 1,
  JESD216_DW15_QER_S1B6 = 2,
  JESD216_DW15_QER_S2B7 = 3,
  JESD216_DW15_QER_S2B1v4 = 4,
  JESD216_DW15_QER_S2B1v5 = 5,
  JESD216_DW15_QER_S2B1v6 = 6,
} JESD216Dw15QerType;

typedef const struct QSPIFlashPart {
  struct {
    uint8_t fast_read;
    uint8_t fast_read_ddr;
    uint8_t read2o;
    uint8_t read2io;
    uint8_t read4o;
    uint8_t read4io;
    uint8_t pp;
    uint8_t pp2o;
    uint8_t pp4o;
    uint8_t pp4io;
    uint8_t erase_sector_4k;
    uint8_t erase_block_64k;
    uint8_t write_enable;
    uint8_t write_disable;
    uint8_t rdsr1;
    uint8_t rdsr2;
    uint8_t wrsr;
    uint8_t wrsr2;
    uint8_t erase_suspend;
    uint8_t erase_resume;
    uint8_t enter_low_power;
    uint8_t exit_low_power;
    uint8_t enter_quad_mode;
    uint8_t exit_quad_mode;
    uint8_t reset_enable;
    uint8_t reset;
    uint8_t qspi_id;
    uint8_t block_lock;
    uint8_t block_lock_status;
    uint8_t block_unlock_all;
    uint8_t write_protection_enable;
    uint8_t read_protection_status;
    uint8_t en4b;
    uint8_t erase_sec;
    uint8_t program_sec;
    uint8_t read_sec;
  } instructions;
  struct {
    uint8_t busy;
    uint8_t write_enable;
  } status_bit_masks;
  struct {
    uint8_t erase_suspend;
  } flag_status_bit_masks;
  struct {
    uint8_t fast_read;
    uint8_t fast_read_ddr;
  } dummy_cycles;
  struct {
    bool has_lock_data;  //<! true ifdata needs to be send along with the block_lock instruction
    uint8_t lock_data;   //<! The data to be sent on a block_lock command, if has_lock_data is true
    uint8_t
        locked_check;  //<! Value block_lock_status instruction should return if sector is locked
    uint8_t protection_enabled_mask;  //<! Mask read_protection_status instr to check if enabled
  } block_lock;
  uint32_t reset_latency_ms;
  uint32_t suspend_to_read_latency_us;
  uint32_t standby_to_low_power_latency_us;
  uint32_t low_power_to_standby_latency_us;
  bool supports_fast_read_ddr;
  bool supports_block_lock;
  JESD216Dw15QerType qer_type;
  uint32_t qspi_id_value;
  uint32_t size;
  const char *name;
} QSPIFlashPart;
