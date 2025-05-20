/*
 * Copyright 2025 Core Devices LLC
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

#include "drivers/otp.h"
#include "drivers/flash.h"

#define FLASH_ERASE_VAL 0xFFU
#define OTP_SLOT_SIZE 32U
#define SEC_REG_IDX 0x0U

static char s_slot[NUM_OTP_SLOTS][OTP_SLOT_SIZE];

char * otp_get_slot(const uint8_t index) {
  const FlashSecurityRegisters *info;
  status_t ret;

  if (index >= NUM_OTP_SLOTS) {
    return NULL;
  }

  info = flash_security_registers_info();
  if (info->num_sec_regs == 0U) {
    return NULL;
  }

  for (uint8_t i = 0U; i < OTP_SLOT_SIZE; i++) {
    ret = flash_read_security_register(info->sec_regs[SEC_REG_IDX] + index * OTP_SLOT_SIZE + i,
                                       (uint8_t *)&s_slot[index][i]);
    if (ret != S_SUCCESS) {
      return NULL;
    }
  }

  return s_slot[index];
}

uint8_t * otp_get_lock(const uint8_t index) {
  return NULL;
}

bool otp_is_locked(const uint8_t index) {
  status_t ret;
  bool locked;

  ret = flash_security_registers_are_locked(&locked);
  if (ret != S_SUCCESS) {
    return false;
  }

  return locked;
}

OtpWriteResult otp_write_slot(const uint8_t index, const char *value) {
  const FlashSecurityRegisters *info;
  char *existing_val;
  status_t ret;
  size_t len;

  if (index >= NUM_OTP_SLOTS) {
    return OtpWriteFailCorrupt;
  }

  len = strlen(value);
  if (len >= OTP_SLOT_SIZE) {
    return OtpWriteFailCorrupt;
  }

  existing_val = otp_get_slot(index);
  for (size_t i = 0U; i < OTP_SLOT_SIZE; i++) {
    if ((uint8_t)existing_val[i] != FLASH_ERASE_VAL) {
      return OtpWriteFailAlreadyWritten;
    }
  }

  info = flash_security_registers_info();
  if (info->num_sec_regs == 0U) {
    return OtpWriteFailCorrupt;
  }

  for (size_t i = 0U; i <= len; i++) {
    ret = flash_write_security_register(info->sec_regs[SEC_REG_IDX] + index * OTP_SLOT_SIZE + i,
                                        (uint8_t)value[i]);
    if (ret != S_SUCCESS) {
      return OtpWriteFailCorrupt;
    }
  }

  return OtpWriteSuccess;
}
