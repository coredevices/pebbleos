/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include <pbl/drivers/otp.h>
#include <pbl/drivers/flash.h>

#define FLASH_ERASE_VAL 0xFFU
#define OTP_SLOT_SIZE 32U
#define SEC_REG_IDX 0x0U

bool cd_flash_active(void);
status_t cd_flash_read_security_register(uint32_t addr, uint8_t *val);
status_t cd_flash_security_register_is_locked(uint32_t addr, bool *locked);

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
    if (cd_flash_active()) {
      ret = cd_flash_read_security_register(info->sec_regs[SEC_REG_IDX] + index * OTP_SLOT_SIZE + i,
                                            (uint8_t *)&s_slot[index][i]);
    } else {
      ret = flash_read_security_register(info->sec_regs[SEC_REG_IDX] + index * OTP_SLOT_SIZE + i,
                                         (uint8_t *)&s_slot[index][i]);
    }
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
  const FlashSecurityRegisters *info;
  status_t ret;
  bool locked;

  info = flash_security_registers_info();
  if (info->num_sec_regs == 0U) {
    return false;
  }

  if (cd_flash_active()) {
    ret = cd_flash_security_register_is_locked(info->sec_regs[SEC_REG_IDX], &locked);
  } else {
    ret = flash_security_register_is_locked(info->sec_regs[SEC_REG_IDX], &locked);
  }
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

  existing_val = otp_get_slot(index);
  if ((existing_val == NULL) || (memcmp(existing_val, value, len + 1) != 0)) {
    return OtpWriteFailCorrupt;
  }

  return OtpWriteSuccess;
}
