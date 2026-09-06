/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <string.h>
#include <stdint.h>

#include <pbl/drivers/otp.h>

// These sizes represent the number of characters in the version
// strings and serial numbers. To store these, one additional byte is
// needed to store the null-terminator.
#define MFG_HW_VERSION_SIZE 9
#define MFG_SERIAL_NUMBER_SIZE 12
#define MFG_PCBA_SERIAL_NUMBER_SIZE 12

//! @return The last written final assembly serial number or "XXXXXXXXXXXX" if
//! no serial number has been written.
const char* mfg_get_serial_number(void);
const char* mfg_get_hw_version(void);
const char* mfg_get_pcba_serial_number(void);

typedef enum MfgSerialsResult {
  MfgSerialsResultSuccess = OtpWriteSuccess,
  MfgSerialsResultAlreadyWritten = OtpWriteFailAlreadyWritten,
  MfgSerialsResultCorrupt = OtpWriteFailCorrupt,
  MfgSerialsResultFailIncorrectLength = 3,
  MfgSerialsResultFailNoMoreSpace = 4,
} MfgSerialsResult;

//! Writes a new final assembly serial number to OTP.
//! There are 3 slots for serial numbers. The last written one is returned
//! from the \ref mfg_get_serial_number() function.
//! @param serial The serial number (zero-terminated string) to be written
//! @param serial_size The length of the buffer in bytes including terminating
//! zero. Must be 13.
//! @param[out] out_index Will contain the OTP index that was used to write the
//! serial number, if the return value was OtpWriteSuccess.
//! @return OtpWriteSuccess if the write was successful or
//! MfgSerialsResultFailNoMoreSpace if all 3 slots were taken already, or
//! MfgSerialsResultFailIncorrectLength if the serial_size was not 13.
MfgSerialsResult mfg_write_serial_number(const char* serial, size_t serial_size, uint8_t *out_index);

#if defined(CONFIG_IS_BIGBOARD)
//! Writes a fake serial number based on the unique identifier of the MCU
void mfg_write_bigboard_serial_number(void);
#endif
