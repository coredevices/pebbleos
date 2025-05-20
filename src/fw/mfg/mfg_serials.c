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

#include <stdio.h>

#include "mfg_serials.h"

#include "console/prompt.h"
#include "util/size.h"

static const uint8_t OTP_SERIAL_SLOT_INDICES[] = {
    OTP_SERIAL1, OTP_SERIAL2, OTP_SERIAL3, OTP_SERIAL4, OTP_SERIAL5
};
static const uint8_t OTP_PCBA_SLOT_INDICES[] = {
    OTP_PCBA_SERIAL1, OTP_PCBA_SERIAL2, OTP_PCBA_SERIAL3
};
#if PLATFORM_SILK || PLATFORM_ASTERIX || PLATFORM_CALCULUS || PLATFORM_ROBERT
static const uint8_t OTP_HWVER_SLOT_INDICES[] = {
    OTP_HWVER1, OTP_HWVER2, OTP_HWVER3, OTP_HWVER4, OTP_HWVER5
};
#else
static const uint8_t OTP_HWVER_SLOT_INDICES[] = {OTP_HWVER1};
#endif

static const char DUMMY_SERIAL[MFG_SERIAL_NUMBER_SIZE + 1] = "XXXXXXXXXXXX";
// FIXME: shouldn't the dummy HWVER be 9 X's?
static const char DUMMY_HWVER[MFG_HW_VERSION_SIZE + 1] = "XXXXXXXX";
static const char DUMMY_PCBA_SERIAL[MFG_PCBA_SERIAL_NUMBER_SIZE + 1] = "XXXXXXXXXXXX";

static void mfg_print_feedback(const MfgSerialsResult result, const uint8_t index, const char *value, const char *name);

const char* mfg_get_serial_number(void) {
  // Trying from "most recent" slot to "least recent":
  for (int i = ARRAY_LENGTH(OTP_SERIAL_SLOT_INDICES) - 1; i >= 0; --i) {
    const uint8_t index = OTP_SERIAL_SLOT_INDICES[i];
    if (otp_is_locked(index)) {
      return otp_get_slot(index);
    }
  }
  return DUMMY_SERIAL;
}

const char* mfg_get_hw_version(void) {
  // Trying from "most recent" slot to "least recent":
  for (int i = ARRAY_LENGTH(OTP_HWVER_SLOT_INDICES) - 1; i >= 0; --i) {
    const uint8_t index = OTP_HWVER_SLOT_INDICES[i];
    if (otp_is_locked(index)) {
      return otp_get_slot(index);
    }
  }
  return DUMMY_HWVER;
}

const char* mfg_get_pcba_serial_number(void) {
  // Trying from "most recent" slot to "least recent":
  for (int i = ARRAY_LENGTH(OTP_PCBA_SLOT_INDICES) - 1; i >= 0; --i) {
    const uint8_t index = OTP_PCBA_SLOT_INDICES[i];
    if (otp_is_locked(index)) {
      return otp_get_slot(index);
    }
  }
  return DUMMY_PCBA_SERIAL;
}

static MfgSerialsResult prv_mfg_write_data_to_slot(const uint8_t *slot_indices, size_t num_slots,
                                                   const char *data, size_t data_size,
                                                   uint8_t *out_index) {
  for (unsigned int i = 0; i < num_slots; ++i) {
    const uint8_t index = slot_indices[i];
    const OtpWriteResult result = otp_write_slot(index, data);
    if (result == OtpWriteSuccess) {
      if (out_index) {
        *out_index = index;
      }
      return MfgSerialsResultSuccess;
    }
    // if OtpWriteFailCorrupt or OtpWriteFailAlreadyWritten, continue to next slot.
  }
  return MfgSerialsResultFailNoMoreSpace;
}

MfgSerialsResult mfg_write_serial_number(const char* serial, size_t serial_size,
                                         uint8_t *out_index) {

  if ((serial_size != (MFG_SERIAL_NUMBER_SIZE)) || (serial[serial_size] != '\0')) {
    return MfgSerialsResultFailIncorrectLength;
  }

  return prv_mfg_write_data_to_slot(OTP_SERIAL_SLOT_INDICES, ARRAY_LENGTH(OTP_SERIAL_SLOT_INDICES),
                                    serial, serial_size, out_index);
}

MfgSerialsResult mfg_write_pcba_serial_number(const char* serial, size_t serial_size,
                                              uint8_t *out_index) {

  if ((serial_size > MFG_PCBA_SERIAL_NUMBER_SIZE) || (serial[serial_size] != '\0')) {
    return MfgSerialsResultFailIncorrectLength;
  }

  return prv_mfg_write_data_to_slot(OTP_PCBA_SLOT_INDICES, ARRAY_LENGTH(OTP_PCBA_SLOT_INDICES),
                                    serial, serial_size, out_index);
}

static MfgSerialsResult prv_mfg_write_hw_version(const char* hwver, size_t hwver_size,
                                                 uint8_t *out_index) {
  if ((hwver_size > MFG_HW_VERSION_SIZE) || hwver[hwver_size] != '\0') {
    return MfgSerialsResultFailIncorrectLength;
  }
  return prv_mfg_write_data_to_slot(OTP_HWVER_SLOT_INDICES, ARRAY_LENGTH(OTP_HWVER_SLOT_INDICES),
                                    hwver, hwver_size, out_index);
}

void command_serial_read(void) {
  prompt_send_response(mfg_get_serial_number());
}

void command_hwver_read(void) {
  prompt_send_response(mfg_get_hw_version());
}

void command_pcba_serial_read(void) {
  prompt_send_response(mfg_get_pcba_serial_number());
}

void command_serial_write(const char *serial) {
  MfgSerialsResult result;
  uint8_t index = 0;

  size_t serial_len = strlen(serial);
  if ((serial_len >= 11) && (serial_len <= MFG_SERIAL_NUMBER_SIZE)) {
    result = mfg_write_serial_number(serial, serial_len, &index);
  } else {
    result = MfgSerialsResultFailIncorrectLength;
  }

  mfg_print_feedback(result, index, serial, "Serial");
}

void command_hwver_write(const char *hwver) {
  MfgSerialsResult result;
  uint8_t index = 0;

  size_t hwver_len = strlen(hwver);
  if (hwver_len > 0) {
    result = prv_mfg_write_hw_version(hwver, hwver_len, &index);
  } else {
    result = MfgSerialsResultFailIncorrectLength;
  }

  mfg_print_feedback(result, index, hwver, "HW version");
}

void command_pcba_serial_write(const char *pcba_serial) {
  MfgSerialsResult result;
  uint8_t index = 0;

  size_t pcba_serial_len = strlen(pcba_serial);
  if ((pcba_serial_len > 0) && (pcba_serial_len <= MFG_PCBA_SERIAL_NUMBER_SIZE)) {
    result = mfg_write_pcba_serial_number(pcba_serial, pcba_serial_len, &index);
  } else {
    result = MfgSerialsResultFailIncorrectLength;
  }

  mfg_print_feedback(result, index, pcba_serial, "PCBA Serial");
}

static void mfg_print_feedback(const MfgSerialsResult result, const uint8_t index,
                               const char *value, const char *name) {
  switch (result) {
    case MfgSerialsResultAlreadyWritten: {
      char buffer[48];
      const char * const field = otp_get_slot(index);
      prompt_send_response_fmt(buffer, sizeof(buffer), "%s already present! %s", name, field);
      break;
    }
    case MfgSerialsResultCorrupt: {
      char buffer[48];
      prompt_send_response_fmt(buffer, sizeof(buffer), "Writing failed; %s may be corrupt!", name);
      break;
    }
    case MfgSerialsResultFailIncorrectLength: {
      prompt_send_response("Incorrect length");
      break;
    }
    case MfgSerialsResultFailNoMoreSpace: {
      prompt_send_response("No more space!");
      break;
    }
    case MfgSerialsResultSuccess:
      prompt_send_response("OK");
      break;
    default:
      break;
  }
}

#if defined(IS_BIGBOARD)

#include <stdio.h>
#include "drivers/rtc.h"
#include "system/logging.h"

#if !MICRO_FAMILY_NRF5
static void prv_get_not_so_unique_serial(char *serial_number) {
  // Contains 96 bits (12 bytes) that uniquely identify the STM32F2/F4 MCUs:
  const uint8_t *DEVICE_ID_REGISTER = (const uint8_t *) 0x1FFF7A10;
  // BBs used the first bytes of the ID registers, which happened to be not very unique...
  for (int i = 2, r = 7; i < MFG_SERIAL_NUMBER_SIZE; i += 2, ++r) {
    sniprintf(&serial_number[i], 3 /* 2 hex digits + zero terminator */, "%02X",
              DEVICE_ID_REGISTER[r]);
  }
  serial_number[MFG_SERIAL_NUMBER_SIZE] = 0;
}
#endif

static bool prv_get_more_unique_serial(char *serial_number) {
  for (int i = 2; i < MFG_SERIAL_NUMBER_SIZE; i += 2) {
    sniprintf(&serial_number[i], 3 /* 2 hex digits + zero terminator */, "%02X", rand());
  }
  serial_number[MFG_SERIAL_NUMBER_SIZE] = 0;
  return true;
}

void mfg_write_bigboard_serial_number(void) {
  char serial_number[MFG_SERIAL_NUMBER_SIZE + 1];
  // Start with underscore, so it's easy to filter out from analytics:
  serial_number[0] = '_';
  serial_number[1] = 'B';
  serial_number[2] = 0;

  // Check whether the previous not-so-unique SN or the no SN ("XXXXXXXXXXXX") has been written:
#if !MICRO_FAMILY_NRF5
  prv_get_not_so_unique_serial(serial_number);
#endif
  const char *current_serial_number = mfg_get_serial_number();
  
  if (strcmp(current_serial_number, serial_number) &&
      strcmp(current_serial_number, DUMMY_SERIAL)) {
    return;
  }

  // Create a "more unique" serial number using rand():
  if (prv_get_more_unique_serial(serial_number)) {
    mfg_write_serial_number(serial_number, MFG_SERIAL_NUMBER_SIZE, NULL);
  }
}
#endif

