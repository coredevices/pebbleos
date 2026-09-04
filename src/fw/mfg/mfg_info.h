/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! @file mfg_info.h
//!
//! Stores information about the physical watch that is encoded during the manufacturing process.

#include "applib/app_watch_info.h"
#include "applib/graphics/gtypes.h"

#include <string.h>

// If you give these functions enough space, they will add the null-terminator for you

void mfg_info_get_serialnumber(char *serial_number, size_t serial_number_size);

void mfg_info_get_pcba_serialnumber(char *serial_number, size_t serial_number_size);

void mfg_info_get_hw_version(char *serial_number, size_t serial_number_size);

//! @addtogroup Foundation
//! @{
//!   @addtogroup WatchInfo
//!   @{

//! Provides the color of the watch.
//! @return {@link WatchInfoColor} representing the color of the watch.
WatchInfoColor mfg_info_get_watch_color(void);

void mfg_info_set_watch_color(WatchInfoColor color);

//!   @} // end addtogroup WatchInfo
//! @} // end addtogroup Foundation

//! @internal
//! Returns the measured frequency of the LSE in mHz.
uint32_t mfg_info_get_rtc_freq(void);

void mfg_info_set_rtc_freq(uint32_t rtc_freq);

//! The number of bytes in our model name, including the null-terminator.
#define MFG_INFO_MODEL_STRING_LENGTH 16

//! Get the model string. Populates a supplied buffer with a null-terminated string.
//! @param buffer a character array that's at least MFG_INFO_MODEL_STRING_LENGTH in size
void mfg_info_get_model(char* buffer);

//! Set the model string to a new value.
//! @param model A null-terminated string that's at most MFG_INFO_MODEL_STRING_LENGTH bytes in
//!              length including the null-terminator. Longer strings will be truncated to fit.
void mfg_info_set_model(const char* model);

//! Set or update any constant data that needs to be written at manufacturing
//! time but which is not customized to the individual unit.
void mfg_info_update_constant_data(void);

typedef enum {
  MfgTest_Vibe,
  MfgTest_Display,
  MfgTest_Buttons,
  MfgTest_ALS,

  MfgTestCount
} MfgTest;

//! Record the pass / fail state of the given test.
void mfg_info_write_test_result(MfgTest test, bool pass);

//! Get the pass / fail state of the given test.
bool mfg_info_get_test_result(MfgTest test);

void mfg_info_write_als_result(uint32_t reading);

uint32_t mfg_info_get_als_result(void);

//! Value returned by mfg_info_get_vibe_cali() when the unit has not been
//! calibrated.
#define MFG_INFO_VIBE_CALI_INVALID 0xFF

//! Get the stored vibration motor calibration (driver-specific trim value).
//! @return the stored value, or MFG_INFO_VIBE_CALI_INVALID if not calibrated.
uint8_t mfg_info_get_vibe_cali(void);

//! Store the vibration motor calibration (driver-specific trim value).
void mfg_info_set_vibe_cali(uint8_t cali);
