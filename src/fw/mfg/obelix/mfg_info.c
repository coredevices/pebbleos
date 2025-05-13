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

#include "mfg/mfg_info.h"
#include "drivers/flash.h"
#include "flash_region/flash_region.h"

#define CURRENT_DATA_VERSION 0

typedef struct {
  uint32_t data_version;

  uint32_t color;
  char model[MFG_INFO_MODEL_STRING_LENGTH]; //!< Null terminated model string
} MfgData;

static void prv_update_struct(const MfgData *data) {
  flash_erase_subsector_blocking(FLASH_REGION_MFG_INFO_BEGIN);
  flash_write_bytes((const uint8_t*) data, FLASH_REGION_MFG_INFO_BEGIN, sizeof(*data));
}

static MfgData prv_fetch_struct(void) {
  MfgData result;

  flash_read_bytes((uint8_t*) &result, FLASH_REGION_MFG_INFO_BEGIN, sizeof(result));

  // Fallback data if not available
  if (result.data_version != CURRENT_DATA_VERSION) {
      result.data_version = CURRENT_DATA_VERSION;
      result.color = WATCH_INFO_COLOR_COREDEVICES_CT2_BLACK;
      strncpy(result.model, "CT2-BK", sizeof(result.model));
      result.model[MFG_INFO_MODEL_STRING_LENGTH - 1] = '\0';
  }

  return result;
}

WatchInfoColor mfg_info_get_watch_color(void) {
  return prv_fetch_struct().color;
}

void mfg_info_set_watch_color(WatchInfoColor color) {
  MfgData data = prv_fetch_struct();
  data.color = color;
  prv_update_struct(&data);
}

void mfg_info_get_model(char* buffer) {
  MfgData data = prv_fetch_struct();
  strcpy(buffer, data.model);
}

void mfg_info_set_model(const char* model) {
  MfgData data = prv_fetch_struct();
  strncpy(data.model, model, sizeof(data.model));
  data.model[MFG_INFO_MODEL_STRING_LENGTH - 1] = '\0';
  prv_update_struct(&data);
}

uint32_t mfg_info_get_rtc_freq(void) {
  // Not implemented.
  return 0U;
}

void mfg_info_set_rtc_freq(uint32_t rtc_freq) {
  // Not implemented.
}

GPoint mfg_info_get_disp_offsets(void) {
  // Not implemented. Can just assume no offset
  return (GPoint) {};
}

void mfg_info_set_disp_offsets(GPoint p) {
  // Not implemented.
}

void mfg_info_update_constant_data(void) {
  // Not implemented
}
