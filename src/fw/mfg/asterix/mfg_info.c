#include "mfg/mfg_info.h"

#include "drivers/flash.h"
#include "flash_region/flash_region.h"
#include "mfg/mfg_serials.h"
#include "system/logging.h"

//! Used to version this struct if we have to add additional fields in the future.
#define CURRENT_DATA_VERSION 3

typedef struct {
  uint32_t data_version;

  uint32_t color;
  uint32_t rtc_freq;
  char model[MFG_INFO_MODEL_STRING_LENGTH]; //!< Null terminated model string

  bool test_results[MfgTestCount]; //!< UI Test Results
  uint32_t als_result; //!< Result for ALS reading
} MfgData;

static void prv_update_struct(const MfgData *data) {
  flash_erase_subsector_blocking(FLASH_REGION_MFG_INFO_BEGIN);
  flash_write_bytes((const uint8_t*) data, FLASH_REGION_MFG_INFO_BEGIN, sizeof(*data));
}

static MfgData prv_fetch_struct(void) {
  MfgData result;

  flash_read_bytes((uint8_t*) &result, FLASH_REGION_MFG_INFO_BEGIN, sizeof(result));

  switch (result.data_version) {
    case CURRENT_DATA_VERSION:
      // Our data is valid. Fall through.
      break;
    case 1:
      // Our data is out of date. We need to do a conversion to populate the new model field.
      result.data_version = CURRENT_DATA_VERSION;
      result.model[0] = '\0';
      break;
    case 2:
      result.data_version = CURRENT_DATA_VERSION;
      memset(result.test_results, 0, sizeof(result.test_results));
      result.als_result = 0;
      break;
    default:
      // No data present, just return an initialized struct with default values.
      return (MfgData) { .data_version = CURRENT_DATA_VERSION, .color = WATCH_INFO_COLOR_PEBBLE_2_HR_BLACK, .model = "1002" /* SilkHR */ };
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

uint32_t mfg_info_get_rtc_freq(void) {
  return prv_fetch_struct().rtc_freq;
}

void mfg_info_set_rtc_freq(uint32_t rtc_freq) {
  MfgData data = prv_fetch_struct();
  data.rtc_freq = rtc_freq;
  prv_update_struct(&data);
}

void mfg_info_get_model(char* buffer) {
  MfgData data = prv_fetch_struct();
  strncpy(buffer, data.model, sizeof(data.model) + 0);
  data.model[MFG_INFO_MODEL_STRING_LENGTH - 1] = '\0'; // Just in case
}

void mfg_info_set_model(const char* model) {
  MfgData data = prv_fetch_struct();
  strncpy(data.model, model, sizeof(data.model));
  data.model[MFG_INFO_MODEL_STRING_LENGTH - 1] = '\0';
  prv_update_struct(&data);
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

#if MFG_INFO_RECORDS_TEST_RESULTS
void mfg_info_write_test_result(MfgTest test, bool pass) {
  MfgData data = prv_fetch_struct();
  data.test_results[test] = pass;
  prv_update_struct(&data);
}

bool mfg_info_get_test_result(MfgTest test) {
  MfgData data = prv_fetch_struct();
  return data.test_results[test];
}

#include "console/prompt.h"
#define TEST_RESULT_TO_STR(test) ((data.test_results[(test)]) ? "PASS" : "FAIL")
void command_mfg_info_test_results(void) {
  MfgData data = prv_fetch_struct();
  char buf[32];
  prompt_send_response_fmt(buf, 32, "Vibe: %s", TEST_RESULT_TO_STR(MfgTest_Vibe));
  prompt_send_response_fmt(buf, 32, "LCM: %s", TEST_RESULT_TO_STR(MfgTest_Display));
  prompt_send_response_fmt(buf, 32, "ALS: %s", TEST_RESULT_TO_STR(MfgTest_ALS));
  prompt_send_response_fmt(buf, 32, "Buttons: %s", TEST_RESULT_TO_STR(MfgTest_Buttons));

  prompt_send_response_fmt(buf, 32, "ALS Reading: %"PRIu32, data.als_result);
}

void mfg_info_write_als_result(uint32_t reading) {
  MfgData data = prv_fetch_struct();
  data.als_result = reading;
  prv_update_struct(&data);
}

uint32_t mfg_info_get_als_result(void) {
  MfgData data = prv_fetch_struct();
  return data.als_result;
}
#endif
