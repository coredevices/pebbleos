/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/hrm/gh3x2x.h>

#include <pbl/drivers/hrm.h>
#include "board/board.h"
#include "kernel/util/sleep.h"
#include <pbl/logging/logging.h>

#ifdef HRM_USE_GH3X2X
#include "math.h"
#include "kernel/util/delay.h"
#include "kernel/events.h"
#include "pbl/services/system_task.h"
#include "pbl/services/hrm/hrm_manager.h"

#include "gh_demo.h"
#include "gh_demo_inner.h"
#include "gh3x2x_demo_mp.h"
#endif // HRM_USE_GH3X2X

PBL_LOG_MODULE_DEFINE(driver_hrm_gh3x2x, CONFIG_DRIVER_HRM_LOG_LEVEL);

void gh3026_reset_pin_ctrl(uint8_t pin_level) {
#if GH3X2X_RESET_PIN_CTRLBY_NPM1300
  NPM1300_OPS.gpio_set(Npm1300_Gpio3, pin_level);
#endif
}

#ifdef HRM_USE_GH3X2X

#define GH3X2X_LOG_ENABLE 0
#define GH3X2X_FIFO_WATERMARK_CONFIG 80
#define GH3X2X_HR_SAMPLING_RATE 25

static volatile uint32_t s_hrm_int_flag = false;
static volatile uint32_t s_hrm_timer_flag = false;

// GH3X2X library glue code

void gh3026_reset_pin_init(void) {}

void gh3026_i2c_init(void) {}

void gh3026_i2c_write(uint8_t device_id, const uint8_t write_buffer[], uint16_t length) {
  i2c_use(HRM->i2c);
  i2c_write_block(HRM->i2c, length, write_buffer);
  i2c_release(HRM->i2c);
}

void gh3026_i2c_read(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length,
                     uint8_t read_buffer[], uint16_t read_length) {
  i2c_use(HRM->i2c);
  i2c_write_block(HRM->i2c, write_length, write_buffer);
  i2c_read_block(HRM->i2c, read_length, read_buffer);
  i2c_release(HRM->i2c);
}

static void prv_conv_fs4g_mg_to_lsb512(AccelRawData* data) {
  //hrm use 512lsb/g, so convert the mg data to lsb by coef 1.953f(1000/512)
  data->x = (int16_t)data->x/1.953f;
  data->y = (int16_t)data->y/1.953f;
  data->z = (int16_t)data->z/1.953f;
}

void gh3026_gsensor_data_get(STGsensorRawdata gsensor_buffer[], GU16 *gsensor_buffer_index) {
  HRMAccelData* acc = hrm_manager_get_accel_data();
  GU16 count = *gsensor_buffer_index = acc->num_samples;
  if(count > __GSENSOR_DATA_BUFFER_SIZE__) count = __GSENSOR_DATA_BUFFER_SIZE__;
  for (uint16_t i = 0; i < count; ++i) {
    prv_conv_fs4g_mg_to_lsb512(&acc->data[i]);
    memcpy(&gsensor_buffer[i], &acc->data[i], sizeof(STGsensorRawdata));
  }
  hrm_manager_release_accel_data();
}

static void gh3026_int_callback_function(void *context) {
  s_hrm_int_flag = false;
  Gh3x2xDemoInterruptProcess();
}

static void gh3026_int_irq_callback(bool *should_context_switch) {
  hal_gh3x2x_int_handler_call_back();

  if (s_hrm_int_flag == false) {
    if (system_task_add_callback_from_isr(gh3026_int_callback_function, NULL,
                                          should_context_switch)) {
      s_hrm_int_flag = true;
    }
  } else {
    *should_context_switch = false;
  }
}

void gh3026_int_pin_init(void) {
  exti_configure_pin(HRM->int_exti, ExtiTrigger_Falling, gh3026_int_irq_callback);
  exti_enable(HRM->int_exti);
}

void gh3x2x_print_fmt(const char *fmt, ...) {
#if GH3X2X_LOG_ENABLE
  char buffer[128];
  va_list ap;
  va_start(ap, fmt);
  vsniprintf(buffer, sizeof(buffer), fmt, ap);
  va_end(ap);
  PBL_LOG_ALWAYS("%s", buffer);
#endif
}

void gh3x2x_hr_result_report(uint8_t bpm, uint8_t quality) {
  HRMData hrm_data = {0};

  PBL_LOG_DBG("GH3X2X BPM %" PRIu8 " (quality=%" PRIu8 ", wear=%u)", bpm, quality, HRM->state->is_wear);

  hrm_data.features = HRMFeature_BPM;

  if (!HRM->state->is_wear) {
    hrm_data.hrm_quality = HRMQuality_OffWrist;
  } else {
    hrm_data.hrm_bpm = bpm;

    if (quality >= 98U) {
      hrm_data.hrm_quality = HRMQuality_Excellent;
    } else if (quality >= 90U) {
      hrm_data.hrm_quality = HRMQuality_Good;
    } else if (quality >= 80U) {
      hrm_data.hrm_quality = HRMQuality_Acceptable;
    } else if (quality >= 70U) {
      hrm_data.hrm_quality = HRMQuality_Poor;
    } else {
      hrm_data.hrm_quality = HRMQuality_Worst;
    }
  }

  hrm_manager_new_data_cb(&hrm_data);
}

void gh3x2x_spo2_result_report(uint8_t pct, uint8_t quality) {
  HRMData hrm_data = {0};

  PBL_LOG_DBG("GH3X2X SpO2 %" PRIu8 " (quality=%" PRIu8 ", wear=%u)", pct, quality, HRM->state->is_wear);

  hrm_data.features = HRMFeature_SpO2;

  if (!HRM->state->is_wear) {
    hrm_data.spo2_quality = HRMQuality_OffWrist;
  } else {
    hrm_data.spo2_percent = pct;

    if (quality >= 98U) {
      hrm_data.spo2_quality = HRMQuality_Excellent;
    } else if (quality >= 90U) {
      hrm_data.spo2_quality = HRMQuality_Good;
    } else if (quality >= 80U) {
      hrm_data.spo2_quality = HRMQuality_Acceptable;
    } else if (quality >= 70U) {
      hrm_data.spo2_quality = HRMQuality_Poor;
    } else {
      hrm_data.spo2_quality = HRMQuality_Worst;
    }
  }

  hrm_manager_new_data_cb(&hrm_data);
}
#ifdef CONFIG_HRM_HRV
void gh3x2x_hrv_result_report(const int32_t *rri, int32_t confidence, int32_t valid_num) {
  PBL_LOG_DBG("GH3X2X HRV n=%" PRId32 " (conf=%" PRId32 ", wear=%u)",
              valid_num, confidence, HRM->state->is_wear);
  if (!HRM->state->is_wear) {
    HRMData hrm_data = {0};
    hrm_data.features = HRMFeature_HRV;
    hrm_data.hrv_quality = HRMQuality_OffWrist;
    hrm_manager_new_data_cb(&hrm_data);
    return;
  }
  if (valid_num > 4) {
    valid_num = 4;
  }
  for (int32_t i = 0; i < valid_num; i++) {
    HRMData hrm_data = {0};
    hrm_data.features = HRMFeature_HRV;
    hrm_data.hrv_ppi_ms = (uint16_t)rri[i];
    if (confidence >= 98) {
      hrm_data.hrv_quality = HRMQuality_Excellent;
    } else if (confidence >= 90) {
      hrm_data.hrv_quality = HRMQuality_Good;
    } else if (confidence >= 80) {
      hrm_data.hrv_quality = HRMQuality_Acceptable;
    } else if (confidence >= 70) {
      hrm_data.hrv_quality = HRMQuality_Poor;
    } else {
      hrm_data.hrv_quality = HRMQuality_Worst;
    }
    hrm_manager_new_data_cb(&hrm_data);
  }
}
#endif

void gh3x2x_wear_evt_notify(bool is_wear) {
  PBL_LOG_DBG("GH3X2X wear state: %d", is_wear);

  HRM->state->is_wear = is_wear;
}

#ifdef CONFIG_GH3X2X_TUNING_SERVICE_ENABLED
void gh3x2x_timer_init(uint32_t period_ms) {
  if (HRM) {
    HRM->state->timer_period_ms = period_ms;
  }
}

static void gh3x2x_timer_callback(void* data) {
  uint32_t param = (uint32_t)data;
  if (param != 0x87965421) {
    // Coalesce repeated timer firings - only queue one callback at a time
    if (s_hrm_timer_flag == false) {
      if (system_task_add_callback(gh3x2x_timer_callback, (void*)0x87965421)) {
        s_hrm_timer_flag = true;
      }
    }
    return;
  }
  s_hrm_timer_flag = false;
  Gh3x2xSerialSendTimerHandle();
}

static void gh3x2x_timer_start_handle(void* arg) {
  if (HRM == NULL || HRM->state->timer != NULL) {
    return;
  }
  if (HRM->state->timer_period_ms == 0) {
    return;
  }
  HRM->state->timer = app_timer_register_repeatable(HRM->state->timer_period_ms, gh3x2x_timer_callback, NULL, true);
}

static void gh3x2x_timer_stop_handle(void* arg) {
  if (HRM && HRM->state->timer) {
    app_timer_cancel(HRM->state->timer);
    HRM->state->timer = NULL;
  }
}

void gh3x2x_timer_start(void) {
  return;
  PebbleEvent e = {
    .type = PEBBLE_CALLBACK_EVENT,
    .callback.callback = gh3x2x_timer_start_handle,
  };
  event_put(&e);
}

void gh3x2x_timer_stop(void) {
  return;
  PebbleEvent e = {
    .type = PEBBLE_CALLBACK_EVENT,
    .callback.callback = gh3x2x_timer_stop_handle,
  };
  event_put(&e);
}

static void gh3x2x_ble_data_recv_handle(void *context) {
  if (context == NULL) {
    return;
  }

  uint32_t data_len;
  uint8_t *p_data = (uint8_t*)context;
  memcpy(&data_len, p_data, sizeof(uint32_t));
  p_data += sizeof(uint32_t);
  Gh3x2xDemoProtocolProcess((GU8*)p_data, data_len);
  free(context);
}

bool gh3x2x_ble_data_recv(void* context) {
  if (context == NULL) {
    return false;
  }

  if (!system_task_add_callback(gh3x2x_ble_data_recv_handle, context)) {
    return false;
  }
  return true;
}
#endif

// GH3X2X calibration/factory testing

void gh3x2x_rawdata_notify(uint32_t *p_rawdata, uint32_t data_count) {
#ifdef CONFIG_MFG
  HRMDevice* p_dev = HRM;
  if (p_dev == NULL || p_dev->state->enabled == false) {
    return;
  }

  GH3x2xFTData* p_factory = p_dev->state->factory;
  if (p_factory == NULL) {
    return;
  }
  uint32_t mode = p_factory->test_mode;
  if (mode == 0) {
    return;
  }

  // save ppg raw data, 80 samples for each channel
  uint32_t idx;
  data_count /= HRM_PPG_CH_NUM;
  while (data_count--) {
    if (p_factory->drop_count > 0) {
      p_factory->drop_count--;
      p_rawdata += HRM_PPG_CH_NUM;
    } else {
      if (p_factory->wpos >= HRM_PPG_FACTORY_TEST_FIFO_LEN) {
        p_factory->wpos = 0;
      }
      for (idx=0; idx<HRM_PPG_CH_NUM; ++idx) {
        p_factory->ppg_array[idx][p_factory->wpos] = *p_rawdata++;
      }
      
      p_factory->wpos++;
      if (p_factory->count < HRM_PPG_FACTORY_TEST_FIFO_LEN) {
        p_factory->count++;
      }
    }
  }
  if (p_factory->count < HRM_PPG_FACTORY_TEST_FIFO_LEN) {
    return;
  }

  uint32_t i;
  uint32_t ppg_avg[HRM_PPG_CH_NUM];
  uint64_t total[HRM_PPG_CH_NUM];
  HRMData hrm_data = {0};
  memset(total, 0, sizeof(total));
  for (idx=0; idx<HRM_PPG_CH_NUM; ++idx) {
    // calcu total values for 80 samples ppg raw data
    for (i=0; i<p_factory->count; ++i) {
      total[idx] += p_factory->ppg_array[idx][i];
    }
    // calcu avr for each channel
    ppg_avg[idx] = total[idx] / p_factory->count;
  }
  
  //let keep the factory test data report 2hz
  static int cnt = 0;
  if (cnt++ % 25) return;
  if (mode == GH3X2X_FUNCTION_TEST1) {
    hrm_data.features = HRMFeature_CTR;
    // calcu CTR:  result = ((ppg_avg-2^23))*1800*1000/(20*10*2*(2^23));
    // Green >= 28; IR >= 36; Red >= 36
    for (i=0; i<HRM_PPG_CH_NUM; ++i) {
      p_factory->result[i] = ((double)(ppg_avg[i] - (1<<23)) * 4500) / (1<<23);
      hrm_data.ctr[i] = p_factory->result[i];
    }
    hrm_manager_new_data_cb(&hrm_data);
  } else if (mode == GH3X2X_FUNCTION_TEST2) {
    hrm_data.features = HRMFeature_Leakage; 
    // calcu leakage: result = (ppg_avg-(2^23))*1800*1000/(20*100*2*(2^23));
    for (i=0; i<HRM_PPG_CH_NUM; ++i) {
      p_factory->result[i] = ((double)(ppg_avg[i] - (1<<23)) * 450) / (1<<23);
      hrm_data.leakage[i] = p_factory->result[i];
    }
    hrm_manager_new_data_cb(&hrm_data);
  } else {
    
    ;
  }
#endif
}

#ifdef CONFIG_MFG
void gh3x2x_factory_test_enable(HRMDevice *dev, GH3x2xFTType test_type) {
  uint32_t mode = 0;
  if (test_type == HRM_FACTORY_TEST_CTR) {                    // CTR
    mode = GH3X2X_FUNCTION_TEST1;
  } else if (test_type == HRM_FACTORY_TEST_LIGHT_LEAK) {      // leakage
    mode = GH3X2X_FUNCTION_TEST2;
  } else if (test_type == HRM_FACTORY_TEST_HSM) {           // noise
    mode = GH3X2X_FUNCTION_HSM;
  } else {
    return;
  }

  uint32_t* ppg_data;
  GH3x2xFTData* p_factory = (GH3x2xFTData*)malloc(sizeof(GH3x2xFTData) + sizeof(uint32_t)*HRM_PPG_FACTORY_TEST_FIFO_LEN*HRM_PPG_CH_NUM);
  if (p_factory == NULL) {
    PBL_LOG_ERR("malloc failed.");
    return;
  }
  memset(p_factory, 0, sizeof(GH3x2xFTData));
  p_factory->drop_count = 30;
  p_factory->test_mode = mode;
  if (dev->state->factory != NULL) {
    free(dev->state->factory);
  }
  ppg_data = (uint32_t*)(p_factory + 1);
  for (uint32_t i=0; i<HRM_PPG_CH_NUM; ++i) {
    p_factory->ppg_array[i] = ppg_data + HRM_PPG_FACTORY_TEST_FIFO_LEN*i;
  }
  dev->state->factory = p_factory;

  dev->state->enabled = true;
  s_hrm_int_flag = false;
  Gh3x2xDemoStopSampling(0xFFFFFFFF);
  Gh3x2xDemoStartSamplingWithCfgSwitch(mode, 1);
}

//shoud be called in system task
static void gh3x2x_ft_ctr_start_handle(void* data) {
  gh3x2x_factory_test_enable(HRM, HRM_FACTORY_TEST_CTR);
}

void gh3x2x_start_ft_ctr(void) {
  system_task_add_callback(gh3x2x_ft_ctr_start_handle, NULL);
}

//shoud be called in system task
static void gh3x2x_ft_leakage_start_handle(void* data) {
  gh3x2x_factory_test_enable(HRM, HRM_FACTORY_TEST_LIGHT_LEAK);
}

void gh3x2x_start_ft_leakage(void) {
  system_task_add_callback(gh3x2x_ft_leakage_start_handle, NULL);
}

//shoud be called in system task
static void gh3x2x_factory_test_disable_handle(void *data) {
  HRMDevice *dev = (HRMDevice *)data;
  dev->state->enabled = false;
  Gh3x2xDemoStopSampling(0xFFFFFFFF);
  if (dev->state->factory != NULL) {
    free(dev->state->factory);
    dev->state->factory = NULL;
  }
}

void gh3x2x_factory_test_disable(void) {
  system_task_add_callback(gh3x2x_factory_test_disable_handle, (void*)HRM);
}

uint8_t gh3x2x_factory_result_get(float* p_result)
{
  HRMDevice* p_dev = HRM;
  if (p_result) {
    GH3x2xFTData* p_factory = p_dev->state->factory;
    if (p_factory != NULL && p_factory->count >= HRM_PPG_FACTORY_TEST_FIFO_LEN) {
      memcpy(p_result, p_factory->result, sizeof(float)*HRM_PPG_FACTORY_TEST_FIFO_LEN);
      return HRM_PPG_FACTORY_TEST_FIFO_LEN;
    }
  }
  return 0;
}

void gh3x2x_set_work_mode(int32_t mode) {
  HRMDeviceState* state = HRM->state;
  //always enable soft adt
  state->work_mode = mode | GH3X2X_FUNCTION_SOFT_ADT_IR;
}
#endif // CONFIG_MFG

#endif // HRM_USE_GH3X2X

// HRM interface

void hrm_init(HRMDevice *dev) {
#ifdef HRM_USE_GH3X2X
  int ret;

  ret = Gh3x2xDemoInit();
  if (ret != 0) {
    PBL_LOG_ERR("GH3X2X failed to initialize");
    return;
  }
#else
  gh3026_reset_pin_ctrl(0);
  gpio_input_init_pull_up_down(&dev->int_input, GPIO_PuPd_DOWN);
#endif

  dev->state->is_wear = false;
  dev->state->initialized = true;
}

bool hrm_enable(HRMDevice *dev, HRMFeature features) {
#ifdef HRM_USE_GH3X2X
  if (!dev->state->initialized) {
    return false;
  }

  s_hrm_int_flag = false;

  dev->state->work_mode = GH3X2X_FUNCTION_HR | GH3X2X_FUNCTION_SOFT_ADT_GREEN;
#ifdef CONFIG_MFG
  dev->state->work_mode = GH3X2X_FUNCTION_HR | GH3X2X_FUNCTION_SPO2 | GH3X2X_FUNCTION_SOFT_ADT_IR;
#endif

#ifdef CONFIG_HRM_HRV
  if (features & HRMFeature_HRV) {
    dev->state->work_mode |= GH3X2X_FUNCTION_HRV;
    // The shipped register config maps PPG channels for HR/SpO2/ADT only.
    // Mirror the HR channel map onto the HRV function so it receives frames.
    const STGh3x2xFrameInfo *hr_info = g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_HR];
    const STGh3x2xFrameInfo *hrv_info = g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_HRV];
    GU8 hr_chnl_num = hr_info->pstFunctionInfo->uchChnlNum;
    GH3x2xSetFunctionChnlNum(hrv_info, hr_chnl_num);
    for (GU8 i = 0; i < hr_chnl_num; i++) {
      GH3x2xSetFunctionChnlMap(hrv_info, i, hr_info->pchChnlMap[i]);
    }
    GH3x2xCalFunctionSlotBit(hrv_info);
  }
#endif

  GH3X2X_FifoWatermarkThrConfig(GH3X2X_FIFO_WATERMARK_CONFIG);
  GH3X2X_SetSoftEvent(GH3X2X_SOFT_EVENT_NEED_FORCE_READ_FIFO);
  Gh3x2xDemoFunctionSampleRateSet(GH3X2X_FUNCTION_HR, GH3X2X_HR_SAMPLING_RATE);
#ifdef CONFIG_HRM_HRV
  if (features & HRMFeature_HRV) {
    Gh3x2xDemoFunctionSampleRateSet(GH3X2X_FUNCTION_HRV, GH3X2X_HR_SAMPLING_RATE);
  }
#endif
  Gh3x2xDemoStartSampling(dev->state->work_mode);

  dev->state->enabled = true;
  return true;
#else
  return false;
#endif
}

void hrm_disable(HRMDevice *dev) {
#ifdef HRM_USE_GH3X2X
  if (!dev->state->initialized) {
    return;
  }

  Gh3x2xDemoStopSampling(0xFFFFFFFF);
#endif
  dev->state->enabled = false;
}

bool hrm_is_enabled(HRMDevice *dev) {
  return dev->state->enabled;
}
