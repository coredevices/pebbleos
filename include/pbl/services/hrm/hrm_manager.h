/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/accel_manager_types.h"
#include "process_management/app_install_types.h"
#include "util/time/time.h"

#include <stdbool.h>
#include <stdint.h>
#include <kernel/pebble_tasks.h>

typedef enum {
  HRMQuality_OffWrist = -1,
  HRMQuality_Worst = 0,
  HRMQuality_Poor,
  HRMQuality_Acceptable,
  HRMQuality_Good,
  HRMQuality_Excellent,
} HRMQuality;

typedef enum {
  HRMFeatureShift_BPM = 0,
  HRMFeatureShift_HRV = 1,
  HRMFeatureShift_SpO2 = 2,
#ifdef CONFIG_MFG
  HRMFeatureShift_CTR = 3,
  HRMFeatureShift_Leakage = 4,
#endif
  HRMFeatureShiftMax
} HRMFeatureShift;

typedef enum {
  HRMFeature_BPM = (1 << HRMFeatureShift_BPM), //!< Collect heartrate BPM.
  HRMFeature_HRV = (1 << HRMFeatureShift_HRV), //!< Collect heartrate variability.
  HRMFeature_SpO2 = (1 << HRMFeatureShift_SpO2), //!< Collect blood oxygen saturation.
#ifdef CONFIG_MFG
  HRMFeature_CTR = (1 << HRMFeatureShift_CTR), //!< Collect ppg CTR test data.
  HRMFeature_Leakage = (1 << HRMFeatureShift_Leakage), //!< Collect ppg leakage test data.
#endif
  HRMFeatureMax
} HRMFeature;

// Hold enough data for 2s worth of samples just in case we miss a handshake
#define HRM_MANAGER_ACCEL_RATE_MILLIHZ (25000)
#define HRM_MANAGER_MAX_ACCEL_SAMPLES ((2 * HRM_MANAGER_ACCEL_RATE_MILLIHZ) / 1000)

// When an app exits, we change its subscription (if any) to expire in this many seconds
#define HRM_MANAGER_APP_EXIT_EXPIRATION_SEC  SECONDS_PER_HOUR

typedef struct {
  AccelRawData data[HRM_MANAGER_MAX_ACCEL_SAMPLES];
  uint32_t num_samples;
} HRMAccelData;

//! Grab the buffer containing accel data for the last 1 second period.
//! This locks the accel sample buffer that lives in the hrm manager.
HRMAccelData * hrm_manager_get_accel_data(void);

//! Unlock the accel sample buffer.
void hrm_manager_release_accel_data(void);

typedef uint32_t HRMSessionRef;
#define HRM_INVALID_SESSION_REF 0

// Send a HRMEvent_SubscriptionExpiring event to the subscriber at least this many seconds before
// the subscription expires (or one subscription interval, whichever is greater)
#define HRM_SUBSCRIPTION_EXPIRING_WARNING_SEC 5

void hrm_manager_init(void);

void hrm_manager_handle_prefs_changed(void);

//! Enable the HRM and subscribe to updates from an app or worker task.
//! This should not be used by KernelBG or KernelMain clients. For KernelBG client subscriptions,
//! please see \ref hrm_manager_subscribe_with_callback. KernelMain clients are not yet supported.
//! If the app/worker is already subscribed, this will update the subscription based on the passed
//! in arguments and return the pre-existing HRMSessionRef.
//! @param app_id the application's AppInstallId
//! @param update_interval_s requested update interval
//! @param expire_s after this many seconds, this subscription will automatically expire
//! @param features A bitfield of the features the subscriber would like updates for.
//! @return the HRMSessionRef for this subscription. NULL on failure
HRMSessionRef sys_hrm_manager_app_subscribe(AppInstallId app_id, uint32_t update_interval_s,
                                            uint16_t expire_s, HRMFeature features);

//! Return the HRMSessionRef for an app or worker subscription, if it exists. This call can not
//! be used for KernelBG subscriptions
//! @param app_id the application's AppInstallId
//! @return the HRMSessionRef for this subscription, or NULL if no subscription exists
HRMSessionRef sys_hrm_manager_get_app_subscription(AppInstallId app_id);

//! Unsubscribe from updates, disabling the HRM device if appropriate.
//! @param session the HRMSessionRef returned by sys_hrm_manager_app_subscribe
//! @return true on success, false on failure
bool sys_hrm_manager_unsubscribe(HRMSessionRef session);

//! Set the enabled features for the given HRM subscription
//! @param session the HRMSessionRef returned by sys_hrm_manager_app_subscribe
//! @param features the desired features
//! @return true on success, false on failure
bool sys_hrm_manager_set_features(HRMSessionRef session, HRMFeature features);

//! Set update interval and expiration time for an existing subscription
//! @param session the HRMSessionRef returned by sys_hrm_manager_app_subscribe
//! @param update_interval_s requested update interval
//! @param expire_s after this many seconds, this subscription will automatically expire
//! @return true on success, false on failure
bool sys_hrm_manager_set_update_interval(HRMSessionRef session, uint32_t update_interval_s,
                                         uint16_t expire_s);

//! Get info on a subscription.
//! @param[in] session the HRMSessionRef returned by sys_hrm_manager_app_subscribe
//! @param[out] app_id if not NULL, the app_id belonging to this subscription is returned here
//! @param[out] update_interval_s if not NULL, the requested update interval is returned here
//! @param[out] expire_s if not NULL, the number of seconds that this subcription will expire in
//! @param[out] features if not NULL, the features of this subscription are returned here
//! @return true if succss, false if subscription was not found
bool sys_hrm_manager_get_subscription_info(HRMSessionRef session, AppInstallId *app_id,
                                           uint32_t *update_interval_s, uint16_t *expire_s,
                                           HRMFeature *features);

//! Returns true if there is an HRM present & accessible to the HRM Manager.
//! @return true on success, false on failure
bool sys_hrm_manager_is_hrm_present(void);

//! Enable or disable the HRM manager.
//! Disabling the HRM manager does not remove subscribers, however subscribers
//! will no longer receive updates until the hrm manager is enabled again.
//! @param on Whether the HRM should be turned on or off.
void hrm_manager_enable(bool on);

//------------------------------------------------------------------------------
// HRM Driver Interface
//------------------------------------------------------------------------------
// The driver needs to provide new data to the service and needs to pull accel data.

//! HRMData will contain all HRM information that is currently available from the device.
typedef struct {
  HRMFeature features;

  uint8_t hrm_bpm;
  HRMQuality hrm_quality;

  uint16_t hrv_ppi_ms;
  HRMQuality hrv_quality;
 
  uint8_t spo2_percent;
  HRMQuality spo2_quality;
  uint8_t spo2_confidence;   //!< raw algorithm confidence coefficient (debug)
  uint8_t spo2_valid_level;  //!< raw algorithm valid level (debug)
  bool spo2_invalid;         //!< raw algorithm invalid flag (debug)

#ifdef CONFIG_MFG
  double ctr[6];
  double leakage[6];
#endif
} HRMData;

//! Callback used by HRM Driver to indicate that new data is available.
//! It can be expected that this will be called approx. 1x per second while the HRM is enabled.
//! This will take the new data and queue up appropriate events given subscribers and features.
void hrm_manager_new_data_cb(const HRMData *data);

//! Cleanup for the given app, which has just exited
void hrm_manager_process_cleanup(PebbleTask task, AppInstallId app_id);
