/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/touch/touch_session.h"

#include <pbl/drivers/rtc.h>
#include <pbl/logging/logging.h>
#include "kernel/ui/modals/modal_manager.h"
#include "pbl/services/light.h"
#include "process_management/app_manager.h"
#include "shell/prefs.h"

PBL_LOG_MODULE_DECLARE(service_touch, CONFIG_SERVICE_TOUCH_LOG_LEVEL);

//! The session lasts exactly as long as the backlight would stay on: arming
//! behaves as if the light had come on, even when ALS or DnD kept it dark.
//! Sampled per arm/extend so a pref change applies immediately.
static RtcTicks prv_timeout_ticks(void) {
  return ((RtcTicks)backlight_get_timeout_ms() * RTC_TICKS_HZ) / 1000;
}

static RtcTicks s_armed_until;

void touch_session_arm(TouchSessionArmSource source) {
  s_armed_until = rtc_get_ticks() + prv_timeout_ticks();
  PBL_LOG_DBG("Touch session armed (source %d)", (int)source);
}

void touch_session_extend(void) {
  if (touch_session_is_active()) {
    s_armed_until = rtc_get_ticks() + prv_timeout_ticks();
  }
}

bool touch_session_is_active(void) {
#ifdef CONFIG_RECOVERY_FW
  // PRF has no touch navigation; keep the mfg touch test's touch-follows-light
  // behavior unconditional.
  return true;
#else
  if (rtc_get_ticks() < s_armed_until) {
    return true;
  }
  // Any UI the user deliberately entered is not guarded; only the idle
  // watchface needs arming.
  if (!app_manager_is_watchface_running()) {
    return true;
  }
  // A focused modal just demanded attention (notification, alarm): the
  // immediate touch response is intentional.
  if (modal_manager_get_enabled() &&
      !(modal_manager_get_properties() & ModalProperty_Unfocused)) {
    return true;
  }
  // A lit backlight means something (button, shake, wake gesture) already
  // signalled engagement.
  return light_is_on();
#endif
}

void touch_session_reset(void) {
  s_armed_until = 0;
}
