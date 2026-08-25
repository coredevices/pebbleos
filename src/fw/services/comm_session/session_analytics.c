/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/comm_session/session_analytics.h"

#include <pbl/drivers/rtc.h>

#include "pbl/services/comm_session/session_internal.h"
#include "pbl/services/analytics/analytics.h"
#include "pbl/services/ping.h"
#include "util/time/time.h"

CommSessionTransportType comm_session_analytics_get_transport_type(CommSession *session) {
  return session->transport_imp->get_type(session->transport);
}

void comm_session_analytics_open_session(CommSession *session) {
  const bool is_system = (session->destination != TransportDestinationApp);
  if (is_system) {
    PBL_ANALYTICS_TIMER_START(connectivity_expected_time_ms);
    PBL_ANALYTICS_TIMER_START(connectivity_connected_time_ms);
  }
  session->open_ticks = rtc_get_ticks();
}

void comm_session_analytics_close_session(CommSession *session, CommSessionCloseReason reason) {
  const bool is_system = (session->destination != TransportDestinationApp);
  if (is_system) {
    PBL_ANALYTICS_TIMER_START(connectivity_expected_time_ms);
    PBL_ANALYTICS_TIMER_STOP(connectivity_connected_time_ms);
  }

}