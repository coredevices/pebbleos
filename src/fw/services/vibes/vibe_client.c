/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/vibes/vibe_client.h"

#include "pbl/services/notifications/alerts_preferences_private.h"
#include "pbl/services/vibes/vibe_score.h"
#include "pbl/services/vibes/vibe_score_info.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DEFINE(service_vibes, CONFIG_SERVICE_VIBES_LOG_LEVEL);

static VibeScoreId prv_get_resource_for_client(VibeClient client) {
  if (client == VibeClient_AlarmsLPM) {
    return VibeScoreId_AlarmsLPM;
  }
  return alerts_preferences_get_vibe_score_for_client(client);
}

VibeScore *vibe_client_get_score(VibeClient client) {
  VibeScoreId id = prv_get_resource_for_client(client);
  if (id == VibeScoreId_Disabled) {
    return NULL;
  }
  VibeScore *score = vibe_score_create_with_resource(vibe_score_info_get_resource_id(id));
  if (!score) {
    PBL_LOG_ERR("Got a null VibeScore resource!");
  }
  return score;
}
