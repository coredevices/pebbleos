/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "apps/system/timeline/layer.h"
#include "pbl/util/attributes.h"

uint16_t WEAK timeline_layer_get_ideal_sidebar_width(void) {
  return 0;
}

bool WEAK timeline_layer_sidebar_is_on_right(void) {
  return true;
}
