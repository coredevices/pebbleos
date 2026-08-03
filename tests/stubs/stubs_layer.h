/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/layer.h"

#include "pbl/util/attributes.h"

#include "stubs_unobstructed_area.h"

WEAK void layer_init(Layer *layer, const GRect *frame) { }

WEAK void layer_deinit(Layer *layer) { }

WEAK void layer_add_child(Layer *parent, Layer *child) { }

WEAK void layer_mark_dirty(Layer *layer) { }

WEAK void layer_set_update_proc(Layer *layer, LayerUpdateProc update_proc) { }

WEAK bool layer_is_status_bar_layer(Layer *layer) {
  return false;
}
