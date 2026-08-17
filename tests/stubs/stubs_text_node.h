/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "apps/system/timeline/text_node.h"

void graphics_text_node_destroy(GTextNode *node) { }

void graphics_text_node_get_size(GTextNode *node, GContext *ctx, const GRect *box,
                                 const GTextNodeDrawConfig *config, GSize *size_out) { }

void graphics_text_node_draw(GTextNode *node, GContext *ctx, const GRect *box,
                             const GTextNodeDrawConfig *config, GSize *size_out) { }

bool graphics_text_node_container_add_child(GTextNodeContainer *parent, GTextNode *child) {
  return 0;
}

GTextNodeText *graphics_text_node_create_text(size_t buffer_size) { return NULL; }

GTextNodeHorizontal *graphics_text_node_create_horizontal(size_t max_nodes) { return NULL; }

GTextNodeCustom *graphics_text_node_create_custom(GTextNodeDrawCallback callback,
                                                  void *user_data) { return NULL; }
