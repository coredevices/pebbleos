/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "action_bar_layer.h"

#include "applib/graphics/gtypes.h"

#include <stdbool.h>
#include <stdint.h>

//! Firmware-only: whether the action bar sits on the right edge of the window.
//! Third-party apps always get the right edge so existing layouts keep working.
//! System and kernel UI follow left-hand display orientation.
bool action_bar_layer_is_on_right(void);

//! X origin of the content area beside the action bar (0 when the bar is on the right).
int16_t action_bar_layer_get_content_origin_x(void);

//! Copy of \a bounds with the action-bar strip removed from the button side.
GRect action_bar_layer_inset_bounds(GRect bounds);

//! Horizontal insets that leave \a other_margin on the content side and room for the bar.
GEdgeInsets action_bar_layer_content_insets(int16_t other_margin);
