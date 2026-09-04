/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/events.h"
#include "pbl/services/compositor/compositor.h"

void watchface_init(void);

void watchface_handle_button_event(PebbleEvent *e);

AppInstallId watchface_get_default_install_id(void);

void watchface_launch_default(const CompositorTransition *animation);

void watchface_set_timeline_peek_enabled(bool visible);

void watchface_reset_click_manager(void);
