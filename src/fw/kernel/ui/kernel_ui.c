/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "kernel_ui.h"

#include "kernel/kernel_applib_state.h"
#include "kernel/pebble_tasks.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/compositor/compositor.h"
#include "system/passert.h"

#include "applib/ui/animation_private.h"

static GContext s_kernel_grahics_context;

T_STATIC ContentIndicatorsBuffer s_kernel_content_indicators_buffer;

static TimelineItemActionSource s_kernel_current_timeline_item_action_source;

void kernel_ui_init(void) {
  graphics_context_init(&s_kernel_grahics_context, compositor_get_framebuffer(),
                        GContextInitializationMode_System);
  animation_private_state_init(kernel_applib_get_animation_state());
  content_indicator_init_buffer(&s_kernel_content_indicators_buffer);
  s_kernel_current_timeline_item_action_source = TimelineItemActionSourceModalNotification;
}

GContext* kernel_ui_get_graphics_context(void) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  return &s_kernel_grahics_context;
}

GContext *graphics_context_get_current_context(void) {
  if (pebble_task_get_current() == PebbleTask_App) {
    return app_state_get_graphics_context();
  } else {
    return kernel_ui_get_graphics_context();
  }
}

ContentIndicatorsBuffer *kernel_ui_get_content_indicators_buffer(void) {
  PBL_ASSERT_TASK(PebbleTask_KernelMain);

  return &s_kernel_content_indicators_buffer;
}

ContentIndicatorsBuffer *content_indicator_get_current_buffer(void) {
  if (pebble_task_get_current() == PebbleTask_App) {
    return app_state_get_content_indicators_buffer();
  } else {
    return kernel_ui_get_content_indicators_buffer();
  }
}

TimelineItemActionSource kernel_ui_get_current_timeline_item_action_source(void) {
  if (pebble_task_get_current() == PebbleTask_App) {
    return app_state_get_current_timeline_item_action_source();
  } else {
    return s_kernel_current_timeline_item_action_source;
  }
}

void kernel_ui_set_current_timeline_item_action_source(TimelineItemActionSource current_source) {
  if (pebble_task_get_current() == PebbleTask_App) {
    app_state_set_current_timeline_item_action_source(current_source);
  } else {
    s_kernel_current_timeline_item_action_source = current_source;
  }
}
