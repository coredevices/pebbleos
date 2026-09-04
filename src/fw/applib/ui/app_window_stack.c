/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "app_window_stack.h"

#include "window.h"
#include "window_stack.h"
#include "window_stack_private.h"

#include "console/prompt.h"
#include "kernel/event_loop.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include <pbl/logging/logging.h>

#include "FreeRTOS.h"
#include "semphr.h"

void app_window_stack_push(Window *window, bool animated) {
  PBL_LOG_DBG("Pushing window %p onto app window stack %p",
      window, app_state_get_window_stack());
  window_stack_push(app_state_get_window_stack(), window, animated);
}

void app_window_stack_insert_next(Window *window) {
  window_stack_insert_next(app_state_get_window_stack(), window);
}

Window *app_window_stack_pop(bool animated) {
  return window_stack_pop(app_state_get_window_stack(), animated);
}

void app_window_stack_pop_all(const bool animated) {
  window_stack_pop_all(app_state_get_window_stack(), animated);
}

bool app_window_stack_remove(Window *window, bool animated) {
  return window_stack_remove(window, animated);
}

Window *app_window_stack_get_top_window(void) {
  return window_stack_get_top_window(app_state_get_window_stack());
}

bool app_window_stack_contains_window(Window *window) {
  return window_stack_contains_window(app_state_get_window_stack(), window);
}

uint32_t app_window_stack_count(void) {
  return window_stack_count(app_state_get_window_stack());
}

// Commands
////////////////////////////////////

typedef struct WindowStackInfoContext {
  SemaphoreHandle_t interlock;
  WindowStackDump *dump;
  size_t count;
} WindowStackInfoContext;

static void prv_window_stack_info_cb(void *ctx) {
  // Note: Because of the nature of modal windows that has us re-using the Window Stack code for
  // everything (for simplicity), while a normal call to any of the stack functions would yield
  // us the appropriate window stack based on our current task, for the sake of this command, we
  // only care about the application's window stack, so we'll work with that directly.
  WindowStackInfoContext *info = ctx;
  WindowStack *stack = app_state_get_window_stack();
  info->count = window_stack_dump(stack, &info->dump);
  xSemaphoreGive(info->interlock);
}

void command_window_stack_info(void) {
  struct WindowStackInfoContext info = {
    .interlock = xSemaphoreCreateBinary(),
  };
  if (!info.interlock) {
    prompt_send_response("Couldn't allocate semaphore for window stack");
    return;
  }
  // FIXME: Dumping the app window stack from another task without a
  // lock exposes us to the possibility of catching the window stack in
  // an inconsistent state. It's been like this for years without issue
  // but we could just be really lucky. Switch to the app task to dump
  // the window stack?
  launcher_task_add_callback(prv_window_stack_info_cb, &info);
  xSemaphoreTake(info.interlock, portMAX_DELAY);
  vSemaphoreDelete(info.interlock);

  if (info.count > 0 && !info.dump) {
    prompt_send_response("Couldn't allocate buffers for window stack data");
    goto cleanup;
  }

  char buffer[128];
  prompt_send_response_fmt(
      buffer, sizeof(buffer),
      "Window Stack, top to bottom: (%zu)", info.count);
  for (size_t i = 0; i < info.count; ++i) {
    prompt_send_response_fmt(buffer, sizeof(buffer), "window %p <%s>",
                             info.dump[i].addr, info.dump[i].name);
  }
cleanup:
  kernel_free(info.dump);
}
