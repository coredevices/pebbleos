/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/voice/dictation_session.h"
#include "applib/voice/dictation_session_private.h"
#include "applib/voice/voice_window.h"
#include "kernel/events.h"
#include "kernel/pebble_tasks.h"
#include "pbl/services/comm_session/session.h"
#include "process_management/app_install_manager.h"
#include "stubs_logging.h"
#include "stubs_passert.h"

#include <stdlib.h>

static EventServiceEventHandler s_result_handler;
static void *s_result_context;
static int s_callback_calls;
static int s_window_reset_calls;
static uint8_t s_window_event_id;

void *_applib_type_malloc_DictationSession(void) {
  return calloc(1, sizeof(DictationSession));
}

void *applib_malloc(size_t size) {
  return malloc(size);
}

void applib_free(void *ptr) {
  free(ptr);
}

void event_service_client_subscribe(EventServiceInfo *service_info) {
  if (service_info->type == PEBBLE_DICTATION_EVENT) {
    s_result_handler = service_info->handler;
    s_result_context = service_info->context;
  }
}

void event_service_client_unsubscribe(EventServiceInfo *service_info) {
}

VoiceWindow *voice_window_create(char *buffer, size_t buffer_size,
                                 VoiceEndpointSessionType session_type) {
  return (VoiceWindow *)1;
}

void voice_window_destroy(VoiceWindow *voice_window) {
}

uint8_t voice_window_get_event_id(VoiceWindow *voice_window) {
  return s_window_event_id;
}

void voice_window_lose_focus(VoiceWindow *voice_window) {
}

void voice_window_regain_focus(VoiceWindow *voice_window) {
}

void voice_window_reset(VoiceWindow *voice_window) {
  s_window_reset_calls++;
}

DictationSessionStatus voice_window_push(VoiceWindow *voice_window) {
  return DictationSessionStatusSuccess;
}

void voice_window_pop(VoiceWindow *voice_window) {
}

void voice_window_set_confirmation_enabled(VoiceWindow *voice_window, bool enabled) {
}

void voice_window_set_error_enabled(VoiceWindow *voice_window, bool enabled) {
}

PebbleTask pebble_task_get_current(void) {
  return PebbleTask_KernelMain;
}

bool app_install_id_from_system(AppInstallId id) {
  return true;
}

AppInstallId sys_process_manager_get_current_process_id(void) {
  return 1;
}

bool sys_system_pp_has_capability(CommSessionCapability capability) {
  return true;
}

static void prv_result_callback(DictationSession *session, DictationSessionStatus status,
                                char *transcription, void *context) {
  s_callback_calls++;
}

void test_dictation_session__initialize(void) {
  s_result_handler = NULL;
  s_result_context = NULL;
  s_callback_calls = 0;
  s_window_reset_calls = 0;
  s_window_event_id = 3;
}

void test_dictation_session__cleanup(void) {
}

void test_dictation_session__ignores_result_from_another_voice_window(void) {
  DictationSession *session = dictation_session_create(0, prv_result_callback, NULL);
  cl_assert(session);
  cl_assert_equal_i(dictation_session_start(session), DictationSessionStatusSuccess);
  cl_assert(s_result_handler);

  PebbleEvent event = {
      .type = PEBBLE_DICTATION_EVENT,
      .dictation = {
          .result = DictationSessionStatusSuccess,
          .source_id = 4,
          .text = "wrong window",
      },
  };
  s_result_handler(&event, s_result_context);

  cl_assert_equal_i(s_callback_calls, 0);
  cl_assert_equal_i(s_window_reset_calls, 0);

  event.dictation.source_id = s_window_event_id;
  event.dictation.text = "right window";
  s_result_handler(&event, s_result_context);

  cl_assert_equal_i(s_callback_calls, 1);
  cl_assert_equal_i(s_window_reset_calls, 1);
  dictation_session_destroy(session);
}
