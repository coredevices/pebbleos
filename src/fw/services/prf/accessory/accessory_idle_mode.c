/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "accessory_idle_mode.h"

#include "drivers/accessory.h"
#include "mfg/mfg_mode/mfg_factory_mode.h"
#include "services/common/system_task.h"
#include "system/logging.h"

#if PLATFORM_SNOWY || PLATFORM_SPALDING
static const char KNOCKING_CODE[] = "sn0wy";
#elif PLATFORM_SILK
static const char KNOCKING_CODE[] = "s1lk";
#elif PLATFORM_ASTERIX
static const char KNOCKING_CODE[] = "aster1x";
#elif PLATFORM_OBELIX
static const char KNOCKING_CODE[] = "0belix";
#elif PLATFORM_ROBERT
static const char KNOCKING_CODE[] = "r0bert";
#elif PLATFORM_CALCULUS
static const char KNOCKING_CODE[] = "c@lculus";
#else
#error "Unknown platform"
#endif

static void prv_knocking_complete(void *data) {
  mfg_enter_mfg_mode_and_launch_app();
}

bool accessory_idle_mode_handle_char(char c) {
  // Note: You're in an interrupt here, be careful

  static int s_knocking_state = 0;

  bool should_context_switch = false;

  if (KNOCKING_CODE[s_knocking_state] == c) {
    // This character matched! We're now looking for the next character.
    ++s_knocking_state;

    PBL_LOG(LOG_LEVEL_DEBUG, "Idle: <%c> Match! State %u", c, s_knocking_state);

    // If we reach the null terminator, we're done!
    if (KNOCKING_CODE[s_knocking_state] == 0) {
      system_task_add_callback_from_isr(
          prv_knocking_complete, NULL, &should_context_switch);

      s_knocking_state = 0;
    }
  } else {
    PBL_LOG(LOG_LEVEL_DEBUG, "Idle: <%c> Mismatch!", c);

    // Wrong character, reset
    s_knocking_state = 0;
  }

  return should_context_switch;
}

