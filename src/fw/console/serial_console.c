/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "serial_console.h"

#include "console/dbgserial_input.h"
#include "console/pulse_protocol_impl.h"
#include "console_internal.h"
#include "prompt.h"

#include "console/pulse_internal.h"
#include "pbl/kernel/types.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

SerialConsoleState s_serial_console_state = SERIAL_CONSOLE_STATE_LOGGING;
static bool s_serial_console_initialized;

static bool s_prompt_enabled = false;

static void logging_handle_character(char c, bool* should_context_switch) {
#ifndef CONFIG_PROMPT
  return;
#endif
  // Remember, you're in an interrupt here!

  if (c == 0x3) { // CTRL-C
    if (!s_prompt_enabled) {
      PBL_LOG_DBG("Ignoring prompt request, not yet ready!");
      return;
    }
    console_switch_to_prompt();
  }
}

void serial_console_init(void) {
  if (s_serial_console_initialized) {
    return;
  }

  dbgserial_register_character_callback(logging_handle_character);

  s_serial_console_state = SERIAL_CONSOLE_STATE_LOGGING;
  s_serial_console_initialized = true;
}

bool serial_console_is_prompt_enabled(void) {
  if (!s_serial_console_initialized) {
    return false;
  }

  return (s_serial_console_state == SERIAL_CONSOLE_STATE_PROMPT);
}

bool serial_console_is_logging_enabled(void) {
  if (!s_serial_console_initialized) {
    return true;
  }

  return s_serial_console_state == SERIAL_CONSOLE_STATE_LOGGING ||
         s_serial_console_state == SERIAL_CONSOLE_STATE_PULSE;
}

void serial_console_enable_prompt(void) {
  s_prompt_enabled = true;
}

void serial_console_write_log_message(const char* msg) {
  while (*msg) {
    dbgserial_putchar(*(msg++));
  }
}

void serial_console_set_state(SerialConsoleState new_state) {
  PBL_ASSERTN(s_serial_console_initialized);
  PBL_ASSERTN(new_state < SERIAL_CONSOLE_NUM_STATES);

  // This function is called from the USART3 IRQ, the new timer thread,
  // and the system task. It thus needs a critical section.
  pbl_irq_lock();

  if (new_state == s_serial_console_state) {
    pbl_irq_unlock();
    return;
  }

  s_serial_console_state = new_state;

  switch (s_serial_console_state) {
#ifdef CONFIG_PROMPT
    case SERIAL_CONSOLE_STATE_PROMPT:
      dbgserial_register_character_callback(prompt_handle_character);
      dbgserial_set_rx_dma_enabled(false);
      break;
#endif
    case SERIAL_CONSOLE_STATE_LOGGING:
      dbgserial_register_character_callback(logging_handle_character);
      dbgserial_set_rx_dma_enabled(false);
      break;
    case SERIAL_CONSOLE_STATE_PULSE:
      dbgserial_register_character_callback(pulse_handle_character);
      dbgserial_set_rx_dma_enabled(true);
      break;
    default:
      WTF; // Don't know this state
  }

  pbl_irq_unlock();
}

SerialConsoleState serial_console_get_state(void) {
  SerialConsoleState state = __atomic_load_n(&s_serial_console_state, __ATOMIC_RELAXED);

  return state;
}

void serial_console_set_rx_enabled(bool enabled) {
  pbl_irq_lock();
  dbgserial_set_input_enabled(enabled);
  pbl_irq_unlock();
}
