/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "prompt.h"
#include "prompt_commands.h"

#include "console_internal.h"
#include "dbgserial.h"
#include <pbl/drivers/rtc.h>
#include "pulse_protocol_impl.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "pbl/util/likely.h"

#include "pbl/services/system_task.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#define PROMPT_RESP_ACK (100)
#define PROMPT_RESP_DONE (101)
#define PROMPT_RESP_MESSAGE (102)

static void start_prompt(void);
static void pulse_send_message(const int message_type, const char* response);
static void prv_pulse_done_command(void);

static void prv_dbgserial_response_callback(const char* response) {
  if (serial_console_get_state() == SERIAL_CONSOLE_STATE_PULSE) {
    pulse_send_message(PROMPT_RESP_MESSAGE, response);
  } else {
    dbgserial_putstr(response);
  }
}

static void prv_dbgserial_command_complete_callback(void) {
  SerialConsoleState state = serial_console_get_state();
  if (state == SERIAL_CONSOLE_STATE_PULSE) {
    prv_pulse_done_command();
  } else if (state == SERIAL_CONSOLE_STATE_PROMPT) {
    start_prompt();
  }
}

static PromptContext s_dbgserial_prompt_context = {
  .response_callback = prv_dbgserial_response_callback,
  .command_complete_callback = prv_dbgserial_command_complete_callback,
};

typedef enum {
  ExecutingCommandNone = 0,
  ExecutingCommandDbgSerial, //!< Currently executing command through dbgserial
  ExecutingCommandPulse,     //!< Currently executing command is through Pulse
  ExecutingCommandContext,   //!< Currently executing command is a custom \ref PromptContext
} ExecutingCommand;

static bool s_command_continues_after_return = false;

static ExecutingCommand s_executing_command = ExecutingCommandNone;

//! Currently used prompt context. This is set so that we know which response
//! and completion callbacks to use.
static PromptContext *s_current_context = NULL;

static void start_prompt(void) {
  dbgserial_putchar('>');
  s_dbgserial_prompt_context.write_index = 0;
}

void console_switch_to_prompt(void) {
  serial_console_set_state(SERIAL_CONSOLE_STATE_PROMPT);
  dbgserial_putstr("");
  start_prompt();
}

/////////////////////////////////////////////////////////////////
// Prompt infrastructure
/////////////////////////////////////////////////////////////////

typedef void (*CommandFuncNoParam)(void);
typedef void (*CommandFuncOneParam)(const char*);
typedef void (*CommandFuncTwoParams)(const char*, const char*);
typedef void (*CommandFuncThreeParams)(const char*, const char*, const char*);
typedef void (*CommandFuncFourParams)(const char*, const char*, const char*, const char*);

#define NUM_SUPPORTED_PARAM_COUNT 4

void command_help(void) {
  prompt_send_response("Available Commands:");
  char buffer[32];
  for (unsigned int i = 0; i < NUM_PROMPT_COMMANDS; ++i) {
    const Command* cmd = &s_prompt_commands[i];
    if (cmd->num_params) {
      prompt_send_response_fmt(buffer, sizeof(buffer),
                               "%s {%u args}", cmd->cmd_str, cmd->num_params);
    } else {
      prompt_send_response(cmd->cmd_str);
    }
  }
}

typedef struct CommandArgs {
  unsigned int num_args;
  const char* args[NUM_SUPPORTED_PARAM_COUNT];
} CommandArgs;

static CommandArgs prv_parse_arguments(char* buffer, char* buffer_end) {
  CommandArgs args;

  for (int i = 0; i < NUM_SUPPORTED_PARAM_COUNT; ++i) {
    // Consume leading whitespace
    while (buffer <= buffer_end && *buffer == ' ') {
      ++buffer;
    }
    if (buffer >= buffer_end) {
      args.num_args = i;
      return args;
    }

    // Yay, a param!
    args.args[i] = buffer;

    // Skip over the param and set us up for the next one.
    while (buffer < buffer_end && *buffer != ' ') {
      ++buffer;
    }
    *(buffer++) = 0;
  }

  args.num_args = NUM_SUPPORTED_PARAM_COUNT;
  return args;
}

static void prv_execute_given_command(const Command* cmd, char* param_str, char* param_str_end) {
  CommandArgs args = prv_parse_arguments(param_str, param_str_end);

  if (args.num_args != cmd->num_params) {
    char buffer[128];
    prompt_send_response_fmt(buffer, sizeof(buffer),
                             "Incorrect number of arguments: Wanted %u Got %u",
                             cmd->num_params, args.num_args);
    goto done;
  }

  if (cmd->num_params == 4) {
    ((CommandFuncFourParams) cmd->func)(args.args[0], args.args[1], args.args[2], args.args[3]);
  } else if (cmd->num_params == 3) {
    ((CommandFuncThreeParams) cmd->func)(args.args[0], args.args[1], args.args[2]);
  } else if (cmd->num_params == 2) {
    ((CommandFuncTwoParams) cmd->func)(args.args[0], args.args[1]);
  } else if (cmd->num_params == 1) {
    ((CommandFuncOneParam) cmd->func)(args.args[0]);
  } else if (cmd->num_params == 0) {
    ((CommandFuncNoParam) cmd->func)();
  }

done:
  if (!s_command_continues_after_return && s_current_context) {
    prompt_command_finish();
  }
}

static void prv_find_and_execute_command(char* cmd, size_t cmd_len, PromptContext *context) {
  if (!cmd_len) {
    // Empty command.
    s_executing_command = ExecutingCommandNone;
    return;
  }

  s_current_context = context;

  bool command_found = false;
  for (unsigned int i = 0; i < NUM_PROMPT_COMMANDS; ++i) {
    const Command *cmd_iter = &s_prompt_commands[i];

    const size_t cmd_iter_length = strlen(cmd_iter->cmd_str);
    if (cmd_len >= cmd_iter_length &&
        memcmp(cmd_iter->cmd_str, cmd, cmd_iter_length) == 0) {

      prv_execute_given_command(cmd_iter, cmd + cmd_iter_length, cmd + cmd_len);

      command_found = true;
      break;
    }
  }

  if (!command_found) {
    cmd[cmd_len] = '\0';

    char buffer[64];
    prompt_send_response_fmt(buffer, sizeof(buffer), "Invalid command <%s>! Try 'help'", cmd);
    prompt_command_finish();
  }
}

void prompt_context_execute(PromptContext *context) {
  s_executing_command = ExecutingCommandContext;
  prv_find_and_execute_command(context->buffer, context->write_index, context);

  context->write_index = 0;
}

static void prv_execute_command_from_dbgserial(void *data) {
  dbgserial_putstr("");

  char* buffer = s_dbgserial_prompt_context.buffer;

  if (s_dbgserial_prompt_context.write_index > 0 && buffer[0] == '!') {
    // Go to log mode immediately.
    serial_console_set_state(SERIAL_CONSOLE_STATE_LOGGING);
    ++buffer;
  }

  const char *end_of_buffer =
      s_dbgserial_prompt_context.buffer + s_dbgserial_prompt_context.write_index;
  const size_t buffer_length = end_of_buffer - buffer;

  prv_find_and_execute_command(buffer, buffer_length,
                               &s_dbgserial_prompt_context);
}

bool prompt_context_append_char(PromptContext *prompt_context, char c) {
  if (UNLIKELY(prompt_context->write_index + 1 >= PROMPT_BUFFER_SIZE_BYTES)) {
    return false;
  }

  prompt_context->buffer[prompt_context->write_index++] = c;
  return true;
}

// Crank up the optimization on this bad boy.
OPTIMIZE_FUNC(2) void prompt_handle_character(char c, bool* should_context_switch) {
  if (UNLIKELY(prompt_command_is_executing())) {
    return;
  }

  if (LIKELY(c >= 0x20 && c < 127)) {
    // Printable character.
    if (!prompt_context_append_char(&s_dbgserial_prompt_context, c)) {
      dbgserial_putchar(0x07); // bell
      return;
    }

    // Echo
    dbgserial_putchar_lazy(c);

    return;
  }

  // Handle unprintable control characters.

  if (UNLIKELY(c == 0x3)) { // CTRL-C
    dbgserial_putstr("");
    start_prompt(); // Start over
    return;
  }

  if (UNLIKELY(c == 0x4)) { // CTRL-D
    dbgserial_putstr("^D");
    serial_console_set_state(SERIAL_CONSOLE_STATE_LOGGING);
    return;
  }

  if (UNLIKELY(c == 0xd)) { // Enter key
    s_executing_command = ExecutingCommandDbgSerial;
    system_task_add_callback_from_isr(prv_execute_command_from_dbgserial, NULL,
                                      should_context_switch);
    return;
  }

  if (UNLIKELY(c == 0x7f)) { // Backspace
    if (s_dbgserial_prompt_context.write_index != 0) {
      s_dbgserial_prompt_context.write_index--;
      dbgserial_putchar(0x8); // move cursor back one character
      dbgserial_putchar(0x20); // replace that character with a space, advancing the cursor
      dbgserial_putchar(0x8); // move the cursor back again
    } else {
      dbgserial_putchar(0x07); // bell
    }
    return;
  }
}

bool prompt_command_is_executing(void) {
  return s_executing_command != ExecutingCommandNone;
}

void prompt_watchdog_feed(void) {
  system_task_watchdog_feed();
}

void prompt_send_response(const char* response) {
  PBL_ASSERTN(s_current_context && s_current_context->response_callback);
  s_current_context->response_callback(response);
}

void prompt_send_response_fmt(char* buffer, size_t buffer_size, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsniprintf(buffer, buffer_size, fmt, ap);
  va_end(ap);

  prompt_send_response(buffer);
}

void prompt_command_continues_after_returning(void) {
  s_command_continues_after_return = true;
}

void prompt_command_finish(void) {
  PBL_ASSERTN(s_current_context);
  PromptContext *last_context = s_current_context;
  s_current_context = NULL;
  s_command_continues_after_return = false;
  s_executing_command = ExecutingCommandNone;
  if (last_context->command_complete_callback) {
    last_context->command_complete_callback();
  }
}

/////////////////////////////////////////////////////////////////
// PULSE infrastructure
/////////////////////////////////////////////////////////////////

#ifndef CONFIG_PULSE_EVERYWHERE
static uint16_t s_latest_cookie = UINT16_MAX;

typedef struct __attribute__((__packed__)) PromptCommand {
  uint8_t cookie;
  char command[];
} PromptCommand;
#endif

typedef struct __attribute__((__packed__)) PromptResponseContents {
  uint8_t message_type;
  uint64_t time_ms;
  char message[];
} PromptResponseContents;

static void pulse_send_message(const int message_type, const char *response) {
  size_t response_length = 0;
#ifdef CONFIG_PULSE_EVERYWHERE
  PromptResponseContents *contents = pulse_reliable_send_begin(
      PULSE2_RELIABLE_PROMPT_PROTOCOL);
  if (!contents) {
    // Transport went down while waiting to send. Just throw away the message;
    // there's not much else we can do.
    return;
  }
#else
  PromptResponseContents *contents = pulse_best_effort_send_begin(
      PULSE_PROTOCOL_PROMPT);
#endif

  if (response) {
    response_length = strlen(response);
  }

  size_t total_size = sizeof(PromptResponseContents) + response_length;
  contents->message_type = message_type;

  time_t time_s;
  uint16_t time_ms;
  rtc_get_time_ms(&time_s, &time_ms);
  contents->time_ms = (uint64_t) time_s * 1000 + time_ms;

  if (response_length > 0) {
    strncpy(contents->message, response, response_length);
  }
#ifdef CONFIG_PULSE_EVERYWHERE
  pulse_reliable_send(contents, total_size);
#else
  pulse_best_effort_send(contents, total_size);
#endif
}

static void prv_pulse_done_command(void) {
  pulse_send_message(PROMPT_RESP_DONE, NULL);
  s_executing_command = ExecutingCommandNone;
}

#ifdef CONFIG_PULSE_EVERYWHERE

void pulse2_prompt_packet_handler(void *packet, size_t length) {
  if (prompt_command_is_executing()) {
    PBL_LOG_DBG("Ignoring prompt command as another command is "
            "currently executing");
    return;
  }

  s_executing_command = ExecutingCommandPulse;
  memcpy(s_dbgserial_prompt_context.buffer, packet, length);
  s_dbgserial_prompt_context.buffer[length] = '\0';
  s_dbgserial_prompt_context.write_index = strlen(s_dbgserial_prompt_context.buffer);
  system_task_add_callback(prv_execute_command_from_dbgserial, NULL);
}

#else

void pulse_prompt_handler(void *packet, size_t length) {
  PromptCommand *command = packet;
  // Check for duplicate command and ignore it
  if (s_latest_cookie == command->cookie) {
    if (s_executing_command == ExecutingCommandPulse) {
      pulse_send_message(PROMPT_RESP_ACK, NULL);
    } else {
      pulse_send_message(PROMPT_RESP_DONE, NULL);
    }

    return;
  }

  // ACK the command
  pulse_send_message(PROMPT_RESP_ACK, NULL);
  s_latest_cookie = command->cookie;
  s_executing_command = ExecutingCommandPulse;

  // Discount the size of the cookie
  size_t command_length = length - sizeof(PromptCommand);
  strncpy(s_dbgserial_prompt_context.buffer, command->command, command_length);
  s_dbgserial_prompt_context.buffer[command_length] = 0;
  s_dbgserial_prompt_context.write_index = strlen(s_dbgserial_prompt_context.buffer);

  system_task_add_callback(prv_execute_command_from_dbgserial, NULL);
}

void pulse_prompt_link_state_handler(PulseLinkState link_state) {
  if (link_state != PulseLinkState_Open) {
    return;
  }

  // Reset the cookie to an 'impossible' value on link open
  s_latest_cookie = UINT16_MAX;
}

#endif
