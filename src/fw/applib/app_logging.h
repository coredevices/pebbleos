/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

/*!
  @file app_logging.h
  @brief Interface for the SDK's App Logging API.
*/

#pragma once

#include "pbl/util/uuid.h"

// FIXME PBL-1629: move needed declarations into applib
#include "logging/logging_private.h"

#include <stdint.h>

//! @addtogroup Foundation
//! @{
//!   @addtogroup Logging Logging
//!   \brief Functions related to logging from apps.
//!
//! This module contains the functions necessary to log messages through
//! Bluetooth.
//! @note It is no longer necessary to enable app logging output from the "settings->about" menu on the Pebble for
//! them to be transmitted!  Instead use the "pebble logs" command included with the SDK to activate logs.  The logs
//! will appear right in your console. Logging
//! over Bluetooth is a fairly power hungry operation that non-developers will
//! not need when your apps are distributed.
//!   @{

// @internal
// Log an app message, takes a va_list rather than varags
// @see app_log
void app_log_vargs(uint8_t log_level, const char *src_filename, int src_line_number,
                   const char *fmt, va_list args);

//! Log an app message.
//! @param log_level
//! @param src_filename The source file where the log originates from
//! @param src_line_number The line number in the source file where the log originates from
//! @param fmt A C formatting string
//! @param ... The arguments for the formatting string
//! \sa snprintf for details about the C formatting string.
#if __clang__
void app_log(uint8_t log_level, const char* src_filename, int src_line_number, const char* fmt,
             ...);
#else
void app_log(uint8_t log_level, const char* src_filename, int src_line_number, const char* fmt,
             ...) __attribute__((format(printf, 4, 5)));
#endif

//! A helper macro that simplifies the use of the app_log function
//! @param level The log level to log output as
//! @param fmt A C formatting string
//! @param args The arguments for the formatting string
#define APP_LOG(level, fmt, args...)                                \
  app_log(level, __FILE_NAME__, __LINE__, fmt, ## args)

//! Suggested log level values
typedef enum {
  //! Error level log message
  APP_LOG_LEVEL_ERROR = 1,
  //! Warning level log message
  APP_LOG_LEVEL_WARNING = 50,
  //! Info level log message
  APP_LOG_LEVEL_INFO = 100,
  //! Debug level log message
  APP_LOG_LEVEL_DEBUG = 200,
  //! Verbose Debug level log message
  APP_LOG_LEVEL_DEBUG_VERBOSE = 255,
} AppLogLevel;

//!   @}
//! @}

// @internal
// Returns true if app logging over Bluetooth is enabled (someone is listening)
bool app_log_is_bt_enabled(void);

typedef enum AppLoggingMode {
  AppLoggingDisabled = 0,
  AppLoggingEnabled = 1,
  NumAppLoggingModes
} AppLoggingMode;

typedef struct __attribute__((__packed__)) AppLogBinaryMessage {
  Uuid uuid;
  LogBinaryMessage log_msg;
} AppLogBinaryMessage;
