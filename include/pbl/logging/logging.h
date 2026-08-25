/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "console/dbgserial.h"
#include "system/die.h"
#include "system/reboot_reason.h"
#include "system/status_codes.h"


#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define SPLIT_64_BIT_ARG(x) (uint32_t)((x >> 32) & 0xFFFFFFFF), (uint32_t)(x & 0xFFFFFFFF)

#define LOG_BUFFER_LENGTH 128

// Minimum amount of stack space required for vsniprintf
#define LOGGING_MIN_STACK_FOR_SPRINTF  240
#define LOGGING_STACK_FULL_MSG  ((const char *)" [STK FULL]")

void pbl_log_hashed_async(const uint32_t packed_loghash, ...);

void pbl_log_hashed_sync(const uint32_t packed_loghash, ...);

// Core Number must be shifted to the correct position.
void pbl_log_hashed_core(const uint32_t core_number, const uint32_t packed_loghash, ...);

// Core Number must be shifted to the correct position.
void pbl_log_hashed_vargs(const bool async, const uint32_t core_number,
                          const uint32_t packed_loghash, va_list fmt_args);

void pbl_log_vargs(uint8_t log_level, const char *src_filename, int src_line_number,
                   const char *fmt, va_list args);

void pbl_log(uint8_t log_level, const char* src_filename, int src_line_number, const char* fmt, ...)
    FORMAT_PRINTF(4, 5);

void pbl_log_sync(uint8_t log_level, const char* src_filename, int src_line_number, const char* fmt, ...)
    FORMAT_PRINTF(4, 5);

int pbl_log_binary_format(char* buffer, int buffer_len, const uint8_t log_level,
    const char* src_filename_path, int src_line_number, const char* fmt, va_list args);

int pbl_log_get_bin_format(char* buffer, int buffer_len, const uint8_t log_level,
    const char* src_filename_path, int src_line_number, const char* fmt, ...);

#define LOG_LEVEL_ALWAYS 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARNING 50
#define LOG_LEVEL_INFO 100
#define LOG_LEVEL_DEBUG 200
#define LOG_LEVEL_DEBUG_VERBOSE 255

#if defined(CONFIG_DEFAULT_LOG_LEVEL_ERROR)
  #define DEFAULT_LOG_LEVEL LOG_LEVEL_ERROR
#elif defined(CONFIG_DEFAULT_LOG_LEVEL_WARNING)
  #define DEFAULT_LOG_LEVEL LOG_LEVEL_WARNING
#elif defined(CONFIG_DEFAULT_LOG_LEVEL_INFO)
  #define DEFAULT_LOG_LEVEL LOG_LEVEL_INFO
#elif defined(CONFIG_DEFAULT_LOG_LEVEL_DEBUG_VERBOSE)
  #define DEFAULT_LOG_LEVEL LOG_LEVEL_DEBUG_VERBOSE
#else
  #define DEFAULT_LOG_LEVEL LOG_LEVEL_DEBUG
#endif

#if defined(CONFIG_FLASH_LOG_LEVEL_ERROR)
  #define FLASH_LOG_LEVEL LOG_LEVEL_ERROR
#elif defined(CONFIG_FLASH_LOG_LEVEL_WARNING)
  #define FLASH_LOG_LEVEL LOG_LEVEL_WARNING
#elif defined(CONFIG_FLASH_LOG_LEVEL_DEBUG)
  #define FLASH_LOG_LEVEL LOG_LEVEL_DEBUG
#elif defined(CONFIG_FLASH_LOG_LEVEL_DEBUG_VERBOSE)
  #define FLASH_LOG_LEVEL LOG_LEVEL_DEBUG_VERBOSE
#else
  #define FLASH_LOG_LEVEL LOG_LEVEL_INFO
#endif

#ifdef CONFIG_LOG_HASHED
  #include <pbl/logging/log_hashing.h>
#endif

#define LOG_COLOR_BLACK          "BLACK"        // Not so useful in general
#define LOG_COLOR_RED            "RED"
#define LOG_COLOR_GREEN          "GREEN"
#define LOG_COLOR_YELLOW         "YELLOW"
#define LOG_COLOR_BLUE           "BLUE"
#define LOG_COLOR_MAGENTA        "MAGENTA"
#define LOG_COLOR_CYAN           "CYAN"
#define LOG_COLOR_GREY           "GREY"
// Reserved for bold. Use sparingly
#define LOG_COLOR_LIGHT_GREY     "LIGHT_GREY"
#define LOG_COLOR_LIGHT_RED      "LIGHT_RED"
#define LOG_COLOR_LIGHT_GREEN    "LIGHT_GREEN"
#define LOG_COLOR_LIGHT_YELLOW   "LIGHT_YELLOW"
#define LOG_COLOR_LIGHT_BLUE     "LIGHT_BLUE"
#define LOG_COLOR_LIGHT_MAGENTA  "LIGHT_MAGENTA"
#define LOG_COLOR_LIGHT_CYAN     "LIGHT_CYAN"
#define LOG_COLOR_WHITE          "WHITE"

// Level-to-color mapping (fixed per level)
#define LOG_COLOR_FOR_ALWAYS  LOG_COLOR_BLUE
#define LOG_COLOR_FOR_ERROR   LOG_COLOR_RED
#define LOG_COLOR_FOR_WARNING LOG_COLOR_YELLOW
#define LOG_COLOR_FOR_INFO    LOG_COLOR_GREEN
#define LOG_COLOR_FOR_DEBUG   LOG_COLOR_GREY
#define LOG_COLOR_FOR_VERBOSE LOG_COLOR_GREY

#define LOG_DOMAIN_MISC               1
#define LOG_DOMAIN_FS                 1
#define LOG_DOMAIN_COMM               1
#define LOG_DOMAIN_TEXT               0
#define LOG_DOMAIN_ANIMATION          0
#define LOG_DOMAIN_ANALYTICS          0
#define LOG_DOMAIN_ACTIVITY           0
#define LOG_DOMAIN_ACTIVITY_INSIGHTS  0
#define LOG_DOMAIN_PROTOBUF           0

#define LOG_DOMAIN_BLOBDB             0

#ifndef LOG_DOMAIN_BT_PAIRING_INFO
#ifndef CONFIG_RELEASE
  #define LOG_DOMAIN_BT_PAIRING_INFO  1
#else
  #define LOG_DOMAIN_BT_PAIRING_INFO  0
#endif
#endif

#ifndef LOG_DOMAIN_BT_STACK
  #define LOG_DOMAIN_BT_STACK        0
#endif

#ifndef LOG_DOMAIN_DATA_LOGGING
  #define LOG_DOMAIN_DATA_LOGGING     0
#endif

#ifndef LOG_DOMAIN_I2C
  #define LOG_DOMAIN_I2C              0
#endif

#ifndef STRINGIFY
  #define STRINGIFY_NX(a) #a
  #define STRINGIFY(a) STRINGIFY_NX(a)
#endif // STRINGIFY

#define STATUS_STRING(s) STRINGIFY(s)

#ifndef DEFAULT_LOG_DOMAIN
  #define DEFAULT_LOG_DOMAIN LOG_DOMAIN_MISC
#endif // DEFAULT_LOG_DOMAIN

// Per-module compile-time log level and name. PBL_LOG_MODULE_DEFINE /
// PBL_LOG_MODULE_DECLARE override these tentative definitions, e.g.
// PBL_LOG_MODULE_DEFINE(service_activity, CONFIG_SERVICE_ACTIVITY_LOG_LEVEL) (see
// Kconfig.template.log_level); level 0 selects DEFAULT_LOG_LEVEL.
__attribute__((unused)) static const uint8_t _pbl_log_module_level;
__attribute__((unused)) static const char *const _pbl_log_module_name;

// Unit tests build with CONFIG_LOG but without the board Kconfig symbols,
// so module levels fall back to the default there.
#if defined(CONFIG_LOG) && !defined(UNITTEST)
  #ifdef CONFIG_LOG_HASHED
    // The MODULE map entry gives the loghash dict generator the
    // file -> module mapping; the module name costs nothing at runtime.
    #define PBL_LOG_MODULE_DEFINE(name, level) \
      __attribute__((unused)) static const uint8_t _pbl_log_module_level = (level); \
      __attribute__((unused)) static const char *const _pbl_log_module_name = #name; \
      __attribute__((used, nocommon, section(".log_strings"))) \
      static const char _pbl_log_module_map[] = "MODULE:" __FILE__ ":" #name
  #else
    #define PBL_LOG_MODULE_DEFINE(name, level) \
      __attribute__((unused)) static const uint8_t _pbl_log_module_level = (level); \
      __attribute__((unused)) static const char *const _pbl_log_module_name = #name
  #endif
#else
  #define PBL_LOG_MODULE_DEFINE(name, level) \
    __attribute__((unused)) static const uint8_t _pbl_log_module_level = 0
#endif

#define PBL_LOG_MODULE_DECLARE(name, level) PBL_LOG_MODULE_DEFINE(name, level)

#define PBL_SHOULD_LOG(level) \
  ((level) <= (_pbl_log_module_level != 0 ? _pbl_log_module_level : DEFAULT_LOG_LEVEL))


// Internal implementation macros (use level-named macros below instead)
#ifdef CONFIG_LOG
  #ifdef CONFIG_LOG_HASHED
    #define PBL_LOG_COLOR_D(domain, level, color, fmt, ...)                       \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          if (domain) { \
            NEW_LOG_HASH(pbl_log_hashed_async, level, color, fmt, ## __VA_ARGS__); \
          } \
        } \
      } while (0)

    #define PBL_LOG_COLOR_D_SYNC(domain, level, color, fmt, ...)                 \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          if (domain) { \
            NEW_LOG_HASH(pbl_log_hashed_sync, level, color, fmt, ## __VA_ARGS__); \
          } \
        } \
      } while (0)
  #else
    #define PBL_LOG_COLOR_D(domain, level, color, fmt, ...) \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          if (domain) { \
            if (_pbl_log_module_name != NULL) { \
              pbl_log(level, __FILE__, __LINE__, "%s: " fmt, _pbl_log_module_name, \
                      ## __VA_ARGS__); \
            } else { \
              pbl_log(level, __FILE__, __LINE__, fmt, ## __VA_ARGS__); \
            } \
          } \
        } \
      } while (0)

    #define PBL_LOG_COLOR_D_SYNC(domain, level, color, fmt, ...) \
      do { \
        if (PBL_SHOULD_LOG(level)) { \
          if (domain) { \
            if (_pbl_log_module_name != NULL) { \
              pbl_log_sync(level, __FILE__, __LINE__, "%s: " fmt, _pbl_log_module_name, \
                           ## __VA_ARGS__); \
            } else { \
              pbl_log_sync(level, __FILE__, __LINE__, fmt, ## __VA_ARGS__); \
            } \
          } \
        } \
      } while (0)
  #endif
#else // !CONFIG_LOG
  #define PBL_LOG_COLOR_D(domain, level, color, fmt, ...)
  #define PBL_LOG_COLOR_D_SYNC(domain, level, color, fmt, ...)
#endif // CONFIG_LOG

// Level-named domain macros (async)
#define PBL_LOG_D_ALWAYS(domain, fmt, ...) \
  PBL_LOG_COLOR_D(domain, LOG_LEVEL_ALWAYS, LOG_COLOR_FOR_ALWAYS, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_ERR(domain, fmt, ...) \
  PBL_LOG_COLOR_D(domain, LOG_LEVEL_ERROR, LOG_COLOR_FOR_ERROR, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_WRN(domain, fmt, ...) \
  PBL_LOG_COLOR_D(domain, LOG_LEVEL_WARNING, LOG_COLOR_FOR_WARNING, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_INFO(domain, fmt, ...) \
  PBL_LOG_COLOR_D(domain, LOG_LEVEL_INFO, LOG_COLOR_FOR_INFO, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_DBG(domain, fmt, ...) \
  PBL_LOG_COLOR_D(domain, LOG_LEVEL_DEBUG, LOG_COLOR_FOR_DEBUG, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_VERBOSE(domain, fmt, ...) \
  PBL_LOG_COLOR_D(domain, LOG_LEVEL_DEBUG_VERBOSE, LOG_COLOR_FOR_VERBOSE, fmt, ## __VA_ARGS__)

// Level-named macros (default domain, async)
#define PBL_LOG_ALWAYS(fmt, ...) \
  PBL_LOG_D_ALWAYS(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_ERR(fmt, ...) \
  PBL_LOG_D_ERR(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_WRN(fmt, ...) \
  PBL_LOG_D_WRN(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_INFO(fmt, ...) \
  PBL_LOG_D_INFO(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_DBG(fmt, ...) \
  PBL_LOG_D_DBG(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_VERBOSE(fmt, ...) \
  PBL_LOG_D_VERBOSE(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)

// Level-named domain sync macros
#define PBL_LOG_D_SYNC_ALWAYS(domain, fmt, ...) \
  PBL_LOG_COLOR_D_SYNC(domain, LOG_LEVEL_ALWAYS, LOG_COLOR_FOR_ALWAYS, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_SYNC_ERR(domain, fmt, ...) \
  PBL_LOG_COLOR_D_SYNC(domain, LOG_LEVEL_ERROR, LOG_COLOR_FOR_ERROR, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_SYNC_WRN(domain, fmt, ...) \
  PBL_LOG_COLOR_D_SYNC(domain, LOG_LEVEL_WARNING, LOG_COLOR_FOR_WARNING, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_SYNC_INFO(domain, fmt, ...) \
  PBL_LOG_COLOR_D_SYNC(domain, LOG_LEVEL_INFO, LOG_COLOR_FOR_INFO, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_SYNC_DBG(domain, fmt, ...) \
  PBL_LOG_COLOR_D_SYNC(domain, LOG_LEVEL_DEBUG, LOG_COLOR_FOR_DEBUG, fmt, ## __VA_ARGS__)
#define PBL_LOG_D_SYNC_VERBOSE(domain, fmt, ...) \
  PBL_LOG_COLOR_D_SYNC(domain, LOG_LEVEL_DEBUG_VERBOSE, LOG_COLOR_FOR_VERBOSE, fmt, ## __VA_ARGS__)

// Level-named sync macros (default domain)
#define PBL_LOG_SYNC_ALWAYS(fmt, ...) \
  PBL_LOG_D_SYNC_ALWAYS(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_ERR(fmt, ...) \
  PBL_LOG_D_SYNC_ERR(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_WRN(fmt, ...) \
  PBL_LOG_D_SYNC_WRN(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_INFO(fmt, ...) \
  PBL_LOG_D_SYNC_INFO(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_DBG(fmt, ...) \
  PBL_LOG_D_SYNC_DBG(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)
#define PBL_LOG_SYNC_VERBOSE(fmt, ...) \
  PBL_LOG_D_SYNC_VERBOSE(DEFAULT_LOG_DOMAIN, fmt, ## __VA_ARGS__)

#ifdef CONFIG_LOG
  #define RETURN_STATUS_D(d, st) \
    do { \
      if (FAILED(st)) { \
        PBL_LOG_D_WRN(d, "%d", (int)(st)); \
      } \
      return st; \
    } while (0)

    #define RETURN_STATUS_UP_D(d, st) \
      return ((st) != E_INVALID_ARGUMENT ? (st) : E_INTERNAL)
#else // CONFIG_LOG
  #define RETURN_STATUS_D(d, st) return (st)
  #define RETURN_STATUS_UP_D(d, st) \
    return ((st) == E_INVALID_ARGUMENT ? E_INTERNAL : (st))
#endif // CONFIG_LOG

#define RETURN_STATUS(s) RETURN_STATUS_D(DEFAULT_LOG_DOMAIN, s)
#define RETURN_STATUS_UP(s) RETURN_STATUS_UP_D(DEFAULT_LOG_DOMAIN, s)
