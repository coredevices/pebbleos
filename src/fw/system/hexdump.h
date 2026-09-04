/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/hexdump.h"

#include <stddef.h>
#include <stdint.h>

void hexdump_log(int level, const uint8_t *data, size_t length);
void hexdump_log_src(const char *src_filename, int src_line_number, int level,
                     const uint8_t *data, size_t length, HexdumpLineCallback cb);

void hexdump_using_serial(int level, const char *src_filename, int src_line_number,
                         const char *line_buffer);
void hexdump_using_prompt(int level, const char *src_filename, int src_line_number,
                         const char *line_buffer);
void hexdump_using_pbllog(int level, const char *src_filename, int src_line_number,
                         const char *line_buffer);

#ifdef CONFIG_LOG
#define PBL_HEXDUMP_D_SERIAL(level, data, length)        \
  hexdump_log_src(__FILE_NAME__, __LINE__, level, data, length, hexdump_using_serial)
#define PBL_HEXDUMP_D_PROMPT(level, data, length)        \
  hexdump_log_src(__FILE_NAME__, __LINE__, level, data, length, hexdump_using_prompt)
#define PBL_HEXDUMP_D(domain, level, data, length)                      \
  if (domain) hexdump_log_src(__FILE_NAME__, __LINE__, level, data, length, hexdump_using_pbllog)
#define PBL_HEXDUMP(level, data, length) \
  hexdump_log_src(__FILE_NAME__, __LINE__, level, data, length, hexdump_using_pbllog);
#else
#define PBL_HEXDUMP_D_SERIAL(level, data, length)
#define PBL_HEXDUMP_D(domain, level, data, length)
#define PBL_HEXDUMP(level, data, length)
#define PBL_HEXDUMP_D_PROMPT(level, data, length)
#endif
