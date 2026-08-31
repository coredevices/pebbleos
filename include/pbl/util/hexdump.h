/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

typedef void (*HexdumpLineCallback)(int level, const char *src_filename, int src_line_number,
                                    const char *line_buffer);

//! Hexdumps data in xxd-style formatting, by repeatedly calling write_line_cb for each line.
//! @note The line_buffer that is passed does not end with any newline characters.
void hexdump(const char *src_filename, int src_line_number, int level,
             const uint8_t *data, size_t length, HexdumpLineCallback write_line_cb);
