/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

//! @internal
//! data type that's used to store row data infos in a space-efficient manner
typedef struct {
  uint32_t offset;  // uint32_t needed for rectangular framebuffers > 65535 bytes
  uint16_t min_x;
  uint16_t max_x;
} GBitmapDataRowInfoInternal;

// FIXME: PBL-21055 Fix SDK exporter failing to crawl framebuffer headers
#if !defined(SDK)

// FIXME: PBL-21049 Fix platform abstraction and board definition scheme
#ifdef UNITTEST
// Do nothing, a unit-test's wscript specifies platforms=[]
// used by tools/waf/pebble_test.py to define these includes per test
#else

#ifdef CONFIG_BOARD_ASTERIX
#include "displays/display_asterix.h"
#elif defined(CONFIG_BOARD_OBELIX_DVT) || defined(CONFIG_BOARD_OBELIX_PVT) || defined(CONFIG_BOARD_OBELIX_BB2)
#include "displays/display_obelix.h"
#elif defined(CONFIG_BOARD_GETAFIX_DVT) || defined(CONFIG_BOARD_GETAFIX_DVT2)
#include "displays/display_getafix.h"
#elif defined(CONFIG_BOARD_QEMU_EMERY)
#include "displays/display_qemu_emery.h"
#elif defined(CONFIG_BOARD_QEMU_FLINT)
#include "displays/display_qemu_flint.h"
#elif defined(CONFIG_BOARD_QEMU_GABBRO)
#include "displays/display_qemu_gabbro.h"
#else
#error "Unknown display definition for board"
#endif // BOARD_*

#endif // UNITTEST

// For backwards compatibility, new code should use PBL_DISPLAY_WIDTH and PBL_DISPLAY_HEIGHT
#if !defined(DISP_COLS) || !defined(DISP_ROWS)
#define DISP_COLS PBL_DISPLAY_WIDTH
#define DISP_ROWS PBL_DISPLAY_HEIGHT
#endif // DISP_COLS || DISP_ROWS

#endif // !SDK
