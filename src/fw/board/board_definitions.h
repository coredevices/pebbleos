/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

// FIXME: PBL-21049 Fix platform abstraction and board definition scheme
#ifdef CONFIG_BOARD_ASTERIX
#include "boards/board_asterix.h"
#elif defined(CONFIG_BOARD_OBELIX_DVT) || defined(CONFIG_BOARD_OBELIX_PVT) || defined(CONFIG_BOARD_OBELIX_BB2)
#include "boards/board_obelix.h"
#elif defined(CONFIG_BOARD_GETAFIX_DVT) || defined(CONFIG_BOARD_GETAFIX_DVT2)
#include "boards/board_getafix.h"
#elif defined(CONFIG_BOARD_QEMU_EMERY)
#include "boards/board_qemu_emery.h"
#elif defined(CONFIG_BOARD_QEMU_FLINT)
#include "boards/board_qemu_flint.h"
#elif defined(CONFIG_BOARD_QEMU_GABBRO)
#include "boards/board_qemu_gabbro.h"
#else
#error "Unknown board definition"
#endif
