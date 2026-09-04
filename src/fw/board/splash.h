/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#if defined(CONFIG_BOARD_OBELIX_DVT) || defined(CONFIG_BOARD_OBELIX_PVT) || defined(CONFIG_BOARD_OBELIX_BB2)
#include "splash/splash_obelix.xbm"
#elif defined(CONFIG_BOARD_GETAFIX_DVT) || defined(CONFIG_BOARD_GETAFIX_DVT2)
#include "splash/splash_getafix.xbm"
#elif defined(CONFIG_BOARD_QEMU_EMERY)
#include "splash/splash_obelix.xbm"
#elif defined(CONFIG_BOARD_QEMU_FLINT)
#include "splash/splash_obelix.xbm"
#elif defined(CONFIG_BOARD_QEMU_GABBRO)
#include "splash/splash_obelix.xbm"
#else
#error "Unknown splash definition for board"
#endif // BOARD_*
