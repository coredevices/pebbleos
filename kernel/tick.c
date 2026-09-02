/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/drivers/rtc.h"
#include "pbl/kernel/types.h"

_Static_assert(PBL_TICK_HZ == RTC_TICKS_HZ, "CONFIG_KERNEL_TICK_HZ must match the RTC tick rate");

pbl_tick_t pbl_ms_to_ticks(uint32_t ms) { return ((uint64_t)ms * PBL_TICK_HZ) / 1000; }

uint32_t pbl_ticks_to_ms(pbl_tick_t ticks) { return ((uint64_t)ticks * 1000) / PBL_TICK_HZ; }
