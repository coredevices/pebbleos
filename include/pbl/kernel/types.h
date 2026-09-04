/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pbl/kernel/backend.h"

typedef uint32_t pbl_tick_t;

#define PBL_TICK_HZ CONFIG_KERNEL_TICK_HZ
#define PBL_TICK_FOREVER UINT32_MAX

//! Timeouts are a distinct type so ticks and milliseconds cannot be mixed.
typedef struct {
  pbl_tick_t ticks;
} pbl_timeout_t;

#define PBL_NO_WAIT ((pbl_timeout_t){ .ticks = 0 })
#define PBL_FOREVER ((pbl_timeout_t){ .ticks = PBL_TICK_FOREVER })
#define PBL_TICKS(t) ((pbl_timeout_t){ .ticks = (t) })
#define PBL_MSEC(ms) ((pbl_timeout_t){ .ticks = pbl_ms_to_ticks(ms) })
#define PBL_SEC(s) PBL_MSEC((s) * 1000U)

pbl_tick_t pbl_ms_to_ticks(uint32_t ms);
uint32_t pbl_ticks_to_ms(pbl_tick_t ticks);

static inline bool pbl_timeout_is_forever(pbl_timeout_t t) { return t.ticks == PBL_TICK_FOREVER; }
static inline bool pbl_timeout_is_no_wait(pbl_timeout_t t) { return t.ticks == 0; }

//! Higher value = more urgent.
typedef uint8_t pbl_prio_t;
#define PBL_PRIO_IDLE ((pbl_prio_t)0)
#define PBL_PRIO_MAX ((pbl_prio_t)(CONFIG_KERNEL_NUM_PRIORITIES - 1))

struct pbl_thread;

