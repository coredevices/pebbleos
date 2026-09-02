/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/backend.h"
#include "pbl/kernel/sched.h"

extern const struct pbl_kobj_init __pbl_kobj_init_start__[];
extern const struct pbl_kobj_init __pbl_kobj_init_end__[];

void pbl_kernel_init(void) {
#if !PBL_KERNEL_KOBJ_RUNTIME_INIT
  return;
#endif
  for (const struct pbl_kobj_init *k = __pbl_kobj_init_start__; k < __pbl_kobj_init_end__; k++) {
    k->init(k->obj);
  }
}
