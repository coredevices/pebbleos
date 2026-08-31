/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! Auto-shutdown when idle in PRF to increase the changes of getting Pebbles shipped
//! that have some level of battery charge in them.

//! Initialize event subscriptions and start the watchdog. Called once at boot from KernelMain.
void prf_idle_watchdog_init(void);

//! Start (or restart) the watchdog timer. Can be called from any task.
void prf_idle_watchdog_start(void);

//! Stop the watchdog timer. Can be called from any task.
void prf_idle_watchdog_stop(void);
