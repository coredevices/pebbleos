/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

// The automated testing framework shouldn't start operating on the system
// after a reset until PebbleOS is ready to handle requests. This function
// handles that notification
void notify_system_ready_for_communication(void);
