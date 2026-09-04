/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

void fw_prepare_for_reset(void);

void fw_reset_into_prf(void);

typedef enum RemoteResetType {
  RemoteResetRegular = 0x00,
  RemoteResetPrf = 0xff,
} RemoteResetType;
