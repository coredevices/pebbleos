/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

void fake_shared_prf_storage_reset_counts(void);
int fake_shared_prf_storage_get_ble_store_count(void);
int fake_shared_prf_storage_get_ble_delete_count(void);

//! How many times the guarded erase was asked to drop the stored pairing. Counted apart from the
//! unconditional erase: which of the two a delete path uses is the thing under test.
int fake_shared_prf_storage_get_ble_delete_if_matches_count(void);
