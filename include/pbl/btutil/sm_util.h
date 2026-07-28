/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdbool.h>

typedef struct SMPairingInfo SMPairingInfo;
typedef struct SM128BitKey SM128BitKey;

bool sm_is_pairing_info_equal_identity(const SMPairingInfo *a, const SMPairingInfo *b);

//! @return True if `stored` still holds `expected`'s encryption key material.
//! @note The LTK is the only field that tells one pairing from another. The identity address and
//! the IRK both survive a re-pair of the same phone, which is exactly the case a deferred delete
//! has to notice.
//! @note Only the halves `expected` marks valid are compared: a deferred delete carries the one
//! half of the bonding the BT driver handed it, while what is stored holds both. No valid half
//! means nothing was compared, and matching on the address alone is what this exists to avoid, so
//! that never matches.
bool sm_pairing_info_encryption_keys_match(const SMPairingInfo *stored,
                                           const SMPairingInfo *expected);

bool sm_is_pairing_info_empty(const SMPairingInfo *p);

bool sm_is_pairing_info_irk_not_used(const SM128BitKey *irk_key);
