/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/btutil/sm_util.h"
#include "pbl/btutil/bt_device.h"

#include <bluetooth/sm_types.h>

#include <stdbool.h>
#include <string.h>

// -------------------------------------------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
bool sm_is_pairing_info_equal_identity(const SMPairingInfo *a, const SMPairingInfo *b) {
  return (a->is_remote_identity_info_valid &&
          b->is_remote_identity_info_valid &&
          bt_device_equal(&a->identity.opaque, &b->identity.opaque) &&
          memcmp(&a->irk, &b->irk, sizeof(SMIdentityResolvingKey)) == 0);
}
#pragma GCC diagnostic pop

// -------------------------------------------------------------------------------------------------
static bool prv_encryption_info_matches(const SMLongTermKey *stored_ltk, uint16_t stored_ediv,
                                        uint64_t stored_rand, const SMLongTermKey *ltk,
                                        uint16_t ediv, uint64_t rand) {
  return (memcmp(stored_ltk, ltk, sizeof(*ltk)) == 0) && (stored_ediv == ediv) &&
         (stored_rand == rand);
}

bool sm_pairing_info_encryption_keys_match(const SMPairingInfo *stored,
                                           const SMPairingInfo *expected) {
  bool compared = false;

  if (expected->is_remote_encryption_info_valid) {
    if (!stored->is_remote_encryption_info_valid ||
        !prv_encryption_info_matches(&stored->remote_encryption_info.ltk,
                                     stored->remote_encryption_info.ediv,
                                     stored->remote_encryption_info.rand,
                                     &expected->remote_encryption_info.ltk,
                                     expected->remote_encryption_info.ediv,
                                     expected->remote_encryption_info.rand)) {
      return false;
    }
    compared = true;
  }

  if (expected->is_local_encryption_info_valid) {
    if (!stored->is_local_encryption_info_valid ||
        !prv_encryption_info_matches(&stored->local_encryption_info.ltk,
                                     stored->local_encryption_info.ediv,
                                     stored->local_encryption_info.rand,
                                     &expected->local_encryption_info.ltk,
                                     expected->local_encryption_info.ediv,
                                     expected->local_encryption_info.rand)) {
      return false;
    }
    compared = true;
  }

  return compared;
}

// -------------------------------------------------------------------------------------------------
bool sm_is_pairing_info_empty(const SMPairingInfo *p) {
  return (!p->is_local_encryption_info_valid &&
          !p->is_remote_encryption_info_valid &&
          !p->is_remote_identity_info_valid &&
          !p->is_remote_signing_info_valid);
}

bool sm_is_pairing_info_irk_not_used(const SMIdentityResolvingKey *irk_key) {
  // Per BLE spec v4.2 section 10.7 "Privacy Feature":
  //
  // "The local or peer’s IRK shall be an all-zero key, if not applicable for the particular
  //  device identity."
  const SMIdentityResolvingKey empty_key = { };
  return (memcmp(irk_key, &empty_key, sizeof(empty_key)) == 0);
}
