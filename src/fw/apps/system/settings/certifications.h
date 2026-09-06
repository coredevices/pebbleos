/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "mfg/mfg_info.h"

#include <stdbool.h>


//! Which regulatory marks and/or IDs a given product should display.
typedef struct RegulatoryFlags {
//! Australia Regulatory Compliance Mark
  bool has_australia_rcm:1;
//! Canada IC ID
  bool has_canada_ic:1;
//! Canada ISED ID
  bool has_canada_ised:1;
//! China CMIIT ID
  bool has_china_cmiit:1;
//! EU CE Mark
  bool has_eu_ce:1;
//! EU WEEE Mark (wastebin with X)
  bool has_eu_weee:1;
//! UKCA Mark
  bool has_ukca:1;
//! Japan TELEC (Telecom Engineering Center) [R] mark and ID
//! (Radio equipment conformity)
  bool has_japan_telec_r:1;
//!  TELEC mark [T] mark and ID (Terminal equipment conformity)
  bool has_japan_telec_t:1;
//! Korea
//!  - KCC mark
//!  - Details window with KCC mark and KCC ID
  bool has_korea_kcc:1;
//! Mexico NOM NYCE mark
  bool has_mexico_nom_nyce:1;
//! USA FCC Mark and FCC ID
  bool has_usa_fcc:1;
} RegulatoryFlags;

typedef struct CertificationIds {
  const char *company_name;
  const char *product_type;
  const char *trademark;
  const char *place_of_origin;
  const char *dc_input;
  const char *rated_voltage;
  const char *milliampere_hour;
  const char *watt_hour;
  const char *canada_ic_id;
  const char *canada_ised_id;
  const char *china_cmiit_id;
  const char *japan_telec_r_id;
  const char *japan_telec_t_id;
  const char *korea_kcc_id;
  const char *mexico_ifetel_id;
  const char *usa_fcc_id;
} CertificationIds;


static const RegulatoryFlags s_regulatory_flags_fallback = {
};

// Certification ID strings used for bigboards and such.
static const CertificationIds s_certification_ids_fallback = {
  .company_name = "ACME Inc.",
  .product_type = "Product Type",
  .trademark = "Product Trademark",
  .place_of_origin = "Country of Origin",
  .dc_input = "XV/YYYmA",
  .rated_voltage = "X.XV",
  .milliampere_hour = "XXXmAh",
  .watt_hour = "X.XXWh",
  .canada_ic_id = "XXXXXX-YYY",
  .canada_ised_id = "XXXXXX-YYYYYYYYYYY",
  .china_cmiit_id = "ABCDEFGHIJ",
  .japan_telec_r_id = "XXX-YYYYYY",
  .japan_telec_t_id = "D XX YYYY ZZZ",
  .korea_kcc_id = "WWWW-XXX-YYY-ZZZ",
  .mexico_ifetel_id = "RCPPEXXXX-YYYY",
  .usa_fcc_id = "XXX-YYY",
};


static const RegulatoryFlags s_regulatory_flags_obelix = {
  .has_canada_ised = true,
  .has_eu_ce = true,
  .has_eu_weee = true,
  .has_ukca = true,
  .has_usa_fcc = true,
};

static const CertificationIds s_certification_ids_obelix = {
  .company_name = "Core Devices LLC",
  .product_type = "Smart watch",
  .trademark = "Pebble",
  .place_of_origin = "Made in China",
  .dc_input = "5 V / 300 mA",
  .rated_voltage = "3.8 V",
  .milliampere_hour = "185 mAh",
  .watt_hour = "0.71 Wh",
  .canada_ised_id = "34223-PEBBLETIME2",
  .usa_fcc_id = "2BQB2-PEBBLETIME2",
};

static const RegulatoryFlags * prv_get_regulatory_flags(void) {
#ifdef CONFIG_BOARD_ASTERIX
  // TODO: add applicable flags
  return &s_regulatory_flags_fallback;
#elif defined(CONFIG_BOARD_OBELIX)
  return &s_regulatory_flags_obelix;
#else
  return &s_regulatory_flags_fallback;
#endif
}

//! Don't call this function directly. Use the prv_get_*_id functions instead.
static const CertificationIds * prv_get_certification_ids(void) {
#ifdef CONFIG_BOARD_ASTERIX
  // TODO: add real certification ids
  return &s_certification_ids_fallback;
#elif defined(CONFIG_BOARD_OBELIX)
  return &s_certification_ids_obelix;
#else
  return &s_certification_ids_fallback;
#endif
}

#define ID_GETTER(ID_KIND) \
  static const char * prv_get_##ID_KIND(void) { \
    return prv_get_certification_ids()->ID_KIND ?: \
      s_certification_ids_fallback.ID_KIND; \
  }

ID_GETTER(company_name)
ID_GETTER(product_type)
ID_GETTER(trademark)
ID_GETTER(place_of_origin)
ID_GETTER(dc_input)
ID_GETTER(rated_voltage)
ID_GETTER(milliampere_hour)
ID_GETTER(watt_hour)
ID_GETTER(canada_ic_id)
ID_GETTER(china_cmiit_id)
ID_GETTER(japan_telec_r_id)
ID_GETTER(japan_telec_t_id)
ID_GETTER(korea_kcc_id)
ID_GETTER(mexico_ifetel_id)
ID_GETTER(usa_fcc_id)
ID_GETTER(canada_ised_id)

#undef ID_GETTER

//! Get the model string from MFG storage
//! @param buffer a character array that's at least MFG_INFO_MODEL_STRING_LENGTH in size
static void prv_get_model(char *buffer) {
  mfg_info_get_model(buffer);
}
