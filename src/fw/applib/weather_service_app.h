/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/weather/weather_snapshot_types.h"

//! @addtogroup Foundation
//! @{
//!   @addtogroup EventService
//!   @{
//!     @addtogroup WeatherService
//!
//! \brief Reads the phone-synced weather forecast for the default location
//!
//! Exposes the same forecast data the system Weather app displays - synced
//! from the phone in the background - directly to third-party apps, with no
//! AppMessage/PebbleKit JS round-trip required.
//! @{

//! Reads a snapshot of the phone-synced weather forecast for the default
//! (usually current) location.
//! @param snapshot_out Pointer to a \ref WeatherServiceSnapshot to fill in
//! @return true if weather data was available and snapshot_out was filled
//! in, false if no forecast is available yet (e.g. the phone hasn't synced
//! any weather data)
bool weather_service_peek(WeatherServiceSnapshot *snapshot_out);

//!     @} // end addtogroup WeatherService
//!   @} // end addtogroup EventService
//! @} // end addtogroup Foundation
