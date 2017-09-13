/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Touch up configuration

#pragma once

// Targets with built-in vtx do not need external vtx
#if defined(VTX_RTC6705) && !defined(VTX_RTC6705_OPTIONAL)
#undef VTX_SMARTAUDIO
#undef VTX_TRAMP
#endif

#if defined(USE_QUAD_MIXER_ONLY) && defined(USE_SERVOS)
#undef USE_SERVOS
#endif

// XXX Followup implicit dependencies among DASHBOARD, display_xxx and USE_I2C.
// XXX This should eventually be cleaned up.
#ifndef USE_I2C
#undef USE_I2C_OLED_DISPLAY
#undef USE_DASHBOARD
#else
#ifdef USE_DASHBOARD
#define USE_I2C_OLED_DISPLAY
#endif
#endif

// XXX Remove USE_BARO_BMP280 and USE_BARO_MS5611 if USE_I2C is not defined.
// XXX This should go away buy editing relevant target.h files
#if !defined(USE_I2C)
#if defined(USE_BARO_BMP280)
#undef USE_BARO_BMP280
#endif
#if defined(USE_BARO_MS5611)
#undef USE_BARO_MS5611
#endif
#endif

#if defined(USE_MSP_OVER_TELEMETRY)
#if !defined(TELEMETRY_SMARTPORT) && !defined(TELEMETRY_CRSF)
#undef USE_MSP_OVER_TELEMETRY
#endif
#endif
