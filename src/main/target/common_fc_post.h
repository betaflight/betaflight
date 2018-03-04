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

#include "build/version.h"

// Targets with built-in vtx do not need external vtx
#if defined(USE_VTX_RTC6705) && !defined(VTX_RTC6705_OPTIONAL)
#undef USE_VTX_SMARTAUDIO
#undef USE_VTX_TRAMP
#endif

#if defined(USE_QUAD_MIXER_ONLY) && defined(USE_SERVOS)
#undef USE_SERVOS
#endif

#ifndef USE_DSHOT
#undef USE_ESC_SENSOR
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
#if !defined(USE_TELEMETRY_SMARTPORT) && !defined(USE_TELEMETRY_CRSF)
#undef USE_MSP_OVER_TELEMETRY
#endif
#endif

// If USE_SERIALRX_SPEKTRUM was dropped by a target, drop all related options
#ifndef USE_SERIALRX_SPEKTRUM
#undef USE_SPEKTRUM_BIND
#undef USE_SPEKTRUM_BIND_PLUG
#undef USE_SPEKTRUM_REAL_RSSI
#undef USE_SPEKTRUM_FAKE_RSSI
#undef USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#undef USE_SPEKTRUM_VTX_CONTROL
#undef USE_SPEKTRUM_VTX_TELEMETRY
#undef USE_SPEKTRUM_CMS_TELEMETRY
#endif

// undefine USE_ALT_HOLD if there is no baro or rangefinder to support it
#if defined(USE_ALT_HOLD) && !defined(USE_BARO) && !defined(USE_RANGEFINDER)
#undef USE_ALT_HOLD
#endif

/* If either VTX_CONTROL or VTX_COMMON is undefined then remove common code and device drivers */
#if !defined(USE_VTX_COMMON) || !defined(USE_VTX_CONTROL)
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#endif

#if defined(USE_RX_FRSKY_SPI_D) || defined(USE_RX_FRSKY_SPI_X)
#define USE_RX_CC2500
#define USE_RX_FRSKY_SPI
#endif

// Burst dshot to default off if not configured explicitly by target
#ifndef ENABLE_DSHOT_DMAR
#define ENABLE_DSHOT_DMAR false
#endif

// Disable filters for IMUF
#ifdef USE_GYRO_IMUF9001
#undef USE_GYRO_FAST_KALMAN
#undef USE_GYRO_BIQUAD_RC_FIR2
#endif
