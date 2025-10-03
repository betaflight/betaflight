/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * WARNING: This is an auto-generated file, please do not edit directly!
 *
 * Generator    : `src/utils/make-build-info.py`
 * Source       : https://build.betaflight.com/api/options/2025.12
 * Input hash   : 6a57c50d7938349a1e8363e85d6741dc
 */

#pragma once

#include "common/streambuf.h"

// Radio Protocols
#define BUILD_OPTION_SERIALRX_CRSF              4097
#define BUILD_OPTION_SERIALRX_FPORT             4098
#define BUILD_OPTION_SERIALRX_GHST              4099
#define BUILD_OPTION_SERIALRX_IBUS              4100
#define BUILD_OPTION_SERIALRX_JETIEXBUS         4101
#define BUILD_OPTION_SERIALRX_MAVLINK           4109
#define BUILD_OPTION_RX_PPM                     4102
#define BUILD_OPTION_SERIALRX_SBUS              4103
#define BUILD_OPTION_SERIALRX_SPEKTRUM          4104
#define BUILD_OPTION_SERIALRX_SRXL2             4105
#define BUILD_OPTION_SERIALRX_SUMD              4106
#define BUILD_OPTION_SERIALRX_SUMH              4107
#define BUILD_OPTION_SERIALRX_XBUS              4108
// Telemetry Protocols
#define BUILD_OPTION_TELEMETRY_FRSKY_HUB        12301
#define BUILD_OPTION_TELEMETRY_HOTT             12302
#define BUILD_OPTION_TELEMETRY_IBUS_EXTENDED    12303
#define BUILD_OPTION_TELEMETRY_LTM              12304
#define BUILD_OPTION_TELEMETRY_MAVLINK          12305
#define BUILD_OPTION_TELEMETRY_SMARTPORT        12306
#define BUILD_OPTION_TELEMETRY_SRXL             12307
// General Options
#define BUILD_OPTION_ACRO_TRAINER               16404
#define BUILD_OPTION_AKK_SMARTAUDIO             16405
#define BUILD_OPTION_ALTITUDE_HOLD              16422
#define BUILD_OPTION_BATTERY_CONTINUE           16406
#define BUILD_OPTION_CAMERA_CONTROL             16407
#define BUILD_OPTION_CHIRP                      16426
#define BUILD_OPTION_DASHBOARD                  16408
#define BUILD_OPTION_EMFAT_TOOLS                16409
#define BUILD_OPTION_ESCSERIAL_SIMONK           16410
#define BUILD_OPTION_GPS                        16412
#define BUILD_OPTION_LED_STRIP                  16413
#define BUILD_OPTION_LED_STRIP_64               16414
#define BUILD_OPTION_MAG                        16415
#define BUILD_OPTION_OSD_SD                     16416
#define BUILD_OPTION_OSD_HD                     16417
#define BUILD_OPTION_FRSKYOSD                   16411
#define BUILD_OPTION_PINIO                      16418
#define BUILD_OPTION_POSITION_HOLD              16425
#define BUILD_OPTION_RACE_PRO                   16419
#define BUILD_OPTION_SOFTSERIAL                 16423
#define BUILD_OPTION_SERVOS                     16420
#define BUILD_OPTION_VTX                        16421
#define BUILD_OPTION_WING                       16424
// Motor Protocols
#define BUILD_OPTION_BRUSHED                    8230
#define BUILD_OPTION_DSHOT                      8231
#define BUILD_OPTION_MULTISHOT                  8232
#define BUILD_OPTION_ONESHOT                    8233
#define BUILD_OPTION_PROSHOT                    8234
#define BUILD_OPTION_PWM_OUTPUT                 8235

void sbufWriteBuildInfoFlags(sbuf_t *dst);
