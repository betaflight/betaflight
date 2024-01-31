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

#pragma once

#include "common/streambuf.h"

// radioProtocols
#define BUILD_OPTION_SERIALRX_CRSF              1000
#define BUILD_OPTION_SERIALRX_FPORT             1001
#define BUILD_OPTION_SERIALRX_GHST              1002
#define BUILD_OPTION_SERIALRX_IBUS              1003
#define BUILD_OPTION_SERIALRX_JETIEXBUS         1004
#define BUILD_OPTION_RX_PPM                     1005
#define BUILD_OPTION_SERIALRX_SBUS              1006
#define BUILD_OPTION_SERIALRX_SPEKTRUM          1007
#define BUILD_OPTION_SERIALRX_SRXL2             1008
#define BUILD_OPTION_SERIALRX_SUMD              1009
#define BUILD_OPTION_SERIALRX_SUMH              1010
#define BUILD_OPTION_SERIALRX_XBUS              1011
// telemetryProtocols
#define BUILD_OPTION_TELEMETRY_FRSKY_HUB        2001
#define BUILD_OPTION_TELEMETRY_HOTT             2002
#define BUILD_OPTION_TELEMETRY_IBUS_EXTENDED    2003
#define BUILD_OPTION_TELEMETRY_LTM              2004
#define BUILD_OPTION_TELEMETRY_MAVLINK          2005
#define BUILD_OPTION_TELEMETRY_SMARTPORT        2006
#define BUILD_OPTION_TELEMETRY_SRXL             2007
// generalOptions
#define BUILD_OPTION_ACRO_TRAINER               3000
#define BUILD_OPTION_AKK_SMARTAUDIO             3001
#define BUILD_OPTION_BATTERY_CONTINUE           3002
#define BUILD_OPTION_CAMERA_CONTROL             3003
#define BUILD_OPTION_DASHBOARD                  3004
#define BUILD_OPTION_EMFAT_TOOLS                3005
#define BUILD_OPTION_ESCSERIAL_SIMONK           3006
#define BUILD_OPTION_FRSKYOSD                   3007
#define BUILD_OPTION_GPS                        3008
#define BUILD_OPTION_LED_STRIP                  3009
#define BUILD_OPTION_LED_STRIP_64               3010
#define BUILD_OPTION_MAG                        3011
#define BUILD_OPTION_OSD_SD                     3012
#define BUILD_OPTION_OSD_HD                     3013
#define BUILD_OPTION_PINIO                      3014
#define BUILD_OPTION_RACE_PRO                   3015
#define BUILD_OPTION_SERVOS                     3016
#define BUILD_OPTION_VTX                        3017
// motorProtocols
#define BUILD_OPTION_BRUSHED                    4000
#define BUILD_OPTION_DSHOT                      4001
#define BUILD_OPTION_MULTISHOT                  4002
#define BUILD_OPTION_ONESHOT                    4003
#define BUILD_OPTION_PROSHOT                    4004
#define BUILD_OPTION_PWM_OUTPUT                 4005

void sbufWriteBuildInfoFlags(sbuf_t *dst);
