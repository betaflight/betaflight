/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/streambuf.h"

typedef enum
{
    // byte 1
    MSP_BUILD_OPTION_FLAG_SERIALRX_CRSF           = 1 << 0,
    MSP_BUILD_OPTION_FLAG_SERIALRX_FPORT          = 1 << 1,
    MSP_BUILD_OPTION_FLAG_SERIALRX_GHST           = 1 << 2,
    MSP_BUILD_OPTION_FLAG_SERIALRX_IBUS           = 1 << 3,
    MSP_BUILD_OPTION_FLAG_SERIALRX_JETIEXBUS      = 1 << 4,
    MSP_BUILD_OPTION_FLAG_RX_PPM                  = 1 << 5,
    MSP_BUILD_OPTION_FLAG_SERIALRX_SBUS           = 1 << 6,
    MSP_BUILD_OPTION_FLAG_SERIALRX_SPEKTRUM       = 1 << 7,
    // byte 2
    MSP_BUILD_OPTION_FLAG_SERIALRX_SRXL2          = 1 << 0,
    MSP_BUILD_OPTION_FLAG_SERIALRX_SUMD           = 1 << 1,
    MSP_BUILD_OPTION_FLAG_SERIALRX_SUMH           = 1 << 2,
    MSP_BUILD_OPTION_FLAG_SERIALRX_XBUS           = 1 << 3,
    MSP_BUILD_OPTION_FLAG_TELEMETRY_FRSKY_HUB     = 1 << 4,
    MSP_BUILD_OPTION_FLAG_TELEMETRY_HOTT          = 1 << 5,
    MSP_BUILD_OPTION_FLAG_TELEMETRY_IBUS_EXTENDED = 1 << 6,
    MSP_BUILD_OPTION_FLAG_TELEMETRY_LTM           = 1 << 7,
    // byte 3
    MSP_BUILD_OPTION_FLAG_TELEMETRY_MAVLINK       = 1 << 0,
    MSP_BUILD_OPTION_FLAG_TELEMETRY_SMARTPORT     = 1 << 1,
    MSP_BUILD_OPTION_FLAG_TELEMETRY_SRXL          = 1 << 2,
    MSP_BUILD_OPTION_FLAG_ACRO_TRAINER            = 1 << 3,
    MSP_BUILD_OPTION_FLAG_AKK_SMARTAUDIO          = 1 << 4,
    MSP_BUILD_OPTION_FLAG_BATTERY_CONTINUE        = 1 << 5,
    MSP_BUILD_OPTION_FLAG_CAMERA_CONTROL          = 1 << 6,
    MSP_BUILD_OPTION_FLAG_DASHBOARD               = 1 << 7,
    // byte 4
    MSP_BUILD_OPTION_FLAG_EMFAT_TOOLS             = 1 << 0,
    MSP_BUILD_OPTION_FLAG_ESCSERIAL_SIMONK        = 1 << 1,
    MSP_BUILD_OPTION_FLAG_FRSKYOSD                = 1 << 2,
    MSP_BUILD_OPTION_FLAG_GPS                     = 1 << 3,
    MSP_BUILD_OPTION_FLAG_LED_STRIP               = 1 << 4,
    MSP_BUILD_OPTION_FLAG_LED_STRIP_64            = 1 << 5,
    MSP_BUILD_OPTION_FLAG_MAG                     = 1 << 6,
    MSP_BUILD_OPTION_FLAG_OSD_SD                  = 1 << 7,
    // byte 5
    MSP_BUILD_OPTION_FLAG_OSD_HD                  = 1 << 0,
    MSP_BUILD_OPTION_FLAG_PINIO                   = 1 << 1,
    MSP_BUILD_OPTION_FLAG_RACE_PRO                = 1 << 2,
    MSP_BUILD_OPTION_FLAG_SERVOS                  = 1 << 3,
    MSP_BUILD_OPTION_FLAG_VTX                     = 1 << 4,
    MSP_BUILD_OPTION_FLAG_BRUSHED                 = 1 << 5,
    MSP_BUILD_OPTION_FLAG_DSHOT                   = 1 << 6,
    MSP_BUILD_OPTION_FLAG_MULTISHOT               = 1 << 7,
    // byte 6
    MSP_BUILD_OPTION_FLAG_ONESHOT                 = 1 << 0,
    MSP_BUILD_OPTION_FLAG_PROSHOT                 = 1 << 1,
    MSP_BUILD_OPTION_FLAG_PWM_OUTPUT              = 1 << 2,
} mspBuildOptionFlags_e;

void sbufWriteBuildInfoFlags(sbuf_t *dst);
