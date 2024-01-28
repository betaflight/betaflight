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

#include <stdint.h>

#include "platform.h"

#include "common/streambuf.h"

#include "msp/msp_build_info.h"

void sbufWriteBuildInfoFlags(sbuf_t *dst) 
{
    uint8_t options = 0;
#ifdef USE_SERIALRX_CRSF
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_CRSF;
#endif
#ifdef USE_SERIALRX_FPORT
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_FPORT;
#endif
#ifdef USE_SERIALRX_GHST
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_GHST;
#endif
#ifdef USE_SERIALRX_IBUS
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_IBUS;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_JETIEXBUS;
#endif
#ifdef USE_RX_PPM
    options |= MSP_BUILD_OPTION_FLAG_RX_PPM;
#endif
#ifdef USE_SERIALRX_SBUS
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_SBUS;
#endif
#ifdef USE_SERIALRX_SPEKTRUM
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_SPEKTRUM;
#endif
    sbufWriteU8(dst, options); // byte 1
    options = 0;
#ifdef USE_SERIALRX_SRXL2
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_SRXL2;
#endif
#ifdef USE_SERIALRX_SUMD
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_SUMD;
#endif
#ifdef USE_SERIALRX_SUMH
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_SUMH;
#endif
#ifdef USE_SERIALRX_XBUS
    options |= MSP_BUILD_OPTION_FLAG_SERIALRX_XBUS;
#endif
#ifdef USE_TELEMETRY_FRSKY_HUB
    options |= MSP_BUILD_OPTION_FLAG_TELEMETRY_FRSKY_HUB;
#endif
#ifdef USE_TELEMETRY_HOTT
    options |= MSP_BUILD_OPTION_FLAG_TELEMETRY_HOTT;
#endif
#ifdef USE_TELEMETRY_IBUS_EXTENDED
    options |= MSP_BUILD_OPTION_FLAG_TELEMETRY_IBUS_EXTENDED;
#endif
#ifdef USE_TELEMETRY_LTM
    options |= MSP_BUILD_OPTION_FLAG_TELEMETRY_LTM;
#endif
    sbufWriteU8(dst, options); // byte 2
    options = 0;
#ifdef USE_TELEMETRY_MAVLINK
    options |= MSP_BUILD_OPTION_FLAG_TELEMETRY_MAVLINK;
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    options |= MSP_BUILD_OPTION_FLAG_TELEMETRY_SMARTPORT;
#endif
#ifdef USE_TELEMETRY_SRXL
    options |= MSP_BUILD_OPTION_FLAG_TELEMETRY_SRXL;
#endif
#ifdef USE_ACRO_TRAINER
    options |= MSP_BUILD_OPTION_FLAG_ACRO_TRAINER;
#endif
#ifdef USE_AKK_SMARTAUDIO
    options |= MSP_BUILD_OPTION_FLAG_AKK_SMARTAUDIO;
#endif
#ifdef USE_BATTERY_CONTINUE
    options |= MSP_BUILD_OPTION_FLAG_BATTERY_CONTINUE;
#endif
#ifdef USE_CAMERA_CONTROL
    options |= MSP_BUILD_OPTION_FLAG_CAMERA_CONTROL;
#endif
#ifdef USE_DASHBOARD
    options |= MSP_BUILD_OPTION_FLAG_DASHBOARD;
#endif
    sbufWriteU8(dst, options); // byte 3
    options = 0;
#ifdef USE_EMFAT_TOOLS
    options |= MSP_BUILD_OPTION_FLAG_EMFAT_TOOLS;
#endif
#ifdef USE_ESCSERIAL_SIMONK
    options |= MSP_BUILD_OPTION_FLAG_ESCSERIAL_SIMONK;
#endif
#ifdef USE_FRSKYOSD
    options |= MSP_BUILD_OPTION_FLAG_FRSKYOSD;
#endif
#ifdef USE_GPS
    options |= MSP_BUILD_OPTION_FLAG_GPS;
#endif
#ifdef USE_LED_STRIP
    options |= MSP_BUILD_OPTION_FLAG_LED_STRIP;
#endif
#ifdef USE_LED_STRIP_64
    options |= MSP_BUILD_OPTION_FLAG_LED_STRIP_64;
#endif
#ifdef USE_MAG
    options |= MSP_BUILD_OPTION_FLAG_MAG;
#endif
#ifdef USE_OSD_SD
    options |= MSP_BUILD_OPTION_FLAG_OSD_SD;
#endif
    sbufWriteU8(dst, options); // byte 4
    options = 0;
#ifdef USE_OSD_HD
    options |= MSP_BUILD_OPTION_FLAG_OSD_HD;
#endif
#ifdef USE_PINIO
    options |= MSP_BUILD_OPTION_FLAG_PINIO;
#endif
#ifdef USE_RACE_PRO
    options |= MSP_BUILD_OPTION_FLAG_RACE_PRO;
#endif
#ifdef USE_SERVOS
    options |= MSP_BUILD_OPTION_FLAG_SERVOS;
#endif
#ifdef USE_VTX
    options |= MSP_BUILD_OPTION_FLAG_VTX;
#endif
#ifdef USE_BRUSHED
    options |= MSP_BUILD_OPTION_FLAG_BRUSHED;
#endif
#ifdef USE_DSHOT
    options |= MSP_BUILD_OPTION_FLAG_DSHOT;
#endif
#ifdef USE_MULTISHOT
    options |= MSP_BUILD_OPTION_FLAG_MULTISHOT;
#endif
    sbufWriteU8(dst, options); // byte 5
    options = 0;
#ifdef USE_ONESHOT
    options |= MSP_BUILD_OPTION_FLAG_ONESHOT;
#endif
#ifdef USE_PROSHOT
    options |= MSP_BUILD_OPTION_FLAG_PROSHOT;
#endif
#ifdef USE_PWM_OUTPUT
    options |= MSP_BUILD_OPTION_FLAG_PWM_OUTPUT;
#endif
    sbufWriteU8(dst, options); // byte 6
}
