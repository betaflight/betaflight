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
 * Source       : https://build.betaflight.com/api/options/4.6.0
 * Input hash   : 5da6ff60951a04dc9a29ea0758aae1df
 */

#include <stdint.h>

#include "platform.h"

#include "common/streambuf.h"

#include "msp/msp_build_info.h"

void sbufWriteBuildInfoFlags(sbuf_t *dst)
{
    static const uint16_t options[] = {
#ifdef USE_SERIALRX_CRSF
        BUILD_OPTION_SERIALRX_CRSF,
#endif
#ifdef USE_SERIALRX_FPORT
        BUILD_OPTION_SERIALRX_FPORT,
#endif
#ifdef USE_SERIALRX_GHST
        BUILD_OPTION_SERIALRX_GHST,
#endif
#ifdef USE_SERIALRX_IBUS
        BUILD_OPTION_SERIALRX_IBUS,
#endif
#ifdef USE_SERIALRX_JETIEXBUS
        BUILD_OPTION_SERIALRX_JETIEXBUS,
#endif
#ifdef USE_RX_PPM
        BUILD_OPTION_RX_PPM,
#endif
#ifdef USE_SERIALRX_SBUS
        BUILD_OPTION_SERIALRX_SBUS,
#endif
#ifdef USE_SERIALRX_SPEKTRUM
        BUILD_OPTION_SERIALRX_SPEKTRUM,
#endif
#ifdef USE_SERIALRX_SRXL2
        BUILD_OPTION_SERIALRX_SRXL2,
#endif
#ifdef USE_SERIALRX_SUMD
        BUILD_OPTION_SERIALRX_SUMD,
#endif
#ifdef USE_SERIALRX_SUMH
        BUILD_OPTION_SERIALRX_SUMH,
#endif
#ifdef USE_SERIALRX_XBUS
        BUILD_OPTION_SERIALRX_XBUS,
#endif
#ifdef USE_TELEMETRY_FRSKY_HUB
        BUILD_OPTION_TELEMETRY_FRSKY_HUB,
#endif
#ifdef USE_TELEMETRY_HOTT
        BUILD_OPTION_TELEMETRY_HOTT,
#endif
#ifdef USE_TELEMETRY_IBUS_EXTENDED
        BUILD_OPTION_TELEMETRY_IBUS_EXTENDED,
#endif
#ifdef USE_TELEMETRY_LTM
        BUILD_OPTION_TELEMETRY_LTM,
#endif
#ifdef USE_TELEMETRY_MAVLINK
        BUILD_OPTION_TELEMETRY_MAVLINK,
#endif
#ifdef USE_TELEMETRY_SMARTPORT
        BUILD_OPTION_TELEMETRY_SMARTPORT,
#endif
#ifdef USE_TELEMETRY_SRXL
        BUILD_OPTION_TELEMETRY_SRXL,
#endif
#ifdef USE_ACRO_TRAINER
        BUILD_OPTION_ACRO_TRAINER,
#endif
#ifdef USE_AKK_SMARTAUDIO
        BUILD_OPTION_AKK_SMARTAUDIO,
#endif
#ifdef USE_ALTITUDE_HOLD
        BUILD_OPTION_ALTITUDE_HOLD,
#endif
#ifdef USE_BATTERY_CONTINUE
        BUILD_OPTION_BATTERY_CONTINUE,
#endif
#ifdef USE_CAMERA_CONTROL
        BUILD_OPTION_CAMERA_CONTROL,
#endif
#ifdef USE_DASHBOARD
        BUILD_OPTION_DASHBOARD,
#endif
#ifdef USE_EMFAT_TOOLS
        BUILD_OPTION_EMFAT_TOOLS,
#endif
#ifdef USE_ESCSERIAL_SIMONK
        BUILD_OPTION_ESCSERIAL_SIMONK,
#endif
#ifdef USE_GPS
        BUILD_OPTION_GPS,
#endif
#ifdef USE_LED_STRIP
        BUILD_OPTION_LED_STRIP,
#endif
#ifdef USE_LED_STRIP_64
        BUILD_OPTION_LED_STRIP_64,
#endif
#ifdef USE_MAG
        BUILD_OPTION_MAG,
#endif
#ifdef USE_OSD_SD
        BUILD_OPTION_OSD_SD,
#endif
#ifdef USE_OSD_HD
        BUILD_OPTION_OSD_HD,
#endif
#ifdef USE_FRSKYOSD
        BUILD_OPTION_FRSKYOSD,
#endif
#ifdef USE_PINIO
        BUILD_OPTION_PINIO,
#endif
#ifdef USE_POSITION_HOLD
        BUILD_OPTION_POSITION_HOLD,
#endif
#ifdef USE_RACE_PRO
        BUILD_OPTION_RACE_PRO,
#endif
#ifdef USE_SOFTSERIAL
        BUILD_OPTION_SOFTSERIAL,
#endif
#ifdef USE_SERVOS
        BUILD_OPTION_SERVOS,
#endif
#ifdef USE_VTX
        BUILD_OPTION_VTX,
#endif
#ifdef USE_WING
        BUILD_OPTION_WING,
#endif
#ifdef USE_BRUSHED
        BUILD_OPTION_BRUSHED,
#endif
#ifdef USE_DSHOT
        BUILD_OPTION_DSHOT,
#endif
#ifdef USE_MULTISHOT
        BUILD_OPTION_MULTISHOT,
#endif
#ifdef USE_ONESHOT
        BUILD_OPTION_ONESHOT,
#endif
#ifdef USE_PROSHOT
        BUILD_OPTION_PROSHOT,
#endif
#ifdef USE_PWM_OUTPUT
        BUILD_OPTION_PWM_OUTPUT,
#endif
    };

    for (unsigned i = 0; i < ARRAYLEN(options); i++)
    {
        sbufWriteU16(dst, options[i]);
    }
}
