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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_TELEMETRY_MAVLINK)

#include "fc/runtime_config.h"

#include "io/gps.h"

#include "telemetry/mavlink_lib.h"
#include "telemetry/mavlink_frame.h"

#define MAVLINK_FRAME_ALT_LIMIT_M 1000000.0f

// The _INT frames are aliases of the plain global frames since MAVLink 2024-03
bool mavlinkFrameIsGlobal(uint8_t frame)
{
    switch (frame) {
    case MAV_FRAME_GLOBAL:
    case MAV_FRAME_GLOBAL_INT:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        return true;
    default:
        return false;
    }
}

mavlinkFrameAltResult_e mavlinkFrameAltToAmslCm(uint8_t frame, float altM, int32_t *altCmAmsl)
{
    if (!isfinite(altM) || fabsf(altM) > MAVLINK_FRAME_ALT_LIMIT_M) {
        return MAVLINK_FRAME_ALT_INVALID;
    }
    const int32_t altCm = lroundf(altM * 100.0f);
    switch (frame) {
    case MAV_FRAME_GLOBAL:
    case MAV_FRAME_GLOBAL_INT:
        *altCmAmsl = altCm;
        return MAVLINK_FRAME_ALT_OK;
    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        if (!STATE(GPS_FIX_HOME)) {
            return MAVLINK_FRAME_ALT_NO_HOME;
        }
        *altCmAmsl = altCm + GPS_home_llh.altCm;
        return MAVLINK_FRAME_ALT_OK;
    default:
        return MAVLINK_FRAME_ALT_UNSUPPORTED;
    }
}

#endif
