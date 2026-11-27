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

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MAVLINK_FRAME_ALT_OK = 0,
    MAVLINK_FRAME_ALT_INVALID,
    MAVLINK_FRAME_ALT_NO_HOME,
    MAVLINK_FRAME_ALT_UNSUPPORTED,
} mavlinkFrameAltResult_e;

bool mavlinkFrameIsGlobal(uint8_t frame);
bool mavlinkFrameIsRelativeAlt(uint8_t frame);
mavlinkFrameAltResult_e mavlinkFrameAltToAmslCm(uint8_t frame, float altM, int32_t *altCmAmsl);
