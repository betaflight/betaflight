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

#define MAVLINK_ATTRATE_DEADBAND_LOW -0.001
#define MAVLINK_ATTRATE_DEADBAND_HIGH  0.001

#define MAVLINK_ATTRATE_BAUDRATE 921600

#define MAVLINK_BUFFER_SIZE 512

#include <stdbool.h>

#include "common/time.h"

float getMavlinkThrottle();
#ifdef USE_MAVLINK_ATTRATE
float getMavlinkAttrateSetpoint(int axis);
void mavlinkAttrateUpdate(timeUs_t currentTimeUs);
bool mavlinkAttrateInit();
#endif // USE_MAVLINK_ATTRATE
