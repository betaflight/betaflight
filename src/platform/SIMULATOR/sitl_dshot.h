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

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"

// Motor speed for a DShot value, negative when the motor turns in reverse
static inline float sitlDshotMotorSpeed(uint16_t value, bool is3D, bool *reversed)
{
    if (value == DSHOT_CMD_SPIN_DIRECTION_NORMAL) {
        *reversed = false;
    } else if (value == DSHOT_CMD_SPIN_DIRECTION_REVERSED) {
        *reversed = true;
    }

    if (value < DSHOT_MIN_THROTTLE) {
        return 0.0f;
    }

    float speed;
    if (!is3D) {
        speed = (float)(value - DSHOT_MIN_THROTTLE) / DSHOT_RANGE;
    } else if (value < DSHOT_3D_FORWARD_MIN_THROTTLE) {
        speed = -(float)(value - DSHOT_MIN_THROTTLE) / (DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - DSHOT_MIN_THROTTLE);
    } else {
        speed = (float)(value - DSHOT_3D_FORWARD_MIN_THROTTLE) / (DSHOT_MAX_THROTTLE - DSHOT_3D_FORWARD_MIN_THROTTLE);
    }

    return *reversed ? -speed : speed;
}
