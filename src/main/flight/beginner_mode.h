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

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BEGINNER_MODE_NONE = 0x00,
    BEGINNER_MODE_ROLL = 0x01,
    BEGINNER_MODE_PITCH = 0x02,
    BEGINNER_MODE_ROLLPITCH = 0x03,
} beginnerMode_e;

typedef struct beginnerModeConfig_d {
    uint8_t enabled_beginnerMode;
    uint8_t maxRoll;
    uint8_t maxPitch;
} beginnerModeConfig_t;

PG_DECLARE(beginnerModeConfig_t, beginnerModeConfig);

void beginnerModeHandleAttitude(float roll, float pitch, float yaw);