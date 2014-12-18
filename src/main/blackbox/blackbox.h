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

typedef struct blackboxValues_t {
    uint32_t time;

    int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];

    int16_t rcCommand[4];
    int16_t gyroData[3];
    int16_t accSmooth[3];
    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];
} blackboxValues_t;

void initBlackbox(void);
void handleBlackbox(void);
void startBlackbox(void);
void finishBlackbox(void);
