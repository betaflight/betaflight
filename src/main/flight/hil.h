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

#ifdef HIL

typedef struct {
    int16_t     rollAngle;  // deg * 10
    int16_t     pitchAngle; // deg * 10
    int16_t     yawAngle;   // deg * 10

    int32_t     baroAlt;    // cm above launch

    int16_t     bodyAccel[3];   // cm/s/s   // forward, right, up
} hilIncomingStateData_t;

typedef struct {
    int16_t     pidCommand[4];
} hilOutgoingStateData_t;

extern bool hilActive;
extern hilIncomingStateData_t hilToFC;
extern hilOutgoingStateData_t hilToSIM;

void hilUpdateControlState(void);

#endif
