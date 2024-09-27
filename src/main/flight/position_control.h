/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "flight/pid.h"

typedef struct positionControlConfig_s {
    uint16_t hover_throttle;      // value used at the start of a rescue or position hold
    uint8_t landing_altitude_m;   // altitude below which landing behaviours can change, metres
    uint8_t altitude_P;
    uint8_t altitude_I;
    uint8_t altitude_D;
    uint8_t altitude_F;
} positionControlConfig_t;

PG_DECLARE(positionControlConfig_t, positionControlConfig);

void positionControlInit(void);
const pidCoefficient_t *getAltitudePidCoeffs(void);
