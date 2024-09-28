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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "platform.h"

#include "flight/pid.h" // for pidCoefficient_t

#include "position_control.h"

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f

static pidCoefficient_t altitudePidCoeffs;

void positionControlInit(void)
{
    altitudePidCoeffs.Kp = positionControlConfig()->altitude_P * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = positionControlConfig()->altitude_I * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = positionControlConfig()->altitude_D * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = positionControlConfig()->altitude_F * ALTITUDE_F_SCALE;
}

const pidCoefficient_t *getAltitudePidCoeffs(void)
{
    return &altitudePidCoeffs;
}
