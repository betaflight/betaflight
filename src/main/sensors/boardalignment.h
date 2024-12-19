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

#include "common/axis.h"
#include "common/maths.h"
#include "common/vector.h"

#include "pg/pg.h"

typedef struct boardAlignment_s {
    int32_t rollDegrees;
    int32_t pitchDegrees;
    int32_t yawDegrees;
} boardAlignment_t;

PG_DECLARE(boardAlignment_t, boardAlignment);

void alignSensorViaMatrix(vector3_t *dest, matrix33_t *rotationMatrix);
void alignSensorViaRotation(vector3_t *dest, sensor_align_e rotation);

void initBoardAlignment(const boardAlignment_t *boardAlignment);
