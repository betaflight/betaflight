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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"

#include "boardalignment.h"

static bool standardBoardAlignment = true;     // board orientation correction
static float boardRotation[3][3];              // matrix

static bool isBoardAlignmentStandard(const boardAlignment_t *boardAlignment)
{
    return !boardAlignment->rollDeciDegrees && !boardAlignment->pitchDeciDegrees && !boardAlignment->yawDeciDegrees;
}

void initBoardAlignment(const boardAlignment_t *boardAlignment)
{
    if (isBoardAlignmentStandard(boardAlignment)) {
        standardBoardAlignment = true;
    }
    else {
        fp_angles_t rotationAngles;

        standardBoardAlignment = false;

        rotationAngles.angles.roll  = DECIDEGREES_TO_RADIANS(boardAlignment->rollDeciDegrees );
        rotationAngles.angles.pitch = DECIDEGREES_TO_RADIANS(boardAlignment->pitchDeciDegrees);
        rotationAngles.angles.yaw   = DECIDEGREES_TO_RADIANS(boardAlignment->yawDeciDegrees  );

        buildRotationMatrix(&rotationAngles, boardRotation);
    }
}

void updateBoardAlignment(boardAlignment_t *boardAlignment, int16_t roll, int16_t pitch)
{
    float sinAlignYaw = sin_approx(DECIDEGREES_TO_RADIANS(boardAlignment->yawDeciDegrees));
    float cosAlignYaw = cos_approx(DECIDEGREES_TO_RADIANS(boardAlignment->yawDeciDegrees));

    boardAlignment->rollDeciDegrees += -sinAlignYaw * pitch + cosAlignYaw * roll;
    boardAlignment->pitchDeciDegrees += cosAlignYaw * pitch + sinAlignYaw * roll;

    initBoardAlignment(boardAlignment);
}

static void alignBoard(int32_t *vec)
{
    int32_t x = vec[X];
    int32_t y = vec[Y];
    int32_t z = vec[Z];

    vec[X] = lrintf(boardRotation[0][X] * x + boardRotation[1][X] * y + boardRotation[2][X] * z);
    vec[Y] = lrintf(boardRotation[0][Y] * x + boardRotation[1][Y] * y + boardRotation[2][Y] * z);
    vec[Z] = lrintf(boardRotation[0][Z] * x + boardRotation[1][Z] * y + boardRotation[2][Z] * z);
}

void alignSensors(int32_t *dest, uint8_t rotation)
{
    const int32_t x = dest[X];
    const int32_t y = dest[Y];
    const int32_t z = dest[Z];

    switch (rotation) {
    default:
    case CW0_DEG:
        dest[X] = x;
        dest[Y] = y;
        dest[Z] = z;
        break;
    case CW90_DEG:
        dest[X] = y;
        dest[Y] = -x;
        dest[Z] = z;
        break;
    case CW180_DEG:
        dest[X] = -x;
        dest[Y] = -y;
        dest[Z] = z;
        break;
    case CW270_DEG:
        dest[X] = -y;
        dest[Y] = x;
        dest[Z] = z;
        break;
    case CW0_DEG_FLIP:
        dest[X] = -x;
        dest[Y] = y;
        dest[Z] = -z;
        break;
    case CW90_DEG_FLIP:
        dest[X] = y;
        dest[Y] = x;
        dest[Z] = -z;
        break;
    case CW180_DEG_FLIP:
        dest[X] = x;
        dest[Y] = -y;
        dest[Z] = -z;
        break;
    case CW270_DEG_FLIP:
        dest[X] = -y;
        dest[Y] = -x;
        dest[Z] = -z;
        break;
    }

    if (!standardBoardAlignment)
        alignBoard(dest);
}
