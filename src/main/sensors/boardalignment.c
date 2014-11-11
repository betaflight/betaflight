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

#include "sensors.h"

#include "boardalignment.h"

static bool standardBoardAlignment = true;     // board orientation correction
static float boardRotation[3][3];              // matrix

static bool isBoardAlignmentStandard(boardAlignment_t *boardAlignment)
{
    return !boardAlignment->rollDegrees && !boardAlignment->pitchDegrees && !boardAlignment->yawDegrees;
}

void initBoardAlignment(boardAlignment_t *boardAlignment)
{
    float roll, pitch, yaw;
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    if (isBoardAlignmentStandard(boardAlignment)) {
        return;
    }

    standardBoardAlignment = false;

    roll = degreesToRadians(boardAlignment->rollDegrees);
    pitch = degreesToRadians(boardAlignment->pitchDegrees);
    yaw = degreesToRadians(boardAlignment->yawDegrees);

    cosx = cosf(roll);
    sinx = sinf(roll);
    cosy = cosf(pitch);
    siny = sinf(pitch);
    cosz = cosf(yaw);
    sinz = sinf(yaw);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    // define rotation matrix
    boardRotation[0][0] = coszcosy;
    boardRotation[0][1] = -cosy * sinz;
    boardRotation[0][2] = siny;

    boardRotation[1][0] = sinzcosx + (coszsinx * siny);
    boardRotation[1][1] = coszcosx - (sinzsinx * siny);
    boardRotation[1][2] = -sinx * cosy;

    boardRotation[2][0] = (sinzsinx) - (coszcosx * siny);
    boardRotation[2][1] = (coszsinx) + (sinzcosx * siny);
    boardRotation[2][2] = cosy * cosx;
}

static void alignBoard(int16_t *vec)
{
    int16_t x = vec[X];
    int16_t y = vec[Y];
    int16_t z = vec[Z];

    vec[X] = lrintf(boardRotation[0][0] * x + boardRotation[1][0] * y + boardRotation[2][0] * z);
    vec[Y] = lrintf(boardRotation[0][1] * x + boardRotation[1][1] * y + boardRotation[2][1] * z);
    vec[Z] = lrintf(boardRotation[0][2] * x + boardRotation[1][2] * y + boardRotation[2][2] * z);
}

void alignSensors(int16_t *src, int16_t *dest, uint8_t rotation)
{
    static uint16_t swap[3];
    memcpy(swap, src, sizeof(swap));

    switch (rotation) {
        case CW0_DEG:
            dest[X] = swap[X];
            dest[Y] = swap[Y];
            dest[Z] = swap[Z];
            break;
        case CW90_DEG:
            dest[X] = swap[Y];
            dest[Y] = -swap[X];
            dest[Z] = swap[Z];
            break;
        case CW180_DEG:
            dest[X] = -swap[X];
            dest[Y] = -swap[Y];
            dest[Z] = swap[Z];
            break;
        case CW270_DEG:
            dest[X] = -swap[Y];
            dest[Y] = swap[X];
            dest[Z] = swap[Z];
            break;
        case CW0_DEG_FLIP:
            dest[X] = -swap[X];
            dest[Y] = swap[Y];
            dest[Z] = -swap[Z];
            break;
        case CW90_DEG_FLIP:
            dest[X] = swap[Y];
            dest[Y] = swap[X];
            dest[Z] = -swap[Z];
            break;
        case CW180_DEG_FLIP:
            dest[X] = swap[X];
            dest[Y] = -swap[Y];
            dest[Z] = -swap[Z];
            break;
        case CW270_DEG_FLIP:
            dest[X] = -swap[Y];
            dest[Y] = -swap[X];
            dest[Z] = -swap[Z];
            break;
        default:
            break;
    }

    if (!standardBoardAlignment)
        alignBoard(dest);
}
