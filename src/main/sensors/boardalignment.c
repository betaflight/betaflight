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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "common/sensor_alignment.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/sensor.h"

#include "boardalignment.h"

static bool standardBoardAlignment = true;     // board orientation correction
static matrix33_t boardRotation;

PG_REGISTER_WITH_RESET_TEMPLATE(boardAlignment_t, boardAlignment, PG_BOARD_ALIGNMENT, 1);

#ifndef DEFAULT_ALIGN_BOARD_ROLL
#define DEFAULT_ALIGN_BOARD_ROLL 0
#endif
#ifndef DEFAULT_ALIGN_BOARD_PITCH
#define DEFAULT_ALIGN_BOARD_PITCH 0
#endif
#ifndef DEFAULT_ALIGN_BOARD_YAW
#define DEFAULT_ALIGN_BOARD_YAW 0
#endif

PG_RESET_TEMPLATE(boardAlignment_t, boardAlignment,
        .rollDegrees = DEFAULT_ALIGN_BOARD_ROLL,
        .pitchDegrees = DEFAULT_ALIGN_BOARD_PITCH,
        .yawDegrees = DEFAULT_ALIGN_BOARD_YAW,
);

static bool isBoardAlignmentStandard(const boardAlignment_t *boardAlignment)
{
    return !boardAlignment->rollDegrees && !boardAlignment->pitchDegrees && !boardAlignment->yawDegrees;
}

void initBoardAlignment(const boardAlignment_t *boardAlignment)
{
    if (isBoardAlignmentStandard(boardAlignment)) {
        return;
    }

    standardBoardAlignment = false;

    fp_angles_t rotationAngles;
    rotationAngles.angles.roll  = DEGREES_TO_RADIANS(boardAlignment->rollDegrees );
    rotationAngles.angles.pitch = DEGREES_TO_RADIANS(boardAlignment->pitchDegrees);
    rotationAngles.angles.yaw   = DEGREES_TO_RADIANS(boardAlignment->yawDegrees  );

    buildRotationMatrix(&boardRotation, &rotationAngles);
}

static void alignBoard(vector3_t *vec)
{
    applyRotationMatrix(vec, &boardRotation);
}

FAST_CODE_NOINLINE void alignSensorViaMatrix(vector3_t *dest, matrix33_t *sensorRotationMatrix)
{
    applyRotationMatrix(dest, sensorRotationMatrix);

    if (!standardBoardAlignment) {
        alignBoard(dest);
    }
}

void alignSensorViaRotation(vector3_t *dest, sensor_align_e rotation)
{
    const vector3_t tmp = *dest;

    switch (rotation) {
    default:
    case CW0_DEG:
        dest->x = tmp.x;
        dest->y = tmp.y;
        dest->z = tmp.z;
        break;
    case CW90_DEG:
        dest->x = tmp.y;
        dest->y = -tmp.x;
        dest->z = tmp.z;
        break;
    case CW180_DEG:
        dest->x = -tmp.x;
        dest->y = -tmp.y;
        dest->z = tmp.z;
        break;
    case CW270_DEG:
        dest->x = -tmp.y;
        dest->y = tmp.x;
        dest->z = tmp.z;
        break;
    case CW0_DEG_FLIP:
        dest->x = -tmp.x;
        dest->y = tmp.y;
        dest->z = -tmp.z;
        break;
    case CW90_DEG_FLIP:
        dest->x = tmp.y;
        dest->y = tmp.x;
        dest->z = -tmp.z;
        break;
    case CW180_DEG_FLIP:
        dest->x = tmp.x;
        dest->y = -tmp.y;
        dest->z = -tmp.z;
        break;
    case CW270_DEG_FLIP:
        dest->x = -tmp.y;
        dest->y = -tmp.x;
        dest->z = -tmp.z;
        break;
    }

    if (!standardBoardAlignment) {
        alignBoard(dest);
    }
}
