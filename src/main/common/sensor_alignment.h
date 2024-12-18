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

typedef enum {
    ALIGN_DEFAULT = 0, // driver-provided alignment

    // the order of these 8 values also correlate to corresponding code in ALIGNMENT_TO_BITMASK.

                            // R, P, Y
    CW0_DEG = 1,            // 00,00,00
    CW90_DEG = 2,           // 00,00,01
    CW180_DEG = 3,          // 00,00,10
    CW270_DEG = 4,          // 00,00,11
    CW0_DEG_FLIP = 5,       // 00,10,00 // _FLIP = 2x90 degree PITCH rotations
    CW90_DEG_FLIP = 6,      // 00,10,01
    CW180_DEG_FLIP = 7,     // 00,10,10
    CW270_DEG_FLIP = 8,     // 00,10,11

    ALIGN_CUSTOM = 9,    // arbitrary sensor angles, e.g. for external sensors
} sensor_align_e;

typedef union sensorAlignment_u {
    // value order is the same as axis_e

    // values are in DECIDEGREES, and should be limited to +/- 3600

    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    };
} sensorAlignment_t;

#define SENSOR_ALIGNMENT(ROLL, PITCH, YAW) ((sensorAlignment_t){\
    .roll = DEGREES_TO_DECIDEGREES(ROLL), \
    .pitch = DEGREES_TO_DECIDEGREES(PITCH), \
    .yaw = DEGREES_TO_DECIDEGREES(YAW), \
})

#define CUSTOM_ALIGN_CW0_DEG         SENSOR_ALIGNMENT( 0,   0,   0 )
#define CUSTOM_ALIGN_CW45_DEG        SENSOR_ALIGNMENT( 0,   0,  45 )
#define CUSTOM_ALIGN_CW90_DEG        SENSOR_ALIGNMENT( 0,   0,  90 )
#define CUSTOM_ALIGN_CW135_DEG       SENSOR_ALIGNMENT( 0,   0, 135 )
#define CUSTOM_ALIGN_CW180_DEG       SENSOR_ALIGNMENT( 0,   0, 180 )
#define CUSTOM_ALIGN_CW225_DEG       SENSOR_ALIGNMENT( 0,   0, 225 )
#define CUSTOM_ALIGN_CW270_DEG       SENSOR_ALIGNMENT( 0,   0, 270 )
#define CUSTOM_ALIGN_CW315_DEG       SENSOR_ALIGNMENT( 0,   0, 315 )
#define CUSTOM_ALIGN_CW0_DEG_FLIP    SENSOR_ALIGNMENT( 0, 180,   0 )
#define CUSTOM_ALIGN_CW45_DEG_FLIP   SENSOR_ALIGNMENT( 0, 180,  45 )
#define CUSTOM_ALIGN_CW90_DEG_FLIP   SENSOR_ALIGNMENT( 0, 180,  90 )
#define CUSTOM_ALIGN_CW135_DEG_FLIP  SENSOR_ALIGNMENT( 0, 180, 135 )
#define CUSTOM_ALIGN_CW180_DEG_FLIP  SENSOR_ALIGNMENT( 0, 180, 180 )
#define CUSTOM_ALIGN_CW225_DEG_FLIP  SENSOR_ALIGNMENT( 0, 180, 225 )
#define CUSTOM_ALIGN_CW270_DEG_FLIP  SENSOR_ALIGNMENT( 0, 180, 270 )
#define CUSTOM_ALIGN_CW315_DEG_FLIP  SENSOR_ALIGNMENT( 0, 180, 315 )

void buildRotationMatrixFromAngles(matrix33_t *rm, const sensorAlignment_t *rpy);
void buildAlignmentFromStandardAlignment(sensorAlignment_t *rpy, sensor_align_e stdAlignment);
