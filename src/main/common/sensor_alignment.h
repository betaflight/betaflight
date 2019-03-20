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

#include "axis.h"
#include "maths.h"

// Legacy alignment values
typedef enum {
    LEGACY_ALIGN_DEFAULT = 0,
    LEGACY_ALIGN_CW0_DEG = 1,
    LEGACY_ALIGN_CW90_DEG = 2,
    LEGACY_ALIGN_CW180_DEG = 3,
    LEGACY_ALIGN_CW270_DEG = 4,
    LEGACY_ALIGN_CW0_DEG_FLIP = 5,
    LEGACY_ALIGN_CW90_DEG_FLIP = 6,
    LEGACY_ALIGN_CW180_DEG_FLIP = 7,
    LEGACY_ALIGN_CW270_DEG_FLIP = 8
} legacy_sensor_align_e;

typedef union sensorAlignment_u {
    // value order is the same as axis_e

    uint16_t raw[XYZ_AXIS_COUNT];
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} sensorAlignment_t;


#define CW0_DEG         ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 0,   .yaw = 0    }})
#define CW90_DEG        ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 0,   .yaw = 90   }})
#define CW180_DEG       ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 0,   .yaw = 180  }})
#define CW270_DEG       ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 0,   .yaw = 270  }})
#define CW0_DEG_FLIP    ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 180, .yaw = 0    }})
#define CW90_DEG_FLIP   ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 180, .yaw = 90   }})
#define CW180_DEG_FLIP  ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 180, .yaw = 180  }})
#define CW270_DEG_FLIP  ((sensorAlignment_t){ .values = { .roll = 0, .pitch = 180, .yaw = 270  }})

void buildRotationMatrixFromAlignment(const sensorAlignment_t* alignment, fp_rotationMatrix_t* rm);
void buildAlignmentFromRotation(sensorAlignment_t* sensorAlignment, legacy_sensor_align_e rotation);
