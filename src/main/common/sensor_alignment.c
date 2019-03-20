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

#include "platform.h"

#include "common/sensor_alignment.h"


void buildRotationMatrixFromAlignment(const sensorAlignment_t* sensorAlignment, fp_rotationMatrix_t* rm)
{
    fp_angles_t rotationAngles;
    rotationAngles.angles.roll  = DEGREES_TO_RADIANS(sensorAlignment->values.roll);
    rotationAngles.angles.pitch = DEGREES_TO_RADIANS(sensorAlignment->values.pitch);
    rotationAngles.angles.yaw   = DEGREES_TO_RADIANS(sensorAlignment->values.yaw);

    buildRotationMatrix(&rotationAngles, rm);
}

void buildAlignmentFromRotation(sensorAlignment_t* sensorAlignment, legacy_sensor_align_e rotation)
{
    *sensorAlignment = CW0_DEG;

    switch (rotation) {
    default:
    case LEGACY_ALIGN_CW0_DEG:
        break;
    case LEGACY_ALIGN_CW90_DEG:
        sensorAlignment->values.yaw += 90;
        break;
    case LEGACY_ALIGN_CW180_DEG:
        sensorAlignment->values.yaw += 180;
        break;
    case LEGACY_ALIGN_CW270_DEG:
        sensorAlignment->values.yaw += 270;
        break;
    case LEGACY_ALIGN_CW0_DEG_FLIP:
        sensorAlignment->values.pitch += 180;
        break;
    case LEGACY_ALIGN_CW90_DEG_FLIP:
        sensorAlignment->values.yaw += 90;
        sensorAlignment->values.pitch += 180;
        break;
    case LEGACY_ALIGN_CW180_DEG_FLIP:
        sensorAlignment->values.yaw += 180;
        sensorAlignment->values.pitch += 180;
        break;
    case LEGACY_ALIGN_CW270_DEG_FLIP:
        sensorAlignment->values.yaw += 270;
        sensorAlignment->values.pitch += 180;
        break;
    }
}
