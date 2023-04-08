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
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/sensor_alignment.h"
#include "common/sensor_alignment_impl.h"

void buildRotationMatrixFromAlignment(const sensorAlignment_t* sensorAlignment, fp_rotationMatrix_t* rm)
{
    fp_angles_t rotationAngles;
    rotationAngles.angles.roll  = DECIDEGREES_TO_RADIANS(sensorAlignment->roll);
    rotationAngles.angles.pitch = DECIDEGREES_TO_RADIANS(sensorAlignment->pitch);
    rotationAngles.angles.yaw   = DECIDEGREES_TO_RADIANS(sensorAlignment->yaw);

    buildRotationMatrix(&rotationAngles, rm);
}


void buildAlignmentFromStandardAlignment(sensorAlignment_t* sensorAlignment, sensor_align_e alignment)
{
    if (alignment == ALIGN_CUSTOM || alignment == ALIGN_DEFAULT) {
        return;
    }

    uint8_t alignmentBits = ALIGNMENT_TO_BITMASK(alignment);

    memset(sensorAlignment, 0x00, sizeof(sensorAlignment_t));

    for (int axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        sensorAlignment->raw[axis] = DEGREES_TO_DECIDEGREES(90) * ALIGNMENT_AXIS_ROTATIONS(alignmentBits, axis);
    }
}
