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

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/filter.h"
#include "common/utils.h"

#include "config/feature.h"

#include "sensors/acceleration_init.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "acceleration.h"

FAST_DATA_ZERO_INIT acc_t acc;                       // acc access functions

static inline void alignAccelerometer(void)
{
    switch (acc.dev.accAlign) {
        case ALIGN_CUSTOM:
            alignSensorViaMatrix(&acc.accADC, &acc.dev.rotationMatrix);
            break;
        default:
            alignSensorViaRotation(&acc.accADC, acc.dev.accAlign);
            break;
    }
}

static inline void calibrateAccelerometer(void)
{
    if (!accIsCalibrationComplete()) {
        // acc.accADC is held at 0 until calibration is completed
        performAccelerometerCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }
}

static inline void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC.x -= accelerationTrims->raw[X];
    acc.accADC.y -= accelerationTrims->raw[Y];
    acc.accADC.z -= accelerationTrims->raw[Z];
}

static inline void postProcessAccelerometer(void)
{
    static vector3_t accAdcPrev;

    for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

        // Apply anti-alias filter for attitude task (if enabled)
        if (axis == gyro.gyroDebugAxis) {
            DEBUG_SET(DEBUG_ACCELEROMETER, 0, lrintf(acc.accADC.v[axis]));
        }

        if (accelerationRuntime.accLpfCutHz) {
            acc.accADC.v[axis] = pt2FilterVec3Apply(&accelerationRuntime.accFilter, acc.accADC.v[axis], axis);
        }

        // Calculate derivative of acc (jerk)
        acc.jerk.v[axis] = (acc.accADC.v[axis] - accAdcPrev.v[axis]) * acc.sampleRateHz;
        accAdcPrev.v[axis] = acc.accADC.v[axis];

        if (axis == gyro.gyroDebugAxis) {
            DEBUG_SET(DEBUG_ACCELEROMETER, 1, lrintf(acc.accADC.v[axis]));
            DEBUG_SET(DEBUG_ACCELEROMETER, 3, lrintf(acc.jerk.v[axis] * 1e-2f));
        }
    }

    acc.accMagnitude = vector3Norm(&acc.accADC) * acc.dev.acc_1G_rec;
    acc.jerkMagnitude = vector3Norm(&acc.jerk) * acc.dev.acc_1G_rec;

    DEBUG_SET(DEBUG_ACCELEROMETER, 2, lrintf(acc.accMagnitude * 1e3f));
    DEBUG_SET(DEBUG_ACCELEROMETER, 4, lrintf(acc.jerkMagnitude * 1e3f));
}

void accUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADC.v[axis] = acc.dev.ADCRaw[axis];
    }

    alignAccelerometer();
    calibrateAccelerometer();
    applyAccelerationTrims(accelerationRuntime.accelerationTrims);
    postProcessAccelerometer();

    acc.isAccelUpdatedAtLeastOnce = true;
}

#endif // USE_ACC
