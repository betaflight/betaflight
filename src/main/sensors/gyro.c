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

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/gyro_sync.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "config/config.h"

gyro_t gyro;                      // gyro access functions

static int32_t gyroZero[XYZ_AXIS_COUNT] = { 0, 0, 0 };
static const gyroConfig_t *gyroConfig;

static uint16_t calibratingG = 0;

static biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];

void gyroInit(const gyroConfig_t *gyroConfigToUse)
{
    gyroConfig = gyroConfigToUse;
    // After refactoring this function is always called after gyro sampling rate is known, so
    // no additional condition is required
    // Set gyro sample rate before driver initialisation
    gyro.dev.lpf = gyroConfig->gyro_lpf;
    gyro.targetLooptime = gyroSetSampleRate(gyroConfig->looptime, gyroConfig->gyro_lpf, gyroConfig->gyroSync, gyroConfig->gyroSyncDenominator);
    // driver initialisation
    gyro.dev.init(&gyro.dev);
    if (gyroConfig->gyro_soft_lpf_hz) {
        for (int axis = 0; axis < 3; axis++) {
        #ifdef ASYNC_GYRO_PROCESSING
            biquadFilterInitLPF(&gyroFilterLPF[axis], gyroConfig->gyro_soft_lpf_hz, getGyroUpdateRate());
        #else
            biquadFilterInitLPF(&gyroFilterLPF[axis], gyroConfig->gyro_soft_lpf_hz, gyro.targetLooptime);
        #endif
        }
    }
}

void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingG = calibrationCyclesRequired;
}

bool isGyroCalibrationComplete(void)
{
    return calibratingG == 0;
}

static bool isOnFinalGyroCalibrationCycle(void)
{
    return calibratingG == 1;
}

static bool isOnFirstGyroCalibrationCycle(void)
{
    return calibratingG == CALIBRATING_GYRO_CYCLES;
}

static void performAcclerationCalibration(uint8_t gyroMovementCalibrationThreshold)
{
    static int32_t g[3];
    static stdev_t var[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle()) {
            g[axis] = 0;
            devClear(&var[axis]);
        }

        // Sum up CALIBRATING_GYRO_CYCLES readings
        g[axis] += gyro.gyroADC[axis];
        devPush(&var[axis], gyro.gyroADC[axis]);

        // Reset global variables to prevent other code from using un-calibrated data
        gyro.gyroADC[axis] = 0;
        gyroZero[axis] = 0;

        if (isOnFinalGyroCalibrationCycle()) {
            float dev = devStandardDeviation(&var[axis]);
            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && dev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
                return;
            }
            gyroZero[axis] = (g[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
        }
    }

    if (isOnFinalGyroCalibrationCycle()) {
        beeper(BEEPER_GYRO_CALIBRATED);
    }
    calibratingG--;

}

void gyroUpdate(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    if (!gyro.dev.read(&gyro.dev)) {
        return;
    }

    // Prepare a copy of int32_t gyroADC for mangling to prevent overflow
    gyro.gyroADC[X] = gyro.dev.gyroADCRaw[X];
    gyro.gyroADC[Y] = gyro.dev.gyroADCRaw[Y];
    gyro.gyroADC[Z] = gyro.dev.gyroADCRaw[Z];

    if (gyroConfig->gyro_soft_lpf_hz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            gyro.gyroADC[axis] = lrintf(biquadFilterApply(&gyroFilterLPF[axis], (float)gyro.gyroADC[axis]));
        }
    }

    if (!isGyroCalibrationComplete()) {
        performAcclerationCalibration(gyroConfig->gyroMovementCalibrationThreshold);
    }

    gyro.gyroADC[X] -= gyroZero[X];
    gyro.gyroADC[Y] -= gyroZero[Y];
    gyro.gyroADC[Z] -= gyroZero[Z];

    alignSensors(gyro.gyroADC, gyro.dev.gyroAlign);
}
