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

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "config/config.h"

gyro_t gyro;                      // gyro access functions
sensor_align_e gyroAlign = 0;

int32_t gyroADC[XYZ_AXIS_COUNT];

static int32_t gyroZero[XYZ_AXIS_COUNT] = { 0, 0, 0 };
static const gyroConfig_t *gyroConfig;

static uint16_t calibratingG = 0;

static biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];
static uint8_t gyroSoftLpfHz = 0;

void gyroUseConfig(const gyroConfig_t *gyroConfigToUse, uint8_t gyro_soft_lpf_hz)
{
    gyroConfig = gyroConfigToUse;
    gyroSoftLpfHz = gyro_soft_lpf_hz;
}

void gyroInit(void)
{
    /*
     * After refactoring this function is always called after gyro sampling rate is known, so
     * no additional condition is required
     */
    if (gyroSoftLpfHz) {
        for (int axis = 0; axis < 3; axis++) {
        #ifdef ASYNC_GYRO_PROCESSING
            biquadFilterInitLPF(&gyroFilterLPF[axis], gyroSoftLpfHz, getGyroUpdateRate());
        #else
            biquadFilterInitLPF(&gyroFilterLPF[axis], gyroSoftLpfHz, gyro.targetLooptime);
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
        g[axis] += gyroADC[axis];
        devPush(&var[axis], gyroADC[axis]);

        // Reset global variables to prevent other code from using un-calibrated data
        gyroADC[axis] = 0;
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

static void applyGyroZero(void)
{
    for (int axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
    }
}

void gyroUpdate(void)
{
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];

    // range: +/- 8192; +/- 2000 deg/sec
    if (!gyro.read(gyroADCRaw)) {
        return;
    }

    // Prepare a copy of int32_t gyroADC for mangling to prevent overflow
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroADC[axis] = gyroADCRaw[axis];
    }

    if (gyroSoftLpfHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            gyroADC[axis] = lrintf(biquadFilterApply(&gyroFilterLPF[axis], (float) gyroADC[axis]));
        }
    }

    if (!isGyroCalibrationComplete()) {
        performAcclerationCalibration(gyroConfig->gyroMovementCalibrationThreshold);
    }

    applyGyroZero();

    alignSensors(gyroADC, gyroADC, gyroAlign);
}
