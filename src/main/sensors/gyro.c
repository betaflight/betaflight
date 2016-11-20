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

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

gyro_t gyro;                      // gyro access functions
sensor_align_e gyroAlign = 0;

int32_t gyroADC[XYZ_AXIS_COUNT];
float gyroADCf[XYZ_AXIS_COUNT];

static int32_t gyroZero[XYZ_AXIS_COUNT] = { 0, 0, 0 };
static const gyroConfig_t *gyroConfig;
static biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];
static biquadFilter_t gyroFilterNotch_1[XYZ_AXIS_COUNT], gyroFilterNotch_2[XYZ_AXIS_COUNT];
static pt1Filter_t gyroFilterPt1[XYZ_AXIS_COUNT];
static firFilterDenoise_t gyroDenoiseState[XYZ_AXIS_COUNT];
static uint8_t gyroSoftLpfType;
static uint16_t gyroSoftNotchHz1, gyroSoftNotchHz2;
static float gyroSoftNotchQ1, gyroSoftNotchQ2;
static uint8_t gyroSoftLpfHz;
static uint16_t calibratingG = 0;

static filterApplyFnPtr softLpfFilterApplyFn;
static filterApplyFnPtr notchFilter1ApplyFn;
static filterApplyFnPtr notchFilter2ApplyFn;

void gyroUseConfig(const gyroConfig_t *gyroConfigToUse,
                   uint8_t gyro_soft_lpf_hz,
                   uint16_t gyro_soft_notch_hz_1,
                   uint16_t gyro_soft_notch_cutoff_1,
                   uint16_t gyro_soft_notch_hz_2,
                   uint16_t gyro_soft_notch_cutoff_2,
                   uint8_t gyro_soft_lpf_type)
{
    gyroConfig = gyroConfigToUse;
    gyroSoftLpfHz = gyro_soft_lpf_hz;
    gyroSoftNotchHz1 = gyro_soft_notch_hz_1;
    gyroSoftNotchHz2 = gyro_soft_notch_hz_2;
    gyroSoftLpfType = gyro_soft_lpf_type;
    gyroSoftNotchQ1 = filterGetNotchQ(gyro_soft_notch_hz_1, gyro_soft_notch_cutoff_1);
    gyroSoftNotchQ2 = filterGetNotchQ(gyro_soft_notch_hz_2, gyro_soft_notch_cutoff_2);
}

void gyroInit(void)
{

    softLpfFilterApplyFn = nullFilterApply;
    notchFilter1ApplyFn = nullFilterApply;
    notchFilter2ApplyFn = nullFilterApply;

    if (gyroSoftLpfHz) {  // Initialisation needs to happen once samplingrate is known
        if (gyroSoftLpfType == FILTER_BIQUAD) {
            softLpfFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = 0; axis < 3; axis++) {
                biquadFilterInitLPF(&gyroFilterLPF[axis], gyroSoftLpfHz, gyro.targetLooptime);
            }
        } else if (gyroSoftLpfType == FILTER_PT1) {
            softLpfFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
            const float gyroDt = (float) gyro.targetLooptime * 0.000001f;
            for (int axis = 0; axis < 3; axis++) {
                pt1FilterInit(&gyroFilterPt1[axis], gyroSoftLpfHz, gyroDt);
            }
        } else {
            softLpfFilterApplyFn = (filterApplyFnPtr)firFilterDenoiseUpdate;
            for (int axis = 0; axis < 3; axis++) {
                firFilterDenoiseInit(&gyroDenoiseState[axis], gyroSoftLpfHz, gyro.targetLooptime);
            }
        }
    }

    if (gyroSoftNotchHz1) {
        notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < 3; axis++) {
            biquadFilterInit(&gyroFilterNotch_1[axis], gyroSoftNotchHz1, gyro.targetLooptime, gyroSoftNotchQ1, FILTER_NOTCH);
        }
    }
    if (gyroSoftNotchHz1) {
        notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < 3; axis++) {
            biquadFilterInit(&gyroFilterNotch_2[axis], gyroSoftNotchHz2, gyro.targetLooptime, gyroSoftNotchQ2, FILTER_NOTCH);
        }
    }
}

bool isGyroCalibrationComplete(void)
{
    return calibratingG == 0;
}

static bool isOnFinalGyroCalibrationCycle(void)
{
    return calibratingG == 1;
}

static uint16_t gyroCalculateCalibratingCycles(void)
{
    return (CALIBRATING_GYRO_CYCLES / gyro.targetLooptime) * CALIBRATING_GYRO_CYCLES;
}

static bool isOnFirstGyroCalibrationCycle(void)
{
    return calibratingG == gyroCalculateCalibratingCycles();
}

void gyroSetCalibrationCycles(void)
{
    calibratingG = gyroCalculateCalibratingCycles();
}

static void performGyroCalibration(uint8_t gyroMovementCalibrationThreshold)
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
                gyroSetCalibrationCycles();
                return;
            }
            gyroZero[axis] = (g[axis] + (gyroCalculateCalibratingCycles() / 2)) / gyroCalculateCalibratingCycles();
        }
    }

    if (isOnFinalGyroCalibrationCycle()) {
        beeper(BEEPER_GYRO_CALIBRATED);
    }
    calibratingG--;

}

void gyroUpdate(void)
{
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];

    // range: +/- 8192; +/- 2000 deg/sec
    if (!gyro.read(gyroADCRaw)) {
        return;
    }

    gyroADC[X] = gyroADCRaw[X];
    gyroADC[Y] = gyroADCRaw[Y];
    gyroADC[Z] = gyroADCRaw[Z];

    alignSensors(gyroADC, gyroADC, gyroAlign);

    if (!isGyroCalibrationComplete()) {
        performGyroCalibration(gyroConfig->gyroMovementCalibrationThreshold);
    }

    gyroADC[X] -= gyroZero[X];
    gyroADC[Y] -= gyroZero[Y];
    gyroADC[Z] -= gyroZero[Z];

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

        if (debugMode == DEBUG_GYRO)
            debug[axis] = gyroADC[axis];

        gyroADCf[axis] = softLpfFilterApplyFn(&gyroDenoiseState[axis], (float) gyroADC[axis]);

        if (debugMode == DEBUG_NOTCH)
            debug[axis] = lrintf(gyroADCf[axis]);

        gyroADCf[axis] = notchFilter1ApplyFn(&gyroFilterNotch_1[axis], gyroADCf[axis]);

        gyroADCf[axis] = notchFilter2ApplyFn(&gyroFilterNotch_2[axis], gyroADCf[axis]);

        gyroADC[axis] = lrintf(gyroADCf[axis]);
    }
}
