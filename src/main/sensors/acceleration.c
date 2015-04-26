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

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "io/beeper.h"
#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/acceleration.h"

int16_t accADC[XYZ_AXIS_COUNT];

acc_t acc;                       // acc access functions
sensor_align_e accAlign = 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

static flightDynamicsTrims_t * accZero;
static flightDynamicsTrims_t * accGain;
static int8_t * accFIRTable = 0L;
static int16_t accFIRState[3][9];

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingA = calibrationCyclesRequired;
}

bool isAccelerationCalibrationComplete(void)
{
    return calibratingA == 0;
}

bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static sensorCalibrationState_t calState;
static bool calibratedAxis[6];
static int32_t accSamples[6][3];
static int  calibratedAxisCount = 0;

int getPrimaryAxisIndex(int16_t sample[3])
{
    if (ABS(sample[Z]) > ABS(sample[X]) && ABS(sample[Z]) > ABS(sample[Y])) {
        //Z-axis
        return (sample[Z] > 0) ? 0 : 1;
    }
    else if (ABS(sample[X]) > ABS(sample[Y]) && ABS(sample[X]) > ABS(sample[Z])) {
        //X-axis
        return (sample[X] > 0) ? 2 : 3;
    }
    else if (ABS(sample[Y]) > ABS(sample[X]) && ABS(sample[Y]) > ABS(sample[Z])) {
        //Y-axis
        return (sample[Y] > 0) ? 4 : 5;
    }
    else 
        return -1;
}

void performAcclerationCalibration(void)
{
    int axisIndex = getPrimaryAxisIndex(accADC);
    uint8_t axis;

    // Check if sample is usable
    if (axisIndex < 0) {
        return;
    }

    // Top-up and first calibration cycle, reset everything
    if (axisIndex == 0 && isOnFirstAccelerationCalibrationCycle()) {
        for (axis = 0; axis < 6; axis++) {
            calibratedAxis[axis] = false;
            accSamples[axis][X] = 0;
            accSamples[axis][Y] = 0;
            accSamples[axis][Z] = 0;
        }

        calibratedAxisCount = 0;
        sensorCalibrationResetState(&calState);
    }

    if (!calibratedAxis[axisIndex]) {
        sensorCalibrationPushSampleForOffsetCalculation(&calState, accADC);
        accSamples[axisIndex][X] += accADC[X];
        accSamples[axisIndex][Y] += accADC[Y];
        accSamples[axisIndex][Z] += accADC[Z];

        if (isOnFinalAccelerationCalibrationCycle()) {
            calibratedAxis[axisIndex] = true;
            calibratedAxisCount++;

            beeperConfirmationBeeps(2);
        }
    }

    if (calibratedAxisCount == 6) {
        float accTmp[3];
        int16_t accSample16[3];

        /* Calculate offset */
        sensorCalibrationSolveForOffset(&calState, accTmp);

        for (axis = 0; axis < 3; axis++) {
            accZero->raw[axis] = lrintf(accTmp[axis]);
        }

        /* Not we can offset our accumulated averages samples and calculate scale factors and calculate gains */
        sensorCalibrationResetState(&calState);

        for (axis = 0; axis < 6; axis++) {
            accSample16[X] = accSamples[axis][X] / CALIBRATING_ACC_CYCLES - accZero->raw[X];
            accSample16[Y] = accSamples[axis][Y] / CALIBRATING_ACC_CYCLES - accZero->raw[Y];
            accSample16[Z] = accSamples[axis][Z] / CALIBRATING_ACC_CYCLES - accZero->raw[Z];

            sensorCalibrationPushSampleForScaleCalculation(&calState, axis / 2, accSample16, acc_1G);
        }

        sensorCalibrationSolveForScale(&calState, accTmp);

        for (axis = 0; axis < 3; axis++) {
            accGain->raw[axis] = lrintf(accTmp[axis] * 4096);
        }

        saveConfigAndNotify();
    }

    calibratingA--;
}

void applyAccelerationZero(flightDynamicsTrims_t * accZero, flightDynamicsTrims_t * accGain)
{
    accADC[X] = (accADC[X] - accZero->raw[X]) * accGain->raw[X] / 4096;
    accADC[Y] = (accADC[Y] - accZero->raw[Y]) * accGain->raw[Y] / 4096;
    accADC[Z] = (accADC[Z] - accZero->raw[Z]) * accGain->raw[Z] / 4096;
}

void updateAccelerationReadings(void)
{
    if (!acc.read(accADC)) {
        return;
    }

    if (accFIRTable) {
        filterApply9TapFIR(accADC, accFIRState, accFIRTable);
    }

    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration();
    }

    alignSensors(accADC, accADC, accAlign);

    applyAccelerationZero(accZero, accGain);
}

void setAccelerationZero(flightDynamicsTrims_t * accZeroToUse)
{
    accZero = accZeroToUse;
}

void setAccelerationGain(flightDynamicsTrims_t * accGainToUse)
{
    accGain = accGainToUse;
}

void setAccelerationFilter(int8_t * filterTableToUse)
{
    accFIRTable = filterTableToUse;
}
