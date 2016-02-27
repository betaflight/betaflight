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
#include "debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"
#include "drivers/gyro_sync.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "io/beeper.h"
#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/acceleration.h"

int32_t accSmooth[XYZ_AXIS_COUNT];

acc_t acc;                       // acc access functions
sensor_align_e accAlign = 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.
uint32_t accTargetLooptime;

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationArmed;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

static flightDynamicsTrims_t *accelerationTrims;

static float accLpfCutHz = 0;
static biquad_t accFilterState[XYZ_AXIS_COUNT];
static bool accFilterInitialised = false;

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

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    rollAndPitchTrims->values.roll = 0;
    rollAndPitchTrims->values.pitch = 0;
}

void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];
    int axis;

    for (axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle())
            a[axis] = 0;

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += accSmooth[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        accSmooth[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveConfigAndNotify();
    }

    calibratingA--;
}

void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    uint8_t axis;
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static rollAndPitchTrims_t angleTrim_saved = { { 0, 0 } };

    // Saving old zeropoints before measurement
    if (InflightcalibratingA == 50) {
        accZero_saved[X] = accelerationTrims->raw[X];
        accZero_saved[Y] = accelerationTrims->raw[Y];
        accZero_saved[Z] = accelerationTrims->raw[Z];
        angleTrim_saved.values.roll = rollAndPitchTrims->values.roll;
        angleTrim_saved.values.pitch = rollAndPitchTrims->values.pitch;
    }
    if (InflightcalibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += accSmooth[axis];
            // Clear global variables for next reading
            accSmooth[axis] = 0;
            accelerationTrims->raw[axis] = 0;
        }
        // all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = false;
            AccInflightCalibrationMeasurementDone = true;
            beeper(BEEPER_ACC_CALIBRATION); // indicate end of calibration
            // recover saved values to maintain current flight behaviour until new values are transferred
            accelerationTrims->raw[X] = accZero_saved[X];
            accelerationTrims->raw[Y] = accZero_saved[Y];
            accelerationTrims->raw[Z] = accZero_saved[Z];
            rollAndPitchTrims->values.roll = angleTrim_saved.values.roll;
            rollAndPitchTrims->values.pitch = angleTrim_saved.values.pitch;
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm) {      // the aircraft is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = false;
        accelerationTrims->raw[X] = b[X] / 50;
        accelerationTrims->raw[Y] = b[Y] / 50;
        accelerationTrims->raw[Z] = b[Z] / 50 - acc_1G;    // for nunchuck 200=1G

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveConfigAndNotify();
    }
}

void applyAccelerationTrims(flightDynamicsTrims_t *accelerationTrims)
{
    accSmooth[X] -= accelerationTrims->raw[X];
    accSmooth[Y] -= accelerationTrims->raw[Y];
    accSmooth[Z] -= accelerationTrims->raw[Z];
}

void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims)
{
	int16_t accADCRaw[XYZ_AXIS_COUNT];
	int axis;

    if (!acc.read(accADCRaw)) {
        return;
    }

    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        if (debugMode == DEBUG_ACCELEROMETER) debug[axis] = accADCRaw[axis];
        accSmooth[axis] = accADCRaw[axis];
    }

    if (accLpfCutHz) {
        if (!accFilterInitialised) {
            if (accTargetLooptime) {  /* Initialisation needs to happen once sample rate is known */
                for (axis = 0; axis < 3; axis++) BiQuadNewLpf(accLpfCutHz, &accFilterState[axis], accTargetLooptime);
                accFilterInitialised = true;
            }
        }

        if (accFilterInitialised) {
            for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) accSmooth[axis] = lrintf(applyBiQuadFilter((float) accSmooth[axis], &accFilterState[axis]));
        }
    }

    alignSensors(accSmooth, accSmooth, accAlign);

    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration(rollAndPitchTrims);
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }

    applyAccelerationTrims(accelerationTrims);
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationTrims = accelerationTrimsToUse;
}

void setAccelerationFilter(float initialAccLpfCutHz)
{
    accLpfCutHz = initialAccLpfCutHz;
}
