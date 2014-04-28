#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"

#include "drivers/accgyro_common.h"
#include "flight_common.h"
#include "sensors_common.h"
#include "buzzer.h"
#include "boardalignment.h"
#include "runtime_config.h"
#include "config.h"

#include "sensors_acceleration.h"

acc_t acc;                       // acc access functions
uint8_t accHardware = ACC_DEFAULT;  // which accel chip is used/detected
sensor_align_e accAlign = 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationArmed;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

int16_flightDynamicsTrims_t *accelerationTrims;

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

void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];
    uint8_t axis;

    for (axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle())
            a[axis] = 0;

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        accADC[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationTrims->raw[FD_ROLL] = (a[FD_ROLL] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[FD_PITCH] = (a[FD_PITCH] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[FD_YAW] = (a[FD_YAW] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveAndReloadCurrentProfileToCurrentProfileSlot();
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
        accZero_saved[FD_ROLL] = accelerationTrims->raw[FD_ROLL];
        accZero_saved[FD_PITCH] = accelerationTrims->raw[FD_PITCH];
        accZero_saved[FD_YAW] = accelerationTrims->raw[FD_YAW];
        angleTrim_saved.trims.roll = rollAndPitchTrims->trims.roll;
        angleTrim_saved.trims.pitch = rollAndPitchTrims->trims.pitch;
    }
    if (InflightcalibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            accelerationTrims->raw[axis] = 0;
        }
        // all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = false;
            AccInflightCalibrationMeasurementDone = true;
            queueConfirmationBeep(2); // buzzer to indicatiing the end of calibration
            // recover saved values to maintain current flight behavior until new values are transferred
            accelerationTrims->raw[FD_ROLL] = accZero_saved[FD_ROLL];
            accelerationTrims->raw[FD_PITCH] = accZero_saved[FD_PITCH];
            accelerationTrims->raw[FD_YAW] = accZero_saved[FD_YAW];
            rollAndPitchTrims->trims.roll = angleTrim_saved.trims.roll;
            rollAndPitchTrims->trims.pitch = angleTrim_saved.trims.pitch;
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm) {      // the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = false;
        accelerationTrims->raw[FD_ROLL] = b[FD_ROLL] / 50;
        accelerationTrims->raw[FD_PITCH] = b[FD_PITCH] / 50;
        accelerationTrims->raw[FD_YAW] = b[FD_YAW] / 50 - acc_1G;    // for nunchuk 200=1G

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveAndReloadCurrentProfileToCurrentProfileSlot();
    }

}

void applyAccelerationTrims(int16_flightDynamicsTrims_t *accelerationTrims)
{
    accADC[FD_ROLL] -= accelerationTrims->raw[FD_ROLL];
    accADC[FD_PITCH] -= accelerationTrims->raw[FD_PITCH];
    accADC[FD_YAW] -= accelerationTrims->raw[FD_YAW];
}

void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims)
{
    acc.read(accADC);
    alignSensors(accADC, accADC, accAlign);

    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration(rollAndPitchTrims);
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }

    applyAccelerationTrims(accelerationTrims);
}

void setAccelerationTrims(int16_flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationTrims = accelerationTrimsToUse;
}
