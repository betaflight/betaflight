#include "board.h"
#include "mw.h"

#include "flight_common.h"

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationArmed;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

void ACC_Common(void)
{
    static int32_t a[3];
    int axis;

    if (calibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == CALIBRATING_ACC_CYCLES)
                a[axis] = 0;
            // Sum up CALIBRATING_ACC_CYCLES readings
            a[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            mcfg.accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (calibratingA == 1) {
            mcfg.accZero[ROLL] = (a[ROLL] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
            mcfg.accZero[PITCH] = (a[PITCH] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
            mcfg.accZero[YAW] = (a[YAW] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc_1G;
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeEEPROM(1, true);      // write accZero in EEPROM
        }
        calibratingA--;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t angleTrim_saved[2] = { 0, 0 };
        // Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
            accZero_saved[ROLL] = mcfg.accZero[ROLL];
            accZero_saved[PITCH] = mcfg.accZero[PITCH];
            accZero_saved[YAW] = mcfg.accZero[YAW];
            angleTrim_saved[ROLL] = cfg.angleTrim[ROLL];
            angleTrim_saved[PITCH] = cfg.angleTrim[PITCH];
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
                mcfg.accZero[axis] = 0;
            }
            // all values are measured
            if (InflightcalibratingA == 1) {
                AccInflightCalibrationActive = false;
                AccInflightCalibrationMeasurementDone = true;
                toggleBeep = 2;      // buzzer for indicatiing the end of calibration
                // recover saved values to maintain current flight behavior until new values are transferred
                mcfg.accZero[ROLL] = accZero_saved[ROLL];
                mcfg.accZero[PITCH] = accZero_saved[PITCH];
                mcfg.accZero[YAW] = accZero_saved[YAW];
                cfg.angleTrim[ROLL] = angleTrim_saved[ROLL];
                cfg.angleTrim[PITCH] = angleTrim_saved[PITCH];
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm) {      // the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = false;
            mcfg.accZero[ROLL] = b[ROLL] / 50;
            mcfg.accZero[PITCH] = b[PITCH] / 50;
            mcfg.accZero[YAW] = b[YAW] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeEEPROM(1, true);          // write accZero in EEPROM
        }
    }

    accADC[ROLL] -= mcfg.accZero[ROLL];
    accADC[PITCH] -= mcfg.accZero[PITCH];
    accADC[YAW] -= mcfg.accZero[YAW];
}

void ACC_getADC(void)
{
    acc.read(accADC);
    ACC_Common();
}

