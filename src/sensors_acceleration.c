#include "board.h"

#include "flight_common.h"

#include "mw.h"

#include "sensors_acceleration.h"

sensor_t acc;                       // acc access functions
uint8_t accHardware = ACC_DEFAULT;  // which accel chip is used/detected

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
            mcfg.accZero[GI_ROLL] = (a[GI_ROLL] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
            mcfg.accZero[GI_PITCH] = (a[GI_PITCH] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
            mcfg.accZero[GI_YAW] = (a[GI_YAW] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc_1G;
            cfg.angleTrim[AI_ROLL] = 0;
            cfg.angleTrim[AI_PITCH] = 0;
            copyCurrentProfileToProfileSlot(mcfg.current_profile);
            writeEEPROM();      // write accZero in EEPROM
            readEEPROMAndNotify();
        }
        calibratingA--;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t angleTrim_saved[2] = { 0, 0 };
        // Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
            accZero_saved[GI_ROLL] = mcfg.accZero[GI_ROLL];
            accZero_saved[GI_PITCH] = mcfg.accZero[GI_PITCH];
            accZero_saved[GI_YAW] = mcfg.accZero[GI_YAW];
            angleTrim_saved[AI_ROLL] = cfg.angleTrim[AI_ROLL];
            angleTrim_saved[AI_PITCH] = cfg.angleTrim[AI_PITCH];
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
                mcfg.accZero[GI_ROLL] = accZero_saved[GI_ROLL];
                mcfg.accZero[GI_PITCH] = accZero_saved[GI_PITCH];
                mcfg.accZero[GI_YAW] = accZero_saved[GI_YAW];
                cfg.angleTrim[AI_ROLL] = angleTrim_saved[AI_ROLL];
                cfg.angleTrim[AI_PITCH] = angleTrim_saved[AI_PITCH];
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm) {      // the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = false;
            mcfg.accZero[GI_ROLL] = b[GI_ROLL] / 50;
            mcfg.accZero[GI_PITCH] = b[GI_PITCH] / 50;
            mcfg.accZero[GI_YAW] = b[GI_YAW] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.angleTrim[AI_ROLL] = 0;
            cfg.angleTrim[AI_PITCH] = 0;
            copyCurrentProfileToProfileSlot(mcfg.current_profile);
            writeEEPROM();          // write accZero in EEPROM
            readEEPROMAndNotify();
        }
    }

    accADC[GI_ROLL] -= mcfg.accZero[GI_ROLL];
    accADC[GI_PITCH] -= mcfg.accZero[GI_PITCH];
    accADC[GI_YAW] -= mcfg.accZero[GI_YAW];
}

void ACC_getADC(void)
{
    acc.read(accADC);
    ACC_Common();
}

