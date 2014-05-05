#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "runtime_config.h"

#include "rc_controls.h"
#include "flight_common.h"
#include "gps_common.h"

extern uint16_t cycleTime;

int16_t heading, magHold;
int16_t axisPID[3];
uint8_t dynP8[3], dynI8[3], dynD8[3];


static int32_t errorGyroI[3] = { 0, 0, 0 };
static int32_t errorAngleI[2] = { 0, 0 };

static void pidMultiWii(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim);

typedef void (* pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim);                // pid controller function prototype

pidControllerFuncPtr pid_controller = pidMultiWii; // which pid controller are we using, defaultMultiWii


void mwDisarm(void)
{
    if (f.ARMED)
        f.ARMED = 0;
}

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    rollAndPitchTrims->trims.roll = 0;
    rollAndPitchTrims->trims.pitch = 0;
}

void resetErrorAngle(void)
{
    errorAngleI[ROLL] = 0;
    errorAngleI[PITCH] = 0;
}

void resetErrorGyro(void)
{
    errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
    errorGyroI[YAW] = 0;
}

static void pidMultiWii(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim)
{
    int axis, prop;
    int32_t error, errorAngle;
    int32_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];
    int32_t deltaSum;
    int32_t delta;

    // **** PITCH & ROLL & YAW PID ****
    prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]
    for (axis = 0; axis < 3; axis++) {
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && (axis == FD_ROLL || axis == FD_PITCH)) { // MODE relying on ACC
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)max_angle_inclination), +max_angle_inclination) - inclination.rawAngles[axis] + angleTrim->raw[axis];
            PTermACC = errorAngle * pidProfile->P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
            PTermACC = constrain(PTermACC, -pidProfile->D8[PIDLEVEL] * 5, +pidProfile->D8[PIDLEVEL] * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * pidProfile->I8[PIDLEVEL]) >> 12;
        }
        if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == FD_YAW) { // MODE relying on GYRO or YAW axis
            error = (int32_t)rcCommand[axis] * 10 * 8 / pidProfile->P8[axis];
            error -= gyroData[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if (abs(gyroData[axis]) > 640)
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * pidProfile->I8[axis]) >> 6;
        }
        if (f.HORIZON_MODE && (axis == FD_ROLL || axis == FD_PITCH)) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } else {
            if (f.ANGLE_MODE && (axis == FD_ROLL || axis == FD_PITCH)) {
                PTerm = PTermACC;
                ITerm = ITermACC;
            } else {
                PTerm = PTermGYRO;
                ITerm = ITermGYRO;
            }
        }

        PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = gyroData[axis] - lastGyro[axis];
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

#define GYRO_I_MAX 256

static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim)
{
    int32_t errorAngle = 0;
    int axis;
    int32_t delta, deltaSum;
    static int32_t delta1[3], delta2[3];
    int32_t PTerm, ITerm, DTerm;
    static int32_t lastError[3] = { 0, 0, 0 };
    int32_t AngleRateTmp, RateError;

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && (axis == FD_PITCH || axis == FD_ROLL)) { // MODE relying on ACC
            // calculate error and limit the angle to max configured inclination
            errorAngle = constrain((rcCommand[axis] << 1) + GPS_angle[axis], -((int)max_angle_inclination), +max_angle_inclination) - inclination.rawAngles[axis] + angleTrim->raw[axis]; // 16 bits is ok here
        }
        if (axis == FD_YAW) { // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = (((int32_t)(controlRateConfig->yawRate + 27) * rcCommand[2]) >> 5);
         } else {
            if (!f.ANGLE_MODE) { //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                AngleRateTmp = ((int32_t) (controlRateConfig->rollPitchRate + 27) * rcCommand[axis]) >> 4;
                if (f.HORIZON_MODE) {
                    // mix up angle error to desired AngleRateTmp to add a little auto-level feel
                    AngleRateTmp += (errorAngle * pidProfile->I8[PIDLEVEL]) >> 8;
                }
            } else { // it's the ANGLE mode - control is angle based, so control loop is needed
                AngleRateTmp = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp - gyroData[axis];

        // -----calculate P component
        PTerm = (RateError * pidProfile->P8[axis]) >> 7;
        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * cycleTime) >> 11) * pidProfile->I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t)-GYRO_I_MAX << 13, (int32_t)+GYRO_I_MAX << 13);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t)0xFFFF / (cycleTime >> 4))) >> 6;
        // add moving average here to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * pidProfile->D8[axis]) >> 8;

        // -----calculate total PID output
        axisPID[axis] = PTerm + ITerm + DTerm;
    }
}

void setPIDController(int type)
{
    switch (type) {
        case 0:
        default:
            pid_controller = pidMultiWii;
            break;
        case 1:
            pid_controller = pidRewrite;
            break;
    }
}

