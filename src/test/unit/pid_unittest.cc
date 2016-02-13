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

#include <stdint.h>
#include <math.h>

extern "C" {
    #include "build_config.h"
    #include "debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "rx/rx.h"
    #include "io/rc_controls.h"

    #include "flight/pid.h"
    #include "flight/imu.h"

    #include "config/runtime_config.h"
    #include "config/config_unittest.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    void pidSetController(pidControllerType_e type);
    typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
            uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);            // pid controller function prototype
    extern pidControllerFuncPtr pid_controller;
    extern uint8_t PIDweight[3];
    extern bool motorLimitReached;
    extern uint32_t rcModeActivationMask;
    float dT; // dT for pidLuxFloat
    float unittest_pidLuxFloat_lastErrorForDelta[3];
    float unittest_pidLuxFloat_delta1[3];
    float unittest_pidLuxFloat_delta2[3];
    float unittest_pidLuxFloat_PTerm[3];
    float unittest_pidLuxFloat_ITerm[3];
    float unittest_pidLuxFloat_DTerm[3];
    int32_t unittest_pidMultiWiiRewrite_lastErrorForDelta[3];
    int32_t targetLooptime; // targetLooptime for pidMultiWiiRewrite
    int32_t unittest_pidMultiWiiRewrite_PTerm[3];
    int32_t unittest_pidMultiWiiRewrite_ITerm[3];
    int32_t unittest_pidMultiWiiRewrite_DTerm[3];
}

static int deltaTotalSamples;

void resetPidProfile(pidProfile_t *pidProfile)
{
    pidProfile->pidController = 1;

    pidProfile->P8[PIDROLL] = 40;
    pidProfile->I8[PIDROLL] = 30;
    pidProfile->D8[PIDROLL] = 23;
    pidProfile->P8[PIDPITCH] = 40;
    pidProfile->I8[PIDPITCH] = 30;
    pidProfile->D8[PIDPITCH] = 23;
    pidProfile->P8[PIDYAW] = 85;
    pidProfile->I8[PIDYAW] = 45;
    pidProfile->D8[PIDYAW] = 0;
    pidProfile->P8[PIDALT] = 50;
    pidProfile->I8[PIDALT] = 0;
    pidProfile->D8[PIDALT] = 0;
    pidProfile->P8[PIDPOS] = 15; // POSHOLD_P * 100;
    pidProfile->I8[PIDPOS] = 0; // POSHOLD_I * 100;
    pidProfile->D8[PIDPOS] = 0;
    pidProfile->P8[PIDPOSR] = 34; // POSHOLD_RATE_P * 10;
    pidProfile->I8[PIDPOSR] = 14; // POSHOLD_RATE_I * 100;
    pidProfile->D8[PIDPOSR] = 53; // POSHOLD_RATE_D * 1000;
    pidProfile->P8[PIDNAVR] = 25; // NAV_P * 10;
    pidProfile->I8[PIDNAVR] = 33; // NAV_I * 100;
    pidProfile->D8[PIDNAVR] = 83; // NAV_D * 1000;
    pidProfile->P8[PIDLEVEL] = 90;
    pidProfile->I8[PIDLEVEL] = 10;
    pidProfile->D8[PIDLEVEL] = 100;
    pidProfile->P8[PIDMAG] = 40;
    pidProfile->P8[PIDVEL] = 120;
    pidProfile->I8[PIDVEL] = 45;
    pidProfile->D8[PIDVEL] = 1;

    pidProfile->yaw_p_limit = YAW_P_LIMIT_MAX;
    pidProfile->dterm_cut_hz = 0;
    pidProfile->deltaMethod = DELTA_FROM_ERROR;

    pidProfile->P_f[FD_ROLL] = 1.4f;     // new PID with preliminary defaults test carefully
    pidProfile->I_f[FD_ROLL] = 0.4f;
    pidProfile->D_f[FD_ROLL] = 0.03f;
    pidProfile->P_f[FD_PITCH] = 1.4f;
    pidProfile->I_f[FD_PITCH] = 0.4f;
    pidProfile->D_f[FD_PITCH] = 0.03f;
    pidProfile->P_f[FD_YAW] = 3.5f;
    pidProfile->I_f[FD_YAW] = 0.4f;
    pidProfile->D_f[FD_YAW] = 0.01f;
    pidProfile->A_level = 5.0f;
    pidProfile->H_level = 3.0f;
    pidProfile->H_sensitivity = 75;

#ifdef GTUNE
    pidProfile->gtune_lolimP[FD_ROLL] = 10;          // [0..200] Lower limit of ROLL P during G tune.
    pidProfile->gtune_lolimP[FD_PITCH] = 10;         // [0..200] Lower limit of PITCH P during G tune.
    pidProfile->gtune_lolimP[FD_YAW] = 10;           // [0..200] Lower limit of YAW P during G tune.
    pidProfile->gtune_hilimP[FD_ROLL] = 100;         // [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[FD_PITCH] = 100;        // [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[FD_YAW] = 100;          // [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_pwr = 0;                    // [0..10] Strength of adjustment
    pidProfile->gtune_settle_time = 450;          // [200..1000] Settle time in ms
    pidProfile->gtune_average_cycles = 16;        // [8..128] Number of looptime cycles used for gyro average calculation
#endif
}

void pidControllerInitLuxFloat(pidProfile_t *pidProfile, controlRateConfig_t *controlRate, uint16_t max_angle_inclination, rollAndPitchTrims_t *rollAndPitchTrims, rxConfig_t *rxConfig)
{
    UNUSED(max_angle_inclination);
    UNUSED(rxConfig);

    pidSetController(PID_CONTROLLER_LUX_FLOAT);
    deltaTotalSamples = 3;
    resetPidProfile(pidProfile);
    resetRollAndPitchTrims(rollAndPitchTrims);
    pidResetErrorAngle();
    pidResetErrorGyro();
    dT = 0.1f;
    // set up the PIDWeights to 100%, so they are neutral in the tests
    PIDweight[FD_ROLL] = 100;
    PIDweight[FD_PITCH] = 100;
    PIDweight[FD_YAW] = 100;
    // set up the control rates for calculation of rate error
    controlRate->rates[ROLL] = 80;
    controlRate->rates[PITCH] = 80;
    controlRate->rates[YAW] = 90;
    // reset the pidLuxFloat static values
    for (int ii = FD_ROLL; ii <= FD_YAW; ++ii) {
        unittest_pidLuxFloat_lastErrorForDelta[ii] = 0.0f;
        unittest_pidLuxFloat_delta1[ii] = 0.0f;
        unittest_pidLuxFloat_delta2[ii] = 0.0f;
    }
}

/*
 * calculate the value of rcCommand[ROLL] required to give a desired rateError
 */
int16_t calcLuxRcCommandRoll(float rateError, const controlRateConfig_t *controlRate) {
    return 50.0f * rateError / (controlRate->rates[ROLL] + 20);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRateRoll(const controlRateConfig_t *controlRate) {
    return ((controlRate->rates[ROLL] + 20) * rcCommand[ROLL]) / 50.0f;
}

/*
 * calculate the value of rcCommand[PITCH] required to give a desired rateError
 */
int16_t calcLuxRcCommandPitch(float rateError, const controlRateConfig_t *controlRate) {
    return 50.0f * rateError / (controlRate->rates[PITCH] + 20);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRatePitch(const controlRateConfig_t *controlRate) {
    return ((controlRate->rates[PITCH] + 20) * rcCommand[PITCH]) / 50.0f;
}

/*
 * calculate the value of rcCommand[YAW] required to give a desired rateError
 */
int16_t calcLuxRcCommandYaw(float rateError, const controlRateConfig_t *controlRate) {
    return 50.0f * rateError / (controlRate->rates[YAW] + 10);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRateYaw(const controlRateConfig_t *controlRate) {
    return ((controlRate->rates[YAW] + 10) * rcCommand[YAW]) / 50.0f;
}

float calcLuxPTerm(pidProfile_t *pidProfile, flight_dynamics_index_t axis, float rateError) {
    return rateError * pidProfile->P_f[axis];
}

float calcLuxITermDelta(pidProfile_t *pidProfile, flight_dynamics_index_t axis, float rateError) {
    return  rateError * dT * pidProfile->I_f[axis] * 10;
}

float calcLuxDTerm(pidProfile_t *pidProfile, flight_dynamics_index_t axis, float rateError) {
    return rateError * pidProfile->D_f[axis] / dT;
}

TEST(PIDUnittest, TestPidLuxFloat)
{
    pidProfile_t pidProfile;
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of zero on all axes
    rcCommand[ROLL] = calcLuxRcCommandRoll(0, &controlRate);
    rcCommand[PITCH] = calcLuxRcCommandPitch(0, &controlRate);
    rcCommand[YAW] = calcLuxRcCommandYaw(0, &controlRate);
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidLuxFloat_PTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidLuxFloat_ITerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidLuxFloat_DTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidLuxFloat_PTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidLuxFloat_ITerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidLuxFloat_DTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidLuxFloat_PTerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidLuxFloat_ITerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidLuxFloat_DTerm[FD_YAW]);

    // set up a rateError of 100 on the roll axis
    rcCommand[ROLL] = calcLuxRcCommandRoll(100, &controlRate);
    const float rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check

    // set up a rateError of 100 on the pitch axis
    rcCommand[PITCH] = calcLuxRcCommandPitch(100, &controlRate);
    const float rateErrorPitch = calcLuxAngleRatePitch(&controlRate);
    EXPECT_EQ(100, rateErrorPitch); // cross check

    // set up a rateError of 100 on the yaw axis
    rcCommand[YAW] = calcLuxRcCommandYaw(100, &controlRate);
    const float rateErrorYaw = calcLuxAngleRateYaw(&controlRate);
    EXPECT_EQ(100, rateErrorYaw); // cross check

    // run the PID controller. Check expected PID values
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    float ITermRoll = calcLuxITermDelta(&pidProfile, FD_ROLL, rateErrorRoll);
    float ITermPitch = calcLuxITermDelta(&pidProfile, FD_PITCH, rateErrorPitch);
    float ITermYaw = calcLuxITermDelta(&pidProfile, FD_YAW, rateErrorYaw);
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloat_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloat_ITerm[FD_ROLL]);
    float expectedDTerm = calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloat_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloat_ITerm[FD_PITCH]);
    expectedDTerm = calcLuxDTerm(&pidProfile, FD_PITCH, rateErrorPitch) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_YAW, rateErrorYaw), unittest_pidLuxFloat_PTerm[FD_YAW]);
    EXPECT_FLOAT_EQ(ITermYaw, unittest_pidLuxFloat_ITerm[FD_YAW]);
    expectedDTerm = calcLuxDTerm(&pidProfile, FD_YAW, rateErrorYaw) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_YAW]);

    // run the PID controller a second time.
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over deltaTotalSamples
    ITermRoll += calcLuxITermDelta(&pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(&pidProfile, FD_PITCH, rateErrorPitch);
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloat_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloat_ITerm[FD_ROLL]);
    expectedDTerm = deltaTotalSamples < 2 ? 0 : calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloat_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloat_ITerm[FD_PITCH]);
    expectedDTerm = deltaTotalSamples < 2 ? 0 : calcLuxDTerm(&pidProfile, FD_PITCH, rateErrorPitch) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_PITCH]);

    // run the PID controller a third time. Error rates unchanged, so expect P and D unchanged, I integrated
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over deltaTotalSamples
    ITermRoll += calcLuxITermDelta(&pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(&pidProfile, FD_PITCH, rateErrorPitch);
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloat_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloat_ITerm[FD_ROLL]);
    expectedDTerm = deltaTotalSamples < 3 ? 0 : calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloat_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloat_ITerm[FD_PITCH]);
    expectedDTerm = deltaTotalSamples < 3 ? 0 : calcLuxDTerm(&pidProfile, FD_PITCH, rateErrorPitch) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_PITCH]);

    // run the PID controller a fourth time.
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over deltaTotalSamples
    ITermRoll += calcLuxITermDelta(&pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(&pidProfile, FD_PITCH, rateErrorPitch);
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloat_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloat_ITerm[FD_ROLL]);
    expectedDTerm = deltaTotalSamples < 4 ? 0 : calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(&pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloat_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloat_ITerm[FD_PITCH]);
    expectedDTerm = deltaTotalSamples < 4 ? 0 : calcLuxDTerm(&pidProfile, FD_PITCH, rateErrorPitch) / deltaTotalSamples;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloat_DTerm[FD_PITCH]);
}

TEST(PIDUnittest, TestPidLuxFloatIntegrationForLinearFunction)
{
    pidProfile_t pidProfile;
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // Test PID integration for a linear function:
    //    rateError = k * t
    // Integral:
    //    IrateError = 0.5 * k * t ^ 2
    // dT = 0.1s, t ranges from 0.0 to 1.0 in steps of 0.1s

    const float k = 100; // arbitrary value of k
    float t = 0.0f;
    // set rateError to k * t
    rcCommand[ROLL] = calcLuxRcCommandRoll(k * t, &controlRate);
    EXPECT_FLOAT_EQ(k * t, calcLuxAngleRateRoll(&controlRate)); // cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    float pidITerm = unittest_pidLuxFloat_ITerm[FD_ROLL]; // integral as estimated by PID
    float actITerm = 0.5 * k * t * t * pidProfile.I_f[ROLL] * 10; // actual value of integral
    EXPECT_FLOAT_EQ(actITerm, pidITerm); // both are zero at this point
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    for (int ii = 0; ii < 10; ++ii) {
        const float actITermPrev = actITerm;
        const float pidITermPrev = pidITerm;
        t += dT;
        // set rateError to k * t
        rcCommand[ROLL] = calcLuxRcCommandRoll(k * t, &controlRate);
        EXPECT_FLOAT_EQ(k * t, calcLuxAngleRateRoll(&controlRate)); // cross check
        pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
        pidITerm = unittest_pidLuxFloat_ITerm[FD_ROLL];
        actITerm = 0.5 * k * t * t * pidProfile.I_f[ROLL] * 10;
        const float pidITermDelta = pidITerm - pidITermPrev;
        const float actITermDelta = actITerm - actITermPrev;
        const float error = fabs(actITermDelta - pidITermDelta);
        // error is limited by rectangle of height k * dT and width dT (then multiplied by pidProfile)
        const float errorLimit = k * dT * dT * pidProfile.I_f[ROLL] * 10;
        EXPECT_GE(errorLimit, error); // ie expect errorLimit >= error
    }
}

TEST(PIDUnittest, TestPidLuxFloatIntegrationForQuadraticFunction)
{
    pidProfile_t pidProfile;
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // Test PID integration for a linear function:
    //    rateError = k * t * t
    // Integral:
    //    IrateError = (1/3) * k * t ^ 3
    // dT = 0.1s, t ranges from 0.0 to 0.6 in steps of 0.1s

    const float k = 800; // arbitrary value of k
    float t = 0.0f;
    // set rateError to k * t * t
    rcCommand[ROLL] = calcLuxRcCommandRoll(k * t * t, &controlRate);
    EXPECT_FLOAT_EQ(k * t, calcLuxAngleRateRoll(&controlRate)); // cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    float pidITerm = unittest_pidLuxFloat_ITerm[FD_ROLL]; // integral as estimated by PID
    float actITerm = (1.0f/3.0f) * k * t * t * t * pidProfile.I_f[ROLL] * 10; // actual value of integral
    EXPECT_FLOAT_EQ(actITerm, pidITerm); // both are zero at this point

    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    // limit to 6 iterations, since at 7th iteration rateError == 392 and so ITerm is constrained
    for (int ii = 0; ii < 6; ++ii) {
        const float actITermPrev = actITerm;
        const float pidITermPrev = pidITerm;
        t += dT;
        // set rateError to k * t * t
        rcCommand[ROLL] = calcLuxRcCommandRoll(k * t * t, &controlRate);
        EXPECT_FLOAT_EQ(k * t * t, calcLuxAngleRateRoll(&controlRate)); // cross check
        pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
        pidITerm = unittest_pidLuxFloat_ITerm[FD_ROLL];
        actITerm = (1.0f/3.0f) * k * t * t * t * pidProfile.I_f[ROLL] * 10;
        const float pidITermDelta = pidITerm - pidITermPrev;
        const float actITermDelta = actITerm - actITermPrev;
        const float error = fabs(actITermDelta - pidITermDelta);
        // error is limited by rectangle of height k * dT and width dT (then multiplied by pidProfile)
        const float errorLimit = k * dT * dT * pidProfile.I_f[ROLL] * 10;
        EXPECT_GE(errorLimit, error); // ie expect errorLimit >= error
    }
}

TEST(PIDUnittest, TestPidLuxFloatITermConstrain)
{
    pidProfile_t pidProfile;
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    // set rateError to zero, ITerm should be zero
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(0, &controlRate);
    float rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(0, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidLuxFloat_ITerm[FD_ROLL]);

    // set rateError to 100, ITerm should not be constrained
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(100, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check
    const float ITerm = calcLuxITermDelta(&pidProfile, FD_ROLL, rateErrorRoll);
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(ITerm, unittest_pidLuxFloat_ITerm[FD_ROLL]);

    // set up a very large rateError to force ITerm to be constrained
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    pidProfile.I8[PIDROLL] = 255;
    rcCommand[ROLL] = calcLuxRcCommandRoll(10000, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(10000, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(PID_LUX_FLOAT_MAX_I, unittest_pidLuxFloat_ITerm[FD_ROLL]);
}

TEST(PIDUnittest, TestPidLuxFloatDTermConstrain)
{
    pidProfile_t pidProfile;
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    // set rateError to zero, DTerm should be zero
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(0, &controlRate);
    float rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(0, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidLuxFloat_DTerm[FD_ROLL]);

    // set rateError to 100, DTerm should not be constrained
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(100, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples, unittest_pidLuxFloat_DTerm[FD_ROLL]);

    // set up a very large rateError to force DTerm to be constrained
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(10000, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(10000, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(PID_LUX_FLOAT_MAX_D, unittest_pidLuxFloat_DTerm[FD_ROLL]);

    // now try a smaller value of dT
    // set rateError to 50, DTerm should not be constrained
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dT = 0.01;
    rcCommand[ROLL] = calcLuxRcCommandRoll(50, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(50, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples, unittest_pidLuxFloat_DTerm[FD_ROLL]);

    // now try a test for dT = 0.001, which is typical for real world case
    // set rateError to 30, DTerm should not be constrained
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dT = 0.001;
    rcCommand[ROLL] = calcLuxRcCommandRoll(30, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(30, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples, unittest_pidLuxFloat_DTerm[FD_ROLL]);

    // set rateError to 32
    pidControllerInitLuxFloat(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dT = 0.001;
    rcCommand[ROLL] = calcLuxRcCommandRoll(32, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(32, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    // following test will fail, since DTerm will be constrained for when dT = 0.001
    //****//EXPECT_FLOAT_EQ(calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / deltaTotalSamples, unittest_pidLuxFloat_DTerm[FD_ROLL]);
}

void pidControllerInitMultiWiiRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRate, uint16_t max_angle_inclination, rollAndPitchTrims_t *rollAndPitchTrims, rxConfig_t *rxConfig)
{
    UNUSED(max_angle_inclination);
    UNUSED(rxConfig);

    pidSetController(PID_CONTROLLER_MWREWRITE);
    deltaTotalSamples = 1;
    resetPidProfile(pidProfile);
    targetLooptime = 2048; // normalised targetLooptime for pidMultiWiiRewrite
    resetRollAndPitchTrims(rollAndPitchTrims);
    pidResetErrorAngle();
    pidResetErrorGyro();
    // set up the PIDWeights to 100%, so they are neutral in the tests
    PIDweight[FD_ROLL] = 100;
    PIDweight[FD_PITCH] = 100;
    PIDweight[FD_YAW] = 100;
    // set up the control rates for calculation of rate error
    controlRate->rates[ROLL] = 73;
    controlRate->rates[PITCH] = 73;
    controlRate->rates[YAW] = 73;
    // reset the pidMultiWiiRewrite static values
    for (int ii = FD_ROLL; ii <= FD_YAW; ++ii) {
        unittest_pidMultiWiiRewrite_lastErrorForDelta[ii] = 0;
    }
}

/*
 * calculate the value of rcCommand[ROLL] required to give a desired rateError
 */
int16_t calcMwrRcCommandRoll(int rateError, const controlRateConfig_t *controlRate) {
    return (rateError << 4) / ((int32_t)(controlRate->rates[ROLL] + 27));
}

/*
 * calculate the angleRate as done in pidMultiWiiRewrite, used for cross checking
 */
int32_t calcMwrAngleRateRoll(const controlRateConfig_t *controlRate) {
    return ((int32_t)(controlRate->rates[ROLL] + 27) * rcCommand[ROLL]) >> 4;
}

/*
 * calculate the value of rcCommand[PITCH] required to give a desired rateError
 */
int16_t calcMwrRcCommandPitch(int rateError, const controlRateConfig_t *controlRate) {
    return (rateError << 4) / ((int32_t)(controlRate->rates[PITCH] + 27));
}

/*
 * calculate the angleRate as done in pidMultiWiiRewrite, used for cross checking
 */
int32_t calcMwrAngleRatePitch(const controlRateConfig_t *controlRate) {
    return ((int32_t)(controlRate->rates[PITCH] + 27) * rcCommand[PITCH]) >> 4;
}

/*
 * calculate the value of rcCommand[YAW] required to give a desired rateError
 */
int16_t calcMwrRcCommandYaw(int rateError, const controlRateConfig_t *controlRate) {
    return (rateError << 5) / ((int32_t)(controlRate->rates[YAW] + 27));
}

/*
 * calculate the angleRate as done in pidMultiWiiRewrite, used for cross checking
 */
int32_t calcMwrAngleRateYaw(const controlRateConfig_t *controlRate) {
    return ((int32_t)(controlRate->rates[YAW] + 27) * rcCommand[YAW]) >> 5;
}

int32_t calcMwrPTerm(pidProfile_t *pidProfile, pidIndex_e axis, int rateError) {
    return (pidProfile->P8[axis] * rateError) >> 7;
}

int32_t calcMwrITermDelta(pidProfile_t *pidProfile, pidIndex_e axis, int rateError) {
    return pidProfile->I8[axis] * (rateError * targetLooptime >> 11) >> 13;
}

int32_t calcMwrDTerm(pidProfile_t *pidProfile, pidIndex_e axis, int rateError) {
    return (pidProfile->D8[axis] * (rateError * ((uint16_t) 0xFFFF / (targetLooptime >> 4))) >> 6) >> 8;
}

TEST(PIDUnittest, TestPidMultiWiiRewrite)
{
    pidProfile_t pidProfile;
    resetPidProfile(&pidProfile);

    controlRateConfig_t controlRate;

    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;
    pidControllerInitMultiWiiRewrite(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of zero on all axes
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_PTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_ITerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_DTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_PTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_ITerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_DTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_PTerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_ITerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_DTerm[FD_YAW]);

    // set up a rateError of 100 on the roll axis
    rcCommand[ROLL] = calcMwrRcCommandRoll(100, &controlRate);
    const int32_t rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll); // cross check
    // set up a rateError of 100 on the pitch axis
    rcCommand[PITCH] = calcMwrRcCommandPitch(100, &controlRate);
    const int32_t rateErrorPitch = calcMwrAngleRatePitch(&controlRate);
    EXPECT_EQ(100, rateErrorPitch); // cross check
    // set up a rateError of 100 on the yaw axis
    rcCommand[YAW] = calcMwrRcCommandYaw(100, &controlRate);
    const int32_t rateErrorYaw = calcMwrAngleRateYaw(&controlRate);
    EXPECT_EQ(100, rateErrorYaw); // cross check

    // run the PID controller. Check expected PID values
    pidControllerInitMultiWiiRewrite(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(calcMwrPTerm(&pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewrite_PTerm[FD_ROLL]);
    EXPECT_EQ(calcMwrITermDelta(&pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewrite_ITerm[FD_ROLL]);
    EXPECT_EQ(calcMwrDTerm(&pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewrite_DTerm[FD_ROLL]);
    EXPECT_EQ(calcMwrPTerm(&pidProfile, PIDPITCH, rateErrorPitch), unittest_pidMultiWiiRewrite_PTerm[FD_PITCH]);
    EXPECT_EQ(calcMwrITermDelta(&pidProfile, PIDPITCH, rateErrorPitch), unittest_pidMultiWiiRewrite_ITerm[FD_PITCH]);
    EXPECT_EQ(calcMwrDTerm(&pidProfile, PIDPITCH, rateErrorPitch), unittest_pidMultiWiiRewrite_DTerm[FD_PITCH]);
    EXPECT_EQ(calcMwrPTerm(&pidProfile, PIDYAW, rateErrorYaw), unittest_pidMultiWiiRewrite_PTerm[FD_YAW]);
    EXPECT_EQ(calcMwrITermDelta(&pidProfile, PIDYAW, rateErrorYaw), unittest_pidMultiWiiRewrite_ITerm[FD_YAW]);
    EXPECT_EQ(calcMwrDTerm(&pidProfile, PIDYAW, rateErrorYaw) , unittest_pidMultiWiiRewrite_DTerm[FD_YAW]);
}

TEST(PIDUnittest, TestPidMultiWiiRewriteITermConstrain)
{
    pidProfile_t pidProfile;
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    // set rateError to zero, ITerm should be zero
    pidControllerInitMultiWiiRewrite(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcMwrRcCommandRoll(0, &controlRate);
    int16_t rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(0, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidMultiWiiRewrite_ITerm[FD_ROLL]);

    // set rateError to 100, ITerm should not be constrained
    pidControllerInitMultiWiiRewrite(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcMwrRcCommandRoll(100, &controlRate);
    rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(calcMwrITermDelta(&pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewrite_ITerm[FD_ROLL]);

    // set up a very large rateError and a large targetLooptime to force ITerm to be constrained
    pidControllerInitMultiWiiRewrite(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    targetLooptime = 8192;
    rcCommand[ROLL] = calcMwrRcCommandRoll(32750, &controlRate); // can't use INT16_MAX, since get rounding error
    rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(32750, rateErrorRoll);// cross check
    pid_controller(&pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(GYRO_I_MAX, unittest_pidMultiWiiRewrite_ITerm[FD_ROLL]);
}

// STUBS

extern "C" {
int16_t GPS_angle[ANGLE_INDEX_COUNT] = { 0, 0 };
int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {return MIN(ABS(rcData[axis] - midrc), 500);}
attitudeEulerAngles_t attitude = { { 0, 0, 0 } };
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims) {rollAndPitchTrims->values.roll = 0;rollAndPitchTrims->values.pitch = 0;};
uint16_t flightModeFlags = 0; // acro mode
uint8_t stateFlags = 0;
uint8_t motorCount = 4;
gyro_t gyro;
int16_t gyroADC[XYZ_AXIS_COUNT];
int16_t rcCommand[4] = {1500,0,0,0};           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
bool motorLimitReached;
uint32_t rcModeActivationMask;
}
