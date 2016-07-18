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
    #include "build/build_config.h"
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "config/parameter_group.h"

    #include "fc/runtime_config.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    #include "rx/rx.h"
    #include "fc/rc_controls.h"
    #include "fc/rate_profile.h"

    #include "flight/pid.h"
    #include "config/config_unittest.h"
    #include "flight/imu.h"

    pidProfile_t testPidProfile;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    void pidSetController(pidControllerType_e type);
    typedef void (*pidControllerFuncPtr)(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
            uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);            // pid controller function prototype
    int16_t pidLuxFloatCore(int axis, const pidProfile_t *pidProfile, float gyroRate, float AngleRate);
    int16_t pidMultiWiiRewriteCore(int axis, const pidProfile_t *pidProfile, int32_t gyroRate, int32_t AngleRate);
    void pidResetITerm(void);
    extern pidControllerFuncPtr pid_controller;
    extern uint8_t PIDweight[3];
    extern bool motorLimitReached;
    extern uint32_t rcModeActivationMask;
    float dT; // dT for pidLuxFloat
    int32_t targetLooptime; // targetLooptime for pidMultiWiiRewrite
    float unittest_pidLuxFloatCore_lastRateForDelta[3];
    float unittest_pidLuxFloatCore_PTerm[3];
    float unittest_pidLuxFloatCore_ITerm[3];
    float unittest_pidLuxFloatCore_DTerm[3];
    int32_t unittest_pidMultiWiiRewriteCore_lastRateForDelta[3];
    int32_t unittest_pidMultiWiiRewriteCore_PTerm[3];
    int32_t unittest_pidMultiWiiRewriteCore_ITerm[3];
    int32_t unittest_pidMultiWiiRewriteCore_DTerm[3];
}

static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 16.4f / 4; // the 16.4 is needed because mwrewrite does not scale according to the gyro model gyro.scale
static const int mwrGyroScaleNum = 4;
static const int mwrGyroScaleDenom = 1;
#define TARGET_LOOPTIME 2048

static const int DTermAverageCount = 1;

void resetPidProfile(pidProfile_t *pidProfile)
{
    pidProfile->pidController = PID_CONTROLLER_MWREWRITE;

    pidProfile->P8[PIDROLL] = 40;
    pidProfile->I8[PIDROLL] = 30;
    pidProfile->D8[PIDROLL] = 10;
    pidProfile->P8[PIDPITCH] = 40;
    pidProfile->I8[PIDPITCH] = 30;
    pidProfile->D8[PIDPITCH] = 10;
    pidProfile->P8[PIDYAW] = 85;
    pidProfile->I8[PIDYAW] = 45;
    pidProfile->D8[PIDYAW] = 10;
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
    pidProfile->dterm_lpf = 0;
    pidProfile->yaw_lpf = 0;
}

void resetRcCommands(void)
{
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;
    rcCommand[THROTTLE] = 0;
}

void resetGyroADC(void)
{
    gyroADC[ROLL] = 0;
    gyroADC[PITCH] = 0;
    gyroADC[YAW] = 0;
}

void pidControllerInitLuxFloatCore(void)
{
    pidSetController(PID_CONTROLLER_LUX_FLOAT);
    resetPidProfile(&testPidProfile);
    pidResetITermAngle();
    pidResetITerm();
    targetLooptime = TARGET_LOOPTIME;
    dT = TARGET_LOOPTIME * 0.000001f;

    gyro.scale = 1.0 / 16.4; // value for 6050 family of gyros
    resetGyroADC();
    // set up the PIDWeights to 100%, so they are neutral in the tests
    PIDweight[FD_ROLL] = 100;
    PIDweight[FD_PITCH] = 100;
    PIDweight[FD_YAW] = 100;
    // reset the pidLuxFloat static values
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        unittest_pidLuxFloatCore_lastRateForDelta[axis] = 0.0f;
    }
}

void pidControllerInitLuxFloat(controlRateConfig_t *controlRate, uint16_t max_angle_inclination, rollAndPitchTrims_t *rollAndPitchTrims, rxConfig_t *rxConfig)
{
    UNUSED(max_angle_inclination);
    UNUSED(rxConfig);

    pidControllerInitLuxFloatCore();
    resetRollAndPitchTrims(rollAndPitchTrims);
    // set up the control rates for calculation of rate error
    controlRate->rates[ROLL] = 173;
    controlRate->rates[PITCH] = 173;
    controlRate->rates[YAW] = 173;
}

/*
 * calculate the value of rcCommand[ROLL] required to give a desired rateError
 */
int16_t calcLuxRcCommandRoll(float rateError, const controlRateConfig_t *controlRate) {
    return 16.0f * rateError / ((float)controlRate->rates[ROLL] + 27.0f);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRateRoll(const controlRateConfig_t *controlRate) {
    return ((controlRate->rates[ROLL] + 27.0f) * rcCommand[ROLL]) / 16.0f;
}

/*
 * calculate the value of rcCommand[PITCH] required to give a desired rateError
 */
int16_t calcLuxRcCommandPitch(float rateError, const controlRateConfig_t *controlRate) {
    return 16.0f * rateError / (controlRate->rates[PITCH] + 27.0f);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRatePitch(const controlRateConfig_t *controlRate) {
    return ((controlRate->rates[PITCH] + 27.0f) * rcCommand[PITCH]) / 16.0f;
}

/*
 * calculate the value of rcCommand[YAW] required to give a desired rateError
 */
int16_t calcLuxRcCommandYaw(float rateError, const controlRateConfig_t *controlRate) {
    return 32.0f * rateError / (controlRate->rates[YAW] + 27.0f);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRateYaw(const controlRateConfig_t *controlRate) {
    return ((controlRate->rates[YAW] + 27.0f) * rcCommand[YAW]) / 32.0f;
}

float calcLuxPTerm(pidProfile_t *pidProfile, flight_dynamics_index_t axis, float rateError) {
    return luxPTermScale * rateError * pidProfile->P8[axis];
}

float calcLuxITermDelta(pidProfile_t *pidProfile, flight_dynamics_index_t axis, float rateError) {
    float ret = luxITermScale * rateError * dT * pidProfile->I8[axis];
    ret = constrainf(ret, -PID_MAX_I, PID_MAX_I);
    return ret;
}

float calcLuxDTerm(pidProfile_t *pidProfile, flight_dynamics_index_t axis, float rateError) {
    float ret = luxDTermScale * rateError * pidProfile->D8[axis] / dT;
    ret = constrainf(ret, -PID_MAX_D, PID_MAX_D);
    return ret;

}

TEST(PIDUnittest, TestPidLuxFloat)
{
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidProfile_t *pidProfile = &testPidProfile;
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of zero on all axes
    resetRcCommands();
    resetGyroADC();
    EXPECT_EQ(0, unittest_pidLuxFloatCore_PTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_PTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_ITerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_DTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_PTerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_ITerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_DTerm[FD_YAW]);

    // set up a rateError of 100 on the roll axis
    const float rateErrorRoll = 100;

    // set up a rateError of 100 on the pitch axis
    const float rateErrorPitch = 100;

    // set up a rateError of 100 on the yaw axis
    const float rateErrorYaw = 100;

    // run the PID controller. Check expected PID values
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    gyroADC[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    gyroADC[PITCH] = -rateErrorPitch / (luxGyroScale * gyro.scale);
    gyroADC[YAW] = -rateErrorYaw / (luxGyroScale * gyro.scale);
    resetRcCommands();
    float ITermRoll = calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    float ITermPitch = calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
    float ITermYaw = calcLuxITermDelta(pidProfile, FD_YAW, rateErrorYaw);
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloatCore_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
    float expectedDTerm = calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloatCore_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloatCore_ITerm[FD_PITCH]);
    expectedDTerm = calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_YAW, rateErrorYaw), unittest_pidLuxFloatCore_PTerm[FD_YAW]);
    EXPECT_FLOAT_EQ(ITermYaw, unittest_pidLuxFloatCore_ITerm[FD_YAW]);
    expectedDTerm = calcLuxDTerm(pidProfile, FD_YAW, rateErrorYaw) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_YAW]);

    // run the PID controller a second time.
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over DTermAverageCount
    ITermRoll += calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloatCore_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
    expectedDTerm = DTermAverageCount < 2 ? 0 : calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloatCore_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloatCore_ITerm[FD_PITCH]);
    expectedDTerm = DTermAverageCount < 2 ? 0 : calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_PITCH]);

    // run the PID controller a third time. Error rates unchanged, so expect P and D unchanged, I integrated
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over DTermAverageCount
    ITermRoll += calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloatCore_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
    expectedDTerm = DTermAverageCount < 3 ? 0 : calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloatCore_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloatCore_ITerm[FD_PITCH]);
    expectedDTerm = DTermAverageCount < 3 ? 0 : calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_PITCH]);

    // run the PID controller a fourth time.
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over DTermAverageCount
    ITermRoll += calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), unittest_pidLuxFloatCore_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
    expectedDTerm = DTermAverageCount < 4 ? 0 : calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), unittest_pidLuxFloatCore_PTerm[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, unittest_pidLuxFloatCore_ITerm[FD_PITCH]);
    expectedDTerm = DTermAverageCount < 4 ? 0 : calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, unittest_pidLuxFloatCore_DTerm[FD_PITCH]);
}

TEST(PIDUnittest, TestPidLuxFloatIntegrationForLinearFunction)
{
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidProfile_t *pidProfile = &testPidProfile;

    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dT = 0.1f; // set large dT so constraints on PID values are not invoked
    EXPECT_FLOAT_EQ(0.1f, dT);

    // Test PID integration for a linear function:
    //    rateError = k * t
    // Integral:
    //    IrateError = 0.5 * k * t ^ 2
    // dT = 0.1s, t ranges from 0.0 to 1.0 in steps of 0.1s

    const float k = 1000; // arbitrary value of k
    float t = 0.0f;
    // set rateError to k * t
    gyroADC[ROLL] = -k * t  / (luxGyroScale * gyro.scale);
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    float pidITerm = unittest_pidLuxFloatCore_ITerm[FD_ROLL]; // integral as estimated by PID
    float actITerm = 0.5 * k * t * t * pidProfile->I8[ROLL] * luxITermScale; // actual value of integral
    EXPECT_FLOAT_EQ(actITerm, pidITerm); // both are zero at this point
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    resetRcCommands();
    dT = 0.01f; // set large dT so constraints on PID values are not invoked

    for (int ii = 0; ii < 10; ++ii) {
        const float actITermPrev = actITerm;
        const float pidITermPrev = pidITerm;
        t += dT;
        // set rateError to k * t
        gyroADC[ROLL] = -k * t / (luxGyroScale * gyro.scale);
        pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
        pidITerm = unittest_pidLuxFloatCore_ITerm[FD_ROLL];
        actITerm = 0.5 * k * t * t * pidProfile->I8[ROLL] * luxITermScale;
        const float pidITermDelta = pidITerm - pidITermPrev;
        const float actITermDelta = actITerm - actITermPrev;
        const float error = fabs(actITermDelta - pidITermDelta);
        // error is limited by rectangle of height k * dT and width dT (then multiplied by pidProfile)
        const float errorLimit = k * dT * dT * pidProfile->I8[ROLL] * luxITermScale;
        EXPECT_GE(errorLimit, error); // ie expect errorLimit >= error
    }
}

TEST(PIDUnittest, TestPidLuxFloatIntegrationForQuadraticFunction)
{
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidProfile_t *pidProfile = &testPidProfile;

    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    resetRcCommands();

    // Test PID integration for a quadratic function:
    //    rateError = k * t * t
    // Integral:
    //    IrateError = (1/3) * k * t ^ 3
    // dT = 0.1s, t ranges from 0.0 to 0.6 in steps of 0.1s

    const float k = 800; // arbitrary value of k
    float t = 0.0f;
    // set rateError to k * t * t
    gyroADC[ROLL] = -k * t * t / (luxGyroScale * gyro.scale);
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    float pidITerm = unittest_pidLuxFloatCore_ITerm[FD_ROLL]; // integral as estimated by PID
    float actITerm = (1.0f/3.0f) * k * t * t * t * pidProfile->I8[ROLL] * luxITermScale; // actual value of integral
    EXPECT_FLOAT_EQ(actITerm, pidITerm); // both are zero at this point

    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    resetRcCommands();
    // limit to 6 iterations, since at 7th iteration rateError == 392 and so ITerm is constrained
    for (int ii = 0; ii < 6; ++ii) {
        const float actITermPrev = actITerm;
        const float pidITermPrev = pidITerm;
        t += dT;
        // set rateError to k * t * t
        gyroADC[ROLL] = -k * t * t / (luxGyroScale * gyro.scale);
        pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
        pidITerm = unittest_pidLuxFloatCore_ITerm[FD_ROLL];
        actITerm = (1.0f/3.0f) * k * t * t * t * pidProfile->I8[ROLL] * luxITermScale;
        const float pidITermDelta = pidITerm - pidITermPrev;
        const float actITermDelta = actITerm - actITermPrev;
        const float error = fabs(actITermDelta - pidITermDelta);
        // error is limited by rectangle of height k * dT and width dT (then multiplied by pidProfile)
        const float errorLimit = k * dT * dT * pidProfile->I8[ROLL] * luxITermScale;
        EXPECT_GE(errorLimit, error); // ie expect errorLimit >= error
    }
}

TEST(PIDUnittest, TestPidLuxFloatITermConstrain)
{
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidProfile_t *pidProfile = &testPidProfile;

    // set rateError to zero, ITerm should be zero
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(0, &controlRate);
    float rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(0, rateErrorRoll);// cross check
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);

    // set rateError to 100, ITerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(100, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check
    const float ITerm = calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(ITerm, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);

    // set up a very large rateError to force ITerm to be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    pidProfile->I8[PIDROLL] = 255;
    rcCommand[ROLL] = calcLuxRcCommandRoll(10000, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(10000, rateErrorRoll);// cross check
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
//    EXPECT_FLOAT_EQ(PID_LUX_FLOAT_MAX_I, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
}

TEST(PIDUnittest, TestPidLuxFloatDTermConstrain)
{
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidProfile_t *pidProfile = &testPidProfile;

    // set rateError to zero, DTerm should be zero
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    float rateErrorRoll = 0;
    gyroADC[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    resetRcCommands();
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);

    // set rateError to 100, DTerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rateErrorRoll = 100;
    gyroADC[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    resetRcCommands();
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);

    // set up a very large rateError to force DTerm to be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rateErrorRoll = 10000;
    gyroADC[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    resetRcCommands();
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    //!!EXPECT_FLOAT_EQ(PID_LUX_FLOAT_MAX_D, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);

    // now try a smaller value of dT
    // set rateError to 50, DTerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dT = 0.01;
    rateErrorRoll = 50;
    gyroADC[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    resetRcCommands();
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);

    // now try a test for dT = 0.001, which is typical for real world case
    // set rateError to 30, DTerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dT = 0.001;
    rateErrorRoll = 30;
    gyroADC[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    resetRcCommands();
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);

    // set rateError to 32
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dT = 0.001;
    rateErrorRoll = 32;
    gyroADC[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    resetRcCommands();
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    // following test will fail, since DTerm will be constrained for when dT = 0.001
    //!!!!//EXPECT_FLOAT_EQ(calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);
}

void pidControllerInitMultiWiiRewriteCore(void)
{
    pidSetController(PID_CONTROLLER_MWREWRITE);
    resetPidProfile(&testPidProfile);
    targetLooptime = TARGET_LOOPTIME; // normalised targetLooptime for pidMultiWiiRewrite
    dT = TARGET_LOOPTIME * 0.000001f;
    pidResetITermAngle();
    pidResetITerm();
    resetGyroADC();
    // set up the PIDWeights to 100%, so they are neutral in the tests
    PIDweight[FD_ROLL] = 100;
    PIDweight[FD_PITCH] = 100;
    PIDweight[FD_YAW] = 100;
    // reset the pidMultiWiiRewrite static values
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        unittest_pidMultiWiiRewriteCore_lastRateForDelta[axis] = 0;
    }
}

void pidControllerInitMultiWiiRewrite(controlRateConfig_t *controlRate, uint16_t max_angle_inclination, rollAndPitchTrims_t *rollAndPitchTrims, rxConfig_t *rxConfig)
{
    UNUSED(max_angle_inclination);
    UNUSED(rxConfig);
    pidControllerInitMultiWiiRewriteCore();
    // set up the control rates for calculation of rate error
    controlRate->rates[ROLL] = 173;
    controlRate->rates[PITCH] = 173;
    controlRate->rates[YAW] = 173;
    resetRollAndPitchTrims(rollAndPitchTrims);
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
    int32_t ret = pidProfile->I8[axis] * (rateError * targetLooptime >> 11) >> 13;
    ret = constrain(ret, -PID_MAX_I, PID_MAX_I);
    return ret;
}

int32_t calcMwrDTerm(pidProfile_t *pidProfile, pidIndex_e axis, int rateError) {
    int32_t ret = (rateError * ((uint16_t)0xFFFF / ((uint16_t)targetLooptime >> 4))) >> 5;
    ret =  (ret * pidProfile->D8[axis]) >> 8;
    ret /= DTermAverageCount;
    ret = constrain(ret, -PID_MAX_D, PID_MAX_D);
    return ret;
}

TEST(PIDUnittest, TestPidMultiWiiRewrite)
{
    pidProfile_t *pidProfile = &testPidProfile;
    resetPidProfile(pidProfile);

    controlRateConfig_t controlRate;

    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of zero on all axes
    resetRcCommands();
    resetGyroADC();
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_PTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_ITerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_DTerm[FD_PITCH]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_PTerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_ITerm[FD_YAW]);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_DTerm[FD_YAW]);

    // set up a rateError of 100 on the roll axis
    const int32_t rateErrorRoll = 100;
    // set up a rateError of 100 on the pitch axis
    const int32_t rateErrorPitch = 100;
    // set up a rateError of 100 on the yaw axis
    const int32_t rateErrorYaw = 100;

    // run the PID controller. Check expected PID values
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    gyroADC[ROLL] = -rateErrorRoll * mwrGyroScaleNum / mwrGyroScaleDenom;
    gyroADC[PITCH] = -rateErrorPitch * mwrGyroScaleNum / mwrGyroScaleDenom;
    gyroADC[YAW] = -rateErrorYaw * mwrGyroScaleNum / mwrGyroScaleDenom;
    resetRcCommands();

    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL]);
    EXPECT_EQ(calcMwrDTerm(pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL]);
    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDPITCH, rateErrorPitch), unittest_pidMultiWiiRewriteCore_PTerm[FD_PITCH]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDPITCH, rateErrorPitch), unittest_pidMultiWiiRewriteCore_ITerm[FD_PITCH]);
    EXPECT_EQ(calcMwrDTerm(pidProfile, PIDPITCH, rateErrorPitch), unittest_pidMultiWiiRewriteCore_DTerm[FD_PITCH]);
    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDYAW, rateErrorYaw), unittest_pidMultiWiiRewriteCore_PTerm[FD_YAW]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDYAW, rateErrorYaw), unittest_pidMultiWiiRewriteCore_ITerm[FD_YAW]);
    EXPECT_EQ(calcMwrDTerm(pidProfile, PIDYAW, rateErrorYaw) , unittest_pidMultiWiiRewriteCore_DTerm[FD_YAW]);
}

TEST(PIDUnittest, TestPidMultiWiiRewriteITermConstrain)
{
    controlRateConfig_t controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    pidProfile_t *pidProfile = &testPidProfile;

    // set rateError to zero, ITerm should be zero
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcMwrRcCommandRoll(0, &controlRate);
    int16_t rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(0, rateErrorRoll);// cross check
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(0, unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL]);

    // set rateError to 100, ITerm should not be constrained
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcMwrRcCommandRoll(100, &controlRate);
    rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL]);

    // set up a very large rateError and a large targetLooptime to force ITerm to be constrained
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    targetLooptime = 8192;
    rcCommand[ROLL] = calcMwrRcCommandRoll(32750, &controlRate); // can't use INT16_MAX, since get rounding error
    rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(32750, rateErrorRoll);// cross check
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(GYRO_I_MAX, unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL]);
}

TEST(PIDUnittest, TestPidMultiWiiRewritePidLuxFloatCoreEquivalence)
{
    pidProfile_t *pidProfile = &testPidProfile;
    const int angleRate = 200;

    pidControllerInitLuxFloatCore();
    EXPECT_EQ(TARGET_LOOPTIME, targetLooptime);
    EXPECT_FLOAT_EQ(TARGET_LOOPTIME * 0.000001f, dT);

    pidLuxFloatCore(FD_ROLL, pidProfile, -angleRate, 0);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, angleRate), unittest_pidLuxFloatCore_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxITermDelta(pidProfile, FD_ROLL, angleRate), unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxDTerm(pidProfile, FD_ROLL, angleRate) / DTermAverageCount, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);

    pidControllerInitMultiWiiRewriteCore();
    EXPECT_EQ(TARGET_LOOPTIME, targetLooptime);
    EXPECT_FLOAT_EQ(TARGET_LOOPTIME * 0.000001f, dT);

    pidMultiWiiRewriteCore(FD_ROLL, pidProfile, -angleRate, 0);
    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDROLL, angleRate), unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDROLL, angleRate), unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL]);
    EXPECT_EQ(calcMwrDTerm(pidProfile, PIDROLL, angleRate), unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL]);

    const float allowedPError = (float)unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL] / 100; // 1% error allowed
    EXPECT_NEAR(unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL], unittest_pidLuxFloatCore_PTerm[FD_ROLL], allowedPError);

    const float allowedIError = 1.0f;
    EXPECT_NEAR(unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL], unittest_pidLuxFloatCore_ITerm[FD_ROLL], allowedIError);

    const float allowedDError = (float)unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL] / 100; // 1% error allowed
    EXPECT_NEAR(unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL], unittest_pidLuxFloatCore_DTerm[FD_ROLL], allowedDError);
}

TEST(PIDUnittest, TestPidMultiWiiRewritePidLuxFloatEquivalence)
{
    pidProfile_t *pidProfile = &testPidProfile;

    controlRateConfig_t controlRate;

    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;


    resetPidProfile(pidProfile);
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(1.0f / 16.4f, gyro.scale);

    // set up a rateError of 200 on the roll axis
    const float rateErrorRollf = 200;
    gyroADC[ROLL] = -rateErrorRollf / (luxGyroScale * gyro.scale);
    resetRcCommands();

    float ITermRoll = calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRollf);

    // run the PID controller. Check expected PID values
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRollf), unittest_pidLuxFloatCore_PTerm[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, unittest_pidLuxFloatCore_ITerm[FD_ROLL]);
    float expectedDTermf = calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRollf) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTermf, unittest_pidLuxFloatCore_DTerm[FD_ROLL]);



    resetPidProfile(pidProfile);
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of 200 on the roll axis
    const int32_t rateErrorRoll = 200;
    gyroADC[ROLL] = -rateErrorRoll * mwrGyroScaleNum / mwrGyroScaleDenom;
    resetRcCommands();

    // run the PID controller. Check expected PID values
    pid_controller(pidProfile, &controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDROLL, rateErrorRoll), unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL]);
    int32_t expectedDTerm = calcMwrDTerm(pidProfile, PIDROLL, rateErrorRoll);
    EXPECT_EQ(expectedDTerm, unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL]);

    const float allowedPError = (float)unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL] / 100; // 1% error allowed
    EXPECT_NEAR(unittest_pidMultiWiiRewriteCore_PTerm[FD_ROLL], unittest_pidLuxFloatCore_PTerm[FD_ROLL], allowedPError);

    const float allowedIError = 1;
    EXPECT_NEAR(unittest_pidMultiWiiRewriteCore_ITerm[FD_ROLL], unittest_pidLuxFloatCore_ITerm[FD_ROLL], allowedIError);

    const float allowedDError = (float)unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL] / 100; // 1% error allowed
    EXPECT_NEAR(unittest_pidMultiWiiRewriteCore_DTerm[FD_ROLL], unittest_pidLuxFloatCore_DTerm[FD_ROLL], allowedDError);
}
// STUBS

extern "C" {
bool rcModeIsActive(boxId_e modeId)  { return rcModeActivationMask & (1 << modeId); }
int16_t GPS_angle[ANGLE_INDEX_COUNT] = { 0, 0 };
int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {return MIN(ABS(rcData[axis] - midrc), 500);}
attitudeEulerAngles_t attitude = { { 0, 0, 0 } };
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims) {rollAndPitchTrims->values.roll = 0;rollAndPitchTrims->values.pitch = 0;};
uint16_t flightModeFlags = 0; // acro mode
uint8_t stateFlags = 0;
uint8_t motorCount = 4;
gyro_t gyro;
int32_t gyroADC[XYZ_AXIS_COUNT];
int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
bool motorLimitReached;
uint32_t rcModeActivationMask;
}
