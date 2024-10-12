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
#include <stdbool.h>
#include <limits.h>
#include <cmath>

#include "unittest_macros.h"
#include "gtest/gtest.h"
#include "build/debug.h"

bool simulatedAirmodeEnabled = true;
float simulatedSetpointRate[3] = { 0,0,0 };
float simulatedPrevSetpointRate[3] = { 0,0,0 };
float simulatedRcDeflection[3] = { 0,0,0 };
float simulatedMaxRcDeflectionAbs = 0;
float simulatedMixerGetRcThrottle = 0;
float simulatedRcCommandDelta[3] = { 1,1,1 };
float simulatedRawSetpoint[3] = { 0,0,0 };
float simulatedMaxRate[3] = { 670,670,670 };
float simulatedFeedforward[3] = { 0,0,0 };
float simulatedMotorMixRange = 0.0f;

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "config/config.h"
    #include "config/config_reset.h"

    #include "drivers/sound_beeper.h"
    #include "drivers/time.h"

    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/pid_init.h"
    #include "flight/position.h"
    
    #include "io/gps.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "pg/rx.h"
    #include "rx/rx.h"

    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"

    acc_t acc;
    gyro_t gyro;
    attitudeEulerAngles_t attitude;

    rxRuntimeState_t rxRuntimeState = {};

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
    PG_REGISTER(positionConfig_t, positionConfig, PG_SYSTEM_CONFIG, 4);

    bool unitLaunchControlActive = false;
    launchControlMode_e unitLaunchControlMode = LAUNCH_CONTROL_MODE_NORMAL;

    float getMotorMixRange(void) { return simulatedMotorMixRange; }
    float getSetpointRate(int axis) { return simulatedSetpointRate[axis]; }
    bool isAirmodeActivated(void) { return simulatedAirmodeEnabled; }
    bool wasThrottleRaised(void) { return simulatedAirmodeEnabled; }
    float getRcDeflectionAbs(int axis) { return fabsf(simulatedRcDeflection[axis]); }

    // used by auto-disarm code
    float getMaxRcDeflectionAbs() { return fabsf(simulatedMaxRcDeflectionAbs); }
    float mixerGetRcThrottle() { return fabsf(simulatedMixerGetRcThrottle); }


    bool isBelowLandingAltitude(void) { return false; }

    void systemBeep(bool) { }
    bool gyroOverflowDetected(void) { return false; }
    float getRcDeflection(int axis) { return simulatedRcDeflection[axis]; }
    float getRcDeflectionRaw(int axis) { return simulatedRcDeflection[axis]; }
    float getRawSetpoint(int axis) { return simulatedRawSetpoint[axis]; }
    float getFeedforward(int axis) {
        return simulatedSetpointRate[axis] - simulatedPrevSetpointRate[axis];
    }
    void beeperConfirmationBeeps(uint8_t) { }
    bool isLaunchControlActive(void) {return unitLaunchControlActive; }
    void disarm(flightLogDisarmReason_e) { }
    float getMaxRcRate(int axis)
    {
        UNUSED(axis);
        float maxRate = simulatedMaxRate[axis];
        return maxRate;
    }
    void initRcProcessing(void) { }
}

pidProfile_t *pidProfile;

int loopIter = 0;

// Always use same defaults for testing in future releases even when defaults change
void setDefaultTestSettings(void)
{
    pgResetAll();
    pidProfile = pidProfilesMutable(1);
    pidProfile->pid[PID_ROLL]  =  { 40, 40, 30, 65, 0 };
    pidProfile->pid[PID_PITCH] =  { 58, 50, 35, 60, 0 };
    pidProfile->pid[PID_YAW]   =  { 70, 45, 20, 60, 0 };
    pidProfile->pid[PID_LEVEL] =  { 50, 50, 75, 50, 0 };

    // Compensate for the upscaling done without 'use_integrated_yaw'
    pidProfile->pid[PID_YAW].I = pidProfile->pid[PID_YAW].I / 2.5f;

    pidProfile->pidSumLimit = PIDSUM_LIMIT;        // 500
    pidProfile->pidSumLimitYaw = PIDSUM_LIMIT_YAW; // 400
    pidProfile->yaw_lowpass_hz = 0;
    pidProfile->dterm_lpf1_static_hz = 100;
    pidProfile->dterm_lpf2_static_hz = 0;
    pidProfile->dterm_notch_hz = 260;
    pidProfile->dterm_notch_cutoff = 160;
    pidProfile->dterm_lpf1_type = FILTER_BIQUAD;
    pidProfile->itermWindup = 80;
    pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;
    pidProfile->angle_limit = 60;
    pidProfile->feedforward_transition = 100;
    pidProfile->yawRateAccelLimit = 100;
    pidProfile->rateAccelLimit = 0;
    pidProfile->anti_gravity_gain = 10;
    pidProfile->crash_time = 500;
    pidProfile->crash_delay = 0;
    pidProfile->crash_recovery_angle = 10;
    pidProfile->crash_recovery_rate = 100;
    pidProfile->crash_dthreshold = 50;
    pidProfile->crash_gthreshold = 400;
    pidProfile->crash_setpoint_threshold = 350;
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_OFF;
    pidProfile->horizon_limit_degrees = 135;
    pidProfile->horizon_ignore_sticks = false;
    pidProfile->crash_limit_yaw = 200;
    pidProfile->itermLimit = 150;
    pidProfile->throttle_boost = 0;
    pidProfile->throttle_boost_cutoff = 15;
    pidProfile->iterm_rotation = false;
    pidProfile->iterm_relax = ITERM_RELAX_OFF,
    pidProfile->iterm_relax_cutoff = 11,
    pidProfile->iterm_relax_type = ITERM_RELAX_SETPOINT,
    pidProfile->abs_control_gain = 0,
    pidProfile->launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
    pidProfile->launchControlGain = 40,
    pidProfile->level_race_mode = false,

    gyro.targetLooptime = 8000;
}

timeUs_t currentTestTime(void)
{
    return targetPidLooptime * loopIter++;
}

void resetTest(void)
{
    loopIter = 0;
    pidRuntime.tpaFactor = 1.0f;
    simulatedMotorMixRange = 0.0f;

    pidStabilisationState(PID_STABILISATION_OFF);
    DISABLE_ARMING_FLAG(ARMED);

    setDefaultTestSettings();
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidData[axis].P = 0;
        pidData[axis].I = 0;
        pidData[axis].D = 0;
        pidData[axis].F = 0;
        pidData[axis].S = 0;
        pidData[axis].Sum = 0;
        simulatedSetpointRate[axis] = 0;
        simulatedRcDeflection[axis] = 0;
        simulatedRawSetpoint[axis] = 0;
        gyro.gyroADCf[axis] = 0;
    }
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;

    flightModeFlags = 0;
    unitLaunchControlActive = false;
    pidProfile->launchControlMode = unitLaunchControlMode;
    pidInit(pidProfile);
    loadControlRateProfile();

    // Run pidloop for a while after reset
    for (int loop = 0; loop < 20; loop++) {
        pidController(pidProfile, currentTestTime());
    }
}

void setStickPosition(int axis, float stickRatio)
{
    simulatedPrevSetpointRate[axis] = simulatedSetpointRate[axis];
    simulatedSetpointRate[axis] = 1998.0f * stickRatio;
    simulatedRcDeflection[axis] = stickRatio;
}

// All calculations will have 10% tolerance
float calculateTolerance(float input)
{
    return fabsf(input * 0.1f);
}

TEST(pidControllerTest, testInitialisation)
{
    resetTest();

    // In initial state PIDsums should be 0
    for (int axis = 0; axis <= FD_YAW; axis++) {
        EXPECT_FLOAT_EQ(0, pidData[axis].P);
        EXPECT_FLOAT_EQ(0, pidData[axis].I);
        EXPECT_FLOAT_EQ(0, pidData[axis].D);
    }
}

TEST(pidControllerTest, testStabilisationDisabled)
{
    ENABLE_ARMING_FLAG(ARMED);
    // Run few loops to make sure there is no error building up when stabilisation disabled

    for (int loop = 0; loop < 10; loop++) {
        pidController(pidProfile, currentTestTime());

        // PID controller should not do anything, while stabilisation disabled
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);
    }
}

TEST(pidControllerTest, testPidLoop)
{
    // Make sure to start with fresh values
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    // Loop 1 - Expecting zero since there is no error
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Add some rotation on ROLL to generate error
    gyro.gyroADCf[FD_ROLL] = 100;
    pidController(pidProfile, currentTestTime());

    // Loop 2 - Expect PID loop reaction to ROLL error
    EXPECT_NEAR(-128.1, pidData[FD_ROLL].P, calculateTolerance(-128.1));
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_NEAR(-7.8, pidData[FD_ROLL].I, calculateTolerance(-7.8));
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_NEAR(-198.4, pidData[FD_ROLL].D, calculateTolerance(-198.4));
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Add some rotation on PITCH to generate error
    gyro.gyroADCf[FD_PITCH] = -100;
    pidController(pidProfile, currentTestTime());

    // Loop 3 - Expect PID loop reaction to PITCH error, ROLL is still in error
    EXPECT_NEAR(-128.1, pidData[FD_ROLL].P, calculateTolerance(-128.1));
    EXPECT_NEAR(185.8, pidData[FD_PITCH].P, calculateTolerance(185.8));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_NEAR(-15.6, pidData[FD_ROLL].I, calculateTolerance(-15.6));
    EXPECT_NEAR(9.8, pidData[FD_PITCH].I, calculateTolerance(9.8));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_NEAR(231.4, pidData[FD_PITCH].D, calculateTolerance(231.4));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Add some rotation on YAW to generate error, but not enough to trigger pidSumLimitYaw
    gyro.gyroADCf[FD_YAW] = 10;
    pidController(pidProfile, currentTestTime());

    // Loop 4 - Expect PID loop reaction to PITCH error, ROLL and PITCH are still in error
    EXPECT_NEAR(-128.1, pidData[FD_ROLL].P, calculateTolerance(-128.1));
    EXPECT_NEAR(185.8, pidData[FD_PITCH].P, calculateTolerance(185.8));
    EXPECT_NEAR(-22.4, pidData[FD_YAW].P, calculateTolerance(-22.4));
    EXPECT_NEAR(-23.5, pidData[FD_ROLL].I, calculateTolerance(-23.5));
    EXPECT_NEAR(19.6, pidData[FD_PITCH].I, calculateTolerance(19.6));
    EXPECT_NEAR(-0.87, pidData[FD_YAW].I, calculateTolerance(-0.87));
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);

    // Simulate Iterm growth if not saturated
    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(-31.3, pidData[FD_ROLL].I, calculateTolerance(-31.3));
    EXPECT_NEAR(29.3, pidData[FD_PITCH].I, calculateTolerance(29.3));
    EXPECT_NEAR(-1.76, pidData[FD_YAW].I, calculateTolerance(-1.76));
        EXPECT_NEAR(-24.2, pidData[FD_YAW].Sum, calculateTolerance(-24.2)); 

    // Match the stick to gyro to stop error
    simulatedSetpointRate[FD_ROLL] = 100;
    simulatedSetpointRate[FD_PITCH] = -100;
    simulatedSetpointRate[FD_YAW] = 10; // error 

    pidController(pidProfile, currentTestTime());
    // Iterm is stalled as it is not accumulating anymore
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_NEAR(-31.3, pidData[FD_ROLL].I, calculateTolerance(-31.3));
    EXPECT_NEAR(29.3, pidData[FD_PITCH].I, calculateTolerance(29.3));
    EXPECT_NEAR(-1.76, pidData[FD_YAW].I, calculateTolerance(-1.76)); 
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);


    for(int loop = 0; loop < 5; loop++) {
        pidController(pidProfile, currentTestTime());
    }
    // Iterm is stalled as it is not accumulating anymore
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_NEAR(-31.3, pidData[FD_ROLL].I, calculateTolerance(-31.3));
    EXPECT_NEAR(29.3, pidData[FD_PITCH].I, calculateTolerance(29.3));
    EXPECT_NEAR(-1.76, pidData[FD_YAW].I, calculateTolerance(-1.76)); 
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Now disable Stabilisation
    pidStabilisationState(PID_STABILISATION_OFF);
    pidController(pidProfile, currentTestTime());

    // Should all be zero again
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);
}

TEST(pidControllerTest, testPidLevel)
{
    // Make sure to start with fresh values
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // Test Angle mode response
    enableFlightMode(ANGLE_MODE);
    float currentPidSetpointRoll = 0;
    float currentPidSetpointPitch = 0;
    float calculatedAngleSetpoint = 0;
    rollAndPitchTrims_t angleTrim = { { 0, 0 } };

    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(0, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(0, calculatedAngleSetpoint);

    currentPidSetpointRoll = 200;
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(51.456356, calculatedAngleSetpoint);
    currentPidSetpointPitch = -200;
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-51.456356, calculatedAngleSetpoint);

    currentPidSetpointRoll = 400;
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(128.94597, calculatedAngleSetpoint);
    currentPidSetpointPitch = -400;
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-128.94597, calculatedAngleSetpoint);

    // Test attitude response
    attitude.values.roll = -275;
    attitude.values.pitch = 275;
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(242.76686, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-242.76686, calculatedAngleSetpoint);

    // Disable ANGLE_MODE
    disableFlightMode(ANGLE_MODE);
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(393.44571, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-392.88422, calculatedAngleSetpoint);

    // Test level mode expo
    enableFlightMode(ANGLE_MODE);
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    currentPidSetpointRoll = 400;
    currentPidSetpointPitch = -400;
    // need to set some rates type and some expo here ??? HELP !!
    calculatedAngleSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpointRoll, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(231.55479, calculatedAngleSetpoint);
    calculatedAngleSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpointPitch, calcHorizonLevelStrength());
    EXPECT_FLOAT_EQ(-231.55479, calculatedAngleSetpoint);
}


TEST(pidControllerTest, testPidHorizon)
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    enableFlightMode(HORIZON_MODE);

    // Test stick response greater than default limit of 0.75
    setStickPosition(FD_ROLL, 0.76f);
    setStickPosition(FD_PITCH, -0.76f);
    EXPECT_FLOAT_EQ(0.0f, calcHorizonLevelStrength());

    // Return sticks to center, should expect some levelling, but will be delayed
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    EXPECT_FLOAT_EQ(0.0078740157, calcHorizonLevelStrength());

    // Test small stick response when flat, considering delay
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    EXPECT_NEAR(0.01457f, calcHorizonLevelStrength(), calculateTolerance(0.01457));

    // Test larger stick response when flat
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.0166, calcHorizonLevelStrength(), calculateTolerance(0.0166));

    // set attitude of craft to 90 degrees
    attitude.values.roll = 900;
    attitude.values.pitch = 900;

    // Test centered sticks at 90 degrees
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    // with gain of 50, and max angle of 135 deg, strength = 0.5 * (135-90) / 90 ie 0.5 * 45/136 or 0.5 * 0.333 = 0.166
    EXPECT_NEAR(0.0193f, calcHorizonLevelStrength(), calculateTolerance(0.0193));

    // Test small stick response at 90 degrees
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    EXPECT_NEAR(0.0213f, calcHorizonLevelStrength(), calculateTolerance(0.0213));

    // Test larger stick response at 90 degrees
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.0218f, calcHorizonLevelStrength(), calculateTolerance(0.0218));

    // set attitude of craft to 120 degrees, inside limit of 135
    attitude.values.roll = 1200;
    attitude.values.pitch = 1200;

    // Test centered sticks at 120 degrees
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    EXPECT_NEAR(0.0224f, calcHorizonLevelStrength(), calculateTolerance(0.0224));

    // Test small stick response at 120 degrees
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    EXPECT_NEAR(0.0228f, calcHorizonLevelStrength(), calculateTolerance(0.0228));

    // Test larger stick response at 120 degrees
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.018f, calcHorizonLevelStrength(), calculateTolerance(0.018));

    // set attitude of craft to 1500 degrees, outside limit of 135
    attitude.values.roll = 1500;
    attitude.values.pitch = 1500;

    // Test centered sticks at 150 degrees - should be zero at any stick angle
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    EXPECT_NEAR(0.0f, calcHorizonLevelStrength(), calculateTolerance(0.0));

    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    EXPECT_NEAR(0.0f, calcHorizonLevelStrength(), calculateTolerance(0.0));

}

// trying to fix



TEST(pidControllerTest, testMixerSaturation)
{
    resetTest();

    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidRuntime.itermLimit = 400;
    pidRuntime.itermLimitYaw = 320;

    // Test full stick response
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    setStickPosition(FD_YAW, 1.0f);
    pidController(pidProfile, currentTestTime());

    // Expect iterm accumulation for all axes because at this point, pidSum is not at limit
    EXPECT_NEAR(156.2f, pidData[FD_ROLL].I, calculateTolerance(156.2f));
    EXPECT_NEAR(-195.3f, pidData[FD_PITCH].I, calculateTolerance(-195.3f));
    EXPECT_NEAR(7.0f, pidData[FD_YAW].I, calculateTolerance(7.0f));

     // ????? why such slow yaw iTerm growth ?? this is not what I see in the real logs == strange

    // Check for iterm growth, should not reach limits yet
    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(312.4f, pidData[FD_ROLL].I, calculateTolerance(312.4f));
    EXPECT_NEAR(-390.6f, pidData[FD_PITCH].I, calculateTolerance(-390.6));
    EXPECT_NEAR(21.1f, pidData[FD_YAW].I, calculateTolerance(21.1));

    // Expect iterm  roll + pitch to stop at limit of 400
    // yaw is still growing, 
    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(400.0f, pidData[FD_ROLL].I, calculateTolerance(400.0f));
    EXPECT_NEAR(-400.0f, pidData[FD_PITCH].I, calculateTolerance(-400.0f));
    EXPECT_NEAR(42.2f, pidData[FD_YAW].I, calculateTolerance(42.2f));

    // run some more loops, check all iTerm values are at their limit
    for (int loop = 0; loop < 7; loop++) {
        pidController(pidProfile, currentTestTime());
    }
    EXPECT_NEAR(400, pidData[FD_ROLL].I, calculateTolerance(400));
    EXPECT_NEAR(-400, pidData[FD_PITCH].I, calculateTolerance(-400));
    EXPECT_NEAR(320, pidData[FD_YAW].I, calculateTolerance(320));

    // Test that the added i term gain from Anti Gravity
    // is also limited
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    pidRuntime.itermLimit = 400;
    pidRuntime.itermLimitYaw = 320;

    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    setStickPosition(FD_YAW, 1.0f);
    const bool prevAgState = pidRuntime.antiGravityEnabled;
    const float prevAgTrhottleD = pidRuntime.antiGravityThrottleD;
    pidRuntime.antiGravityEnabled = true;
    pidRuntime.antiGravityThrottleD = 1.0;

    pidController(pidProfile, currentTestTime());

    // Expect more iterm accumulation than before on pitch and roll, no change on yaw
    // without antigravity values were 156, 195, 7
    EXPECT_NEAR(210.6, pidData[FD_ROLL].I, calculateTolerance(210.6f));
    EXPECT_NEAR(-249.6f, pidData[FD_PITCH].I, calculateTolerance(-249.6f));
    EXPECT_NEAR(7.0f, pidData[FD_YAW].I, calculateTolerance(7.0f));

    // run again and should expect to hit the limit on pitch and roll but yaw unaffected
    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(400, pidData[FD_ROLL].I, calculateTolerance(400));
    EXPECT_NEAR(-400, pidData[FD_PITCH].I, calculateTolerance(-400));
    EXPECT_NEAR(21.0f, pidData[FD_YAW].I, calculateTolerance(21.0f));

    pidRuntime.antiGravityEnabled = prevAgState;
    pidRuntime.antiGravityThrottleD = prevAgTrhottleD;

    // Test that i term is limited on yaw at 320  when only yaw is saturated
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidRuntime.itermLimit = 400;
    pidRuntime.itermLimitYaw = 320;

    pidStabilisationState(PID_STABILISATION_ON);
    setStickPosition(FD_ROLL, 0.0f);
    setStickPosition(FD_PITCH, 0.0f);
    setStickPosition(FD_YAW, 0.5f);

    for (int loop = 0; loop < 7; loop++) {
        pidController(pidProfile, currentTestTime());
    }

    EXPECT_NEAR(0, pidData[FD_ROLL].I, calculateTolerance(0));
    EXPECT_NEAR(0, pidData[FD_PITCH].I, calculateTolerance(0));
    EXPECT_NEAR(197, pidData[FD_YAW].I, calculateTolerance(197));

    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(253, pidData[FD_YAW].I, calculateTolerance(320));

    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(320, pidData[FD_YAW].I, calculateTolerance(320));

    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(0, pidData[FD_ROLL].I, calculateTolerance(0));
    EXPECT_NEAR(0, pidData[FD_PITCH].I, calculateTolerance(0));
    EXPECT_NEAR(320, pidData[FD_YAW].I, calculateTolerance(320));
}

TEST(pidControllerTest, testiTermWindup)
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    // simulate the outcome with iterm_windup of 50
    pidRuntime.itermLimit = 200;
    pidRuntime.itermLimitYaw = 160;

    pidStabilisationState(PID_STABILISATION_ON);
    setStickPosition(FD_ROLL, 0.12f);
    setStickPosition(FD_PITCH, 0.12f);
    setStickPosition(FD_YAW, 0.12f);

    for (int loop = 0; loop < 7; loop++) {
        pidController(pidProfile, currentTestTime());
    }

    EXPECT_NEAR(131, pidData[FD_ROLL].I, calculateTolerance(131));
    EXPECT_NEAR(164, pidData[FD_PITCH].I, calculateTolerance(164));
    EXPECT_NEAR(126, pidData[FD_YAW].I, calculateTolerance(126));

    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(150, pidData[FD_ROLL].I, calculateTolerance(150));
    EXPECT_NEAR(200, pidData[FD_PITCH].I, calculateTolerance(200));
    EXPECT_NEAR(137, pidData[FD_YAW].I, calculateTolerance(137));

    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(169, pidData[FD_ROLL].I, calculateTolerance(200));
    EXPECT_NEAR(200, pidData[FD_PITCH].I, calculateTolerance(200));
    EXPECT_NEAR(160, pidData[FD_YAW].I, calculateTolerance(160));

    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(200, pidData[FD_ROLL].I, calculateTolerance(200));
    EXPECT_NEAR(200, pidData[FD_PITCH].I, calculateTolerance(200));
    EXPECT_NEAR(160, pidData[FD_YAW].I, calculateTolerance(320));
}

// TODO - Add more scenarios
TEST(pidControllerTest, testCrashRecoveryMode)
{
    resetTest();
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_ON;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    sensorsSet(SENSOR_ACC);

    EXPECT_FALSE(crashRecoveryModeActive());

    int loopsToCrashTime = (int)((pidProfile->crash_time * 1000) / targetPidLooptime) + 1;

    // generate crash detection for roll axis
    gyro.gyroADCf[FD_ROLL]  = 800;
    simulatedMotorMixRange = 1.2f;
    for (int loop =0; loop <= loopsToCrashTime; loop++) {
        gyro.gyroADCf[FD_ROLL] += gyro.gyroADCf[FD_ROLL];
        // advance the time to avoid initialized state prevention of crash recovery
        pidController(pidProfile, currentTestTime() + 2000000);
    }

    EXPECT_TRUE(crashRecoveryModeActive());
    // Add additional verifications
}

TEST(pidControllerTest, testFeedForward)
// NOTE: THIS DOES NOT TEST THE FEEDFORWARD CALCULATIONS, which are now in rc.c, and return setpointDelta
// This test only validates the feedforward value calculated in pid.c for a given simulated setpointDelta
{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);

    // Move the sticks fully
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    setStickPosition(FD_YAW, -1.0f);

    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(17.86, pidData[FD_ROLL].F, calculateTolerance(17.86));
    EXPECT_NEAR(-16.49, pidData[FD_PITCH].F, calculateTolerance(-16.49));
    EXPECT_NEAR(-16.49, pidData[FD_YAW].F, calculateTolerance(-16.49));

    // Bring sticks back to half way
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    setStickPosition(FD_YAW, -0.5f);

    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(-8.93, pidData[FD_ROLL].F, calculateTolerance(-8.93));
    EXPECT_NEAR(8.24, pidData[FD_PITCH].F, calculateTolerance(8.24));
    EXPECT_NEAR(8.24, pidData[FD_YAW].F, calculateTolerance(8.24));

    // Bring sticks back to two tenths out
    setStickPosition(FD_ROLL, 0.2f);
    setStickPosition(FD_PITCH, -0.2f);
    setStickPosition(FD_YAW, -0.2f);

    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(-5.36, pidData[FD_ROLL].F, calculateTolerance(-5.36));
    EXPECT_NEAR(4.95, pidData[FD_PITCH].F, calculateTolerance(4.95));
    EXPECT_NEAR(4.95, pidData[FD_YAW].F, calculateTolerance(4.95));

    // Bring sticks back to one tenth out, to give a difference of 0.1
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    setStickPosition(FD_YAW, -0.1f);

    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(-1.79, pidData[FD_ROLL].F, calculateTolerance(-1.79));
    EXPECT_NEAR(1.65, pidData[FD_PITCH].F, calculateTolerance(1.65));
    EXPECT_NEAR(1.65, pidData[FD_YAW].F, calculateTolerance(1.65));

    // Center the sticks, giving the same delta value as before, should return the same feedforward
    setStickPosition(FD_ROLL, 0.0f);
    setStickPosition(FD_PITCH, 0.0f);
    setStickPosition(FD_YAW, 0.0f);

    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(-1.79, pidData[FD_ROLL].F, calculateTolerance(-1.79));
    EXPECT_NEAR(1.65, pidData[FD_PITCH].F, calculateTolerance(1.65));
    EXPECT_NEAR(1.65, pidData[FD_YAW].F, calculateTolerance(1.65));

    // Keep centered
    setStickPosition(FD_ROLL, 0.0f);
    setStickPosition(FD_PITCH, 0.0f);
    setStickPosition(FD_YAW, 0.0f);

    pidController(pidProfile, currentTestTime());

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);
}

TEST(pidControllerTest, testItermRelax)
{
    resetTest();
    pidProfile->iterm_relax = ITERM_RELAX_RP;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidProfile->iterm_relax_type = ITERM_RELAX_SETPOINT;
    pidInit(pidProfile);

    float itermErrorRate = 0;
    float currentPidSetpoint = 0;
    float gyroRate = 0;

    applyItermRelax(FD_PITCH, 0, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, 0);
    itermErrorRate = -10;
    currentPidSetpoint = 10;
    pidData[FD_PITCH].I = 10;

    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);

    EXPECT_NEAR(-8.16, itermErrorRate, calculateTolerance(-8.16));
    currentPidSetpoint += ITERM_RELAX_SETPOINT_THRESHOLD;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(0, itermErrorRate, calculateTolerance(0));
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(0, itermErrorRate, calculateTolerance(0));

    pidProfile->iterm_relax_type = ITERM_RELAX_GYRO;
    pidInit(pidProfile);

    currentPidSetpoint = 100;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, 0);
    gyroRate = 10;
    itermErrorRate = -10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(7, itermErrorRate, calculateTolerance(7));
    gyroRate += 100;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(-10, itermErrorRate, calculateTolerance(-10));

    pidProfile->iterm_relax = ITERM_RELAX_RP_INC;
    pidInit(pidProfile);

    itermErrorRate = -10;
    pidData[FD_PITCH].I = 10;
    currentPidSetpoint = 10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, -10);
    itermErrorRate = 10;
    pidData[FD_PITCH].I = -10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, 10);
    itermErrorRate = -10;
    currentPidSetpoint = 10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, -100);

    pidProfile->iterm_relax_type = ITERM_RELAX_SETPOINT;
    pidInit(pidProfile);

    itermErrorRate = -10;
    currentPidSetpoint = ITERM_RELAX_SETPOINT_THRESHOLD;
    applyItermRelax(FD_YAW, pidData[FD_YAW].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, -10);

    pidProfile->iterm_relax = ITERM_RELAX_RPY;
    pidInit(pidProfile);
    applyItermRelax(FD_YAW, pidData[FD_YAW].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(-3.6, itermErrorRate, calculateTolerance(-3.6));
}

// TODO - Add more tests
TEST(pidControllerTest, testAbsoluteControl)
{
    resetTest();
    pidProfile->abs_control_gain = 10;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    float gyroRate = 0;

    float itermErrorRate = 10;
    float currentPidSetpoint = 10;

    applyAbsoluteControl(FD_PITCH, gyroRate, &currentPidSetpoint, &itermErrorRate);

    EXPECT_NEAR(10.8, itermErrorRate, calculateTolerance(10.8));
    EXPECT_NEAR(10.8, currentPidSetpoint, calculateTolerance(10.8));

    applyAbsoluteControl(FD_PITCH, gyroRate, &currentPidSetpoint, &itermErrorRate);
    EXPECT_NEAR(10.8, itermErrorRate, calculateTolerance(10.8));
    EXPECT_NEAR(10.8, currentPidSetpoint, calculateTolerance(10.8));

    gyroRate = -53;
    axisError[FD_PITCH] = -60;
    applyAbsoluteControl(FD_PITCH, gyroRate, &currentPidSetpoint, &itermErrorRate);
    EXPECT_NEAR(-79.2, itermErrorRate, calculateTolerance(-79.2));
    EXPECT_NEAR(-79.2, currentPidSetpoint, calculateTolerance(-79.2));
}

TEST(pidControllerTest, testDtermFiltering)
{
// TODO
}

TEST(pidControllerTest, testItermRotationHandling)
{
    resetTest();
    pidInit(pidProfile);

    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_PITCH].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_YAW].I, 0);

    pidProfile->iterm_rotation = true;
    pidInit(pidProfile);

    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_PITCH].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_YAW].I, 0);

    pidData[FD_ROLL].I = 10;
    pidData[FD_PITCH].I = 1000;
    pidData[FD_YAW].I = 1000;
    gyro.gyroADCf[FD_ROLL] = -1000;
    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 10);
    EXPECT_NEAR(860.37, pidData[FD_PITCH].I, calculateTolerance(860.37));
    EXPECT_NEAR(1139.6, pidData[FD_YAW].I, calculateTolerance(1139.6));

    pidProfile->abs_control_gain = 10;
    pidInit(pidProfile);
    pidData[FD_ROLL].I = 10;
    pidData[FD_PITCH].I = 1000;
    pidData[FD_YAW].I = 1000;

    gyro.gyroADCf[FD_ROLL] = -1000;
    // FIXME - axisError changes don't affect the system. This is a potential bug or intendend behaviour?
    axisError[FD_PITCH] = 1000;
    axisError[FD_YAW] = 1000;
    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 10);
    EXPECT_NEAR(860.37, pidData[FD_PITCH].I, calculateTolerance(860.37));
    EXPECT_NEAR(1139.6, pidData[FD_YAW].I, calculateTolerance(1139.6));
}

TEST(pidControllerTest, testLaunchControl)
{
    // The launchControlGain is indirectly tested since when launch control is active the
    // the gain overrides the PID settings. If the logic to use launchControlGain wasn't
    // working then any I calculations would be incorrect.

    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // test that feedforward and D are disabled (always zero) when launch control is active
    // set initial state
    pidController(pidProfile, currentTestTime());

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Move the sticks to induce feedforward
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    setStickPosition(FD_YAW, -0.5f);

    // add gyro activity to induce D
    gyro.gyroADCf[FD_ROLL] = -1000;
    gyro.gyroADCf[FD_PITCH] = 1000;
    gyro.gyroADCf[FD_YAW] = -1000;

    pidController(pidProfile, currentTestTime());

    // validate that feedforwad is still 0
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);

    // validate that D is still 0
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // test NORMAL mode - expect P/I on roll and pitch, P on yaw but I == 0
    unitLaunchControlMode = LAUNCH_CONTROL_MODE_NORMAL;
    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    gyro.gyroADCf[FD_ROLL] = -20;
    gyro.gyroADCf[FD_PITCH] = 20;
    gyro.gyroADCf[FD_YAW] = -20;
    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(25.62,  pidData[FD_ROLL].P,  calculateTolerance(25.62));
    EXPECT_NEAR(1.56,   pidData[FD_ROLL].I,  calculateTolerance(1.56));
    EXPECT_NEAR(-37.15, pidData[FD_PITCH].P, calculateTolerance(-37.15));
    EXPECT_NEAR(-1.56,  pidData[FD_PITCH].I, calculateTolerance(-1.56));
    EXPECT_NEAR(44.84,  pidData[FD_YAW].P,   calculateTolerance(44.84));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);

    // test PITCHONLY mode - expect P/I only on pitch; I cannot go negative
    unitLaunchControlMode = LAUNCH_CONTROL_MODE_PITCHONLY;
    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    // first test that pitch I is prevented from going negative
    gyro.gyroADCf[FD_ROLL] = 0;
    gyro.gyroADCf[FD_PITCH] = 20;
    gyro.gyroADCf[FD_YAW] = 0;
    pidController(pidProfile, currentTestTime());

    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);

    gyro.gyroADCf[FD_ROLL] = 20;
    gyro.gyroADCf[FD_PITCH] = -20;
    gyro.gyroADCf[FD_YAW] = 20;
    pidController(pidProfile, currentTestTime());

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_NEAR(37.15, pidData[FD_PITCH].P, calculateTolerance(37.15));
    EXPECT_NEAR(1.56,  pidData[FD_PITCH].I, calculateTolerance(1.56));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);

    // test FULL mode - expect P/I on all axes
    unitLaunchControlMode = LAUNCH_CONTROL_MODE_FULL;
    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    gyro.gyroADCf[FD_ROLL] = -20;
    gyro.gyroADCf[FD_PITCH] = 20;
    gyro.gyroADCf[FD_YAW] = -20;
    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(25.62,  pidData[FD_ROLL].P,  calculateTolerance(25.62));
    EXPECT_NEAR(1.56,   pidData[FD_ROLL].I,  calculateTolerance(1.56));
    EXPECT_NEAR(-37.15, pidData[FD_PITCH].P, calculateTolerance(-37.15));
    EXPECT_NEAR(-1.56,  pidData[FD_PITCH].I, calculateTolerance(-1.56));
    EXPECT_NEAR(44.84,  pidData[FD_YAW].P,   calculateTolerance(44.84));
    EXPECT_NEAR(1.56,   pidData[FD_YAW].I,  calculateTolerance(1.56));
}

TEST(pidControllerTest, testTpaClassic)
{
    resetTest();

    pidProfile->tpa_curve_type = TPA_CURVE_CLASSIC;
    pidProfile->tpa_rate = 30;
    pidProfile->tpa_breakpoint = 1600;
    pidProfile->tpa_low_rate = -50;
    pidProfile->tpa_low_breakpoint = 1200;
    pidProfile->tpa_low_always = 1;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(1.5f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.1f);
    EXPECT_FLOAT_EQ(1.25f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.2f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.6f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.8f);
    EXPECT_FLOAT_EQ(0.85f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(1.0f);
    EXPECT_FLOAT_EQ(0.7f, pidRuntime.tpaFactor);


    pidProfile->tpa_curve_type = TPA_CURVE_CLASSIC;
    pidProfile->tpa_rate = 30;
    pidProfile->tpa_breakpoint = 1600;
    pidProfile->tpa_low_rate = -50;
    pidProfile->tpa_low_breakpoint = 1000;
    pidProfile->tpa_low_always = 1;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.1f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.2f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.6f);
    EXPECT_FLOAT_EQ(1.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.8f);
    EXPECT_FLOAT_EQ(0.85f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(1.0f);
    EXPECT_FLOAT_EQ(0.7f, pidRuntime.tpaFactor);
}

TEST(pidControllerTest, testTpaHyperbolic)
{
    resetTest();

    // curve sligly down - edge case where internal expo -> inf
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 50;
    pidProfile->tpa_curve_pid_thr0 = 500;
    pidProfile->tpa_curve_expo = 10;
    pidProfile->tpa_curve_stall_throttle = 30;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(5.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_FLOAT_EQ(5.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(2.588f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(0.693f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.5f, pidRuntime.tpaFactor, 0.01f);

    // linear curve
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 10;
    pidProfile->tpa_curve_pid_thr0 = 300;
    pidProfile->tpa_curve_expo = 0;
    pidProfile->tpa_curve_stall_throttle = 0;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(3.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_NEAR(2.565f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(1.550f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(0.390f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.1f, pidRuntime.tpaFactor, 0.01f);

    // curve bends up
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 60;
    pidProfile->tpa_curve_pid_thr0 = 1000;
    pidProfile->tpa_curve_expo = -50;
    pidProfile->tpa_curve_stall_throttle = 40;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(10.0f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_NEAR(10.0f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(9.700f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(7.364f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.625f, pidRuntime.tpaFactor, 0.01f);

    // curve bends down
    pidProfile->tpa_curve_type = TPA_CURVE_HYPERBOLIC;
    pidProfile->tpa_curve_pid_thr100 = 90;
    pidProfile->tpa_curve_pid_thr0 = 250;
    pidProfile->tpa_curve_expo = 60;
    pidProfile->tpa_curve_stall_throttle = 60;

    pidInit(pidProfile);

    pidUpdateTpaFactor(0.0f);
    EXPECT_FLOAT_EQ(2.5f, pidRuntime.tpaFactor);

    pidUpdateTpaFactor(0.15f);
    EXPECT_NEAR(2.5f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.5);
    EXPECT_NEAR(2.5f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(0.9);
    EXPECT_NEAR(0.954f, pidRuntime.tpaFactor, 0.01f);

    pidUpdateTpaFactor(1.0);
    EXPECT_NEAR(0.9f, pidRuntime.tpaFactor, 0.01f);
}
