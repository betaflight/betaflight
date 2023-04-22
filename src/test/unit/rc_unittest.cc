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

// fake pid returns
float simulatedRawSetpoint[3] = { 0,0,0 };
float simulatedSetpointRate[3] = { 0,0,0 };
float simulatedRcDeflection[3] = { 0,0,0 };
float simulatedMaxRate[3] = { 670,670,670 };
float simulatedGetFeedforward[3] = { 0,0,0};
float feedforward = 0.0f;

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

// pid requirements
float simulatedMotorMixRange = 0.0f;

bool simulatedAirmodeEnabled = true;

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

    #include "io/gps.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"

    gyro_t gyro;
    attitudeEulerAngles_t attitude;

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);

    // pid requirements
    bool unitLaunchControlActive = false;
    bool isAirmodeActivated(void) { return simulatedAirmodeEnabled; }
    bool isLaunchControlActive(void) {return unitLaunchControlActive; }
    launchControlMode_e unitLaunchControlMode = LAUNCH_CONTROL_MODE_NORMAL;
    bool gyroOverflowDetected(void) { return false; }
    float getSetpointRate(int axis) { return simulatedSetpointRate[axis]; }
    float getRcDeflection(int axis) { return simulatedRcDeflection[axis]; }
    float getMotorMixRange(void) { return simulatedMotorMixRange; }
    float getFeedforward(int axis)
    {
        UNUSED(axis);
        return simulatedGetFeedforward[axis];
    }
    float getMaxRcRate(int axis)
    {
        UNUSED(axis);
        return simulatedMaxRate[axis];
    }

    void systemBeep(bool) { }
    void beeperConfirmationBeeps(uint8_t) { }
    void disarm(flightLogDisarmReason_e) { }

    void initRcProcessing(void) { }
}

pidProfile_t *pidProfile;
pidRuntime_t pidRuntime;

int loopIter = 0;

timeUs_t currentTestTime(void)
{
    return targetPidLooptime * loopIter++;
}

void resetTest(void)
{
    loopIter = 0;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidData[axis].P = 0;
        pidData[axis].I = 0;
        pidData[axis].D = 0;
        pidData[axis].F = 120;
        pidData[axis].Sum = 0;
        gyro.gyroADCf[axis] = 0;
    }

    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;

    flightModeFlags = 0;
    loadControlRateProfile();
}

// All calculations will have 10% tolerance
float calculateTolerance(float input)
{
    return fabsf(input * 0.1f);
}


// *******  do the feedforward tests here ***********
TEST(rcUnitTest, testFeedForward)

{
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    calculateFeedforward(&pidRuntime, FD_ROLL); // **** FAILS !! *******


    EXPECT_NEAR(0.1, feedforward, calculateTolerance(0.1));

    // typical tests
    // EXPECT_NEAR(-3.6, itermErrorRate, calculateTolerance(-3.6));
    // EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    // EXPECT_EQ(0, ALIGNMENT_YAW_ROTATIONS(bits));
}
