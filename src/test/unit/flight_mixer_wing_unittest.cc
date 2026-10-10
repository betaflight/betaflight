/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "common/filter.h"
    #include "common/maths.h"

    #include "config/feature.h"

    #include "fc/controlrate_profile.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/mixer.h"
    #include "flight/mixer_init.h"
    #include "flight/pid.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "rx/rx.h"

    #include "io/gps.h"

    #include "sensors/gyro.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static const float DISARMED_OUTPUT = 900.0f;

static bool testMotorStopRequested;
static uint32_t testFeatures;

extern "C" {
    uint8_t armingFlags;
    uint16_t flightModeFlags;
    uint8_t stateFlags;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    pidAxisData_t pidData[XYZ_AXIS_COUNT];
    gyro_t gyro;
    gpsSolutionData_t gpsSol;
    mixerRuntime_t mixerRuntime;
    float throttleBoost;
    pt1Filter_t throttleLpf;
    static pidProfile_t testPidProfile;
    pidProfile_t *currentPidProfile = &testPidProfile;
    static controlRateConfig_t testRateProfile;
    controlRateConfig_t *currentControlRateProfile = &testRateProfile;

    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);

    bool autopilotWingMotorStopRequested(void) { return testMotorStopRequested; }
    bool featureIsEnabled(const uint32_t mask) { return (testFeatures & mask) != 0; }
    bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
    bool isAirmodeEnabled(void) { return false; }
    bool isLaunchControlActive(void) { return false; }
    bool isCrashFlipModeActive(void) { return false; }
    bool isMotorsReversed(void) { return false; }
    bool gyroYawSpinDetected(void) { return false; }
    bool failsafeIsActive(void) { return false; }
    bool sensors(uint32_t) { return true; }
    bool autopilotThrottleValid(void) { return true; }
    float getAutopilotThrottle(void) { return 0.4f; }
    bool launchWingThrottleValid(void) { return false; }
    float launchWingGetThrottle(void) { return 0.0f; }
    float launchWingHandoverFactor(void) { return 1.0f; }
    float getCosTiltAngle(void) { return 1.0f; }
    float getRcDeflection(int) { return 0.0f; }
    float getRcDeflectionAbs(int) { return 0.0f; }
    float getMaxRcDeflectionAbs(void) { return 0.0f; }
    uint16_t getBatterySagCellVoltage(void) { return 0; }
    void delay(uint32_t) {}
    void motorWriteAll(float *) {}
    void dynLpfGyroUpdate(float) {}
    void dynLpfDTermUpdate(float) {}
    void pidResetIterm(void) {}
    void pidUpdateAntiGravityThrottleFilter(float) {}
    void pidUpdateTpaFactor(float) {}
    float pidApplyThrustLinearization(float motorOutput) { return motorOutput; }
    float pidCompensateThrustLinearization(float throttle) { return throttle; }
    bool mixerIsTricopter(void) { return false; }
    float mixerTricopterMotorCorrection(int) { return 0.0f; }
    bool motorIsEnabled(void) { return true; }
    bool motorIsMotorEnabled(unsigned) { return true; }
    uint16_t motorConvertToExternal(float motorValue) { return motorValue; }
}

class MixerWingTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        pgResetAll();
        memset(&mixerRuntime, 0, sizeof(mixerRuntime));
        mixerRuntime.motorCount = 1;
        mixerRuntime.currentMixer[0].throttle = 1.0f;
        mixerRuntime.motorOutputLow = 1000.0f;
        mixerRuntime.motorOutputHigh = 2000.0f;
        mixerRuntime.disarmMotorOutput = DISARMED_OUTPUT;
        memset(&testPidProfile, 0, sizeof(testPidProfile));
        testPidProfile.pidSumLimit = 500;
        testPidProfile.pidSumLimitYaw = 400;
        rxConfigMutable()->mincheck = 1050;
        rcData[THROTTLE] = 1000.0f;
        rcCommand[THROTTLE] = 1000.0f;
        testMotorStopRequested = false;
        testFeatures = 0;
        armingFlags = 0;
        ENABLE_ARMING_FLAG(ARMED);
        flightModeFlags = ALT_HOLD_MODE;
    }
};

TEST_F(MixerWingTest, TheAutopilotStopsTheMotorWithoutTheMotorStopFeature)
{
    mixTable(0);
    EXPECT_GT(motor[0], 1000.0f);       // altitude hold's throttle

    testMotorStopRequested = true;
    mixTable(0);
    EXPECT_FLOAT_EQ(DISARMED_OUTPUT, motor[0]);

    testFeatures = FEATURE_MOTOR_STOP;
    mixTable(0);
    EXPECT_FLOAT_EQ(DISARMED_OUTPUT, motor[0]);
}

TEST_F(MixerWingTest, OnlyArmedDoesTheAutopilotStopTheMotor)
{
    DISABLE_ARMING_FLAG(ARMED);
    testMotorStopRequested = true;
    mixTable(0);
    EXPECT_NE(DISARMED_OUTPUT, motor[0]);
}
