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

#include <cmath>
#include <cstring>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "common/axis.h"
#include "config/config.h"
#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/mixer_init.h"
#include "flight/pid.h"
#include "io/gps.h"
#include "pg/rpm_filter.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

motorConfig_t motorConfig_System;
rpmFilterConfig_t rpmFilterConfig_System;
rxConfig_t rxConfig_System;
flight3DConfig_t flight3DConfig_System;
pidProfile_t testPidProfile;
pidProfile_t *currentPidProfile = &testPidProfile;
controlRateConfig_t testRateProfile;
controlRateConfig_t *currentControlRateProfile = &testRateProfile;
pidAxisData_t pidData[3];
gpsSolutionData_t gpsSol;
gyro_t gyro;
uint8_t armingFlags;
uint16_t flightModeFlags;
uint8_t stateFlags;
float rcCommand[4];
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
bool useDshotTelemetry;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;
static timeUs_t nowUs;

timeUs_t micros(void) { return nowUs; }
void delay(uint32_t) {}
bool featureIsEnabled(uint8_t) { return false; }
bool isLaunchControlActive(void) { return false; }
bool isAirmodeEnabled(void) { return false; }
bool isCrashFlipModeActive(void) { return false; }
bool isMotorsReversed(void) { return false; }
bool failsafeIsActive(void) { return false; }
bool isMotorProtocolDshot(void) { return true; }
bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
bool sensors(uint32_t) { return false; }
float getCosTiltAngle(void) { return 1; }
float getRcDeflection(int) { return 0; }
float getRcDeflectionAbs(int) { return 0; }
float getMaxRcDeflectionAbs(void) { return 0; }
float pidGetDT(void) { return 0.001f; }
float pidGetPidFrequency(void) { return 1000.0f; }
void pidResetIterm(void) {}
void pidUpdateAntiGravityThrottleFilter(float) {}
void pidUpdateTpaFactor(float) {}
float motorEstimateMaxRpm(void) { return 20000.0f; }
void motorWriteAll(float *) {}
void motorInitEndpoints(const motorConfig_t *, float, float *, float *, float *, float *, float *) {}
float mixerTricopterMotorCorrection(int) { return 0; }
void mixerTricopterInit(void) {}
}

#include "gtest/gtest.h"

class RpmLimiterTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        std::memset(&mixerRuntime, 0, sizeof(mixerRuntime));
        std::memset(&dshotTelemetryState, 0, sizeof(dshotTelemetryState));
        std::memset(debug, 0, sizeof(debug));
        mixerRuntime.motorCount = 4;
        mixerRuntime.motorOutputHigh = 1;
        for (unsigned i = 0; i < 4; i++) {
            mixerRuntime.currentMixer[i].throttle = 1;
        }
        dshotMotorCount = 4;
        useDshotTelemetry = true;
        motorConfig_System.dev.useDshotTelemetry = true;
        motorConfig_System.motorPoleCount = 14;
        motorConfig_System.maxthrottle = 2000;
        rxConfig_System.maxcheck = 1900;
        rxConfig_System.mincheck = 1050;
        armingFlags = ARMED;
        rcCommand[THROTTLE] = 1600;
        rcData[THROTTLE] = 1600;
        mixerConfigMutable()->rpm_limit = true;
        mixerConfigMutable()->rpm_limit_value = 18000;
        mixerConfigMutable()->rpm_limit_p = 25;
        mixerConfigMutable()->rpm_limit_i = 10;
        mixerConfigMutable()->rpm_limit_d = 8;
        mixerConfigMutable()->mixer_type = MIXER_LEGACY;
        debugMode = DEBUG_RPM_LIMIT;
        nowUs = 1000;
        initDshotTelemetry(1000);
        mixerInitProfile();
    }

    void tick(bool valid, uint16_t rawValue = 450)
    {
        nowUs += 1000;
        // Model the actual transport contract: invalid/missing replies do not
        // overwrite per-motor rawValue, but the cycle is marked unprocessed.
        if (valid) {
            for (unsigned i = 0; i < 4; i++) {
                dshotTelemetryState.motorState[i].rawValue = rawValue;
            }
        }
        dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
        updateDshotTelemetry();
        mixTable(nowUs);
    }
};

TEST_F(RpmLimiterTest, LostTelemetryCannotContinueSuppressingThrottle)
{
    for (unsigned i = 0; i < 100; i++) {
        tick(true);
    }
    ASSERT_TRUE(mixerRuntime.rpmLimiterTelemetryFresh);
    ASSERT_NEAR(19043, getDshotRpmAverage(), 1);
    ASSERT_GT(mixerRuntime.rpmLimiterI, 0);

    for (unsigned i = 0; i < 30000; i++) {
        tick(false);
    }
    EXPECT_FALSE(mixerRuntime.rpmLimiterTelemetryFresh);
    EXPECT_FLOAT_EQ(0, mixerRuntime.rpmLimiterI);
    EXPECT_FLOAT_EQ(0, mixerRuntime.rpmLimiterPreviousError);
    for (unsigned i = 0; i < 4; i++) {
        EXPECT_NEAR(0.6f, motor[i], 1e-6f);
    }
}

TEST_F(RpmLimiterTest, RecoveryDerivativeStartsFromClearedError)
{
    // Fresh zero-RPM packets create a negative previous error, exposing the
    // positive derivative kick that retained history could cause on recovery.
    tick(true, 0x0fff);
    ASSERT_FLOAT_EQ(-18000, mixerRuntime.rpmLimiterPreviousError);

    for (unsigned i = 0; i < 102; i++) {
        tick(false);
    }
    ASSERT_FALSE(mixerRuntime.rpmLimiterTelemetryFresh);
    EXPECT_FLOAT_EQ(0, mixerRuntime.rpmLimiterPreviousError);

    tick(true);
    ASSERT_TRUE(mixerRuntime.rpmLimiterTelemetryFresh);
    const float recoveredError = mixerRuntime.rpmLimiterPreviousError;
    EXPECT_LT(recoveredError, 0);
    EXPECT_EQ(std::lrint(recoveredError * mixerRuntime.rpmLimiterDGain * 100), debug[7]);
    EXPECT_LT(debug[7], 0);
    EXPECT_NEAR(0.54f, motor[0], 1e-6f);
}

TEST_F(RpmLimiterTest, DisabledLimiterPreservesThrottleWithFreshTelemetry)
{
    mixerConfigMutable()->rpm_limit = false;
    for (unsigned i = 0; i < 100; i++) {
        tick(true);
    }
    EXPECT_NEAR(0.6f, motor[0], 1e-6f);
    EXPECT_FALSE(mixerRuntime.rpmLimiterTelemetryFresh);
}
