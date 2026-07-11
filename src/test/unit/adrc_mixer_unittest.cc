/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <cstring>

extern "C" {
#include "platform.h"

#include "build/debug.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"
}

#include "gtest/gtest.h"

namespace {

constexpr float TEST_EPSILON = 1.0e-6f;

uint32_t testFeatureMask;
bool testAirmodeEnabled;
bool testCrashFlipModeActive;
bool testYawSpinDetected;
float testAutopilotThrottle;
float testRcDeflection[XYZ_AXIS_COUNT];

int adrcFeedbackCallCount;
const pidProfile_t *adrcFeedbackProfile;
float adrcFeedbackAxisScale;
float adrcFeedbackYawLimit;

pidProfile_t testPidProfile;
controlRateConfig_t testControlRateProfile;

void setRcThrottle(const float normalizedThrottle)
{
    rcCommand[THROTTLE] = PWM_RANGE_MIN + normalizedThrottle * PWM_RANGE;
    rcData[THROTTLE] = rcCommand[THROTTLE];
}

void expectAdrcFeedback(const float expectedAxisScale, const float expectedYawLimit)
{
    EXPECT_EQ(adrcFeedbackCallCount, 1);
    EXPECT_EQ(adrcFeedbackProfile, &testPidProfile);
    EXPECT_NEAR(adrcFeedbackAxisScale, expectedAxisScale, TEST_EPSILON);
    EXPECT_NEAR(adrcFeedbackYawLimit, expectedYawLimit, TEST_EPSILON);
}

struct QuadXMotorComponents {
    float collective;
    float roll;
    float pitch;
    float yaw;
};

QuadXMotorComponents decomposeQuadXMotorOutput(void)
{
    return {
        (motor[0] + motor[1] + motor[2] + motor[3]) * 0.25f,
        (-motor[0] - motor[1] + motor[2] + motor[3]) * 0.25f,
        (motor[0] - motor[1] + motor[2] - motor[3]) * 0.25f,
        (-motor[0] + motor[1] + motor[2] - motor[3]) * 0.25f,
    };
}

class AdrcMixerUnittest : public ::testing::Test {
protected:
    void SetUp() override
    {
        std::memset(&mixerConfig_System, 0, sizeof(mixerConfig_System));
        std::memset(&rxConfig_System, 0, sizeof(rxConfig_System));
        std::memset(&flight3DConfig_System, 0, sizeof(flight3DConfig_System));
        std::memset(&mixerRuntime, 0, sizeof(mixerRuntime));
        std::memset(&testPidProfile, 0, sizeof(testPidProfile));
        std::memset(&testControlRateProfile, 0, sizeof(testControlRateProfile));
        std::memset(pidData, 0, sizeof(pidData));
        std::memset(rcCommand, 0, sizeof(rcCommand));
        std::memset(rcData, 0, sizeof(rcData));
        std::memset(motor, 0, sizeof(motor));
        std::memset(motor_disarmed, 0, sizeof(motor_disarmed));
        std::memset(&gyro, 0, sizeof(gyro));
        std::memset(&gpsSol, 0, sizeof(gpsSol));
        std::memset(testRcDeflection, 0, sizeof(testRcDeflection));

        armingFlags = ARMED;
        flightModeFlags = 0;
        stateFlags = 0;

        testFeatureMask = 0;
        testAirmodeEnabled = true;
        testCrashFlipModeActive = false;
        testYawSpinDetected = false;
        testAutopilotThrottle = 0.0f;

        adrcFeedbackCallCount = 0;
        adrcFeedbackProfile = nullptr;
        adrcFeedbackAxisScale = 0.0f;
        adrcFeedbackYawLimit = 0.0f;

        currentPidProfile = &testPidProfile;
        testPidProfile.pid_type = PID_TYPE_ADRC;
        testPidProfile.pidSumLimit = PIDSUM_LIMIT_MAX;
        testPidProfile.pidSumLimitYaw = 400;

        currentControlRateProfile = &testControlRateProfile;
        testControlRateProfile.throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF;
        testControlRateProfile.throttle_limit_percent = 100;

        mixerConfigMutable()->mixerMode = MIXER_QUADX;
        mixerConfigMutable()->mixer_type = MIXER_LEGACY;

        rxConfigMutable()->midrc = PWM_RANGE_MIDDLE;
        rxConfigMutable()->mincheck = 1050;
        rxConfigMutable()->maxcheck = 1900;

        mixerRuntime.motorCount = QUAD_MOTOR_COUNT;
        mixerRuntime.motorOutputLow = 0.0f;
        mixerRuntime.motorOutputHigh = 1.0f;
        mixerRuntime.disarmMotorOutput = -1.0f;
        mixerRuntime.feature3dEnabled = false;

        const motorMixer_t quadX[QUAD_MOTOR_COUNT] = {
            { 1.0f, -1.0f,  1.0f, -1.0f },
            { 1.0f, -1.0f, -1.0f,  1.0f },
            { 1.0f,  1.0f,  1.0f,  1.0f },
            { 1.0f,  1.0f, -1.0f, -1.0f },
        };
        std::memcpy(mixerRuntime.currentMixer, quadX, sizeof(quadX));

        setRcThrottle(0.5f);
    }
};

TEST_F(AdrcMixerUnittest, LegacyMixerPublishesNormalizationAppliedToMotorMix)
{
    pidData[FD_ROLL].Sum = PIDSUM_LIMIT_MAX;

    mixTable(1000);

    expectAdrcFeedback(0.5f, testPidProfile.pidSumLimitYaw);
    EXPECT_NEAR(mixerGetAdrcThrottle(), 0.5f, TEST_EPSILON);
    EXPECT_NEAR(motor[0], 0.0f, TEST_EPSILON);
    EXPECT_NEAR(motor[2], 1.0f, TEST_EPSILON);
}

TEST_F(AdrcMixerUnittest, LinearMixerPublishesNormalizationAppliedToMotorMix)
{
    mixerConfigMutable()->mixer_type = MIXER_LINEAR;
    pidData[FD_ROLL].Sum = PIDSUM_LIMIT_MAX;

    mixTable(2000);

    expectAdrcFeedback(0.5f, testPidProfile.pidSumLimitYaw);
    EXPECT_NEAR(mixerGetAdrcThrottle(), 0.5f, TEST_EPSILON);
    EXPECT_NEAR(motor[0], 0.0f, TEST_EPSILON);
    EXPECT_NEAR(motor[2], 1.0f, TEST_EPSILON);
}

TEST_F(AdrcMixerUnittest, DynamicMixerPublishesUniformScaleAndLeavesRedistributionAsPlantDisturbance)
{
    mixerConfigMutable()->mixer_type = MIXER_DYNAMIC;
    setRcThrottle(0.25f);
    pidData[FD_ROLL].Sum = 400.0f;
    pidData[FD_PITCH].Sum = 400.0f;

    mixTable(2500);

    const float expectedAxisScale = 1.0f / 1.6f;
    expectAdrcFeedback(expectedAxisScale, testPidProfile.pidSumLimitYaw);
    EXPECT_NEAR(mixerGetAdrcThrottle(), 0.25f, TEST_EPSILON);

    // MIXER_DYNAMIC adds k * abs(motorMix) per motor, where k = 1 - 2 * throttle.
    // For equal roll and pitch on QuadX this residual contains collective and yaw terms.
    const QuadXMotorComponents components = decomposeQuadXMotorOutput();
    EXPECT_NEAR(components.collective, 0.375f, TEST_EPSILON);
    EXPECT_NEAR(components.roll, 0.4f * expectedAxisScale, TEST_EPSILON);
    EXPECT_NEAR(components.pitch, 0.4f * expectedAxisScale, TEST_EPSILON);
    EXPECT_NEAR(components.yaw, 0.125f, TEST_EPSILON);
}

TEST_F(AdrcMixerUnittest, EzLandingPublishesExactUniformAxisScale)
{
    mixerConfigMutable()->mixer_type = MIXER_EZLANDING;
    mixerRuntime.ezLandingLimit = 0.15f;
    setRcThrottle(0.0f);
    pidData[FD_ROLL].Sum = 400.0f;
    pidData[FD_PITCH].Sum = 400.0f;

    mixTable(2750);

    const float baseNormalizationFactor = 1.0f / 1.6f;
    const float ezLandFactor = 0.15f / (0.5f + 1.0e-6f);
    const float expectedAxisScale = baseNormalizationFactor * ezLandFactor;
    expectAdrcFeedback(expectedAxisScale, testPidProfile.pidSumLimitYaw);
    EXPECT_NEAR(mixerGetAdrcThrottle(), 0.15f, TEST_EPSILON);

    const QuadXMotorComponents components = decomposeQuadXMotorOutput();
    EXPECT_NEAR(components.collective, mixerGetAdrcThrottle(), TEST_EPSILON);
    EXPECT_NEAR(components.roll, 0.4f * expectedAxisScale, TEST_EPSILON);
    EXPECT_NEAR(components.pitch, 0.4f * expectedAxisScale, TEST_EPSILON);
    EXPECT_NEAR(components.yaw, 0.0f, TEST_EPSILON);
}

TEST_F(AdrcMixerUnittest, NoAirmodePublishesLowThrottleAuthorityAttenuation)
{
    testAirmodeEnabled = false;
    setRcThrottle(0.0f);
    pidData[FD_ROLL].Sum = 500.0f;

    mixTable(3000);

    expectAdrcFeedback(0.5f, testPidProfile.pidSumLimitYaw);
    EXPECT_NEAR(mixerGetAdrcThrottle(), 0.25f, TEST_EPSILON);
    EXPECT_NEAR(motor[0], 0.0f, TEST_EPSILON);
    EXPECT_NEAR(motor[2], 0.5f, TEST_EPSILON);
}

TEST_F(AdrcMixerUnittest, YawSpinPublishesEffectiveYawLimitAndNormalization)
{
    pidData[FD_YAW].Sum = 900.0f;

    mixTable(4000);

    expectAdrcFeedback(1.0f, testPidProfile.pidSumLimitYaw);

    adrcFeedbackCallCount = 0;
    testYawSpinDetected = true;

    mixTable(5000);

    expectAdrcFeedback(1.0f / 1.8f, PIDSUM_LIMIT_MAX);
    EXPECT_NEAR(mixerGetAdrcThrottle(), 0.5f, TEST_EPSILON);
}

TEST_F(AdrcMixerUnittest, MotorStopPublishesZeroAppliedOutput)
{
    testFeatureMask = FEATURE_MOTOR_STOP;
    testAirmodeEnabled = false;
    setRcThrottle(0.0f);
    rcData[THROTTLE] = PWM_RANGE_MIN;
    pidData[FD_ROLL].Sum = 500.0f;

    mixTable(6000);

    expectAdrcFeedback(0.0f, testPidProfile.pidSumLimitYaw);
    EXPECT_FLOAT_EQ(mixerGetAdrcThrottle(), 0.0f);
    for (int i = 0; i < QUAD_MOTOR_COUNT; ++i) {
        EXPECT_FLOAT_EQ(motor[i], mixerRuntime.disarmMotorOutput);
    }
}

TEST_F(AdrcMixerUnittest, CrashFlipPublishesZeroAppliedOutput)
{
    testCrashFlipModeActive = true;
    setRcThrottle(0.0f);

    mixTable(7000);

    expectAdrcFeedback(0.0f, testPidProfile.pidSumLimitYaw);
    EXPECT_FLOAT_EQ(mixerGetAdrcThrottle(), 0.0f);
    for (int i = 0; i < QUAD_MOTOR_COUNT; ++i) {
        EXPECT_FLOAT_EQ(motor[i], mixerRuntime.disarmMotorOutput);
    }
}

TEST_F(AdrcMixerUnittest, AltHoldPublishesPostOverrideThrottle)
{
    setRcThrottle(0.8f);
    testAutopilotThrottle = 0.35f;
    flightModeFlags = ALT_HOLD_MODE;

    mixTable(8000);

    expectAdrcFeedback(1.0f, testPidProfile.pidSumLimitYaw);
    EXPECT_NEAR(mixerGetAdrcThrottle(), testAutopilotThrottle, TEST_EPSILON);
}

TEST_F(AdrcMixerUnittest, GpsRescuePublishesPostOverrideThrottle)
{
    setRcThrottle(0.2f);
    testAutopilotThrottle = 0.75f;
    flightModeFlags = GPS_RESCUE_MODE;

    mixTable(9000);

    expectAdrcFeedback(1.0f, testPidProfile.pidSumLimitYaw);
    EXPECT_NEAR(mixerGetAdrcThrottle(), testAutopilotThrottle, TEST_EPSILON);
}

} // namespace

extern "C" {

mixerConfig_t mixerConfig_System;
rxConfig_t rxConfig_System;
flight3DConfig_t flight3DConfig_System;

mixerRuntime_t mixerRuntime;
pidProfile_t *currentPidProfile;
controlRateConfig_t *currentControlRateProfile;
pidAxisData_t pidData[XYZ_AXIS_COUNT];

uint8_t armingFlags;
uint16_t flightModeFlags;
uint8_t stateFlags;

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

float rcCommand[NON_AUX_CHANNEL_COUNT];
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

gyro_t gyro;
gpsSolutionData_t gpsSol;

bool featureIsEnabled(const uint32_t mask)
{
    return testFeatureMask & mask;
}

bool IS_RC_MODE_ACTIVE(boxId_e boxId)
{
    UNUSED(boxId);
    return false;
}

bool isAirmodeEnabled(void)
{
    return testAirmodeEnabled;
}

bool isCrashFlipModeActive(void)
{
    return testCrashFlipModeActive;
}

bool isLaunchControlActive(void)
{
    return false;
}

bool isMotorsReversed(void)
{
    return false;
}

bool gyroYawSpinDetected(void)
{
    return testYawSpinDetected;
}

bool failsafeIsActive(void)
{
    return false;
}

bool sensors(uint32_t mask)
{
    UNUSED(mask);
    return false;
}

float getRcDeflection(int axis)
{
    return testRcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
    return std::fabs(testRcDeflection[axis]);
}

float getMaxRcDeflectionAbs(void)
{
    return std::fmax(std::fabs(testRcDeflection[FD_ROLL]),
        std::fmax(std::fabs(testRcDeflection[FD_PITCH]), std::fabs(testRcDeflection[FD_YAW])));
}

float getCosTiltAngle(void)
{
    return 1.0f;
}

float getAutopilotThrottle(void)
{
    return testAutopilotThrottle;
}

bool mixerIsTricopter(void)
{
    return false;
}

float mixerTricopterMotorCorrection(int motorIndex)
{
    UNUSED(motorIndex);
    return 0.0f;
}

void motorWriteAll(float *motorValues)
{
    UNUSED(motorValues);
}

void delay(timeMs_t milliseconds)
{
    UNUSED(milliseconds);
}

void pidResetIterm(void)
{
}

void pidUpdateAntiGravityThrottleFilter(float normalizedThrottle)
{
    UNUSED(normalizedThrottle);
}

void pidUpdateTpaFactor(float normalizedThrottle)
{
    UNUSED(normalizedThrottle);
}

void pidUpdateAdrcAppliedOutput(const pidProfile_t *pidProfile, float axisScale, float yawSumLimit)
{
    ++adrcFeedbackCallCount;
    adrcFeedbackProfile = pidProfile;
    adrcFeedbackAxisScale = axisScale;
    adrcFeedbackYawLimit = yawSumLimit;
}

} // extern "C"
