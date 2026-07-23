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

#include "gtest/gtest.h"

extern "C" {
#include "common/axis.h"

#include "fc/runtime_config.h"

#include "flight/adrc.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "sensors/gyro.h"
}

// The focused target links pid_unittest.cc to reuse its established Betaflight platform stubs.
extern float simulatedThrottle;
extern float simulatedSetpointRate[XYZ_AXIS_COUNT];
extern pidProfile_t *pidProfile;

void resetTest(void);
timeUs_t currentTestTime(void);

namespace {

constexpr float TEST_AUTHORITY_SCALE = 1.0f;
constexpr float TEST_GROUND_SETPOINT_DPS = 300.0f;

void prepareAdrcFeedbackLoop(void)
{
    resetTest();

    pidProfile->pid_type = PID_TYPE_ADRC;
    pidInitConfig(pidProfile);

    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    simulatedThrottle = 0.0f;
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        simulatedSetpointRate[axis] = 0.0f;
        gyro.gyroADCf[axis] = 0.0f;
        adrcResetState(&pidRuntime.adrc, axis);
    }
    adrcResetGate(&pidRuntime.adrc);
}

float publishGroundConstrainedCommand(void)
{
    simulatedSetpointRate[FD_ROLL] = TEST_GROUND_SETPOINT_DPS;
    pidController(pidProfile, currentTestTime());
    EXPECT_FALSE(pidRuntime.adrc.liftoff);
    EXPECT_GT(std::fabs(pidData[FD_ROLL].Sum), 1.0f);

    pidUpdateAdrcAppliedOutput(pidProfile, TEST_AUTHORITY_SCALE, pidProfile->pidSumLimitYaw);
    const float staleOutput = pidRuntime.adrc.lastOutput[FD_ROLL];
    EXPECT_GT(std::fabs(staleOutput), 1.0f);
    return staleOutput;
}

void isolateGroundEpochFeedback(const float staleOutput)
{
    simulatedSetpointRate[FD_ROLL] = 0.0f;
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        adrcResetState(&pidRuntime.adrc, axis);
    }
    pidRuntime.adrc.lastOutput[FD_ROLL] = staleOutput;
}

void expectNeutralOpenAndNextObserverLoop(void)
{
    simulatedThrottle = 0.6f;
    pidController(pidProfile, currentTestTime());

    ASSERT_TRUE(pidRuntime.adrc.liftoff);
    EXPECT_FLOAT_EQ(0.0f, pidRuntime.adrc.lastOutput[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, pidRuntime.adrc.z2[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].Sum);

    // This is the real mixer-to-observer handoff. mixTable() computes the authority scale and
    // calls this function after applying its constraints; the observer consumes it one loop later.
    pidUpdateAdrcAppliedOutput(pidProfile, TEST_AUTHORITY_SCALE, pidProfile->pidSumLimitYaw);
    EXPECT_FLOAT_EQ(0.0f, pidRuntime.adrc.lastOutput[FD_ROLL]);

    pidController(pidProfile, currentTestTime());
    EXPECT_TRUE(pidRuntime.adrc.liftoff);
    EXPECT_FLOAT_EQ(0.0f, pidRuntime.adrc.z2[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0.0f, pidData[FD_ROLL].Sum);
}

TEST(AdrcGateE2eTest, FirstOpenDropsGroundMixerFeedbackBeforeObserverUsesIt)
{
    prepareAdrcFeedbackLoop();

    const float staleOutput = publishGroundConstrainedCommand();
    isolateGroundEpochFeedback(staleOutput);
    expectNeutralOpenAndNextObserverLoop();
}

} // namespace
