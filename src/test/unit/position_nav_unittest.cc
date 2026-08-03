/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static positionEstimate3d_t makeEstimate(float eastCm, float northCm, float velEastCmS, float velNorthCmS, float upCm = 0.0f, float velUpCmS = 0.0f)
{
    positionEstimate3d_t est;
    memset(&est, 0, sizeof(est));
    est.position.x = eastCm;
    est.position.y = northCm;
    est.position.z = upCm;
    est.velocity.x = velEastCmS;
    est.velocity.y = velNorthCmS;
    est.velocity.z = velUpCmS;
    est.isValidXY = true;
    est.trustXY = 1.0f;
    return est;
}

static int callbackCount;
static void *lastCallbackUserData;

static void testCallback(void *userData)
{
    callbackCount++;
    lastCallbackUserData = userData;
}

class PositionNavTest : public ::testing::Test {
protected:
    void SetUp() override {
        positionNavInit();
        callbackCount = 0;
        lastCallbackUserData = NULL;
    }
};

// --- Direction correctness ---

TEST_F(PositionNavTest, EastTargetProducesEastwardVelocity)
{
    const vector3_t target = {{ 10.0f, 0.0f, 0.0f }};  // 10m east
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_GT(vel.x, 0.0f);
    EXPECT_NEAR(vel.y, 0.0f, 0.01f);
}

TEST_F(PositionNavTest, NorthTargetProducesNorthwardVelocity)
{
    const vector3_t target = {{ 0.0f, 10.0f, 0.0f }};  // 10m north
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(vel.x, 0.0f, 0.01f);
    EXPECT_GT(vel.y, 0.0f);
}

TEST_F(PositionNavTest, DiagonalTargetProducesBothAxes)
{
    const vector3_t target = {{ 10.0f, 10.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_GT(vel.x, 0.0f);
    EXPECT_GT(vel.y, 0.0f);
    EXPECT_NEAR(fabsf(vel.x), fabsf(vel.y), 1.0f);
}

TEST_F(PositionNavTest, UpTargetProducesUpwardVelocityWhenIncludeAltitude)
{
    const vector3_t target = {{ 0.0f, 0.0f, 10.0f }};  // 10 m up
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, true, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_GT(vel.z, 0.0f);
    EXPECT_NEAR(vel.x, 0.0f, 1.0f);
    EXPECT_NEAR(vel.y, 0.0f, 1.0f);
}

TEST_F(PositionNavTest, UpComponentIgnoredWhenIncludeAltitudeFalse)
{
    const vector3_t target = {{ 0.0f, 0.0f, 10.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(vel.z, 0.0f, 0.01f);
}

// --- Speed limiting ---

TEST_F(PositionNavTest, SpeedIsCappedAtCruiseSpeed)
{
    const vector3_t target = {{ 1000.0f, 0.0f, 0.0f }};  // 1km east
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    const float speedCmS = sqrtf(vel.x * vel.x + vel.y * vel.y);
    EXPECT_LE(speedCmS, 5.0f * 100.0f + 1.0f);
}

TEST_F(PositionNavTest, SpeedRampsDownNearTarget)
{
    const vector3_t target = {{ 0.5f, 0.0f, 0.0f }};  // 0.5m east (close)
    // acceptanceRadius must be < 0.5m or the first update marks arrived (error inside zone)
    positionNavSetTargetEf(&target, 10.0f, 0.1f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    const float speedCmS = sqrtf(vel.x * vel.x + vel.y * vel.y);
    // At 0.5m with Kp=1.0, desired speed = 0.5 m/s = 50 cm/s (much less than cruise)
    EXPECT_LT(speedCmS, 10.0f * 100.0f);
    EXPECT_NEAR(speedCmS, 50.0f, 5.0f);
}

// --- Braking behaviour ---

TEST_F(PositionNavTest, BrakingLimitsSpeedNearTarget)
{
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};  // 1m east
    positionNavSetTargetEf(&target, 10.0f, 0.5f, 0.3f, false, NULL, NULL);
    positionNavSetAccelLimits(0.0f, 2.0f);  // decel = 2 m/s^2

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    const float speedCmS = sqrtf(vel.x * vel.x + vel.y * vel.y);
    // braking speed = sqrt(2 * 2.0 * 1.0) = 2.0 m/s = 200 cm/s
    // Kp speed = 1.0 * 1.0 = 1.0 m/s = 100 cm/s
    // min(10, 1.0, 2.0) = 1.0 → 100 cm/s
    EXPECT_NEAR(speedCmS, 100.0f, 5.0f);
}

TEST_F(PositionNavTest, BrakingDominatesWhenVeryClose)
{
    const vector3_t target = {{ 0.1f, 0.0f, 0.0f }};  // 0.1m east
    positionNavSetTargetEf(&target, 10.0f, 0.05f, 0.3f, false, NULL, NULL);
    positionNavSetAccelLimits(0.0f, 2.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    const float speedCmS = sqrtf(vel.x * vel.x + vel.y * vel.y);
    // braking speed = sqrt(2 * 2.0 * 0.1) = 0.632 m/s = 63.2 cm/s
    // Kp speed = 1.0 * 0.1 = 0.1 m/s = 10 cm/s
    // min(10, 0.1, 0.632) → 0.1 m/s = 10 cm/s
    EXPECT_NEAR(speedCmS, 10.0f, 2.0f);
}

// --- Arrival detection ---

TEST_F(PositionNavTest, ArrivalDetectedWhenWithinRadiusAndSlow)
{
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    EXPECT_FALSE(positionNavTargetReached());

    // Simulate craft at target position, slow velocity
    positionEstimate3d_t est = makeEstimate(100.0f, 0.0f, 10.0f, 0.0f);  // 1m east, 0.1 m/s
    positionNavUpdate(0.01f, &est);

    EXPECT_TRUE(positionNavTargetReached());
}

TEST_F(PositionNavTest, NoArrivalWhenFar)
{
    const vector3_t target = {{ 10.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    EXPECT_FALSE(positionNavTargetReached());
}

TEST_F(PositionNavTest, NoArrivalWhenCloseButFast)
{
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    // At target position but moving 2 m/s (above completionSpeedMps)
    positionEstimate3d_t est = makeEstimate(100.0f, 0.0f, 200.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    EXPECT_FALSE(positionNavTargetReached());
}

// --- Callback fires once ---

TEST_F(PositionNavTest, CallbackFiresExactlyOnce)
{
    int flag = 42;
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, testCallback, &flag);

    EXPECT_EQ(callbackCount, 0);

    // Arrive
    positionEstimate3d_t est = makeEstimate(100.0f, 0.0f, 10.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    EXPECT_EQ(callbackCount, 1);
    EXPECT_EQ(lastCallbackUserData, &flag);

    // Update again — callback must not fire again
    positionNavUpdate(0.01f, &est);
    EXPECT_EQ(callbackCount, 1);
}

TEST_F(PositionNavTest, NoCallbackWhenNoneSet)
{
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(100.0f, 0.0f, 10.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    EXPECT_TRUE(positionNavTargetReached());
}

// --- Zero-distance safety ---

TEST_F(PositionNavTest, ZeroDistanceProducesZeroVelocity)
{
    const vector3_t target = {{ 0.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(vel.x, 0.0f, 0.01f);
    EXPECT_NEAR(vel.y, 0.0f, 0.01f);
    EXPECT_FALSE(isnan(vel.x));
    EXPECT_FALSE(isnan(vel.y));
}

// --- Hysteresis ---

TEST_F(PositionNavTest, HysteresisPreventsPrematureExit)
{
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 5.0f, false, NULL, NULL);  // generous completionSpeed

    // Enter acceptance radius
    positionEstimate3d_t est = makeEstimate(100.0f, 0.0f, 10.0f, 0.0f);
    positionNavUpdate(0.01f, &est);
    EXPECT_TRUE(positionNavTargetReached());

    // Now set a new target and move slightly outside acceptance (within hysteresis band)
    const vector3_t target2 = {{ 5.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target2, 5.0f, 1.0f, 5.0f, false, NULL, NULL);

    // 1.2m from target (within 1.5x acceptance radius)
    est = makeEstimate(380.0f, 0.0f, 10.0f, 0.0f);
    positionNavUpdate(0.01f, &est);
    // Should enter acceptance zone since 1.2 > 1.0 (not yet in zone)
    EXPECT_FALSE(positionNavTargetReached());

    // Move inside acceptance
    est = makeEstimate(490.0f, 0.0f, 10.0f, 0.0f);
    positionNavUpdate(0.01f, &est);
    EXPECT_TRUE(positionNavTargetReached());
}

// --- Auto-clear on reach ---

TEST_F(PositionNavTest, AutoClearDeactivatesOnReach)
{
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);
    positionNavSetAutoClearOnReach(true);

    EXPECT_TRUE(positionNavHasActiveTarget());

    positionEstimate3d_t est = makeEstimate(100.0f, 0.0f, 10.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    EXPECT_FALSE(positionNavHasActiveTarget());
}

// --- Clear target ---

TEST_F(PositionNavTest, ClearTargetDeactivates)
{
    const vector3_t target = {{ 10.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    EXPECT_TRUE(positionNavHasActiveTarget());
    positionNavClearTarget();
    EXPECT_FALSE(positionNavHasActiveTarget());
}

TEST_F(PositionNavTest, ClearTargetZerosVelocity)
{
    const vector3_t target = {{ 10.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t velBefore = positionNavGetTargetVelocityCmS();
    EXPECT_GT(fabsf(velBefore.x), 0.0f);

    positionNavClearTarget();

    const vector3_t velAfter = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(velAfter.x, 0.0f, 0.01f);
    EXPECT_NEAR(velAfter.y, 0.0f, 0.01f);
}

// --- Completed target produces zero velocity ---

TEST_F(PositionNavTest, CompletedTargetOutputsZeroVelocity)
{
    const vector3_t target = {{ 1.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    // Arrive
    positionEstimate3d_t est = makeEstimate(100.0f, 0.0f, 10.0f, 0.0f);
    positionNavUpdate(0.01f, &est);
    EXPECT_TRUE(positionNavTargetReached());

    // Subsequent update should output zero velocity
    positionNavUpdate(0.01f, &est);
    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(vel.x, 0.0f, 0.01f);
    EXPECT_NEAR(vel.y, 0.0f, 0.01f);
}

// --- Acceleration limiting ---

TEST_F(PositionNavTest, AccelerationLimitingClampsVelocityChange)
{
    const vector3_t target = {{ 100.0f, 0.0f, 0.0f }};  // far target
    positionNavSetTargetEf(&target, 10.0f, 1.0f, 0.5f, false, NULL, NULL);
    positionNavSetAccelLimits(1.0f, 0.0f);  // 1 m/s^2 accel limit

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    const float dt = 0.01f;

    // First update: from 0 velocity, limited to maxAccel * dt = 1.0 * 0.01 = 0.01 m/s = 1 cm/s
    positionNavUpdate(dt, &est);
    const vector3_t vel1 = positionNavGetTargetVelocityCmS();
    EXPECT_LE(fabsf(vel1.x), 1.5f);

    // Second update: can increase by another 1 cm/s
    positionNavUpdate(dt, &est);
    const vector3_t vel2 = positionNavGetTargetVelocityCmS();
    EXPECT_LE(fabsf(vel2.x), 3.0f);
    EXPECT_GT(fabsf(vel2.x), fabsf(vel1.x) - 0.01f);
}

// --- No active target produces zero velocity ---

TEST_F(PositionNavTest, NoActiveTargetProducesZeroVelocity)
{
    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 50.0f, 50.0f);
    positionNavUpdate(0.01f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(vel.x, 0.0f, 0.01f);
    EXPECT_NEAR(vel.y, 0.0f, 0.01f);
}

// --- Active flag and reached flag ---

TEST_F(PositionNavTest, ActiveAndReachedFlagsAreCorrect)
{
    EXPECT_FALSE(positionNavHasActiveTarget());
    EXPECT_FALSE(positionNavTargetReached());

    const vector3_t target = {{ 10.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);

    EXPECT_TRUE(positionNavHasActiveTarget());
    EXPECT_FALSE(positionNavTargetReached());

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);

    EXPECT_TRUE(positionNavHasActiveTarget());
    EXPECT_FALSE(positionNavTargetReached());
}
