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

// --- Vertical channel: rate-limited ramp, not a stepped altitude target ---

TEST_F(PositionNavTest, VerticalProfileSeedsTheCommandedRateBeforeAnyUpdate)
{
    const vector3_t target = {{ 0.0f, 0.0f, 30.0f }};   // 30 m above the craft
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, true, NULL, NULL);
    positionNavSetVerticalProfile(1.5f, 0.0f);

    // The altitude controller's feedforward is consumed by a task that can run before the next
    // positionNavUpdate(): it must already see the rate this leg is climbing at, and an altitude
    // target still at the craft rather than 30 m above it.
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, 150.0f, 1.0f);
    EXPECT_NEAR(positionNavGetTargetAltitudeCm(), 0.0f, 0.1f);
    EXPECT_NEAR(positionNavGetVerticalRateLimitCmS(), 150.0f, 0.1f);
}

TEST_F(PositionNavTest, VerticalProfileRampsTheAltitudeTargetAtTheCommandedRate)
{
    const vector3_t target = {{ 0.0f, 0.0f, 30.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, true, NULL, NULL);
    positionNavSetVerticalProfile(1.5f, 0.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 150.0f);
    float previousAltCm = positionNavGetTargetAltitudeCm();
    for (int i = 0; i < 20; i++) {
        est.position.z += 15.0f;                 // craft climbing at the commanded 1.5 m/s
        positionNavUpdate(0.1f, &est);
        const float altCm = positionNavGetTargetAltitudeCm();
        EXPECT_LE(altCm - previousAltCm, 15.1f); // never more than rate * dt in one cycle
        EXPECT_LE(altCm - est.position.z, 151.0f); // and never leads the craft by more than the leash
        previousAltCm = altCm;
    }
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, 150.0f, 1.0f);
}

TEST_F(PositionNavTest, ShortClimbIsNotArrivedBeforeItStarts)
{
    // A station-keeping climb leg: 2 m up, and the acceptance radius that goes with it is also 2 m.
    // Vertical arrival cannot borrow the horizontal radius - the leg would be arrived on its first
    // update, having climbed nothing, and a rescue would set off for home at the altitude it was
    // triggered at.
    const vector3_t target = {{ 0.0f, 0.0f, 2.0f }};
    positionNavSetTargetEf(&target, 5.0f, 2.0f, 0.5f, true, NULL, NULL);
    positionNavSetVerticalProfile(5.0f, 0.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.1f, &est);
    EXPECT_FALSE(positionNavTargetReached());

    est.position.z = 100.0f;   // half way up, still not there
    positionNavUpdate(0.1f, &est);
    EXPECT_FALSE(positionNavTargetReached());

    est.position.z = 200.0f;
    positionNavUpdate(0.1f, &est);
    EXPECT_TRUE(positionNavTargetReached());
}

TEST_F(PositionNavTest, CompletedLegWalksItsRampOnToTheLegAltitude)
{
    // A leg that completes before its altitude is reached (no altitude gate) must not hand alt hold
    // the leg altitude in one step, nor leave it holding wherever the ramp had got to: the ramp
    // carries on walking to the leg altitude at the leg's rate.
    const vector3_t target = {{ 0.0f, 0.0f, 10.0f }};
    positionNavSetTargetEf(&target, 5.0f, 2.0f, 1000.0f, true, NULL, NULL);
    positionNavSetAltitudeArrivalRequired(false);
    positionNavSetVerticalProfile(1.0f, 0.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f);
    positionNavUpdate(0.1f, &est);
    ASSERT_TRUE(positionNavTargetReached());
    float previousAltCm = positionNavGetTargetAltitudeCm();
    EXPECT_NEAR(previousAltCm, 10.0f, 0.5f);

    for (int i = 0; i < 150; i++) {
        est.position.z = positionNavGetTargetAltitudeCm();   // craft tracking the ramp
        positionNavUpdate(0.1f, &est);
        const float altCm = positionNavGetTargetAltitudeCm();
        EXPECT_LE(altCm - previousAltCm, 10.0f + 0.01f);    // 1 m/s over 0.1 s
        EXPECT_GE(altCm, previousAltCm - 0.01f);
        previousAltCm = altCm;
    }
    EXPECT_NEAR(positionNavGetTargetAltitudeCm(), 1000.0f, 0.1f);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().x, 0.0f, 0.01f);    // nothing horizontal to fly
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, 0.0f, 0.01f);
}

TEST_F(PositionNavTest, HandOverToAShorterLeashWalksTheRampInRatherThanSnapping)
{
    // A climbing leg let the altitude target lead a lagging craft by its own leash; the landing that
    // takes over has a much shorter one. The target must be walked back inside it, not dropped onto
    // it in a single cycle.
    const vector3_t climb = {{ 0.0f, 0.0f, 100.0f }};
    positionNavSetTargetEf(&climb, 1.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(1.5f, 21.5f);

    const vector3_t landing = {{ 0.0f, 0.0f, -180.0f }};
    positionNavSetTargetEf(&landing, 1.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(0.5f, 21.5f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 150.0f);
    float previousAltCm = positionNavGetTargetAltitudeCm();
    for (int i = 0; i < 300; i++) {
        positionNavUpdate(0.01f, &est);
        const float altCm = positionNavGetTargetAltitudeCm();
        EXPECT_LE(fabsf(altCm - previousAltCm), 1.5f + 0.5f + 0.01f);   // ramp rate plus walk, over 10 ms
        previousAltCm = altCm;
    }
    EXPECT_LE(positionNavGetTargetAltitudeCm(), 2000.0f + 100.0f + 0.01f);   // inside the 1 m leash by now
}

TEST_F(PositionNavTest, ShortClimbBrakesIntoTheLegAltitudeRatherThanLagging)
{
    // The ramp brakes into the leg altitude. Scaling the rate on the whole remaining error made
    // every climb shorter than the leg rate in metres a one-second lag - 2 m at a commanded 5 m/s
    // set off at 2 m/s and closed the last stretch asymptotically.
    const vector3_t target = {{ 0.0f, 0.0f, 2.0f }};
    positionNavSetTargetEf(&target, 5.0f, 0.5f, 0.5f, true, NULL, NULL);
    positionNavSetVerticalProfile(5.0f, 0.0f);

    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, 283.0f, 5.0f);   // sqrt(2 * 2 m/s^2 * 2 m)

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 280.0f);
    for (int i = 0; i < 12; i++) {
        est.position.z = positionNavGetTargetAltitudeCm();   // craft tracking the ramp
        positionNavUpdate(0.1f, &est);
    }
    EXPECT_NEAR(positionNavGetTargetAltitudeCm(), 200.0f, 1.0f);      // arrived inside 1.2 s
}

TEST_F(PositionNavTest, LegTakingOverSlewsIntoItsVerticalRate)
{
    // A landing taking over from a leg holding altitude: the altitude target starts where that leg
    // left it and the vertical rate slews into the descent, rather than the feedforward flipping to
    // the full descent rate in one frame.
    const vector3_t level = {{ 0.0f, 0.0f, 10.0f }};
    positionNavSetTargetEf(&level, 1.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(2.0f, 10.0f);

    const vector3_t target = {{ 0.0f, 0.0f, -190.0f }};
    positionNavSetTargetEf(&target, 1.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(2.0f, 10.0f);

    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, 0.0f, 0.01f);
    EXPECT_NEAR(positionNavGetTargetAltitudeCm(), 1000.0f, 0.01f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 950.0f, 0.0f);
    float previousRateCmS = 0.0f;
    float previousAltCm = positionNavGetTargetAltitudeCm();
    for (int i = 0; i < 150; i++) {
        positionNavUpdate(0.01f, &est);
        const float rateCmS = positionNavGetTargetVelocityCmS().z;
        const float altCm = positionNavGetTargetAltitudeCm();
        EXPECT_LE(fabsf(rateCmS - previousRateCmS), 2.0f + 0.01f);   // 2 m/s^2 over 10 ms
        EXPECT_LE(altCm, previousAltCm + 0.01f);
        EXPECT_GE(altCm, previousAltCm - 2.0f - 0.01f);
        previousRateCmS = rateCmS;
        previousAltCm = altCm;
    }
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, -200.0f, 0.5f);
}

TEST_F(PositionNavTest, LegTakingOverFromAClimbTurnsTheRateRound)
{
    // Handed over while still climbing to a leg below: the ramp carries on up while the rate turns
    // round, instead of the feedforward flipping from climb to descent in one frame or the ramp
    // being clamped onto a target 200 m below the moment it is seen to be past it.
    const vector3_t climb = {{ 0.0f, 0.0f, 100.0f }};
    positionNavSetTargetEf(&climb, 1.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(1.0f, 10.0f);

    const vector3_t target = {{ 0.0f, 0.0f, -190.0f }};
    positionNavSetTargetEf(&target, 1.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(2.0f, 10.0f);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, 100.0f, 0.01f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 100.0f);
    positionNavUpdate(0.01f, &est);
    EXPECT_NEAR(positionNavGetTargetAltitudeCm(), 1001.0f, 0.1f);
    EXPECT_GT(positionNavGetTargetVelocityCmS().z, 90.0f);
}

TEST_F(PositionNavTest, RampLandingOnTheLegAltitudeStopsItsRateThere)
{
    // Handed over climbing at 5 m/s to a leg only 1 m above the ramp: the slew brakes the ramp onto
    // the leg altitude, harder than its usual rate as it has to, and the rate the feedforward reports
    // comes down with it and stops there rather than arriving at speed or carrying on decaying
    // against a target that no longer moves.
    const vector3_t climb = {{ 0.0f, 0.0f, 60.0f }};
    positionNavSetTargetEf(&climb, 5.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(5.0f, 30.0f);
    ASSERT_NEAR(positionNavGetTargetVelocityCmS().z, 500.0f, 0.01f);

    const vector3_t next = {{ 0.0f, 0.0f, 31.0f }};
    positionNavSetTargetEf(&next, 5.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(5.0f, 30.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 2900.0f, 500.0f);
    float previousAltCm = positionNavGetTargetAltitudeCm();
    float previousRateCmS = positionNavGetTargetVelocityCmS().z;
    for (int i = 0; i < 100; i++) {
        positionNavUpdate(0.01f, &est);
        const float altCm = positionNavGetTargetAltitudeCm();
        const float rateCmS = positionNavGetTargetVelocityCmS().z;
        EXPECT_NEAR(rateCmS, (altCm - previousAltCm) / 0.01f, 0.5f) << "at " << i;
        EXPECT_LE(altCm, 3100.0f);
        EXPECT_LE(fabsf(rateCmS - previousRateCmS), 30.0f) << "at " << i;   // 5 m/s shed over 1 m
        previousAltCm = altCm;
        previousRateCmS = rateCmS;
    }
    EXPECT_NEAR(positionNavGetTargetAltitudeCm(), 3100.0f, 0.01f);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, 0.0f, 0.01f);
}

TEST_F(PositionNavTest, ApproachSlowdownTapersFromTheStatedRange)
{
    // Bleed speed from a stated range instead of waiting for the position gain to bite: linear in
    // distance, so the speed decays exponentially in time, which is the shape the legacy rescue
    // flew home on.
    const vector3_t target = {{ 0.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);
    positionNavSetApproachSlowdown(20.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 10000.0f, 0.0f, 0.0f);   // 100 m out
    positionNavUpdate(0.1f, &est);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, -500.0f, 1.0f);       // outside the range: cruise

    est.position.y = 1000.0f;                                             // 10 m: half the range
    positionNavUpdate(0.1f, &est);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, -250.0f, 1.0f);

    est.position.y = 200.0f;                                              // 2 m
    positionNavUpdate(0.1f, &est);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, -50.0f, 1.0f);
}

TEST_F(PositionNavTest, NewTargetDoesNotNotchTheCommandedVelocity)
{
    // A leg change used to zero the commanded velocity until the next update, and the position
    // controller answers a one-cycle notch with a pitch jerk. The craft is still moving and the
    // next target is in much the same direction: hold the command until it is recomputed.
    const vector3_t target = {{ 0.0f, 100.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, NULL, NULL);
    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 500.0f);
    positionNavUpdate(0.1f, &est);
    const float flying = positionNavGetTargetVelocityCmS().y;
    ASSERT_GT(flying, 100.0f);

    const vector3_t next = {{ 0.0f, 200.0f, 0.0f }};
    positionNavSetTargetEf(&next, 5.0f, 1.0f, 0.5f, false, NULL, NULL);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, flying, 0.01f);
}

static const vector3_t handoverTarget = {{ 0.0f, 5000.0f, 0.0f }};

static void handOverCallback(void *userData)
{
    UNUSED(userData);
    callbackCount++;
    positionNavSetTargetEf(&handoverTarget, 5.0f, 1.0f, 0.5f, false, NULL, NULL);
}

TEST_F(PositionNavTest, CallbackHandOverDoesNotNotchTheCommandedVelocity)
{
    // The leg completes and its callback issues the next one in the same update. The velocity this
    // cycle computed stands until the next update computes the new leg's, rather than the target
    // dropping to zero for a cycle between them.
    const vector3_t target = {{ 0.0f, 150.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 2.0f, 10.0f, false, handOverCallback, NULL);
    positionEstimate3d_t est = makeEstimate(0.0f, 14900.0f, 0.0f, 100.0f);   // 1 m short, inside the radius

    positionNavUpdate(0.01f, &est);
    ASSERT_EQ(callbackCount, 1);
    EXPECT_TRUE(positionNavHasActiveTarget());
    EXPECT_FALSE(positionNavTargetReached());
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, 100.0f, 1.0f);    // POS_TO_VEL_KP * 1 m

    positionNavUpdate(0.01f, &est);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, 500.0f, 1.0f);    // now the next leg's cruise
}

TEST_F(PositionNavTest, CompletionWithoutAHandOverStillStops)
{
    const vector3_t target = {{ 0.0f, 150.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 2.0f, 10.0f, false, testCallback, NULL);
    positionEstimate3d_t est = makeEstimate(0.0f, 14900.0f, 0.0f, 100.0f);

    positionNavUpdate(0.01f, &est);
    ASSERT_EQ(callbackCount, 1);
    EXPECT_TRUE(positionNavTargetReached());
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, 0.0f, 0.01f);
}

TEST_F(PositionNavTest, VerticalProfileGovernsDescentRateToADeepTarget)
{
    // The landing target sits far below ground so vertical arrival never triggers; the descent is
    // governed by the leg's rate, not by how deep that target is.
    const vector3_t target = {{ 0.0f, 0.0f, -200.0f }};
    positionNavSetTargetEf(&target, 1.0f, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetVerticalProfile(1.0f, 50.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f, 5000.0f, -100.0f);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, -100.0f, 1.0f);
    for (int i = 0; i < 10; i++) {
        est.position.z -= 10.0f;
        positionNavUpdate(0.1f, &est);
        EXPECT_NEAR(positionNavGetTargetVelocityCmS().z, -100.0f, 1.0f);
        EXPECT_LE(est.position.z - positionNavGetTargetAltitudeCm(), 101.0f);
    }
}

TEST_F(PositionNavTest, ClimbDoesNotRobTheHorizontalCruiseSpeed)
{
    // Horizontal and vertical are separate budgets: a steep leg flies its cruise speed and climbs
    // at its own rate, instead of splitting one 3D speed between the two.
    const vector3_t target = {{ 100.0f, 0.0f, 50.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, true, NULL, NULL);
    positionNavSetVerticalProfile(2.0f, 0.0f);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.1f, &est);

    const vector3_t vel = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(vel.x, 500.0f, 1.0f);
    EXPECT_NEAR(vel.z, 200.0f, 1.0f);
}

// --- Velocity feedforward: a target flown at the velocity its owner states ---

TEST_F(PositionNavTest, VelocityFeedforwardIsTheCommandedVelocityWhateverTheGap)
{
    // A carrot 3 m ahead of a craft flying behind it: the command is the carrot's own velocity,
    // not the chase law on the gap (which would ask for 3 m/s against the carrot's 4).
    const vector3_t carrot = {{ 0.0f, 3.0f, 0.0f }};
    positionNavSetTargetEf(&carrot, 10.0f, -1.0f, 1000.0f, false, NULL, NULL);
    positionNavSetApproachSlowdown(20.0f);
    positionNavSetAccelLimits(0.0f, 0.3f);
    const vector2_t carrotVelMps = {{ 0.0f, 4.0f }};
    positionNavSetVelocityFeedforward(&carrotVelMps);

    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 400.0f);
    for (int i = 0; i < 5; i++) {
        positionNavUpdate(0.01f, &est);
        EXPECT_NEAR(positionNavGetTargetVelocityCmS().x, 0.0f, 0.01f);
        EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, 400.0f, 0.01f);
    }
}

TEST_F(PositionNavTest, VelocityFeedforwardIsFlownAsItStandsAcrossAHandOver)
{
    // Handed over from a leg flying north to one stating east: the stated velocity is the command
    // from the first cycle, acceleration limit or not. Its owner shapes how it changes.
    const vector3_t first = {{ 0.0f, 100.0f, 0.0f }};
    positionNavSetTargetEf(&first, 4.0f, -1.0f, 1000.0f, false, NULL, NULL);
    const vector2_t northMps = {{ 0.0f, 4.0f }};
    positionNavSetVelocityFeedforward(&northMps);
    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 400.0f);
    positionNavUpdate(0.01f, &est);
    ASSERT_NEAR(positionNavGetTargetVelocityCmS().y, 400.0f, 0.01f);

    const vector3_t second = {{ 100.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&second, 4.0f, -1.0f, 1000.0f, false, NULL, NULL);
    positionNavSetAccelLimits(2.5f, 0.0f);
    const vector2_t eastMps = {{ 4.0f, 0.0f }};
    positionNavSetVelocityFeedforward(&eastMps);
    positionNavUpdate(0.01f, &est);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().x, 400.0f, 0.01f);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, 0.0f, 0.01f);
}

TEST_F(PositionNavTest, VelocityFeedforwardWalksItsTarget)
{
    // Between its owner's moves the target walks at the stated velocity, at the rate positionNav
    // runs, and a move puts it wherever the owner says.
    const vector3_t carrot = {{ 1.0f, 2.0f, 0.0f }};
    positionNavSetTargetEf(&carrot, 5.0f, -1.0f, 1000.0f, false, NULL, NULL);
    const vector2_t velMps = {{ 3.0f, -4.0f }};
    positionNavSetVelocityFeedforward(&velMps);
    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 5; i++) {
        positionNavUpdate(0.01f, &est);
    }
    EXPECT_NEAR(positionNavGetActiveCommand()->targetPosEfM.x, 1.15f, 0.001f);
    EXPECT_NEAR(positionNavGetActiveCommand()->targetPosEfM.y, 1.80f, 0.001f);

    const vector3_t moved = {{ 5.0f, 5.0f, 0.0f }};
    positionNavMoveTargetEf(&moved);
    positionNavUpdate(0.01f, &est);
    EXPECT_NEAR(positionNavGetActiveCommand()->targetPosEfM.x, 5.03f, 0.001f);
    EXPECT_NEAR(positionNavGetActiveCommand()->targetPosEfM.y, 4.96f, 0.001f);
}

TEST_F(PositionNavTest, NewTargetClearsTheVelocityFeedforward)
{
    const vector3_t carrot = {{ 0.0f, 50.0f, 0.0f }};
    positionNavSetTargetEf(&carrot, 5.0f, -1.0f, 1000.0f, false, NULL, NULL);
    const vector2_t velMps = {{ 3.0f, 0.0f }};
    positionNavSetVelocityFeedforward(&velMps);
    EXPECT_TRUE(positionNavGetActiveCommand()->velocityFfValid);

    positionNavSetTargetEf(&carrot, 5.0f, 1.0f, 0.5f, false, NULL, NULL);
    EXPECT_FALSE(positionNavGetActiveCommand()->velocityFfValid);
    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    positionNavUpdate(0.01f, &est);
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().x, 0.0f, 0.01f);     // back on the chase law, north
    EXPECT_NEAR(positionNavGetTargetVelocityCmS().y, 500.0f, 0.01f);

    positionNavReset();
    positionNavSetVelocityFeedforward(&velMps);                        // no command: nothing to state it on
    EXPECT_FALSE(positionNavGetActiveCommand()->velocityFfValid);
}

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

// --- Move target ---

TEST_F(PositionNavTest, MoveTargetPreservesRampAndRedirectsVelocity)
{
    const vector3_t target = {{ 10.0f, 0.0f, 0.0f }};
    positionNavSetTargetEf(&target, 5.0f, 1.0f, 0.5f, false, testCallback, NULL);
    positionNavSetAccelLimits(1.0f, 0.0f);

    // Build up a ramped velocity toward the first target.
    positionEstimate3d_t est = makeEstimate(0.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 100; i++) {
        positionNavUpdate(0.01f, &est);
    }
    const vector3_t velBefore = positionNavGetTargetVelocityCmS();
    EXPECT_NEAR(velBefore.x, 100.0f, 5.0f);  // ~1 m/s after 1 s at 1 m/s^2

    // Move the target north: the very next update must not restart the ramp
    // from zero — the accel limit only bends the existing velocity.
    const vector3_t moved = {{ 0.0f, 10.0f, 0.0f }};
    positionNavMoveTargetEf(&moved);
    EXPECT_TRUE(positionNavHasActiveTarget());
    EXPECT_NEAR(positionNavGetActiveCommand()->targetPosEfM.y, 10.0f, 0.001f);

    positionNavUpdate(0.01f, &est);
    const vector3_t velAfter = positionNavGetTargetVelocityCmS();
    EXPECT_GT(vector3Norm(&velAfter), 90.0f);
    EXPECT_EQ(callbackCount, 0);
}

TEST_F(PositionNavTest, MoveTargetWithoutActiveCommandIsNoOp)
{
    const vector3_t moved = {{ 5.0f, 5.0f, 0.0f }};
    positionNavMoveTargetEf(&moved);
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
