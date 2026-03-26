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
#include <stdbool.h>
#include <limits.h>
#include <math.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "common/maths.h"
    #include "common/vector.h"

    #include "flight/autopilot_guidance.h"

    #include "pg/autopilot.h"
    #include "pg/rx.h"

    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t millisRW;
uint32_t millis() {
    return millisRW;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void setupL1Config(void)
{
    autopilotConfig_t *cfg = autopilotConfigMutable();
    cfg->l1Period = 20;              // 2.0 seconds
    cfg->l1MinLookahead = 1000;      // 10m
    cfg->l1MaxLookahead = 10000;     // 100m
    cfg->l1MaxCrossTrackError = 10000; // 100m
    cfg->l1TurnRate = 8;             // 8 deg/s arc transition
}

static const float DT = 0.1f;       // 10 Hz GPS rate for tests

// Tolerance for float comparisons (cm-level precision)
static const float TOLERANCE = 1.0f;

// =========================================================================
// Init / Reset
// =========================================================================

TEST(AutopilotGuidanceUnittest, InitializationTest)
{
    l1GuidanceInit();

    EXPECT_FALSE(l1GuidanceIsActive());
    EXPECT_FLOAT_EQ(l1GuidanceGetCrossTrackError(), 0.0f);
    EXPECT_FLOAT_EQ(l1GuidanceGetAlongTrackDistance(), 0.0f);
    EXPECT_FLOAT_EQ(l1GuidanceGetLookaheadDistance(), 0.0f);
}

TEST(AutopilotGuidanceUnittest, ResetClearsState)
{
    l1GuidanceInit();
    l1GuidanceSetActive(true);

    l1GuidanceReset();

    EXPECT_FALSE(l1GuidanceIsActive());
    EXPECT_FLOAT_EQ(l1GuidanceGetCrossTrackError(), 0.0f);
    EXPECT_FLOAT_EQ(l1GuidanceGetAlongTrackDistance(), 0.0f);
    EXPECT_FLOAT_EQ(l1GuidanceGetLookaheadDistance(), 0.0f);
}

// =========================================================================
// SetPath
// =========================================================================

TEST(AutopilotGuidanceUnittest, SetPathBasic)
{
    l1GuidanceInit();

    // Horizontal path: 1000cm (10m) along X axis
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 1000.0f, .y = 0.0f };

    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // After SetPath, the path should be active-ready
    EXPECT_TRUE(l1GuidanceIsActive());
}

TEST(AutopilotGuidanceUnittest, SetPathTooShort)
{
    l1GuidanceInit();

    // Path shorter than L1_MIN_PATH_LENGTH_CM (10cm)
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 5.0f, .y = 0.0f };

    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Update should return immediately for short path (no state change)
    setupL1Config();
    vector2_t pos = { .x = 0.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Cross-track error should remain 0 (update skipped)
    EXPECT_FLOAT_EQ(l1GuidanceGetCrossTrackError(), 0.0f);
}

TEST(AutopilotGuidanceUnittest, SetPathDiagonal)
{
    l1GuidanceInit();

    // 45-degree path: (0,0) → (1000,1000)
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 1000.0f, .y = 1000.0f };

    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    setupL1Config();

    // Position on the path at the midpoint
    vector2_t pos = { .x = 500.0f, .y = 500.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Cross-track error should be ~0 for point on path
    EXPECT_NEAR(l1GuidanceGetCrossTrackError(), 0.0f, TOLERANCE);
}

// =========================================================================
// Cross-Track Error sign convention
// =========================================================================

TEST(AutopilotGuidanceUnittest, CrossTrackErrorRightOfPath)
{
    // Path along X axis (North), position to the East (positive Y) → right of path
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 10000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position 500cm to the right (East) of path
    vector2_t pos = { .x = 5000.0f, .y = 500.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Cross-track error should be positive (right of path)
    EXPECT_GT(l1GuidanceGetCrossTrackError(), 0.0f);
    EXPECT_NEAR(l1GuidanceGetCrossTrackError(), 500.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, CrossTrackErrorLeftOfPath)
{
    // Path along X axis (North), position to the West (negative Y) → left of path
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 10000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position 300cm to the left (West) of path
    vector2_t pos = { .x = 5000.0f, .y = -300.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Cross-track error should be negative (left of path)
    EXPECT_LT(l1GuidanceGetCrossTrackError(), 0.0f);
    EXPECT_NEAR(l1GuidanceGetCrossTrackError(), -300.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, CrossTrackErrorOnPath)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 10000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position exactly on the path
    vector2_t pos = { .x = 3000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    EXPECT_NEAR(l1GuidanceGetCrossTrackError(), 0.0f, TOLERANCE);
}

// =========================================================================
// Carrot point on straight path
// =========================================================================

TEST(AutopilotGuidanceUnittest, CarrotPointOnStraightPath)
{
    l1GuidanceInit();
    setupL1Config();

    // Long path along X axis
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position at x=5000 on path
    vector2_t pos = { .x = 5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    const vector2_t *carrot = l1GuidanceGetCarrot();

    // Carrot should be ahead on the path
    EXPECT_GT(carrot->x, 5000.0f);
    // Carrot should be on path (y ≈ 0)
    EXPECT_NEAR(carrot->y, 0.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, CarrotPointClampedToEnd)
{
    l1GuidanceInit();
    setupL1Config();

    // Short path (200cm) along X axis
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 200.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position near the start with min lookahead (1000cm) >> path length (200cm).
    // Carrot clamps to the endpoint so the position PID converges on the waypoint.
    vector2_t pos = { .x = 50.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    const vector2_t *carrot = l1GuidanceGetCarrot();

    // Carrot should be clamped to end of path
    EXPECT_NEAR(carrot->x, 200.0f, TOLERANCE);
    EXPECT_NEAR(carrot->y, 0.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, CarrotPointClampedToStart)
{
    l1GuidanceInit();
    setupL1Config();

    // Long path along X
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position BEHIND the path start
    // alongTrack = dot(pos-start, dir) = -5000 (negative)
    // carrotAlongTrack = -5000 + lookahead ≈ -5000 + 1000 = -4000 → clamped to 0
    vector2_t pos = { .x = -5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    const vector2_t *carrot = l1GuidanceGetCarrot();

    // Carrot should be clamped to start of path
    EXPECT_NEAR(carrot->x, 0.0f, TOLERANCE);
    EXPECT_NEAR(carrot->y, 0.0f, TOLERANCE);
}

// =========================================================================
// Lookahead clamping
// =========================================================================

TEST(AutopilotGuidanceUnittest, LookaheadMinClamping)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Very low speed → lookahead = V * T / (2π) = 10 * 2.0 / 6.28 ≈ 3.2 cm
    // Should be clamped to l1MinLookahead = 1000 cm
    vector2_t pos = { .x = 5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 10.0f, DT);

    EXPECT_NEAR(l1GuidanceGetLookaheadDistance(), 1000.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, LookaheadMaxClamping)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 500000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Very high speed → lookahead = 100000 * 2.0 / 6.28 ≈ 31847 cm
    // Should be clamped to l1MaxLookahead = 10000 cm
    vector2_t pos = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 100000.0f, DT);

    EXPECT_NEAR(l1GuidanceGetLookaheadDistance(), 10000.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, LookaheadFormulaVerification)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 500000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // L1 = V * T / (2π)
    // V = 5000 cm/s, T = 2.0s
    // Expected = 5000 * 2.0 / (2 * π) ≈ 1591.5 cm
    // This should be within [1000, 10000] → not clamped
    vector2_t pos = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 5000.0f, DT);

    float expected = 5000.0f * 2.0f / (2.0f * M_PIf);
    EXPECT_NEAR(l1GuidanceGetLookaheadDistance(), expected, 1.0f);
}

// =========================================================================
// Update lifecycle
// =========================================================================

TEST(AutopilotGuidanceUnittest, UpdateReturnsWhenInactive)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 10000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    // NOT setting active

    vector2_t pos = { .x = 5000.0f, .y = 500.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Cross-track error should remain 0 (update skipped)
    EXPECT_FLOAT_EQ(l1GuidanceGetCrossTrackError(), 0.0f);
}

TEST(AutopilotGuidanceUnittest, UpdateStaysActiveWithLargeCrossTrackUsingIntercept)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position 15000cm to the right of path (large XTE)
    vector2_t pos = { .x = 25000.0f, .y = 15000.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // L1 should remain active — intercept logic handles large XTE
    EXPECT_TRUE(l1GuidanceIsActive());

    // Carrot should be ahead on path, creating a shallow intercept angle (~30°)
    // interceptLookahead = 15000 / tan(30°) ≈ 25981cm
    // carrotAlongTrack ≈ 25000 + 25981 ≈ 50000 (clamped to path end)
    const vector2_t *carrot = l1GuidanceGetCarrot();
    EXPECT_GT(carrot->x, 40000.0f);  // carrot is well ahead
}

TEST(AutopilotGuidanceUnittest, StaysActiveNearEndpointWithCarrotClamped)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 10000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position close to end: alongTrack ≈ 9500
    // L1 stays active (no self-deactivation) with the carrot clamped to
    // the endpoint, giving the position PID a convergence target.
    vector2_t pos = { .x = 9500.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    EXPECT_TRUE(l1GuidanceIsActive());

    // Carrot should be clamped to the path endpoint
    const vector2_t *carrot = l1GuidanceGetCarrot();
    EXPECT_NEAR(carrot->x, 10000.0f, TOLERANCE);
    EXPECT_NEAR(carrot->y, 0.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, UpdateStaysActiveWhenFarFromEnd)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position far from end: alongTrack ≈ 5000
    // distanceToEnd = 50000 - 5000 = 45000
    // lookahead at 500 → clamped to 1000
    // 45000 > 1000 → stays active
    vector2_t pos = { .x = 5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    EXPECT_TRUE(l1GuidanceIsActive());
}

// =========================================================================
// Along-track distance
// =========================================================================

TEST(AutopilotGuidanceUnittest, AlongTrackDistanceAtStart)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    vector2_t pos = { .x = 0.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    EXPECT_NEAR(l1GuidanceGetAlongTrackDistance(), 0.0f, TOLERANCE);
}

TEST(AutopilotGuidanceUnittest, AlongTrackDistanceMidway)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    vector2_t pos = { .x = 25000.0f, .y = 100.0f }; // slightly off path
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Along-track should be ~25000 regardless of cross-track offset
    EXPECT_NEAR(l1GuidanceGetAlongTrackDistance(), 25000.0f, TOLERANCE);
}

// =========================================================================
// SetActive / IsActive
// =========================================================================

TEST(AutopilotGuidanceUnittest, SetActiveToggle)
{
    l1GuidanceInit();

    EXPECT_FALSE(l1GuidanceIsActive());

    l1GuidanceSetActive(true);
    EXPECT_TRUE(l1GuidanceIsActive());

    l1GuidanceSetActive(false);
    EXPECT_FALSE(l1GuidanceIsActive());
}

// =========================================================================
// Arc Transition
// =========================================================================

TEST(AutopilotGuidanceUnittest, ArcTransitionActivatesOnLargeBearingChange)
{
    l1GuidanceInit();
    setupL1Config();

    // First path: heading North (0°)
    vector2_t start1 = { .x = 0.0f, .y = 0.0f };
    vector2_t end1   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start1, &end1);
    l1GuidanceSetActive(true);

    // Update to establish desired bearing
    vector2_t pos = { .x = 5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);
    EXPECT_FALSE(l1GuidanceIsArcActive());

    // Set arc start at heading 0° (North), then set new path heading 60° (NE)
    l1GuidanceSetArcStart(0.0f);
    vector2_t start2 = { .x = 5000.0f, .y = 0.0f };
    // Path bearing 60°: cos(60°)=0.5, sin(60°)=0.866
    vector2_t end2   = { .x = 5000.0f + 25000.0f, .y = 43301.0f };
    l1GuidanceSetPath(&start2, &end2);
    l1GuidanceSetActive(true);

    // Arc should be active (60° change: > 20° min and < 90° max)
    EXPECT_TRUE(l1GuidanceIsArcActive());
}

TEST(AutopilotGuidanceUnittest, ArcTransitionSmallAngleSkipped)
{
    l1GuidanceInit();
    setupL1Config();

    // Arc start at 0°, new path at 10° — below 20° threshold
    l1GuidanceSetArcStart(0.0f);
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    // Path bearing = atan2(1736, 9848) ≈ 10° (sin(10°)=0.1736, cos(10°)=0.9848)
    vector2_t end = { .x = 9848.0f, .y = 1736.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    EXPECT_FALSE(l1GuidanceIsArcActive());
}

TEST(AutopilotGuidanceUnittest, ArcTransitionDisabledWhenTurnRateZero)
{
    l1GuidanceInit();
    setupL1Config();
    autopilotConfigMutable()->l1TurnRate = 0;

    l1GuidanceSetArcStart(0.0f);
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    // Path bearing 60°: cos(60°)=0.5, sin(60°)=0.866
    vector2_t end = { .x = 25000.0f, .y = 43301.0f };  // 60° change
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    EXPECT_FALSE(l1GuidanceIsArcActive());
}

TEST(AutopilotGuidanceUnittest, ArcTransitionCarrotRotates)
{
    l1GuidanceInit();
    setupL1Config();

    // Arc start at 0° (North), new path heading 60° (NE)
    l1GuidanceSetArcStart(0.0f);
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    // Path bearing 60°: cos(60°)=0.5, sin(60°)=0.866
    vector2_t end = { .x = 25000.0f, .y = 43301.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    EXPECT_TRUE(l1GuidanceIsArcActive());

    // First update: arc bearing starts at 0°, advances by 8 * 0.1 = 0.8°
    vector2_t pos = { .x = 5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Arc should still be active (only 0.8° into a 60° turn)
    EXPECT_TRUE(l1GuidanceIsArcActive());

    // Desired bearing should be near 0° + small advance (in centideg)
    float bearing = l1GuidanceGetDesiredBearing();
    EXPECT_GT(bearing, 0.0f);       // Advancing clockwise toward 90°
    EXPECT_LT(bearing, 500.0f);     // Should be less than 5° (500 centideg)
}

TEST(AutopilotGuidanceUnittest, ArcTransitionConverges)
{
    l1GuidanceInit();
    setupL1Config();

    // Arc from 0° to 60° (NE)
    l1GuidanceSetArcStart(0.0f);
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    // Path bearing 60°: cos(60°)=0.5, sin(60°)=0.866
    vector2_t end = { .x = 25000.0f, .y = 43301.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Simulate many updates at 8 deg/s, dt=0.1 → 0.8 deg/update
    // 60° / 0.8° = ~75 updates to complete
    vector2_t pos = { .x = 0.0f, .y = 0.0f };
    for (int i = 0; i < 200; i++) {
        l1GuidanceUpdate(&pos, 500.0f, DT);
        if (!l1GuidanceIsArcActive()) {
            break;
        }
    }

    // Arc should have converged to normal L1
    EXPECT_FALSE(l1GuidanceIsArcActive());
    EXPECT_TRUE(l1GuidanceIsActive());
}

TEST(AutopilotGuidanceUnittest, ArcTransitionShortestTurnLeft)
{
    l1GuidanceInit();
    setupL1Config();

    // Arc from 350° to 270° — shortest turn is counter-clockwise (-80°)
    l1GuidanceSetArcStart(350.0f);
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 0.0f, .y = -50000.0f };  // West = 270°
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    EXPECT_TRUE(l1GuidanceIsArcActive());

    // After one update, bearing should decrease (counter-clockwise)
    vector2_t pos = { .x = 0.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    // Bearing should be less than 350° (moving toward 270° counter-clockwise)
    float bearing = l1GuidanceGetDesiredBearing() * 0.01f;  // centideg → deg
    EXPECT_LT(bearing, 350.0f);
    EXPECT_GT(bearing, 340.0f);  // Should be around 349.2° (350 - 0.8)
}

TEST(AutopilotGuidanceUnittest, ArcTransitionLargeTurnUsesBaseRate)
{
    l1GuidanceInit();
    setupL1Config();

    // Arc from 0° to 180° (South) — large turn, uses base rate (8 deg/s).
    // Rate scaling was removed: the craft couldn't follow a fast arc with
    // the buildup angle limit active, causing huge cross-track error.
    l1GuidanceSetArcStart(0.0f);
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = -50000.0f, .y = 0.0f };  // South = 180°
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    EXPECT_TRUE(l1GuidanceIsArcActive());

    // After one update (dt=0.1), arc should advance by 8 * 0.1 = 0.8°
    // Same rate as a small turn — no scaling
    vector2_t pos = { .x = 5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    float bearing = l1GuidanceGetDesiredBearing() * 0.01f;  // centideg → deg
    EXPECT_NEAR(bearing, 0.8f, 0.2f);
}

TEST(AutopilotGuidanceUnittest, ArcTransitionSmallTurnUsesBaseRate)
{
    l1GuidanceInit();
    setupL1Config();

    // Arc from 0° to 30° — below the 45° reference, uses base rate (8 deg/s)
    l1GuidanceSetArcStart(0.0f);
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    // Path bearing 30°: cos(30°)=0.866, sin(30°)=0.5
    vector2_t end = { .x = 43301.0f, .y = 25000.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    EXPECT_TRUE(l1GuidanceIsArcActive());

    // After one update, arc advances by 8 * 0.1 = 0.8°
    vector2_t pos = { .x = 5000.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f, DT);

    float bearing = l1GuidanceGetDesiredBearing() * 0.01f;
    EXPECT_NEAR(bearing, 0.8f, 0.2f);
}

// =========================================================================
// STUBS
// =========================================================================

extern "C" {
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) {
        UNUSED(input);
        UNUSED(rxConfig);
    }
}
