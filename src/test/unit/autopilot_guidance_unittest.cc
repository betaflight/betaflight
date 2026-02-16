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
}

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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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

    // Short path (200cm)
    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 200.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position near the start with large lookahead
    // Even with min lookahead (1000cm), carrot would be past end
    vector2_t pos = { .x = 50.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 10.0f);

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
    l1GuidanceUpdate(&pos, 100000.0f);

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
    l1GuidanceUpdate(&pos, 5000.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

    // Cross-track error should remain 0 (update skipped)
    EXPECT_FLOAT_EQ(l1GuidanceGetCrossTrackError(), 0.0f);
}

TEST(AutopilotGuidanceUnittest, UpdateDeactivatesOnExcessiveCrossTrack)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 50000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position > l1MaxCrossTrackError (10000cm) to the right of path
    vector2_t pos = { .x = 25000.0f, .y = 15000.0f };
    l1GuidanceUpdate(&pos, 500.0f);

    // L1 should have been deactivated
    EXPECT_FALSE(l1GuidanceIsActive());
}

TEST(AutopilotGuidanceUnittest, UpdateDeactivatesNearEndpoint)
{
    l1GuidanceInit();
    setupL1Config();

    vector2_t start = { .x = 0.0f, .y = 0.0f };
    vector2_t end   = { .x = 10000.0f, .y = 0.0f };
    l1GuidanceSetPath(&start, &end);
    l1GuidanceSetActive(true);

    // Position close to end: alongTrack ≈ 9500
    // distanceToEnd = 10000 - 9500 = 500
    // lookahead at 500cm/s → V*T/(2π) = 500*2.0/6.28 ≈ 159 → clamped to min 1000
    // distanceToEnd (500) < lookahead (1000) → deactivate
    vector2_t pos = { .x = 9500.0f, .y = 0.0f };
    l1GuidanceUpdate(&pos, 500.0f);

    EXPECT_FALSE(l1GuidanceIsActive());
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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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
    l1GuidanceUpdate(&pos, 500.0f);

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
