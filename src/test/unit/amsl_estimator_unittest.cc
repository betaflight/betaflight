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
#include <string.h>
#include <math.h>

#ifdef USE_GPS

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "pg/pg_ids.h"

#include "common/maths.h"
#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/position_estimator.h"

#include "io/gps.h"

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];

gpsSolutionData_t gpsSol;

// --- Estimator-layer simulations ------------------------------------------------
// positionUpdate() in position.c calls these; the AMSL altitude is computed
// from their results. Minimal implementation: Z validity, relative altitude,
// and an optional GPS alt offset.

static bool simulationZValid = false;
static float simulationAltitudeCm = 0.0f;
static bool simulationGpsAltOffsetAvailable = false;
static float simulationGpsAltOffsetCm = 0.0f;

void positionEstimatorInit(void) {}
void positionEstimatorUpdate(void) {}
void positionEstimatorResetZ(void) {}

bool positionEstimatorIsValidZ(void) { return simulationZValid; }
float positionEstimatorGetAltitudeCm(void) { return simulationAltitudeCm; }
float positionEstimatorGetVerticalVelocity(void) { return 0.0f; }
float positionEstimatorGetVerticalAcceleration(void) { return 0.0f; }

bool positionEstimatorGetGpsAltOffsetCm(float *offsetCm)
{
    if (!simulationGpsAltOffsetAvailable) {
        return false;
    }
    *offsetCm = simulationGpsAltOffsetCm;
    return true;
}
}

#include "gtest/gtest.h"

class PositionAltitudeAslTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(debug, 0, sizeof(debug));

        armingFlags = ARMED;
        stateFlags = 0;
        flightModeFlags = 0;

        simulationZValid = false;
        simulationAltitudeCm = 0.0f;
        simulationGpsAltOffsetAvailable = false;
        simulationGpsAltOffsetCm = 0.0f;

        // Fast LPF: the settle helper converges the filter in a handful of steps.
        positionConfigMutable()->altitude_lpf = 10000;
        positionConfigMutable()->altitude_d_lpf = 10000;

        positionInit();
    }

    // Slow PT2 (1 Hz cutoff): makes the filter's lag behind its input measurable.
    void useSlowFilter()
    {
        positionConfigMutable()->altitude_lpf = 100;
        positionConfigMutable()->altitude_d_lpf = 100;
        positionInit();
    }

    // Drive the display filter to the given altitude and mark Z valid.
    // 1000 steps at a 10000 Hz cutoff settle well below 1 cm of error; at a
    // 1 Hz cutoff the residual is likewise negligible (e^-59).
    void settleDisplayAltitude(float altitudeCm, int iterations = 1000)
    {
        simulationZValid = true;
        simulationAltitudeCm = altitudeCm;
        for (int i = 0; i < iterations; i++) {
            positionUpdate();
        }
    }
};

// --- Basic getAltitudeAsl behaviour ---------------------------------------

TEST_F(PositionAltitudeAslTest, GpsFixReturnsRawGpsAltitudeAsNormal)
{
    stateFlags = GPS_FIX;
    gpsSol.llh.altCm = 12345;

    // Even with a valid estimator and an offset, GPS_FIX takes priority.
    simulationZValid = true;
    simulationGpsAltOffsetAvailable = true;
    simulationGpsAltOffsetCm = 500.0f;

    int32_t altitudeAsl = -1;
    EXPECT_EQ(getAltitudeAsl(&altitudeAsl), ALTITUDE_ASL_NORMAL);
    EXPECT_EQ(altitudeAsl, 12345);
}

TEST_F(PositionAltitudeAslTest, NoGpsFixWithGpsAltOffsetReturnsEstimated)
{
    stateFlags = 0;
    simulationGpsAltOffsetAvailable = true;
    simulationGpsAltOffsetCm = 120.0f;

    settleDisplayAltitude(500.0f);

    const int32_t estimatedAltitude = getEstimatedAltitudeCm();

    int32_t altitudeAsl = 0;
    EXPECT_EQ(getAltitudeAsl(&altitudeAsl), ALTITUDE_ASL_ESTIMATED);
    EXPECT_EQ(altitudeAsl, estimatedAltitude + 120);
}

TEST_F(PositionAltitudeAslTest, NoGpsFixWithoutGpsAltOffsetReturnsRelative)
{
    stateFlags = 0;
    simulationGpsAltOffsetAvailable = false;

    settleDisplayAltitude(500.0f);

    const int32_t estimatedAltitude = getEstimatedAltitudeCm();

    int32_t altitudeAsl = 12345;
    EXPECT_EQ(getAltitudeAsl(&altitudeAsl), ALTITUDE_ASL_RELATIVE);
    EXPECT_EQ(altitudeAsl, estimatedAltitude);
}

TEST_F(PositionAltitudeAslTest, NoGpsFixAndInvalidZReturnsFaultAndZero)
{
    stateFlags = 0;
    simulationZValid = false;

    int32_t altitudeAsl = 12345;
    EXPECT_EQ(getAltitudeAsl(&altitudeAsl), ALTITUDE_ASL_FAULT);
    EXPECT_EQ(altitudeAsl, 0);
}

// --- AMSL across a Z-frame rebase -----------------------------------------
//
// In ESTIMATED mode, AMSL altitude = relative estimate + GPS alt offset.
// On rebase the estimator shifts both terms in opposite directions (the
// estimate moves with the frame, the offset moves against it) so the sum does
// not step. The tests below assert exactly that invariant: it keeps GPS Rescue
// from seeing false altitude steps on every rebase.

TEST_F(PositionAltitudeAslTest, EstimatedAslIsContinuousAcrossRebase)
{
    stateFlags = 0;                     // no fix, but the offset from a past fix is present
    simulationGpsAltOffsetAvailable = true;
    simulationGpsAltOffsetCm = 1000.0f;
    settleDisplayAltitude(500.0f);

    int32_t before = 0;
    ASSERT_EQ(getAltitudeAsl(&before), ALTITUDE_ASL_ESTIMATED);
    ASSERT_NEAR(before, 1500, 1);       // 500 relative + 1000 offset

    // The estimator rebased the Z frame by +260 cm: the relative estimate
    // rose by 260, the offset tying it to GPS ASL dropped by 260.
    constexpr float deltaCm = 260.0f;
    rebaseAltitudeFilterState(deltaCm);
    simulationAltitudeCm += deltaCm;
    simulationGpsAltOffsetCm -= deltaCm;

    positionUpdate();                   // one step suffices: the filter moved with the frame

    int32_t after = 0;
    EXPECT_EQ(getAltitudeAsl(&after), ALTITUDE_ASL_ESTIMATED);
    EXPECT_NEAR(after, before, 2) << "AMSL must not step on rebase";
}

// The flip side: if the display filter is NOT rebased along with the frame,
// the offset has already moved while the display still lags on its old value,
// so AMSL drops by deltaCm. Pins the contract: skipping
// rebaseAltitudeFilterState() while the offset actually changes produces a
// phantom AMSL step. The test above cannot pass by accident.
TEST_F(PositionAltitudeAslTest, EstimatedAslJumpsIfDisplayFilterIsNotRebased)
{
    useSlowFilter();

    stateFlags = 0;
    simulationZValid = true;
    simulationGpsAltOffsetAvailable = true;
    simulationGpsAltOffsetCm = 1000.0f;

    simulationAltitudeCm = 500.0f;
    for (int i = 0; i < 2000; i++) {
        positionUpdate();
    }

    int32_t before = 0;
    ASSERT_EQ(getAltitudeAsl(&before), ALTITUDE_ASL_ESTIMATED);
    ASSERT_EQ(before, 1500);

    // Rebase the offset and the input, but NOT the filter state.
    constexpr float deltaCm = 260.0f;
    simulationAltitudeCm += deltaCm;
    simulationGpsAltOffsetCm -= deltaCm;
    positionUpdate();

    int32_t after = 0;
    EXPECT_EQ(getAltitudeAsl(&after), ALTITUDE_ASL_ESTIMATED);
    // The display barely moved (~515) while the offset dropped by 260, so AMSL
    // fell by hundreds of cm against the true 1500.
    EXPECT_LT(after, before - 100)
        << "skipping the display rebase must show a phantom drop";
}

// In RELATIVE mode there is no offset, the frame is not tied to anything, so
// a rebase legitimately shifts the relative altitude by deltaCm. Expected
// behaviour, but it is worth pinning so it is not confused with ESTIMATED.
TEST_F(PositionAltitudeAslTest, RelativeAslShiftsWithTheFrameOnRebase)
{
    stateFlags = 0;
    simulationGpsAltOffsetAvailable = false;   // no absolute frame
    settleDisplayAltitude(500.0f);

    int32_t before = 0;
    ASSERT_EQ(getAltitudeAsl(&before), ALTITUDE_ASL_RELATIVE);
    ASSERT_NEAR(before, 500, 1);

    constexpr float deltaCm = 260.0f;
    rebaseAltitudeFilterState(deltaCm);
    simulationAltitudeCm += deltaCm;
    positionUpdate();

    int32_t after = 0;
    EXPECT_EQ(getAltitudeAsl(&after), ALTITUDE_ASL_RELATIVE);
    EXPECT_NEAR(after, 760, 2) << "relative altitude follows the frame shift";
}

// With a GPS fix the altitude comes straight from the receiver, so a Z-frame
// rebase is entirely invisible to getAltitudeAsl. Guarantees the estimated
// path stays a fallback rather than becoming a second source of truth.
TEST_F(PositionAltitudeAslTest, GpsFixMasksRebaseEntirely)
{
    stateFlags = GPS_FIX;
    gpsSol.llh.altCm = 2500;
    simulationGpsAltOffsetAvailable = true;
    simulationGpsAltOffsetCm = 1000.0f;
    settleDisplayAltitude(500.0f);

    int32_t before = 0;
    ASSERT_EQ(getAltitudeAsl(&before), ALTITUDE_ASL_NORMAL);
    ASSERT_EQ(before, 2500);

    constexpr float deltaCm = 260.0f;
    rebaseAltitudeFilterState(deltaCm);
    simulationAltitudeCm += deltaCm;
    simulationGpsAltOffsetCm -= deltaCm;
    positionUpdate();

    int32_t after = 0;
    EXPECT_EQ(getAltitudeAsl(&after), ALTITUDE_ASL_NORMAL);
    EXPECT_EQ(after, 2500) << "GPS ASL must not be touched by an estimator rebase";
}

// Additive invariant: successive rebases do not accumulate AMSL error — a
// pair of opposing shifts returns the original value to within lrintf. Catches
// both a sign error in the offset and rounding accumulation if lrintf is ever
// applied to each term separately.
TEST_F(PositionAltitudeAslTest, SuccessiveRebasesDoNotAccumulateAslError)
{
    stateFlags = 0;
    simulationGpsAltOffsetAvailable = true;
    simulationGpsAltOffsetCm = 1000.0f;
    settleDisplayAltitude(500.0f);

    int32_t baseline = 0;
    ASSERT_EQ(getAltitudeAsl(&baseline), ALTITUDE_ASL_ESTIMATED);

    for (float d : { +260.0f, -180.0f, +75.0f, -155.0f, +40.0f, -40.0f }) {
        rebaseAltitudeFilterState(d);
        simulationAltitudeCm += d;
        simulationGpsAltOffsetCm -= d;
        positionUpdate();

        int32_t now = 0;
        ASSERT_EQ(getAltitudeAsl(&now), ALTITUDE_ASL_ESTIMATED);
        EXPECT_NEAR(now, baseline, 2) << "drift after rebase of " << d << " cm";
    }
}

#endif // USE_GPS
