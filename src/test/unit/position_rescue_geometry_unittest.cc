/*
 * This file is part of Betaflight.
 *
 * Tests geometry matching gps_rescue_multirotor rescueBearingToHomeDecideg
 * and velocity projection toward home (estimator ENU frame).
 */

#include <math.h>
#include <stdint.h>

extern "C" {
#include "platform.h"
#include "common/maths.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Mirror flight/gps_rescue_multirotor.c rescueBearingToHomeDecideg (decidegrees).
static int16_t bearingToHomeDecideg(float posEastCm, float posNorthCm)
{
    const float dLat = -posNorthCm;
    const float dLon = -posEastCm;
    float bearing = 9000.0f - RADIANS_TO_DEGREES(atan2_approx(dLat, dLon)) * 100.0f;
    while (bearing < 0.0f) {
        bearing += 36000.0f;
    }
    while (bearing >= 36000.0f) {
        bearing -= 36000.0f;
    }
    return (int16_t)lrintf(bearing / 10.0f);
}

static float velocityTowardHomeCmS(float posEastCm, float posNorthCm, float velEast, float velNorth)
{
    const float distCm = hypotf(posEastCm, posNorthCm);
    if (distCm <= 10.0f) {
        return 0.0f;
    }
    const float invDist = 1.0f / distCm;
    const float ux = -posEastCm * invDist;
    const float uy = -posNorthCm * invDist;
    return velEast * ux + velNorth * uy;
}

TEST(PositionRescueGeometry, bearingSouthWhenNorthOfHome)
{
    EXPECT_NEAR(bearingToHomeDecideg(0.0f, 100.0f), 1800, 5);
}

TEST(PositionRescueGeometry, bearingWestWhenEastOfHome)
{
    EXPECT_NEAR(bearingToHomeDecideg(100.0f, 0.0f), 2700, 5);
}

TEST(PositionRescueGeometry, bearingNorthWhenSouthOfHome)
{
    EXPECT_NEAR(bearingToHomeDecideg(0.0f, -100.0f), 0, 5);
}

TEST(PositionRescueGeometry, velocityTowardHomePositiveWhenClosing)
{
    // 100 cm east of home, moving west (negative East velocity) -> toward home
    const float v = velocityTowardHomeCmS(100.0f, 0.0f, -50.0f, 0.0f);
    EXPECT_GT(v, 0.0f);
    EXPECT_NEAR(v, 50.0f, 1.0f);
}
