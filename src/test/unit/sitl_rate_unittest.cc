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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "gtest/gtest.h"

extern "C" {
#include "sitl_rate.h"
}

TEST(SitlRateUnittest, WaitsForWarmup)
{
    sitlRateEstimator_t estimator = {};
    double rate = 0.0;

    for (unsigned i = 0; i < 99; i++) {
        EXPECT_FALSE(sitlRateEstimatorUpdate(&estimator, 0.001, 0.001, &rate));
    }
    EXPECT_TRUE(sitlRateEstimatorUpdate(&estimator, 0.001, 0.001, &rate));
    EXPECT_DOUBLE_EQ(rate, 1.0);
}

static void feedRealTime(sitlRateEstimator_t *estimator, double *rate)
{
    for (unsigned i = 0; i < 200; i++) {
        sitlRateEstimatorUpdate(estimator, 0.001, 0.001, rate);
    }
}

TEST(SitlRateUnittest, BurstsGiveTheAverageRate)
{
    sitlRateEstimator_t estimator = {};
    double rate = 0.0;
    feedRealTime(&estimator, &rate);

    // ten steps of 1 ms arrive together every 10 ms
    for (unsigned i = 0; i < 4000; i++) {
        sitlRateEstimatorUpdate(&estimator, 0.001, i % 10 ? 0.000001 : 0.01 - 9 * 0.000001, &rate);
        if (i >= 2000) {
            EXPECT_NEAR(rate, 1.0, 0.0001);
        }
    }
}

// Some simulators send more than one packet for each step
TEST(SitlRateUnittest, RepeatedTimestampsKeepTheirRealTime)
{
    sitlRateEstimator_t estimator = {};
    double rate = 0.0;

    for (unsigned i = 0; i < 1000; i++) {
        sitlRateEstimatorUpdate(&estimator, 0.002, 0.001, &rate);
        sitlRateEstimatorUpdate(&estimator, 0.0, 0.001, &rate);
    }
    EXPECT_NEAR(rate, 1.0, 0.001);
}

TEST(SitlRateUnittest, PauseKeepsTheRate)
{
    sitlRateEstimator_t estimator = {};
    double rate = 0.0;
    feedRealTime(&estimator, &rate);

    for (unsigned i = 0; i < 500; i++) {
        sitlRateEstimatorUpdate(&estimator, 0.0, 0.001, &rate);
    }
    feedRealTime(&estimator, &rate);
    EXPECT_NEAR(rate, 1.0, 0.001);
}

TEST(SitlRateUnittest, TracksSustainedFastSimulation)
{
    sitlRateEstimator_t estimator = {};
    double rate = 0.0;

    for (unsigned i = 0; i < 20; i++) {
        sitlRateEstimatorUpdate(&estimator, 0.01, 0.005, &rate);
    }

    EXPECT_DOUBLE_EQ(rate, 2.0);
}

TEST(SitlRateUnittest, ResetDiscardsPreviousSamples)
{
    sitlRateEstimator_t estimator = {};
    double rate = 0.0;

    for (unsigned i = 0; i < 20; i++) {
        sitlRateEstimatorUpdate(&estimator, 0.01, 0.005, &rate);
    }
    sitlRateEstimatorReset(&estimator);

    EXPECT_FALSE(sitlRateEstimatorUpdate(&estimator, 0.01, 0.01, &rate));
}

TEST(SitlRateUnittest, PreservesWarmupAcrossExcludedGaps)
{
    sitlRateEstimator_t estimator = {};
    double rate = 1.0;

    for (unsigned cycle = 0; cycle < 3; cycle++) {
        for (unsigned sample = 0; sample < 8; sample++) {
            sitlRateEstimatorUpdate(&estimator, 0.01, 0.005, &rate);
        }
        EXPECT_FALSE(sitlRateEstimatorUpdate(&estimator, 0.03, 0.015, &rate));
    }

    EXPECT_TRUE(estimator.initialized);
    EXPECT_DOUBLE_EQ(rate, 2.0);
}

TEST(SitlRateUnittest, PausesWhenTimestampStops)
{
    sitlPause_t pause = {};

    for (unsigned i = 0; i < 99; i++) {
        EXPECT_FALSE(sitlPauseUpdate(&pause, 0.0, 0.001));
    }
    EXPECT_FALSE(pause.paused);

    EXPECT_TRUE(sitlPauseUpdate(&pause, 0.0, 0.001));
    EXPECT_TRUE(pause.paused);

    for (unsigned i = 0; i < 5000; i++) {
        EXPECT_FALSE(sitlPauseUpdate(&pause, 0.0, 0.001));
    }

    EXPECT_TRUE(sitlPauseUpdate(&pause, 0.001, 0.001));
    EXPECT_FALSE(pause.paused);
}

// Some simulators send more than one packet for each step
TEST(SitlRateUnittest, RepeatedTimestampsAreNotAPause)
{
    sitlPause_t pause = {};

    for (unsigned i = 0; i < 5000; i++) {
        sitlPauseUpdate(&pause, 0.002, 0.001);
        sitlPauseUpdate(&pause, 0.0, 0.001);
        EXPECT_FALSE(pause.paused);
    }
}
