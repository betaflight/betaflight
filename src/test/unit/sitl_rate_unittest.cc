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

    for (unsigned i = 0; i < 9; i++) {
        EXPECT_FALSE(sitlRateEstimatorUpdate(&estimator, 0.01, 0.01, &rate));
    }
    EXPECT_TRUE(sitlRateEstimatorUpdate(&estimator, 0.01, 0.01, &rate));
    EXPECT_DOUBLE_EQ(rate, 1.0);
}

TEST(SitlRateUnittest, LimitsOneShortGap)
{
    sitlRateEstimator_t estimator = {};
    double rate = 0.0;

    for (unsigned i = 0; i < 10; i++) {
        sitlRateEstimatorUpdate(&estimator, 0.01, 0.01, &rate);
    }

    EXPECT_TRUE(sitlRateEstimatorUpdate(&estimator, 0.01, 0.000001, &rate));
    EXPECT_NEAR(rate, 1.0101, 0.0001);
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
