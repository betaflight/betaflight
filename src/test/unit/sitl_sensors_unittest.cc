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

#include "gtest/gtest.h"

extern "C" {
#include "sitl_sensors.h"
}

// The conversions of voltage.c and current.c, with a 3.3 V reference
static int centivoltsFromSample(uint16_t sample, int scale, int divider, int multiplier)
{
    return (((uint32_t)sample * scale * 3300 / 10 + 0xFFF * 5) / (0xFFF * divider)) / multiplier;
}

static int centiampsFromSample(uint16_t sample, int scale, int offset)
{
    const int32_t millivolts = ((uint32_t)sample * 3300) / 4096;
    return (millivolts * 10000 / scale + offset) / 10;
}

TEST(SitlSensorsUnittest, VoltageReadsBack)
{
    for (int cv = 0; cv <= 3600; cv += 7) {
        EXPECT_NEAR(centivoltsFromSample(sitlAdcVoltageSample(cv / 100.0, 110, 10, 1), 110, 10, 1), cv, 1) << cv;
    }
    // 12S
    for (int cv = 0; cv <= 6000; cv += 7) {
        EXPECT_NEAR(centivoltsFromSample(sitlAdcVoltageSample(cv / 100.0, 200, 10, 1), 200, 10, 1), cv, 2) << cv;
    }
}

TEST(SitlSensorsUnittest, CurrentReadsBack)
{
    // current.c reads whole millivolts: 2.5 cA steps at the default scale, 10 cA at scale 100
    for (int ca = 0; ca <= 8200; ca += 13) {
        EXPECT_NEAR(centiampsFromSample(sitlAdcCurrentSample(ca / 100.0, 400, 0), 400, 0), ca, 4) << ca;
    }
    for (int ca = 0; ca <= 33000; ca += 37) {
        EXPECT_NEAR(centiampsFromSample(sitlAdcCurrentSample(ca / 100.0, 100, 0), 100, 0), ca, 15) << ca;
    }
    for (int ca = 50; ca <= 8000; ca += 13) {
        EXPECT_NEAR(centiampsFromSample(sitlAdcCurrentSample(ca / 100.0, 400, 500), 400, 500), ca, 4) << ca;
    }
}

TEST(SitlSensorsUnittest, NotSimulatedReadsZero)
{
    EXPECT_EQ(sitlAdcVoltageSample(-1.0, 110, 10, 1), 0);
    EXPECT_EQ(sitlAdcCurrentSample(-1.0, 400, 0), 0);
    EXPECT_EQ(sitlAdcCurrentSample(-1.0, -400, -500), 0);
}

TEST(SitlSensorsUnittest, NotANumberReadsZero)
{
    EXPECT_FALSE(sitlIsFinite(NAN));
    EXPECT_FALSE(sitlIsFinite(INFINITY));
    EXPECT_FALSE(sitlIsFinite(-INFINITY));
    EXPECT_TRUE(sitlIsFinite(0.0));
    EXPECT_TRUE(sitlIsFinite(-12.5));
    EXPECT_EQ(sitlAdcVoltageSample(NAN, 110, 10, 1), 0);
    EXPECT_EQ(sitlAdcCurrentSample(NAN, 400, 0), 0);
}

TEST(SitlSensorsUnittest, ClampsAtFullScale)
{
    EXPECT_EQ(sitlAdcVoltageSample(50.0, 110, 10, 1), 4095);
    EXPECT_EQ(sitlAdcCurrentSample(100.0, 400, 0), 4095);
    EXPECT_LT(sitlAdcCurrentSample(300.0, 100, 0), 4095);
}

TEST(SitlSensorsUnittest, MagNotSimulated)
{
    const double field[3] = { 0.0, 0.0, 0.0 };
    int16_t samples[3] = { 1, 2, 3 };
    EXPECT_FALSE(sitlMagSamples(field, samples));
    EXPECT_EQ(samples[0], 1);

    const double nanField[3] = { 0.3, NAN, 0.4 };
    EXPECT_FALSE(sitlMagSamples(nanField, samples));
}

TEST(SitlSensorsUnittest, MagLevelNorth)
{
    const double field[3] = { 2046.8 / 8192.0, 71.5 / 8192.0, 3547.2 / 8192.0 };  // north, east, down
    int16_t samples[3];
    EXPECT_TRUE(sitlMagSamples(field, samples));
    EXPECT_EQ(samples[0], 2047);
    EXPECT_EQ(samples[1], -72);
    EXPECT_EQ(samples[2], -3547);
}

TEST(SitlSensorsUnittest, MagClampsAtFullScale)
{
    const double field[3] = { 5.0, 5.0, -5.0 };
    int16_t samples[3];
    EXPECT_TRUE(sitlMagSamples(field, samples));
    EXPECT_EQ(samples[0], 32767);
    EXPECT_EQ(samples[1], -32767);
    EXPECT_EQ(samples[2], 32767);
}
