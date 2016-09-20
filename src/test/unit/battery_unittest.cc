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

#include <limits.h>

extern "C" {
    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "common/filter.h"

    #include "sensors/voltage.h"
    #include "sensors/amperage.h"
    #include "sensors/battery.h"

    #include "io/beeper.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint16_t currentVoltage;
amperageMeter_t *amperageMeter;

typedef struct batteryAdcToBatteryStateExpectation_s
{
    uint16_t voltage;
    batteryState_e expectedBatteryState;
} batteryAdcToBatteryStateExpectation_t;

TEST(BatteryTest, BatteryStateAndHysteresis)
{
    // batteryInit() reads a bunch of fields including vbatscale, so set up the config with useful initial values:
    batteryConfig_t testBatteryConfig = {
        .vbatmaxcellvoltage = 43,
        .vbatmincellvoltage = 33,
        .vbatwarningcellvoltage = 35,
        .batteryCapacity = 0,    // UNUSED
        .amperageMeterSource = 0  // UNUSED
    };

    memcpy(batteryConfig(), &testBatteryConfig, sizeof(*batteryConfig()));

    batteryInit();


    batteryAdcToBatteryStateExpectation_t batteryAdcToBatteryStateExpectations[] = {
            // using 3-Cell voltages.
            {126, BATTERY_OK},
            // fall down to battery warning level
            {105, BATTERY_OK},
            {104, BATTERY_WARNING},
            // creep back up to battery ok
            {105, BATTERY_WARNING},
            {106, BATTERY_WARNING},
            {107, BATTERY_OK},
            // fall down to battery critical level
            {104, BATTERY_WARNING},
            {98, BATTERY_CRITICAL},
            // creep back up to battery warning
            {99, BATTERY_CRITICAL},
            {100, BATTERY_CRITICAL},
            {101, BATTERY_WARNING},

    };
    uint8_t testIterationCount = sizeof(batteryAdcToBatteryStateExpectations) / sizeof(batteryAdcToBatteryStateExpectation_t);

    // expect
    for (uint8_t index = 0; index < testIterationCount; index ++) {
        batteryAdcToBatteryStateExpectation_t *batteryAdcToBatteryStateExpectation = &batteryAdcToBatteryStateExpectations[index];

        currentVoltage = batteryAdcToBatteryStateExpectation->voltage;

        batteryUpdate();

        batteryState_e batteryState = getBatteryState();
        EXPECT_EQ(batteryAdcToBatteryStateExpectation->expectedBatteryState, batteryState);
    }
}

typedef struct cellCountExpectation_s
{
    uint16_t voltage;
    uint8_t cellCount;
} cellCountExpectation_t;

// FIXME as-is this test barely scratches the surface of using the min/max cell voltages, expand to cover 1-8 cell detection.
// it does not cover, for example, the voltages used by different battery technologies which use different min/max cell voltages
TEST(BatteryTest, LipoCellCount)
{
    // batteryInit() reads a bunch of fields including vbatscale, so set up the config with useful initial values:
    batteryConfig_t testBatteryConfig = {
        // Standard LIPO voltages
        .vbatmaxcellvoltage = 43,
        .vbatmincellvoltage = 33,
        .vbatwarningcellvoltage = 35,
        .batteryCapacity = 0,    // UNUSED
        .amperageMeterSource = 0  // UNUSED
    };

    memcpy(batteryConfig(), &testBatteryConfig, sizeof(*batteryConfig()));

    cellCountExpectation_t cellCountExpectations[] = {
            {42,    1},
            {84,    2},
            {126,   3},
            {168,   4},
            {210,   5},
            {252,   6},
            {294,   7},
            {336,   8},
    };
    uint8_t testIterationCount = sizeof(cellCountExpectations) / sizeof(cellCountExpectation_t);

    // expect
    for (uint8_t index = 0; index < testIterationCount; index ++) {
        cellCountExpectation_t *cellCountExpectation = &cellCountExpectations[index];

        currentVoltage = cellCountExpectation->voltage;

        batteryInit();
        batteryUpdate();

        EXPECT_EQ(cellCountExpectation->cellCount, batteryCellCount);
    }
}


//#define DEBUG_ROLLOVER_PATTERNS
/**
 * These next two tests do not test any production code (!) but serves as an example of how to use a signed variable for timing purposes.
 *
 * The 'signed diff timing' pattern is followed in a few places in the production codebase.
 */
TEST(BatteryTest, RollOverPattern1)
{
    uint16_t now = 0;
    uint16_t servicedAt = 0;
    uint16_t serviceInterval = 5000;
    int serviceCount = 0;
    int rolloverCount = 0;

    while(rolloverCount < 3) {

        int16_t diff = (now - servicedAt);
        if (diff >= serviceInterval) {

            if (now < servicedAt) {
                rolloverCount++;
            }

            servicedAt = now;
#ifdef DEBUG_ROLLOVER_PATTERNS
            printf("servicedAt: %d, diff: %d\n", servicedAt, diff);
#endif
            serviceCount++;

            EXPECT_GT(diff, 0);
            EXPECT_GE(diff, serviceInterval); // service interval must have passed
            EXPECT_LT(diff, serviceInterval + 95 + 10); // but never more than the service interval + ticks + jitter
        }

        now += 95 + (rand() % 10); // simulate 100 ticks +/- 5 ticks of jitter, this can rollover
    }
    EXPECT_GT(serviceCount, 0);
}

TEST(BatteryTest, RollOverPattern2)
{
    uint16_t now = 0;
    uint16_t serviceAt = 0;
    uint16_t serviceInterval = 5000;
    int serviceCount = 0;
    int rolloverCount = 0;

    while(rolloverCount < 3) {

        int16_t diff = (now - serviceAt);
        if (diff >= 0) {

            if (now < serviceAt) {
                rolloverCount++;
            }

            serviceAt = now + serviceInterval; // this can rollover
#ifdef DEBUG_ROLLOVER_PATTERNS
            printf("servicedAt: %d, nextServiceAt: %d, diff: %d\n", now, serviceAt, diff);
#endif

            serviceCount++;

            EXPECT_GE(diff, 0);
            EXPECT_LE(diff, serviceInterval);
            EXPECT_LT(diff, 95 + 10); // never more than the ticks + jitter
        }

        now += 95 + (rand() % 10); // simulate 100 ticks +/- 5 ticks of jitter, this can rollover
    }
    EXPECT_GT(serviceCount, 0);
}


// STUBS

extern "C" {

uint16_t getVoltageForADCChannel(uint8_t channel)
{
    UNUSED(channel);
    return currentVoltage;
}

uint16_t getLatestVoltageForADCChannel(uint8_t channel)
{
    UNUSED(channel);
    return currentVoltage;
}

void delay(uint32_t ms)
{
    UNUSED(ms);
    return;
}

void beeper(beeperMode_e mode)
{
    UNUSED(mode);
}

void voltageMeterUpdate(void) {}

amperageMeter_t *getAmperageMeter(amperageMeter_e  index)
{
    UNUSED(index);
    return amperageMeter;
}



}
