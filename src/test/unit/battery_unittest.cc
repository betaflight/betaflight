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

//#define DEBUG_BATTERY

extern "C" {
    #include "sensors/battery.h"
    #include "io/rc_controls.h"
    #include "io/beeper.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef struct batteryAdcToVoltageExpectation_s {
    uint16_t adcReading;
    uint16_t expectedVoltageInDeciVoltSteps;
    uint8_t scale;
} batteryAdcToVoltageExpectation_t;

#define ELEVEN_TO_ONE_VOLTAGE_DIVIDER 110 // (10k:1k) * 10 for 0.1V

TEST(BatteryTest, BatteryADCToVoltage)
{
    // batteryInit() reads a bunch of fields including vbatscale, so set up the config with useful initial values:
    batteryConfig_t batteryConfig = {
        .vbatscale = VBAT_SCALE_DEFAULT,
        .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
        .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
        .vbatmaxcellvoltage = 43,
        .vbatmincellvoltage = 33,
        .vbatwarningcellvoltage = 35,
        .currentMeterScale = 400,
        .currentMeterOffset = 0,
        .currentMeterType = CURRENT_SENSOR_NONE,
        .multiwiiCurrentMeterOutput = 0,
        .batteryCapacity = 2200,
    };

    batteryInit(&batteryConfig);

    batteryAdcToVoltageExpectation_t batteryAdcToVoltageExpectations[] = {
            {1420, 126 /*125.88*/, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1430, 127 /*126.76*/, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1440, 128 /*127.65*/, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1890, 168 /*167.54*/, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1900, 168 /*168.42*/, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1910, 169 /*169.31*/, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {   0,   0 /*  0.00*/, VBAT_SCALE_MAX},
            {4096, 842 /*841.71*/, VBAT_SCALE_MAX}
    };
    uint8_t testIterationCount = sizeof(batteryAdcToVoltageExpectations) / sizeof(batteryAdcToVoltageExpectation_t);

    // expect

    for (uint8_t index = 0; index < testIterationCount; index ++) {
        batteryAdcToVoltageExpectation_t *batteryAdcToVoltageExpectation = &batteryAdcToVoltageExpectations[index];
        batteryConfig.vbatscale = batteryAdcToVoltageExpectation->scale;
#ifdef DEBUG_BATTERY
        printf("adcReading: %d, vbatscale: %d\n",
                batteryAdcToVoltageExpectation->adcReading,
                batteryAdcToVoltageExpectation->scale
        );
#endif
        uint16_t pointOneVoltSteps = batteryAdcToVoltage(batteryAdcToVoltageExpectation->adcReading);

        EXPECT_EQ(batteryAdcToVoltageExpectation->expectedVoltageInDeciVoltSteps, pointOneVoltSteps);
    }
}

uint16_t currentADCReading;


typedef struct batteryAdcToBatteryStateExpectation_s
{
    uint16_t adcReading;
    uint16_t expectedVoltageInDeciVoltSteps;
    batteryState_e expectedBatteryState;
    uint8_t scale;
} batteryAdcToBatteryStateExpectation_t;

/* Test the battery state and hysteresis code */
TEST(BatteryTest, BatteryState)
{
    // batteryInit() reads a bunch of fields including vbatscale, so set up the config with useful initial values:
    batteryConfig_t batteryConfig = {
        .vbatscale = VBAT_SCALE_DEFAULT,
        .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
        .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
        .vbatmaxcellvoltage = 43,
        .vbatmincellvoltage = 33,
        .vbatwarningcellvoltage = 35,
        .currentMeterScale = 400,
        .currentMeterOffset = 0,
        .currentMeterType = CURRENT_SENSOR_NONE,
        .multiwiiCurrentMeterOutput = 0,
        .batteryCapacity = 2200,
    };

    batteryInit(&batteryConfig);

    batteryAdcToBatteryStateExpectation_t batteryAdcToBatteryStateExpectations[] = {
            {1420, 126, BATTERY_OK, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            /* fall down to battery warning level */
            {1185, 105, BATTERY_OK, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1175, 104, BATTERY_WARNING, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            /* creep back up to battery ok */
            {1185, 105, BATTERY_WARNING, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1195, 106, BATTERY_WARNING, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1207, 107, BATTERY_OK, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            /* fall down to battery critical level */
            {1175, 104, BATTERY_WARNING, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1108, 98, BATTERY_CRITICAL, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            /* creep back up to battery warning */
            {1115, 99, BATTERY_CRITICAL, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1130, 100, BATTERY_CRITICAL, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},
            {1145, 101, BATTERY_WARNING, ELEVEN_TO_ONE_VOLTAGE_DIVIDER},

    };
    uint8_t testIterationCount = sizeof(batteryAdcToBatteryStateExpectations) / sizeof(batteryAdcToBatteryStateExpectation_t);

    // expect
    for (uint8_t index = 0; index < testIterationCount; index ++) {
        batteryAdcToBatteryStateExpectation_t *batteryAdcToBatteryStateExpectation = &batteryAdcToBatteryStateExpectations[index];
        batteryConfig.vbatscale = batteryAdcToBatteryStateExpectation->scale;
        currentADCReading = batteryAdcToBatteryStateExpectation->adcReading;
        updateBattery( );
        batteryState_e batteryState = getBatteryState();
        EXPECT_EQ(batteryAdcToBatteryStateExpectation->expectedBatteryState, batteryState);
    }
}

typedef struct batteryAdcToCellCountExpectation_s
{
    uint16_t adcReading;
    uint16_t expectedVoltageInDeciVoltSteps;
    uint8_t scale;
    uint8_t cellCount;
} batteryAdcToCellCountExpectation_t;

/* Test the cell count is correctly detected if we start at 0V */
TEST(BatteryTest, CellCount)
{
    // batteryInit() reads a bunch of fields including vbatscale, so set up the config with useful initial values:
    batteryConfig_t batteryConfig = {
        .vbatscale = VBAT_SCALE_DEFAULT,
        .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
        .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
        .vbatmaxcellvoltage = 43,
        .vbatmincellvoltage = 33,
        .vbatwarningcellvoltage = 35,
        .currentMeterScale = 400,
        .currentMeterOffset = 0,
        .currentMeterType = CURRENT_SENSOR_NONE,
        .multiwiiCurrentMeterOutput = 0,
        .batteryCapacity = 2200,
    };

    batteryInit(&batteryConfig);

    batteryAdcToCellCountExpectation_t batteryAdcToCellCountExpectations[] = {
            {0, 0, ELEVEN_TO_ONE_VOLTAGE_DIVIDER, 1},
            {1420, 126, ELEVEN_TO_ONE_VOLTAGE_DIVIDER, 3},
    };
    uint8_t testIterationCount = sizeof(batteryAdcToCellCountExpectations) / sizeof(batteryAdcToCellCountExpectation_t);

    // expect
    for (uint8_t index = 0; index < testIterationCount; index ++) {
        batteryAdcToCellCountExpectation_t *batteryAdcToCellCountExpectation = &batteryAdcToCellCountExpectations[index];
        batteryConfig.vbatscale = batteryAdcToCellCountExpectation->scale;
        currentADCReading = batteryAdcToCellCountExpectation->adcReading;
        updateBattery( );
        EXPECT_EQ(batteryAdcToCellCountExpectation->cellCount, batteryCellCount);
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

uint8_t armingFlags = 0;
int16_t rcCommand[4] = {0,0,0,0};


bool feature(uint32_t mask)
{
    UNUSED(mask);
    return false;
}

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    UNUSED(*rxConfig);
    UNUSED(deadband3d_throttle);
    return THROTTLE_HIGH;
}

uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return currentADCReading;
}

void delay(uint32_t ms)
{
    UNUSED(ms);
    return;
}

int32_t lowpassFixed(lowpass_t *filter, int32_t in, int16_t freq)
{
    UNUSED(filter);
    UNUSED(freq);
    return in;
}

void beeper(beeperMode_e mode)
{
    UNUSED(mode);
}

}
