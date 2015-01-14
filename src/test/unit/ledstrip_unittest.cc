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
#include <stdlib.h>

#include <limits.h>

extern "C" {
    #include "build_config.h"

    #include "common/color.h"
    #include "common/axis.h"
    #include "flight/flight.h"

    #include "sensors/battery.h"
    #include "config/runtime_config.h"
    #include "config/config.h"

    #include "rx/rx.h"

    #include "drivers/light_ws2811strip.h"
    #include "io/ledstrip.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    extern ledConfig_t *ledConfigs;
    extern uint8_t highestYValueForNorth;
    extern uint8_t lowestYValueForSouth;
    extern uint8_t highestXValueForWest;
    extern uint8_t lowestXValueForEast;
    extern uint8_t ledGridWidth;
    extern uint8_t ledGridHeight;

    void determineLedStripDimensions(void);
    void determineOrientationLimits(void);

    ledConfig_t systemLedConfigs[MAX_LED_STRIP_LENGTH];
}

TEST(LedStripTest, parseLedStripConfig)
{
    /*
     * 0..5   - rear right cluster,  0..2 rear 3..5 right
     * 6..11  - front right cluster, 6..8 rear, 9..11 front
     * 12..15 - front center cluster
     * 16..21 - front left cluster,  16..18 front, 19..21 rear
     * 22..27 - rear left cluster,   22..24 left, 25..27 rear
     */

    // given
    static const ledConfig_t expectedLedStripConfig[WS2811_LED_STRIP_LENGTH] = {
            { CALCULATE_LED_XY( 9,  9), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
            { CALCULATE_LED_XY(10, 10), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
            { CALCULATE_LED_XY(11, 11), LED_DIRECTION_SOUTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY(11, 11), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY(10, 10), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY( 9,  9), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },

            { CALCULATE_LED_XY(10,  5), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY(11,  4), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY(12,  3), LED_DIRECTION_SOUTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY(12,  2), LED_DIRECTION_NORTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY(11,  1), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY(10,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE },

            { CALCULATE_LED_XY( 7,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
            { CALCULATE_LED_XY( 6,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
            { CALCULATE_LED_XY( 5,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
            { CALCULATE_LED_XY( 4,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },

            { CALCULATE_LED_XY( 2,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY( 1,  1), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY( 0,  2), LED_DIRECTION_NORTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY( 0,  3), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY( 1,  4), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY( 2,  5), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },

            { CALCULATE_LED_XY( 2,  9), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY( 1, 10), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
            { CALCULATE_LED_XY( 0, 11), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY( 0, 11), LED_DIRECTION_SOUTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
            { CALCULATE_LED_XY( 1, 10), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
            { CALCULATE_LED_XY( 2,  9), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },

            { 0, 0 },
            { 0, 0 },
            { 0, 0 },
            { 0, 0 },
    };

    // and
    const char *ledStripConfigCommands[] = {
            // Spider quad

            // right rear cluster
            "9,9:S:FW",
            "10,10:S:FW",
            "11,11:S:IA",
            "11,11:E:IA",
            "10,10:E:F",
            "9,9:E:F",

            // right front cluster
            "10,5:S:F",
            "11,4:S:F",
            "12,3:S:IA",
            "12,2:N:IA",
            "11,1:N:F",
            "10,0:N:F",

            // center front cluster
            "7,0:N:FW",
            "6,0:N:FW",
            "5,0:N:FW",
            "4,0:N:FW",

            // left front cluster
            "2,0:N:F",
            "1,1:N:F",
            "0,2:N:IA",
            "0,3:W:IA",
            "1,4:W:F",
            "2,5:W:F",

            // left rear cluster
            "2,9:W:F",
            "1,10:W:F",
            "0,11:W:IA",
            "0,11:S:IA",
            "1,10:S:FW",
            "2,9:S:FW"
    };
    // and
    memset(&systemLedConfigs, 0, sizeof(systemLedConfigs));
    ledConfigs = systemLedConfigs;

    // and
    bool ok = false;

    // when
    for (uint8_t index = 0; index < (sizeof(ledStripConfigCommands) / sizeof(ledStripConfigCommands[0])); index++) {
        ok |= parseLedStripConfig(index, ledStripConfigCommands[index]);
    }

    // then
    EXPECT_EQ(true, ok);
    EXPECT_EQ(28, ledCount);


    // and
    for (uint8_t index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
        printf("iteration: %d\n", index);

        EXPECT_EQ(expectedLedStripConfig[index].xy, ledConfigs[index].xy);
        EXPECT_EQ(expectedLedStripConfig[index].flags, ledConfigs[index].flags);
    }

    // then
    EXPECT_EQ(13, ledGridWidth);
    EXPECT_EQ(12, ledGridHeight);

    // then
    EXPECT_EQ(5, highestXValueForWest);
    EXPECT_EQ(7, lowestXValueForEast);
    EXPECT_EQ(5, highestYValueForNorth);
    EXPECT_EQ(6, lowestYValueForSouth);
}

TEST(LedStripTest, smallestGridWithCenter)
{
    // given
    memset(&systemLedConfigs, 0, sizeof(systemLedConfigs));
    ledConfigs = systemLedConfigs;

    // and
    static const ledConfig_t testLedConfigs[] = {
        { CALCULATE_LED_XY( 2,  2), LED_DIRECTION_SOUTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 2,  1), LED_DIRECTION_EAST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 2,  0), LED_DIRECTION_NORTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 1,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 0,  0), LED_DIRECTION_NORTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 0,  1), LED_DIRECTION_WEST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 0,  2), LED_DIRECTION_SOUTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 1,  2), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING }
    };
    memcpy(&systemLedConfigs, &testLedConfigs, sizeof(testLedConfigs));

    // when
    determineLedStripDimensions();

    // then
    EXPECT_EQ(3, ledGridWidth);
    EXPECT_EQ(3, ledGridHeight);

    // when
    determineOrientationLimits();

    // then
    EXPECT_EQ(0, highestXValueForWest);
    EXPECT_EQ(2, lowestXValueForEast);
    EXPECT_EQ(0, highestYValueForNorth);
    EXPECT_EQ(2, lowestYValueForSouth);
}

TEST(LedStripTest, smallestGrid)
{
    // given
    memset(&systemLedConfigs, 0, sizeof(systemLedConfigs));
    ledConfigs = systemLedConfigs;

    // and
    static const ledConfig_t testLedConfigs[] = {
        { CALCULATE_LED_XY( 1,  1), LED_DIRECTION_SOUTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 1,  0), LED_DIRECTION_NORTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 0,  0), LED_DIRECTION_NORTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 0,  1), LED_DIRECTION_SOUTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_FLIGHT_MODE },
    };
    memcpy(&systemLedConfigs, &testLedConfigs, sizeof(testLedConfigs));

    // when
    determineLedStripDimensions();

    // then
    EXPECT_EQ(2, ledGridWidth);
    EXPECT_EQ(2, ledGridHeight);

    // when
    determineOrientationLimits();

    // then
    EXPECT_EQ(0, highestXValueForWest);
    EXPECT_EQ(1, lowestXValueForEast);
    EXPECT_EQ(0, highestYValueForNorth);
    EXPECT_EQ(1, lowestYValueForSouth);
}

/*
        { CALCULATE_LED_XY( 1, 14), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_FLIGHT_MODE },

        { CALCULATE_LED_XY( 0, 13), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 0, 12), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { CALCULATE_LED_XY( 0, 11), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 0, 10), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 0,  9), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 0,  8), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 0,  7), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 0,  6), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 0,  5), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 0,  4), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 0,  3), LED_DIRECTION_WEST  | LED_FUNCTION_FLIGHT_MODE },

        { CALCULATE_LED_XY( 0,  2), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 0,  1), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { CALCULATE_LED_XY( 1,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 2,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 3,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { CALCULATE_LED_XY( 4,  1), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 4,  2), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { CALCULATE_LED_XY( 4,  3), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 4,  4), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 4,  5), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 4,  6), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 4,  7), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 4,  8), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
        { CALCULATE_LED_XY( 4,  9), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 4, 10), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },
        { CALCULATE_LED_XY( 4, 11), LED_DIRECTION_EAST  | LED_FUNCTION_FLIGHT_MODE },

        { CALCULATE_LED_XY( 4, 12), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { CALCULATE_LED_XY( 4, 13), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { CALCULATE_LED_XY( 3, 14), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

 */

hsvColor_t testColors[CONFIGURABLE_COLOR_COUNT];

extern hsvColor_t *colors;

#define TEST_COLOR_COUNT 4

TEST(ColorTest, parseColor)
{
    // given
    colors = testColors;
    memset(colors, 0, sizeof(colors) * CONFIGURABLE_COLOR_COUNT);

    // and
    const hsvColor_t expectedColors[TEST_COLOR_COUNT] = {
            //  H    S    V
            {   0,   0,   0 },
            {   1,   1,   1 },
            { 359, 255, 255 },
            { 333,  22,   1 }
    };

    const char *testColors[TEST_COLOR_COUNT] = {
            "0,0,0",
            "1,1,1",
            "359,255,255",
            "333,22,1"
    };

    // when
    for (uint8_t index = 0; index < TEST_COLOR_COUNT; index++) {
        printf("parse iteration: %d\n", index);
        parseColor(index, testColors[index]);
    }

    // then

    for (uint8_t index = 0; index < TEST_COLOR_COUNT; index++) {
        printf("iteration: %d\n", index);

        EXPECT_EQ(expectedColors[index].h, colors[index].h);
        EXPECT_EQ(expectedColors[index].s, colors[index].s);
        EXPECT_EQ(expectedColors[index].v, colors[index].v);
    }
}

extern "C" {

uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
int16_t rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

batteryState_e calculateBatteryState(void) {
    return BATTERY_OK;
}

void ws2811LedStripInit(void) {}
void ws2811UpdateStrip(void) {}

void setLedValue(uint16_t index, const uint8_t value) {
    UNUSED(index);
    UNUSED(value);
}

void setLedHsv(uint16_t index, const hsvColor_t *color) {
    UNUSED(index);
    UNUSED(color);
}

void getLedHsv(uint16_t index, hsvColor_t *color) {
    UNUSED(index);
    UNUSED(color);
}


void scaleLedValue(uint16_t index, const uint8_t scalePercent) {
    UNUSED(index);
    UNUSED(scalePercent);
}

void setStripColor(const hsvColor_t *color) {
    UNUSED(color);
}

void setStripColors(const hsvColor_t *colors) {
    UNUSED(colors);
}

bool isWS2811LedStripReady(void) { return false; }

void delay(uint32_t ms)
{
    UNUSED(ms);
    return;
}

uint32_t micros(void) { return 0; }
bool shouldSoundBatteryAlarm(void) { return false; }
bool feature(uint32_t mask) {
    UNUSED(mask);
    return false;
}

void tfp_sprintf(char *, char*, ...) { }

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax) {
    UNUSED(x);
    UNUSED(srcMin);
    UNUSED(srcMax);
    UNUSED(destMin);
    UNUSED(destMax);

    return 0;
}

}
