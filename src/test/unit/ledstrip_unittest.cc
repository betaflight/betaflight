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

//#define DEBUG_LEDSTRIP

extern "C" {
    #include "build_config.h"

    #include "common/color.h"
    #include "common/axis.h"
    #include "common/utils.h"

    #include "sensors/battery.h"
    #include "config/runtime_config.h"
    #include "config/config.h"

    #include "rx/rx.h"

    #include "io/rc_controls.h"

    #include "drivers/light_ws2811strip.h"
    #include "io/ledstrip.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    extern uint8_t highestYValueForNorth;
    extern uint8_t lowestYValueForSouth;
    extern uint8_t highestXValueForWest;
    extern uint8_t lowestXValueForEast;
    extern uint8_t ledGridWidth;
    extern uint8_t ledGridHeight;
    extern ledCounts_t ledCounts;

    void updateLedCount(void);
    void determineLedStripDimensions(void);
    void determineOrientationLimits(void);
}

// utility macros
#define LF(name) LED_FUNCTION_ ## name
#define LO(name) LED_FLAG_OVERLAY(LED_OVERLAY_ ## name)
#define LD(name) LED_FLAG_DIRECTION(LED_DIRECTION_ ## name)

TEST(LedStripTest, parseLedStripConfig)
{
    // given
    static const ledConfig_t expectedLedStripConfig[WS2811_LED_STRIP_LENGTH] = {
            DEFINE_LED( 9,  9, 0, LD(SOUTH) , LF(FLIGHT_MODE) , LO(WARNING) , 0),
            DEFINE_LED(10, 10, 0, LD(SOUTH) , LF(FLIGHT_MODE) , LO(WARNING) , 0),
            DEFINE_LED(11, 11, 0, LD(SOUTH) , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED(11, 11, 0, LD(EAST)  , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED(10, 10, 0, LD(EAST)  , LF(FLIGHT_MODE) , 0, 0),

            DEFINE_LED(10,  5, 0, LD(SOUTH) , LF(FLIGHT_MODE) , 0, 0),
            DEFINE_LED(11,  4, 0, LD(SOUTH) , LF(FLIGHT_MODE) , 0, 0),
            DEFINE_LED(12,  3, 0, LD(SOUTH) , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED(12,  2, 0, LD(NORTH) , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED(11,  1, 0, LD(NORTH) , LF(FLIGHT_MODE) , 0, 0),
            DEFINE_LED(10,  0, 0, LD(NORTH) , LF(FLIGHT_MODE) , 0, 0),

            DEFINE_LED( 7,  0, 0, LD(NORTH) , LF(FLIGHT_MODE) , LO(WARNING) , 0),
            DEFINE_LED( 6,  0, 1, LD(NORTH) , LF(COLOR) , LO(WARNING) , 0),
            DEFINE_LED( 5,  0, 1, LD(NORTH) , LF(COLOR) , LO(WARNING) , 0),
            DEFINE_LED( 4,  0, 0, LD(NORTH) , LF(FLIGHT_MODE) , LO(WARNING) , 0),

            DEFINE_LED( 2,  0, 0, LD(NORTH) , LF(FLIGHT_MODE) , 0, 0),
            DEFINE_LED( 1,  1, 0, LD(NORTH) , LF(FLIGHT_MODE) , 0, 0),
            DEFINE_LED( 0,  2, 0, LD(NORTH) , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED( 0,  3, 0, LD(WEST)  , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED( 1,  4, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0, 0),
            DEFINE_LED( 2,  5, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0, 0),

            DEFINE_LED( 1, 10, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0, 0),
            DEFINE_LED( 0, 11, 0, LD(WEST)  , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED( 0, 11, 0, LD(SOUTH) , LF(ARM_STATE) , LO(INDICATOR) , 0),
            DEFINE_LED( 1, 10, 0, LD(SOUTH) , LF(FLIGHT_MODE) , LO(WARNING) , 0),
            DEFINE_LED( 2,  9, 0, LD(SOUTH) , LF(FLIGHT_MODE) , LO(WARNING) , 0),

            DEFINE_LED( 7,  7, 14, 0, LF(THRUST_RING) , 0 , 0),
            DEFINE_LED( 8,  7, 15, 0, LF(THRUST_RING) , 0 , 0),
            DEFINE_LED( 8,  8, 14, 0, LF(THRUST_RING) , 0 , 0),
            DEFINE_LED( 7,  8, 15, 0, LF(THRUST_RING) , 0 , 0),

            DEFINE_LED( 0, 0, 0, 0 , 0 , 0 , 0),
            DEFINE_LED( 0, 0, 0, 0 , 0 , 0 , 0),
    };

    // and
    const char *ledStripConfigCommands[] = {
            // Spider quad

            // right rear cluster
            "9,9:S:FW:0",
            "10,10:S:FW:0",
            "11,11:S:AI:0",
            "11,11:E:AI:0",
            "10,10:E:F:0",

            // right front cluster
            "10,5:S:F:0",
            "11,4:S:F:0",
            "12,3:S:AI:0",
            "12,2:N:AI:0",
            "11,1:N:F:0",
            "10,0:N:F:0",

            // center front cluster
            "7,0:N:FW:0",
            "6,0:N:CW:1",
            "5,0:N:CW:1",
            "4,0:N:FW:0",

            // left front cluster
            "2,0:N:F:0",
            "1,1:N:F:0",
            "0,2:N:AI:0",
            "0,3:W:AI:0",
            "1,4:W:F:0",
            "2,5:W:F:0",

            // left rear cluster
            "1,10:W:F:0",
            "0,11:W:AI:0",
            "0,11:S:AI:0",
            "1,10:S:FW:0",
            "2,9:S:FW:0",

            // thrust ring
            "7,7::R:14",
            "8,7::R:15",
            "8,8::R:14",
            "7,8::R:15"
    };
    // and
    memset(ledConfigs_arr(), 0, sizeof(*ledConfigs_arr()));

    // and
    bool result = true;

    // when
    for (unsigned index = 0; index < ARRAYLEN(ledStripConfigCommands); index++) {
        result = result && parseLedStripConfig(index, ledStripConfigCommands[index]);
    }

    // then
    EXPECT_EQ(true, result);
    EXPECT_EQ(30, ledCounts.count);
    EXPECT_EQ(4, ledCounts.ring);


    // and
    for (int index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
#ifdef DEBUG_LEDSTRIP
        printf("iteration: %d\n", index);
#endif
        EXPECT_EQ(ledGetXY(&expectedLedStripConfig[index]), ledGetXY(ledConfigs(index)));
        EXPECT_EQ(ledGetFunction(&expectedLedStripConfig[index]), ledGetFunction(ledConfigs(index)));
        EXPECT_EQ(ledGetOverlay(&expectedLedStripConfig[index]), ledGetOverlay(ledConfigs(index)));
        EXPECT_EQ(ledGetColor(&expectedLedStripConfig[index]), ledGetColor(ledConfigs(index)));
        EXPECT_EQ(ledGetDirection(&expectedLedStripConfig[index]), ledGetDirection(ledConfigs(index)));
        EXPECT_EQ(ledGetParams(&expectedLedStripConfig[index]), ledGetParams(ledConfigs(index)));
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
    memset(ledConfigs_arr(), 0, sizeof(*ledConfigs_arr()));

    // and
    static const ledConfig_t testLedConfigs[] = {
        DEFINE_LED( 2,  2, 0, LD(SOUTH) | LD(EAST) , LF(ARM_STATE)   , LO(INDICATOR), 0),
        DEFINE_LED( 2,  1, 0, LD(EAST)             , LF(FLIGHT_MODE) , LO(WARNING)  , 0),
        DEFINE_LED( 2,  0, 0, LD(NORTH) | LD(EAST) , LF(ARM_STATE)   , LO(INDICATOR), 0),
        DEFINE_LED( 1,  0, 0, LD(NORTH)            , LF(FLIGHT_MODE) , LO(WARNING)  , 0),
        DEFINE_LED( 0,  0, 0, LD(NORTH) | LD(WEST) , LF(ARM_STATE)   , LO(INDICATOR), 0),
        DEFINE_LED( 0,  1, 0, LD(WEST)             , LF(FLIGHT_MODE) , LO(WARNING)  , 0),
        DEFINE_LED( 0,  2, 0, LD(SOUTH) | LD(WEST) , LF(ARM_STATE)   , LO(INDICATOR), 0),
        DEFINE_LED( 1,  2, 0, LD(SOUTH)            , LF(FLIGHT_MODE) , LO(WARNING)  , 0),
    };
    memcpy(ledConfigs_arr(), &testLedConfigs, sizeof(testLedConfigs));

    // when
    updateLedCount();
    determineLedStripDimensions();

    // then
    EXPECT_EQ(8, ledCounts.count);
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
    memset(ledConfigs_arr(), 0, sizeof(*ledConfigs_arr()));

    // and
    static const ledConfig_t testLedConfigs[] = {
        DEFINE_LED( 1,  1, 0, LD(SOUTH) | LD(EAST) , LF(FLIGHT_MODE) , LO(INDICATOR), 0),
        DEFINE_LED( 1,  0, 0, LD(NORTH) | LD(EAST) , LF(FLIGHT_MODE) , LO(INDICATOR), 0),
        DEFINE_LED( 0,  0, 0, LD(NORTH) | LD(WEST) , LF(FLIGHT_MODE) , LO(INDICATOR), 0),
        DEFINE_LED( 0,  1, 0, LD(SOUTH) | LD(WEST) , LF(FLIGHT_MODE) , LO(INDICATOR), 0),
    };
    memcpy(ledConfigs_arr(), &testLedConfigs, sizeof(testLedConfigs));

    // when
    updateLedCount();
    determineLedStripDimensions();

    // then
    EXPECT_EQ(4, ledCounts.count);
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
        LED( 1, 14, 0, LD(SOUTH) , LF(FLIGHT_MODE) , LO(INDICATOR) , 0 ),

        LED( 0, 13, 0, LD(WEST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),
        LED( 0, 12, 0, LD(WEST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),

        LED( 0, 11, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 0, 10, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 0,  9, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 0,  8, 0, LD(WEST)  , LF(FLIGHT_MODE) , LO(WARNING) , 0 ),
        LED( 0,  7, 0, LD(WEST)  , LF(FLIGHT_MODE) , LO(WARNING) , 0 ),
        LED( 0,  6, 0, LD(WEST)  , LF(FLIGHT_MODE) , LO(WARNING) , 0 ),
        LED( 0,  5, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 0,  4, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 0,  3, 0, LD(WEST)  , LF(FLIGHT_MODE) , 0 , 0 ),

        LED( 0,  2, 0, LD(WEST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),
        LED( 0,  1, 0, LD(WEST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),

        LED( 1,  0, 0, LD(NORTH) , LF(FLIGHT_MODE) , LO(INDICATOR)  , 0 ),
        LED( 2,  0, 0, LD(NORTH) , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 3,  0, 0, LD(NORTH) , LF(FLIGHT_MODE) , LO(INDICATOR) , 0 ),

        LED( 4,  1, 0, LD(EAST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),
        LED( 4,  2, 0, LD(EAST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),

        LED( 4,  3, 0, LD(EAST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 4,  4, 0, LD(EAST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 4,  5, 0, LD(EAST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 4,  6, 0, LD(EAST)  , LF(FLIGHT_MODE) , LO(WARNING) , 0 ),
        LED( 4,  7, 0, LD(EAST)  , LF(FLIGHT_MODE) , LO(WARNING) , 0 ),
        LED( 4,  8, 0, LD(EAST)  , LF(FLIGHT_MODE) , LO(WARNING) , 0 ),
        LED( 4,  9, 0, LD(EAST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 4, 10, 0, LD(EAST)  , LF(FLIGHT_MODE) , 0 , 0 ),
        LED( 4, 11, 0, LD(EAST)  , LF(FLIGHT_MODE) , 0 , 0 ),

        LED( 4, 12, 0, LD(EAST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),
        LED( 4, 13, 0, LD(EAST)  , LF(ARM_STATE) , LO(INDICATOR)  , 0 ),

        LED( 3, 14, 0, LD(SOUTH) , LF(FLIGHT_MODE) , LO(INDICATOR) , 0 ),

 */

hsvColor_t testColors[LED_CONFIGURABLE_COLOR_COUNT];

#define TEST_COLOR_COUNT 4

TEST(ColorTest, parseColor)
{
    // given
    memset(colors_arr(), 0, sizeof(*colors_arr()));

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
    for (int index = 0; index < TEST_COLOR_COUNT; index++) {
#ifdef DEBUG_LEDSTRIP
        printf("parse iteration: %d\n", index);
#endif

        parseColor(index, testColors[index]);
    }

    // then

    for (int index = 0; index < TEST_COLOR_COUNT; index++) {
#ifdef DEBUG_LEDSTRIP
        printf("iteration: %d\n", index);
#endif

        EXPECT_EQ(expectedColors[index].h, colors(index)->h);
        EXPECT_EQ(expectedColors[index].s, colors(index)->s);
        EXPECT_EQ(expectedColors[index].v, colors(index)->v);
    }
}

extern "C" {

uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
int16_t rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
uint32_t rcModeActivationMask;

batteryState_e getBatteryState(void) {
    return BATTERY_OK;
}

void ws2811LedStripInit(void) {}
void ws2811UpdateStrip(void) {}

void setLedValue(int index, const uint8_t value) {
    UNUSED(index);
    UNUSED(value);
}

void setLedHsv(int index, const hsvColor_t *color) {
    UNUSED(index);
    UNUSED(color);
}

void getLedHsv(int index, hsvColor_t *color) {
    UNUSED(index);
    UNUSED(color);
}


void scaleLedValue(int index, const uint8_t scalePercent) {
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

bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
bool failsafeIsActive() { return false; }
bool rxIsReceivingSignal() { return true; }
bool sensors(uint32_t mask) { UNUSED(mask); return true; }
uint8_t calculateBatteryCapacityRemainingPercentage(void) { return 40; }

uint8_t GPS_numSat;
uint8_t stateFlags;
uint16_t rssi;

}
