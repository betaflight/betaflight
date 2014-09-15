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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include <platform.h>

#include <build_config.h>

#ifdef LED_STRIP

#include "drivers/light_ws2811strip.h"
#include "drivers/system.h"
#include "drivers/serial.h"

#include <common/maths.h>
#include <common/printf.h>
#include <common/typeconversion.h>


#include "sensors/battery.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "rx/rx.h"
#include "io/rc_controls.h"

#include "io/ledstrip.h"

#if MAX_LED_STRIP_LENGTH > WS2811_LED_STRIP_LENGTH
#error "Led strip length must match driver"
#endif

//#define USE_LED_ANIMATION

#define LED_WHITE   {255, 255, 255}
#define LED_BLACK   {0,   0,   0  }

#define LED_RED          {255, 0,   0  }
#define LED_ORANGE       {255, 128, 0  }
#define LED_YELLOW       {255, 255, 0  }
#define LED_LIME_GREEN   {128, 255, 0  }
#define LED_CYAN         {0,   255, 255}
#define LED_GREEN        {0,   255, 0  }
#define LED_LIGHT_BLUE   {0,   128, 255}
#define LED_BLUE         {0,   0,   255}
#define LED_DARK_MAGENTA {128, 0,   128}
#define LED_PINK         {255, 0,   255}
#define LED_DARK_VIOLET  {128, 0,   255}
#define LED_DEEP_PINK    {255, 0,   128}

const rgbColor24bpp_t black = { LED_BLACK };
const rgbColor24bpp_t white = { LED_WHITE };

const rgbColor24bpp_t red = { LED_RED };
const rgbColor24bpp_t orange = { LED_ORANGE };
const rgbColor24bpp_t green = { LED_GREEN };
const rgbColor24bpp_t blue = { LED_BLUE };


uint8_t ledGridWidth;
uint8_t ledGridHeight;
uint8_t ledCount;

ledConfig_t *ledConfigs;

const ledConfig_t defaultLedStripConfig[] = {
    { CALCULATE_LED_XY( 2,  2), LED_DIRECTION_SOUTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY( 2,  1), LED_DIRECTION_EAST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_BATTERY },
    { CALCULATE_LED_XY( 2,  0), LED_DIRECTION_NORTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY( 1,  0), LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE },
    { CALCULATE_LED_XY( 0,  0), LED_DIRECTION_NORTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY( 0,  1), LED_DIRECTION_WEST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_BATTERY },
    { CALCULATE_LED_XY( 0,  2), LED_DIRECTION_SOUTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY( 1,  2), LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_BATTERY },
    { CALCULATE_LED_XY( 1,  1), LED_DIRECTION_UP | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_BATTERY },
    { CALCULATE_LED_XY( 1,  1), LED_DIRECTION_UP | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_BATTERY },
    { CALCULATE_LED_XY( 1,  1), LED_DIRECTION_DOWN | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_BATTERY },
    { CALCULATE_LED_XY( 1,  1), LED_DIRECTION_DOWN | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_BATTERY },
};


/*
 * 6 coords @nn,nn
 * 4 direction @##
 * 6 modes @####
 * = 16 bytes per led
 * 16 * 32 leds = 512 bytes storage needed worst case.
 * = not efficient to store led configs as strings in flash.
 * = becomes a problem to send all the data via cli due to serial/cli buffers
 */

typedef enum {
    X_COORDINATE,
    Y_COORDINATE,
    DIRECTIONS,
    FUNCTIONS
} parseState_e;

#define PARSE_STATE_COUNT 4

static const char chunkSeparators[PARSE_STATE_COUNT] = {',', ':', ':', '\0' };

static const char directionCodes[] = { 'N', 'E', 'S', 'W', 'U', 'D' };
#define DIRECTION_COUNT (sizeof(directionCodes) / sizeof(directionCodes[0]))
static const uint8_t directionMappings[DIRECTION_COUNT] = {
    LED_DIRECTION_NORTH,
    LED_DIRECTION_EAST,
    LED_DIRECTION_SOUTH,
    LED_DIRECTION_WEST,
    LED_DIRECTION_UP,
    LED_DIRECTION_DOWN
};

static const char functionCodes[] = { 'I', 'B', 'F', 'A' };
#define FUNCTION_COUNT (sizeof(functionCodes) / sizeof(functionCodes[0]))
static const uint16_t functionMappings[FUNCTION_COUNT] = {
    LED_FUNCTION_INDICATOR,
    LED_FUNCTION_BATTERY,
    LED_FUNCTION_FLIGHT_MODE,
    LED_FUNCTION_ARM_STATE
};

// grid offsets
uint8_t highestYValueForNorth;
uint8_t lowestYValueForSouth;
uint8_t highestXValueForWest;
uint8_t lowestXValueForEast;

void determineLedStripDimensions(void)
{
    ledGridWidth = 0;
    ledGridHeight = 0;

    uint8_t ledIndex;
    const ledConfig_t *ledConfig;

    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        ledConfig = &ledConfigs[ledIndex];

        if (GET_LED_X(ledConfig) >= ledGridWidth) {
            ledGridWidth = GET_LED_X(ledConfig) + 1;
        }
        if (GET_LED_Y(ledConfig) >= ledGridHeight) {
            ledGridHeight = GET_LED_Y(ledConfig) + 1;
        }
    }
}

void determineOrientationLimits(void)
{
    bool isOddHeight = (ledGridHeight & 1);
    bool isOddWidth = (ledGridWidth & 1);
    uint8_t heightModifier = isOddHeight ? 1 : 0;
    uint8_t widthModifier = isOddWidth ? 1 : 0;

    highestYValueForNorth = (ledGridHeight / 2) - 1;
    lowestYValueForSouth = (ledGridHeight / 2) + heightModifier;
    highestXValueForWest = (ledGridWidth / 2) - 1;
    lowestXValueForEast = (ledGridWidth / 2) + widthModifier;
}

void updateLedCount(void)
{
    uint8_t ledIndex;
    ledCount = 0;
    for (ledIndex = 0; ledIndex < MAX_LED_STRIP_LENGTH; ledIndex++) {
        if (ledConfigs[ledIndex].flags == 0 && ledConfigs[ledIndex].xy == 0) {
            break;
        }
        ledCount++;
    }
}

static void reevalulateLedConfig(void)
{
    updateLedCount();
    determineLedStripDimensions();
    determineOrientationLimits();
}

#define CHUNK_BUFFER_SIZE 10

#define NEXT_PARSE_STATE(parseState) ((parseState + 1) % PARSE_STATE_COUNT)


bool parseLedStripConfig(uint8_t ledIndex, const char *config)
{
    char chunk[CHUNK_BUFFER_SIZE];
    uint8_t chunkIndex;
    uint8_t val;

    uint8_t parseState = X_COORDINATE;
    bool ok = true;

    if (ledIndex >= MAX_LED_STRIP_LENGTH) {
        return !ok;
    }

    ledConfig_t *ledConfig = &ledConfigs[ledIndex];
    memset(ledConfig, 0, sizeof(ledConfig_t));

    while (ok) {

        char chunkSeparator = chunkSeparators[parseState];

        memset(&chunk, 0, sizeof(chunk));
        chunkIndex = 0;

        while (*config && chunkIndex < CHUNK_BUFFER_SIZE && *config != chunkSeparator) {
            chunk[chunkIndex++] = *config++;
        }

        if (*config++ != chunkSeparator) {
            ok = false;
            break;
        }

        switch((parseState_e)parseState) {
            case X_COORDINATE:
                val = atoi(chunk);
                ledConfig->xy |= CALCULATE_LED_X(val);
                break;
            case Y_COORDINATE:
                val = atoi(chunk);
                ledConfig->xy |= CALCULATE_LED_Y(val);
                break;
            case DIRECTIONS:
                for (chunkIndex = 0; chunk[chunkIndex] && chunkIndex < CHUNK_BUFFER_SIZE; chunkIndex++) {
                    for (uint8_t mappingIndex = 0; mappingIndex < DIRECTION_COUNT; mappingIndex++) {
                        if (directionCodes[mappingIndex] == chunk[chunkIndex]) {
                            ledConfig->flags |= directionMappings[mappingIndex];
                            break;
                        }
                    }
                }
                break;
            case FUNCTIONS:
                for (chunkIndex = 0; chunk[chunkIndex] && chunkIndex < CHUNK_BUFFER_SIZE; chunkIndex++) {
                    for (uint8_t mappingIndex = 0; mappingIndex < FUNCTION_COUNT; mappingIndex++) {
                        if (functionCodes[mappingIndex] == chunk[chunkIndex]) {
                            ledConfig->flags |= functionMappings[mappingIndex];
                            break;
                        }
                    }
                }
                break;
        }

        parseState++;
        if (parseState >= PARSE_STATE_COUNT) {
            break;
        }
    }

    if (!ok) {
        memset(ledConfig, 0, sizeof(ledConfig_t));
    }

    reevalulateLedConfig();

    return ok;
}

void generateLedConfig(uint8_t ledIndex, char *ledConfigBuffer, size_t bufferSize)
{
    char functions[FUNCTION_COUNT];
    char directions[DIRECTION_COUNT];
    uint8_t index;
    uint8_t mappingIndex;
    ledConfig_t *ledConfig = &ledConfigs[ledIndex];

    memset(ledConfigBuffer, 0, bufferSize);
    memset(&functions, 0, sizeof(functions));
    memset(&directions, 0, sizeof(directions));

    for (mappingIndex = 0, index = 0; mappingIndex < FUNCTION_COUNT; mappingIndex++) {
        if (ledConfig->flags & functionMappings[mappingIndex]) {
            functions[index++] = functionCodes[mappingIndex];
        }
    }

    for (mappingIndex = 0, index = 0; mappingIndex < DIRECTION_COUNT; mappingIndex++) {
        if (ledConfig->flags & directionMappings[mappingIndex]) {
            directions[index++] = directionCodes[mappingIndex];
        }
    }

    sprintf(ledConfigBuffer, "%u,%u:%s:%s", GET_LED_X(ledConfig), GET_LED_Y(ledConfig), directions, functions);
}

// timers
uint32_t nextAnimationUpdateAt = 0;
uint32_t nextIndicatorFlashAt = 0;
uint32_t nextBatteryFlashAt = 0;

#define LED_STRIP_20HZ ((1000 * 1000) / 20)
#define LED_STRIP_10HZ ((1000 * 1000) / 10)
#define LED_STRIP_5HZ ((1000 * 1000) / 5)

#define LED_DIRECTION_COUNT 6

struct modeColors_s {
    rgbColor24bpp_t north;
    rgbColor24bpp_t east;
    rgbColor24bpp_t south;
    rgbColor24bpp_t west;
    rgbColor24bpp_t up;
    rgbColor24bpp_t down;
};

typedef union {
    rgbColor24bpp_t raw[LED_DIRECTION_COUNT];
    struct modeColors_s colors;
} modeColors_t;

static const modeColors_t orientationModeColors = {
    .raw = {
        {LED_WHITE},
        {LED_DARK_VIOLET},
        {LED_RED},
        {LED_DEEP_PINK},
        {LED_BLUE},
        {LED_ORANGE}
    }
};

static const modeColors_t headfreeModeColors = {
    .raw = {
        {LED_LIME_GREEN},
        {LED_DARK_VIOLET},
        {LED_ORANGE},
        {LED_DEEP_PINK},
        {LED_BLUE},
        {LED_ORANGE}
    }
};

static const modeColors_t horizonModeColors = {
    .raw = {
        {LED_BLUE},
        {LED_DARK_VIOLET},
        {LED_YELLOW},
        {LED_DEEP_PINK},
        {LED_BLUE},
        {LED_ORANGE}
    }
};

static const modeColors_t angleModeColors = {
    .raw = {
        {LED_CYAN},
        {LED_DARK_VIOLET},
        {LED_YELLOW},
        {LED_DEEP_PINK},
        {LED_BLUE},
        {LED_ORANGE}
    }
};

static const modeColors_t magModeColors = {
    .raw = {
        {LED_PINK},
        {LED_DARK_VIOLET},
        {LED_ORANGE},
        {LED_DEEP_PINK},
        {LED_BLUE},
        {LED_ORANGE}
    }
};

static const modeColors_t baroModeColors = {
    .raw = {
        {LED_LIGHT_BLUE},
        {LED_DARK_VIOLET},
        {LED_RED},
        {LED_DEEP_PINK},
        {LED_BLUE},
        {LED_ORANGE}
    }
};

void applyDirectionalModeColor(const uint8_t ledIndex, const ledConfig_t *ledConfig, const modeColors_t *modeColors)
{
    // apply up/down colors regardless of quadrant.
    if ((ledConfig->flags & LED_DIRECTION_UP)) {
        setLedColor(ledIndex, &modeColors->colors.up);
    }

    if ((ledConfig->flags & LED_DIRECTION_DOWN)) {
        setLedColor(ledIndex, &modeColors->colors.down);
    }

    // override with n/e/s/w colors to each n/s e/w half - bail at first match.
    if ((ledConfig->flags & LED_DIRECTION_WEST) && GET_LED_X(ledConfig) <= highestXValueForWest) {
        setLedColor(ledIndex, &modeColors->colors.west);
    }

    if ((ledConfig->flags & LED_DIRECTION_EAST) && GET_LED_X(ledConfig) >= lowestXValueForEast) {
        setLedColor(ledIndex, &modeColors->colors.east);
    }

    if ((ledConfig->flags & LED_DIRECTION_NORTH) && GET_LED_Y(ledConfig) <= highestYValueForNorth) {
        setLedColor(ledIndex, &modeColors->colors.north);
    }

    if ((ledConfig->flags & LED_DIRECTION_SOUTH) && GET_LED_Y(ledConfig) >= lowestYValueForSouth) {
        setLedColor(ledIndex, &modeColors->colors.south);
    }

}

typedef enum {
    QUADRANT_NORTH_EAST = 1,
    QUADRANT_SOUTH_EAST,
    QUADRANT_SOUTH_WEST,
    QUADRANT_NORTH_WEST
} quadrant_e;

void applyQuadrantColor(const uint8_t ledIndex, const ledConfig_t *ledConfig, const quadrant_e quadrant, const rgbColor24bpp_t *color)
{
    switch (quadrant) {
        case QUADRANT_NORTH_EAST:
            if (GET_LED_Y(ledConfig) <= highestYValueForNorth && GET_LED_X(ledConfig) >= lowestXValueForEast) {
                setLedColor(ledIndex, color);
            }
            return;

        case QUADRANT_SOUTH_EAST:
            if (GET_LED_Y(ledConfig) >= lowestYValueForSouth && GET_LED_X(ledConfig) >= lowestXValueForEast) {
                setLedColor(ledIndex, color);
            }
            return;

        case QUADRANT_SOUTH_WEST:
            if (GET_LED_Y(ledConfig) >= lowestYValueForSouth && GET_LED_X(ledConfig) <= highestXValueForWest) {
                setLedColor(ledIndex, color);
            }
            return;

        case QUADRANT_NORTH_WEST:
            if (GET_LED_Y(ledConfig) <= highestYValueForNorth && GET_LED_X(ledConfig) <= highestXValueForWest) {
                setLedColor(ledIndex, color);
            }
            return;
    }
}

void applyLedModeLayer(void)
{
    const ledConfig_t *ledConfig;

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        setLedColor(ledIndex, &black);

        if (!(ledConfig->flags & LED_FUNCTION_FLIGHT_MODE)) {
            if (ledConfig->flags & LED_FUNCTION_ARM_STATE) {
                if (!ARMING_FLAG(ARMED)) {
                    setLedColor(ledIndex, &green);
                } else {
                    setLedColor(ledIndex, &blue);
                }
            }
            continue;
        }

        applyDirectionalModeColor(ledIndex, ledConfig, &orientationModeColors);

        if (FLIGHT_MODE(HEADFREE_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &headfreeModeColors);
#ifdef MAG
        } else if (FLIGHT_MODE(MAG_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &magModeColors);
#endif
#ifdef BARO
        } else if (FLIGHT_MODE(BARO_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &baroModeColors);
#endif
        } else if (FLIGHT_MODE(HORIZON_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &horizonModeColors);
        } else if (FLIGHT_MODE(ANGLE_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &angleModeColors);
        }
    }
}

void applyLedLowBatteryLayer(uint8_t batteryFlashState)
{
    const ledConfig_t *ledConfig;

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (!(ledConfig->flags & LED_FUNCTION_BATTERY)) {
            continue;
        }

        if (batteryFlashState == 0) {
            setLedColor(ledIndex, &red);
        } else {
            setLedColor(ledIndex, &black);
        }
    }
}

void applyLedIndicatorLayer(uint8_t indicatorFlashState)
{
    const ledConfig_t *ledConfig;
    static const rgbColor24bpp_t *flashColor;


    if (indicatorFlashState == 0) {
        flashColor = &orange;
    } else {
        flashColor = &black;
    }


    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (!(ledConfig->flags & LED_FUNCTION_INDICATOR)) {
            continue;
        }

        if (rcCommand[ROLL] > 50) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_EAST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_EAST, flashColor);
        }

        if (rcCommand[ROLL] < -50) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_WEST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_WEST, flashColor);
        }

        if (rcCommand[PITCH] > 50) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_EAST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_WEST, flashColor);
        }

        if (rcCommand[PITCH] < -50) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_EAST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_WEST, flashColor);
        }
    }
}

static uint8_t frameCounter = 0;

static uint8_t previousRow;
static uint8_t currentRow;
static uint8_t nextRow;

static void updateLedAnimationState(void)
{
    uint8_t animationFrames = ledGridHeight;

    previousRow = (frameCounter + animationFrames - 1) % animationFrames;
    currentRow = frameCounter;
    nextRow = (frameCounter + 1) % animationFrames;

    frameCounter = (frameCounter + 1) % animationFrames;
}

#ifdef USE_LED_ANIMATION
static void applyLedAnimationLayer(void)
{
    const ledConfig_t *ledConfig;

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (GET_LED_Y(ledConfig) == previousRow) {
            setLedColor(ledIndex, &white);
            setLedBrightness(ledIndex, 50);

        } else if (GET_LED_Y(ledConfig) == currentRow) {
            setLedColor(ledIndex, &white);
        } else if (GET_LED_Y(ledConfig) == nextRow) {
            setLedBrightness(ledIndex, 50);
        }
    }
}
#endif

void updateLedStrip(void)
{
    if (!isWS2811LedStripReady()) {
        return;
    }

    uint32_t now = micros();

    bool animationUpdateNow = (int32_t)(now - nextAnimationUpdateAt) >= 0L;
    bool indicatorFlashNow = (int32_t)(now - nextIndicatorFlashAt) >= 0L;
    bool batteryFlashNow = (int32_t)(now - nextBatteryFlashAt) >= 0L;

    if (!(batteryFlashNow || indicatorFlashNow || animationUpdateNow)) {
        return;
    }

    static uint8_t indicatorFlashState = 0;
    static uint8_t batteryFlashState = 0;
    static bool batteryWarningEnabled = false;

    // LAYER 1

    applyLedModeLayer();

    // LAYER 2

    if (batteryFlashNow) {
        nextBatteryFlashAt = now + LED_STRIP_10HZ;

        if (batteryFlashState == 0) {
            batteryFlashState = 1;

            batteryWarningEnabled = feature(FEATURE_VBAT) && shouldSoundBatteryAlarm();
        } else {
            batteryFlashState = 0;

        }
    }

    if (batteryWarningEnabled) {
        applyLedLowBatteryLayer(batteryFlashState);
    }

    // LAYER 3

    if (indicatorFlashNow) {

        uint8_t rollScale = abs(rcCommand[ROLL]) / 50;
        uint8_t pitchScale = abs(rcCommand[PITCH]) / 50;
        uint8_t scale = max(rollScale, pitchScale);
        nextIndicatorFlashAt = now + (LED_STRIP_5HZ / max(1, scale));

        if (indicatorFlashState == 0) {
            indicatorFlashState = 1;
        } else {
            indicatorFlashState = 0;
        }
    }

    applyLedIndicatorLayer(indicatorFlashState);

    if (animationUpdateNow) {
        nextAnimationUpdateAt = now + LED_STRIP_20HZ;
        updateLedAnimationState();
    }

#ifdef USE_LED_ANIMATION
    applyLedAnimationLayer();
#endif
    ws2811UpdateStrip();
}

void applyDefaultLedStripConfig(ledConfig_t *ledConfigs)
{
    memset(ledConfigs, 0, MAX_LED_STRIP_LENGTH * sizeof(ledConfig_t));
    memcpy(ledConfigs, &defaultLedStripConfig, sizeof(defaultLedStripConfig));
    reevalulateLedConfig();
}

void ledStripInit(ledConfig_t *ledConfigsToUse)
{
    ledConfigs = ledConfigsToUse;
    reevalulateLedConfig();
}
#endif
