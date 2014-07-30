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

#include <platform.h>

#ifdef LED_STRIP
#include <common/maths.h>

#include "drivers/light_ws2811strip.h"
#include "drivers/system.h"

#include "sensors/battery.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "rx/rx.h"
#include "io/rc_controls.h"

#include "io/ledstrip.h"

#define LED_WHITE  {255, 255, 255}
#define LED_BLACK  {0,   0,   0  }
#define LED_RED    {255, 0,   0  }
#define LED_GREEN  {0,   255, 0  }
#define LED_BLUE   {0,   0,   255}
#define LED_CYAN   {0,   255, 255}
#define LED_YELLOW {255, 255, 0  }
#define LED_ORANGE {255, 128, 0  }
#define LED_PINK   {255, 0,   128}
#define LED_PURPLE {192, 64,  255}

const rgbColor24bpp_t black = { LED_BLACK };
const rgbColor24bpp_t red = { LED_RED };
const rgbColor24bpp_t orange = { LED_ORANGE };
const rgbColor24bpp_t white = { LED_WHITE };
const rgbColor24bpp_t green = { LED_GREEN };
const rgbColor24bpp_t blue = { LED_BLUE };

/*
 * 0..5   - rear right cluster,  0..2 rear 3..5 right
 * 6..11  - front right cluster, 6..8 rear, 9..11 front
 * 12..15 - front center cluster
 * 16..21 - front left cluster,  16..18 front, 19..21 rear
 * 22..27 - rear left cluster,   22..24 left, 25..27 rear
 */

typedef enum {
    LED_DISABLED = 0,
    LED_DIRECTION_NORTH    = (1 << 0),
    LED_DIRECTION_EAST     = (1 << 1),
    LED_DIRECTION_SOUTH    = (1 << 2),
    LED_DIRECTION_WEST     = (1 << 3),
    LED_DIRECTION_UP       = (1 << 4),
    LED_DIRECTION_DOWN     = (1 << 5),
    LED_FUNCTION_INDICATOR = (1 << 6),
    LED_FUNCTION_BATTERY   = (1 << 7),
    LED_FUNCTION_MODE      = (1 << 8),
    LED_FUNCTION_ARM_STATE = (1 << 9)
} ledFlag_e;

#define LED_X_BIT_OFFSET 4
#define LED_Y_BIT_OFFSET 0

#define LED_XY_MASK (0x0F)

#define LED_X(ledConfig) ((ledConfig->xy >> LED_X_BIT_OFFSET) & LED_XY_MASK)
#define LED_Y(ledConfig) ((ledConfig->xy >> LED_Y_BIT_OFFSET) & LED_XY_MASK)

#define LED_XY(x,y) (((x & LED_XY_MASK) << LED_X_BIT_OFFSET) | ((y & LED_XY_MASK) << LED_Y_BIT_OFFSET))

typedef struct ledConfig_s {
    uint8_t xy; // see LED_X/Y_MASK defines
    uint16_t flags; // see ledFlag_e
} ledConfig_t;

static uint8_t ledGridWidth;
static uint8_t ledGridHeight;

#ifdef USE_ALTERNATE_LED_LAYOUT
static const ledConfig_t ledConfigs[WS2811_LED_STRIP_LENGTH] = {
        { LED_XY( 1, 14), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { LED_XY( 0, 13), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 0, 12), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { LED_XY( 0, 11), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 0, 10), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 0,  9), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 0,  8), LED_DIRECTION_WEST  | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 0,  7), LED_DIRECTION_WEST  | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 0,  6), LED_DIRECTION_WEST  | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 0,  5), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 0,  4), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 0,  3), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },

        { LED_XY( 0,  2), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 0,  1), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { LED_XY( 1,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 2,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE | LED_FUNCTION_ARM_STATE },
        { LED_XY( 3,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { LED_XY( 4,  1), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 4,  2), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { LED_XY( 4,  3), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },
        { LED_XY( 4,  4), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },
        { LED_XY( 4,  5), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },
        { LED_XY( 4,  6), LED_DIRECTION_EAST  | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 4,  7), LED_DIRECTION_EAST  | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 4,  8), LED_DIRECTION_EAST  | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 4,  9), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },
        { LED_XY( 4, 10), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },
        { LED_XY( 4, 11), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },

        { LED_XY( 4, 12), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 4, 13), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },

        { LED_XY( 3, 14), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
};
#else
static const ledConfig_t ledConfigs[WS2811_LED_STRIP_LENGTH] = {
        { LED_XY( 9,  9), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY(10, 10), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY(11, 11), LED_DIRECTION_SOUTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY(11, 11), LED_DIRECTION_EAST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY(10, 10), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },
        { LED_XY( 9,  9), LED_DIRECTION_EAST  | LED_FUNCTION_MODE },

        { LED_XY(10,  5), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE },
        { LED_XY(11,  4), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE },
        { LED_XY(12,  3), LED_DIRECTION_SOUTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY(12,  2), LED_DIRECTION_NORTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY(11,  1), LED_DIRECTION_NORTH | LED_FUNCTION_MODE },
        { LED_XY(10,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE },

        { LED_XY( 7,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 6,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 5,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 4,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },

        { LED_XY( 2,  0), LED_DIRECTION_NORTH | LED_FUNCTION_MODE },
        { LED_XY( 1,  1), LED_DIRECTION_NORTH | LED_FUNCTION_MODE },
        { LED_XY( 0,  2), LED_DIRECTION_NORTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 0,  3), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 1,  4), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 2,  5), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },

        { LED_XY( 2,  9), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 1, 10), LED_DIRECTION_WEST  | LED_FUNCTION_MODE },
        { LED_XY( 0, 11), LED_DIRECTION_WEST  | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 0, 11), LED_DIRECTION_SOUTH | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
        { LED_XY( 1, 10), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY },
        { LED_XY( 2,  9), LED_DIRECTION_SOUTH | LED_FUNCTION_MODE | LED_FUNCTION_BATTERY }
};
#endif

// grid offsets
uint8_t highestYValueForNorth;
uint8_t lowestYValueForSouth;
uint8_t highestXValueForWest;
uint8_t lowestXValueForEast;

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
        {LED_BLUE},
        {LED_RED},
        {LED_GREEN},
        {LED_PURPLE},
        {LED_CYAN}
    }
};

static const modeColors_t headfreeModeColors = {
    .raw = {
        {LED_PINK},
        {LED_BLACK},
        {LED_ORANGE},
        {LED_BLACK},
        {LED_BLACK},
        {LED_BLACK}
    }
};

static const modeColors_t horizonModeColors = {
    .raw = {
        {LED_BLUE},
        {LED_BLACK},
        {LED_YELLOW},
        {LED_BLACK},
        {LED_BLACK},
        {LED_BLACK}
    }
};

static const modeColors_t angleModeColors = {
    .raw = {
        {LED_CYAN},
        {LED_BLACK},
        {LED_YELLOW},
        {LED_BLACK},
        {LED_BLACK},
        {LED_BLACK}
    }
};

static const modeColors_t magModeColors = {
    .raw = {
        {LED_PURPLE},
        {LED_BLACK},
        {LED_ORANGE},
        {LED_BLACK},
        {LED_BLACK},
        {LED_BLACK}
    }
};

void applyDirectionalModeColor(const uint8_t ledIndex, const ledConfig_t *ledConfig, const modeColors_t *modeColors)
{
    if (ledConfig->flags & LED_DIRECTION_NORTH && LED_Y(ledConfig) < highestYValueForNorth) {
        setLedColor(ledIndex, &modeColors->colors.north);
        return;
    }

    if (ledConfig->flags & LED_DIRECTION_SOUTH && LED_Y(ledConfig) >= lowestYValueForSouth) {
        setLedColor(ledIndex, &modeColors->colors.south);
        return;
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
            if (LED_Y(ledConfig) <= highestYValueForNorth && LED_X(ledConfig) >= lowestXValueForEast) {
                setLedColor(ledIndex, color);
            }
            return;

        case QUADRANT_SOUTH_EAST:
            if (LED_Y(ledConfig) >= lowestYValueForSouth && LED_X(ledConfig) >= lowestXValueForEast) {
                setLedColor(ledIndex, color);
            }
            return;

        case QUADRANT_SOUTH_WEST:
            if (LED_Y(ledConfig) >= lowestYValueForSouth && LED_X(ledConfig) <= highestXValueForWest) {
                setLedColor(ledIndex, color);
            }
            return;

        case QUADRANT_NORTH_WEST:
            if (LED_Y(ledConfig) <= highestYValueForNorth && LED_X(ledConfig) <= highestXValueForWest) {
                setLedColor(ledIndex, color);
            }
            return;
    }
}

void applyLedModeLayer(void)
{
    const ledConfig_t *ledConfig;

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < WS2811_LED_STRIP_LENGTH; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        setLedColor(ledIndex, &black);

        if (!(ledConfig->flags & LED_FUNCTION_MODE)) {
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
    for (ledIndex = 0; ledIndex < WS2811_LED_STRIP_LENGTH; ledIndex++) {

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
    for (ledIndex = 0; ledIndex < WS2811_LED_STRIP_LENGTH; ledIndex++) {

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

static void applyLedAnimationLayer(void)
{
    const ledConfig_t *ledConfig;

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < WS2811_LED_STRIP_LENGTH; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (LED_Y(ledConfig) == previousRow) {
            setLedColor(ledIndex, &white);
            setLedBrightness(ledIndex, 50);

        } else if (LED_Y(ledConfig) == currentRow) {
            setLedColor(ledIndex, &white);
        } else if (LED_Y(ledConfig) == nextRow) {
            setLedBrightness(ledIndex, 50);
        }
    }
}

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

    applyLedAnimationLayer();

    ws2811UpdateStrip();
}

void determineLedStripDimensions()
{
    ledGridWidth = 0;
    ledGridHeight = 0;

    uint8_t ledIndex;
    const ledConfig_t *ledConfig;

    for (ledIndex = 0; ledIndex < WS2811_LED_STRIP_LENGTH; ledIndex++) {
        ledConfig = &ledConfigs[ledIndex];

        if (LED_X(ledConfig) >= ledGridWidth) {
            ledGridWidth = LED_X(ledConfig) + 1;
        }
        if (LED_Y(ledConfig) >= ledGridHeight) {
            ledGridHeight = LED_Y(ledConfig) + 1;
        }
    }
}

void determineOrientationLimits(void)
{
    highestYValueForNorth = (ledGridHeight / 2) - 1;
    if (highestYValueForNorth > 1) { // support small grid (e.g. gridwidth 5)
        highestYValueForNorth &= ~(1 << 0); // make even
    }

    lowestYValueForSouth = (ledGridHeight / 2) - 1;
    if (lowestYValueForSouth & 1) {
        lowestYValueForSouth = min(lowestYValueForSouth + 1, ledGridHeight - 1);
    }

    highestXValueForWest = (ledGridWidth / 2) - 1;
    if (highestXValueForWest > 1) { // support small grid (e.g. gridwidth 5)
        highestXValueForWest &= ~(1 << 0); // make even
    }

    lowestXValueForEast = (ledGridWidth / 2) - 1;
    if (lowestXValueForEast & 1) {
        lowestXValueForEast = min(lowestXValueForEast + 1, ledGridWidth - 1);
    }
}

void ledStripInit(void)
{
    determineLedStripDimensions();
    determineOrientationLimits();
}
#endif
