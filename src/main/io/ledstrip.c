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

#include "build/build_config.h"

#ifdef LED_STRIP

#include <common/color.h>
#include <common/maths.h>
#include <common/typeconversion.h>
#include <common/printf.h>
#include <common/axis.h>
#include <common/utils.h>

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"

#include "drivers/light_ws2811strip.h"
#include "drivers/system.h"
#include "drivers/serial.h"

#include "flight/pid.h"
#include "flight/failsafe.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"

#include "io/gps.h"
#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "io/ledstrip.h"


PG_REGISTER_ARR_WITH_RESET_FN(ledConfig_t, LED_MAX_STRIP_LENGTH, ledConfigs, PG_LED_STRIP_CONFIG, 0);
PG_REGISTER_ARR_WITH_RESET_FN(hsvColor_t, LED_CONFIGURABLE_COLOR_COUNT, colors, PG_COLOR_CONFIG, 0);
PG_REGISTER_ARR_WITH_RESET_FN(modeColorIndexes_t, LED_MODE_COUNT, modeColors, PG_MODE_COLOR_CONFIG, 0);
PG_REGISTER_ARR_WITH_RESET_FN(specialColorIndexes_t, 1, specialColors, PG_SPECIAL_COLOR_CONFIG, 0);

static bool ledStripInitialised = false;
static bool ledStripEnabled = true;

static void ledStripDisable(void);

//#define USE_LED_ANIMATION
//#define USE_LED_RING_DEFAULT_CONFIG

#define LED_STRIP_HZ(hz) ((int32_t)((1000 * 1000) / (hz)))
#define LED_STRIP_MS(ms) ((int32_t)(1000 * (ms)))

#if LED_MAX_STRIP_LENGTH > WS2811_LED_STRIP_LENGTH
# error "Led strip length must match driver"
#endif

typedef enum {
    COLOR_BLACK = 0,
    COLOR_WHITE,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_LIME_GREEN,
    COLOR_GREEN,
    COLOR_MINT_GREEN,
    COLOR_CYAN,
    COLOR_LIGHT_BLUE,
    COLOR_BLUE,
    COLOR_DARK_VIOLET,
    COLOR_MAGENTA,
    COLOR_DEEP_PINK,
} colorId_e;

const hsvColor_t hsv[] = {
    //                        H    S    V
    [COLOR_BLACK] =        {  0,   0,   0},
    [COLOR_WHITE] =        {  0, 255, 255},
    [COLOR_RED] =          {  0,   0, 255},
    [COLOR_ORANGE] =       { 30,   0, 255},
    [COLOR_YELLOW] =       { 60,   0, 255},
    [COLOR_LIME_GREEN] =   { 90,   0, 255},
    [COLOR_GREEN] =        {120,   0, 255},
    [COLOR_MINT_GREEN] =   {150,   0, 255},
    [COLOR_CYAN] =         {180,   0, 255},
    [COLOR_LIGHT_BLUE] =   {210,   0, 255},
    [COLOR_BLUE] =         {240,   0, 255},
    [COLOR_DARK_VIOLET] =  {270,   0, 255},
    [COLOR_MAGENTA] =      {300,   0, 255},
    [COLOR_DEEP_PINK] =    {330,   0, 255},
};
// macro to save typing on default colors
#define HSV(color) (hsv[COLOR_ ## color])

STATIC_UNIT_TESTED uint8_t ledGridWidth;
STATIC_UNIT_TESTED uint8_t ledGridHeight;
// grid offsets
STATIC_UNIT_TESTED uint8_t highestYValueForNorth;
STATIC_UNIT_TESTED uint8_t lowestYValueForSouth;
STATIC_UNIT_TESTED uint8_t highestXValueForWest;
STATIC_UNIT_TESTED uint8_t lowestXValueForEast;

uint8_t ledCount;
uint8_t ledRingCount;
static uint8_t ledRingSeqLen;

// macro for initializer
#define LF(name) LED_FLAG_FUNCTION(LED_FUNCTION_ ## name)
#define LD(name) LED_FLAG_DIRECTION(LED_DIRECTION_ ## name)

#ifdef USE_LED_RING_DEFAULT_CONFIG
static const ledConfig_t defaultLedStripConfig[] = {
    { CALCULATE_LED_XY( 2,  2), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 2,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 2,  0), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  0), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 0,  0), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 0,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 0,  2), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  2), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 1,  1), 3, LF(THRUST_RING)},
};
#else
static const ledConfig_t defaultLedStripConfig[] = {
    { CALCULATE_LED_XY(15, 15), 0, LD(SOUTH) | LD(EAST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY(15,  8), 0, LD(EAST)             | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY(15,  7), 0, LD(EAST)             | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY(15,  0), 0, LD(NORTH) | LD(EAST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY( 8,  0), 0, LD(NORTH)            | LF(FLIGHT_MODE) },
    { CALCULATE_LED_XY( 7,  0), 0, LD(NORTH)            | LF(FLIGHT_MODE) },

    { CALCULATE_LED_XY( 0,  0), 0, LD(NORTH) | LD(WEST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY( 0,  7), 0, LD(WEST)             | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 0,  8), 0, LD(WEST)             | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY( 0, 15), 0, LD(SOUTH) | LD(WEST) | LF(INDICATOR) | LF(ARM_STATE) },

    { CALCULATE_LED_XY( 7, 15), 0, LD(SOUTH)            | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 8, 15), 0, LD(SOUTH)            | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY( 7,  7), 0, LD(UP)               | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 8,  7), 0, LD(UP)               | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 7,  8), 0, LD(DOWN)             | LF(FLIGHT_MODE) | LF(WARNING) },
    { CALCULATE_LED_XY( 8,  8), 0, LD(DOWN)             | LF(FLIGHT_MODE) | LF(WARNING) },

    { CALCULATE_LED_XY( 8,  9), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 9, 10), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY(10, 11), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY(10, 12), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 9, 13), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 8, 14), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 7, 14), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 6, 13), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 5, 12), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 5, 11), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 6, 10), 3, LF(THRUST_RING)},
    { CALCULATE_LED_XY( 7,  9), 3, LF(THRUST_RING)},

};
#endif

#undef LD
#undef LF

static const modeColorIndexes_t defaultModeColors[] = {
    //                          NORTH             EAST               SOUTH            WEST             UP          DOWN
    [LED_MODE_ORIENTATION] = {{ COLOR_WHITE,      COLOR_DARK_VIOLET, COLOR_RED,       COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_HEADFREE]    = {{ COLOR_LIME_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_HORIZON]     = {{ COLOR_BLUE,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_ANGLE]       = {{ COLOR_CYAN,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_MAG]         = {{ COLOR_MINT_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_BARO]        = {{ COLOR_LIGHT_BLUE, COLOR_DARK_VIOLET, COLOR_RED,       COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
};

static const specialColorIndexes_t defaultSpecialColors[] = {
    {{ [LED_SCOLOR_DISARMED]        = COLOR_GREEN,
       [LED_SCOLOR_ARMED]           = COLOR_BLUE,
       [LED_SCOLOR_ANIMATION]       = COLOR_WHITE,
       [LED_SCOLOR_BACKGROUND]      = COLOR_BLACK,
       [LED_SCOLOR_BLINKBACKGROUND] = COLOR_BLACK,
       [LED_SCOLOR_GPSNOSATS]       = COLOR_RED,
       [LED_SCOLOR_GPSNOLOCK]       = COLOR_ORANGE,
       [LED_SCOLOR_GPSLOCKED]       = COLOR_GREEN,
    }}
};

void pgResetFn_ledConfigs(ledConfig_t *instance)
{
    memcpy_fn(instance, &defaultLedStripConfig, sizeof(defaultLedStripConfig));
}

/*
 * 6 coords @nn,nn
 * 4 direction @##
 * 6 modes @####
 * = 16 bytes per led
 * 16 * 32 leds = 512 bytes storage needed worst case.
 * = not efficient to store led configs as strings in flash.
 * = becomes a problem to send all the data via cli due to serial/cli buffers
 */



static void updateLedRingCounts(void);

STATIC_UNIT_TESTED void determineLedStripDimensions(void)
{
    int maxX = 0;
    int maxY = 0;

    for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        const ledConfig_t *ledConfig = ledConfigs(ledIndex);

        maxX = MAX(ledGetX(ledConfig), maxX);
        maxY = MAX(ledGetY(ledConfig), maxY);
    }
    ledGridWidth = maxX + 1;
    ledGridHeight = maxY + 1;
}

STATIC_UNIT_TESTED void determineOrientationLimits(void)
{
    highestYValueForNorth = (ledGridHeight / 2) - 1;
    lowestYValueForSouth = ((ledGridHeight + 1) / 2);
    highestXValueForWest = (ledGridWidth / 2) - 1;
    lowestXValueForEast = ((ledGridWidth + 1) / 2);
}

static void updateLedCount(void)
{
    int count = 0, countRing = 0;

    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        const ledConfig_t *ledConfig = ledConfigs(ledIndex);
        if (ledConfig->flags == 0 && ledConfig->xy == 0)
            break;
        count++;
        if ((ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_THRUST_RING)))
            countRing++;
    }
    ledCount = count;
    ledRingCount = countRing;
}

void reevalulateLedConfig(void)
{
    updateLedCount();
    determineLedStripDimensions();
    determineOrientationLimits();
    updateLedRingCounts();
}

// get specialColor by index
static hsvColor_t *getSC(ledSpecialColorIds_e index)
{
    return colors(specialColors(0)->color[index]);
}

static const char directionCodes[LED_DIRECTION_COUNT] = { 'N', 'E', 'S', 'W', 'U', 'D' };
static const char functionCodes[LED_FUNCTION_COUNT]   = { 'I', 'W', 'F', 'A', 'T', 'R', 'C', 'G', 'S', 'B' };

#define CHUNK_BUFFER_SIZE 11

bool parseLedStripConfig(int ledIndex, const char *config)
{
    if (ledIndex >= LED_MAX_STRIP_LENGTH)
        return false;

    enum parseState_e {
        X_COORDINATE,
        Y_COORDINATE,
        DIRECTIONS,
        FUNCTIONS,
        RING_COLORS,
        PARSE_STATE_COUNT
    };
    static const char chunkSeparators[PARSE_STATE_COUNT] = {',', ':', ':',':', '\0'};

    ledConfig_t *ledConfig = ledConfigs(ledIndex);
    memset(ledConfig, 0, sizeof(ledConfig_t));

    int x = 0, y = 0, color = 0;   // initialize to prevent warnings
    int flags = 0;
    for(enum parseState_e parseState = 0; parseState < PARSE_STATE_COUNT; parseState++) {
        char chunk[CHUNK_BUFFER_SIZE];
        {
            char chunkSeparator = chunkSeparators[parseState];
            int chunkIndex = 0;
            while (*config  && *config != chunkSeparator && chunkIndex < CHUNK_BUFFER_SIZE-1) {
                chunk[chunkIndex++] = *config++;
            }
            chunk[chunkIndex++] = 0; // zero-terminate chunk
            if (*config != chunkSeparator) {
                return false;
            }
            config++;   // skip separator
        }
        switch(parseState) {
            case X_COORDINATE:
                x = atoi(chunk);
                break;
            case Y_COORDINATE:
                y = atoi(chunk);
                break;
            case DIRECTIONS:
                for (char* ch = chunk; *ch; ch++) {
                    for (ledDirectionId_e dir = 0; dir < LED_DIRECTION_COUNT; dir++) {
                        if (directionCodes[dir] == *ch) {
                            flags |= LED_FLAG_DIRECTION(dir);
                            break;
                        }
                    }
                }
                break;
            case FUNCTIONS:
                for (char* ch = chunk; *ch; ch++) {
                    for (ledFunctionId_e fn = 0; fn < LED_FUNCTION_COUNT; fn++) {
                        if (functionCodes[fn] == *ch) {
                            flags |= LED_FLAG_FUNCTION(fn);
                            break;
                        }
                    }
                }
                break;
            case RING_COLORS:
                color = atoi(chunk);
                if (color >= LED_CONFIGURABLE_COLOR_COUNT)
                    color = 0;
                break;
            case PARSE_STATE_COUNT:; // prevent warning
        }
    }
    ledSetXY(ledConfig, x, y);
    ledConfig->color = color;
    ledConfig->flags = flags;

    reevalulateLedConfig();
    return true;
}

void generateLedConfig(int ledIndex, char *ledConfigBuffer, size_t bufferSize)
{
    char functions[LED_FUNCTION_COUNT + 1];
    char directions[LED_DIRECTION_COUNT + 1];

    ledConfig_t *ledConfig = ledConfigs(ledIndex);

    memset(ledConfigBuffer, 0, bufferSize);
    char *fptr = functions;
    for (ledFunctionId_e fn = 0; fn < LED_FUNCTION_COUNT; fn++) {
        if (ledConfig->flags & LED_FLAG_FUNCTION(fn)) {
            *fptr++ = functionCodes[fn];
        }
    }
    *fptr = 0;
    char *dptr = directions;
    for (ledDirectionId_e dir = 0; dir < LED_DIRECTION_COUNT; dir++) {
        if (ledConfig->flags & LED_FLAG_DIRECTION(dir)) {
            *dptr++ = directionCodes[dir];
        }
    }
    *dptr = 0;
    // TODO - check buffer length
    sprintf(ledConfigBuffer, "%u,%u:%s:%s:%u", ledGetX(ledConfig), ledGetY(ledConfig), directions, functions, ledConfig->color);
}

typedef enum {
    // the ordering is important, see below how NSEW is mapped to  NE/SE/NW/SW
    QUADRANT_NORTH      = 1 << 0,
    QUADRANT_SOUTH      = 1 << 1,
    QUADRANT_EAST       = 1 << 2,
    QUADRANT_WEST       = 1 << 3,
    QUADRANT_NORTH_EAST = 1 << 4,
    QUADRANT_SOUTH_EAST = 1 << 5,
    QUADRANT_NORTH_WEST = 1 << 6,
    QUADRANT_SOUTH_WEST = 1 << 7,
    QUADRANT_NONE       = 1 << 8,
    QUADRANT_NOTDIAG    = 1 << 9,  // not in NE/SE/NW/SW
    // values for test
    QUADRANT_ANY        = QUADRANT_NORTH | QUADRANT_SOUTH | QUADRANT_EAST | QUADRANT_WEST | QUADRANT_NONE,
} quadrant_e;

static quadrant_e getLedQuadrant(const int ledIndex)
{
    const ledConfig_t *ledConfig = ledConfigs(ledIndex);

    int quad = 0;
    if (ledGetY(ledConfig) <= highestYValueForNorth)
        quad |= QUADRANT_NORTH;
    else if (ledGetY(ledConfig) >= lowestYValueForSouth)
        quad |= QUADRANT_SOUTH;
    if (ledGetX(ledConfig) >= lowestXValueForEast)
        quad |= QUADRANT_EAST;
    else if (ledGetX(ledConfig) <= highestXValueForWest)
        quad |= QUADRANT_WEST;

    if((quad & (QUADRANT_NORTH | QUADRANT_SOUTH))
       && (quad & (QUADRANT_EAST | QUADRANT_WEST)) ) { // is led  in one of NE/SE/NW/SW?
        quad |= 1 << (4 + ((quad & QUADRANT_SOUTH) ? 1 : 0) + ((quad & QUADRANT_WEST) ? 2 : 0));
    } else {
        quad |= QUADRANT_NOTDIAG;
    }

    if((quad & (QUADRANT_NORTH | QUADRANT_SOUTH | QUADRANT_EAST | QUADRANT_WEST))  == 0)
        quad |= QUADRANT_NONE;

    return quad;
}

static const struct {
    uint8_t dir;             // ledDirectionId_e
    uint16_t quadrantMask;   // quadrant_e
} directionQuadrantMap[] = {
    {LED_DIRECTION_SOUTH, QUADRANT_SOUTH},
    {LED_DIRECTION_NORTH, QUADRANT_NORTH},
    {LED_DIRECTION_EAST,  QUADRANT_EAST},
    {LED_DIRECTION_WEST,  QUADRANT_WEST},
    {LED_DIRECTION_DOWN,  QUADRANT_ANY},
    {LED_DIRECTION_UP,    QUADRANT_ANY},
};

static hsvColor_t * getDirectionalModeColor(const int ledIndex, const modeColorIndexes_t *modeColors)
{
    const ledConfig_t *ledConfig = ledConfigs(ledIndex);

    quadrant_e quad = getLedQuadrant(ledIndex);
    for(unsigned i = 0; i < ARRAYLEN(directionQuadrantMap); i++) {
        ledDirectionId_e dir = directionQuadrantMap[i].dir;
        quadrant_e quadMask = directionQuadrantMap[i].quadrantMask;

        if((ledConfig->flags & LED_FLAG_DIRECTION(dir))
           && (quad & quadMask))
            return colors(modeColors->color[dir]);
    }
    return NULL;
}


// map flight mode to led mode, in order of priority
// flightMode == 0 is always active
static const struct {
    uint16_t flightMode;
    uint8_t ledMode;
} flightModeToLed[] = {
    {HEADFREE_MODE, LED_MODE_HEADFREE},
#ifdef MAG
    {MAG_MODE,      LED_MODE_MAG},
#endif
#ifdef BARO
    {BARO_MODE,     LED_MODE_BARO},
#endif
    {HORIZON_MODE,  LED_MODE_HORIZON},
    {ANGLE_MODE,    LED_MODE_ANGLE},
    {0,             LED_MODE_ORIENTATION},
};

static void applyLedModeLayer(void)
{
    for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        const ledConfig_t *ledConfig = ledConfigs(ledIndex);
        const hsvColor_t* color = NULL;

        if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_THRUST_RING))) {
            if (ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_COLOR)) {
                color = colors(ledConfig->color);
            } else {
                color = getSC(LED_SCOLOR_BACKGROUND);
            }
        }

        if (ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_FLIGHT_MODE)) {
            for(unsigned i = 0; i < ARRAYLEN(flightModeToLed); i++)
                if(!flightModeToLed[i].flightMode || FLIGHT_MODE(flightModeToLed[i].flightMode)) {
                    color = getDirectionalModeColor(ledIndex, modeColors(flightModeToLed[i].ledMode));
                    break; // stop on first match
                }
        } else if (ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_ARM_STATE)) {
            color = ARMING_FLAG(ARMED) ? getSC(LED_SCOLOR_ARMED) : getSC(LED_SCOLOR_DISARMED);
        }

        if(color)
            setLedHsv(ledIndex, color);
    }
}

static void applyLedHue(ledFunctionId_e flag, int16_t value, int16_t minRange, int16_t maxRange)
{
    int scaled = scaleRange(value, minRange, maxRange, -60, +60);
    scaled += HSV_HUE_MAX;   // wrap negative values correctly

    for (int i = 0; i < ledCount; ++i) {
        const ledConfig_t *ledConfig = ledConfigs(i);
        if (!(ledConfig->flags & LED_FLAG_FUNCTION(flag)))
            continue;

        hsvColor_t color;
        getLedHsv(i, &color);
        color.h = (color.h + scaled) % (HSV_HUE_MAX + 1);
        setLedHsv(i, &color);
    }
}

static void applyLedHueLayer(void)
{
    applyLedHue(LED_FUNCTION_THROTTLE, rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
    applyLedHue(LED_FUNCTION_RSSI, rssi, 0, 1023);
}

typedef enum {
    WARNING_ARMING_DISABLED,
    WARNING_LOW_BATTERY,
    WARNING_FAILSAFE,
} warningFlags_e;


static void applyLedWarningLayer(bool updateNow, uint32_t *timer)
{
    static uint8_t warningFlashCounter = 0;
    static uint8_t warningFlags = 0;          // non-zero during blinks

    if (updateNow) {
        // keep counter running, so it stays in sync with blink
        warningFlashCounter++;
        if (warningFlashCounter >= 20) {
            warningFlashCounter = 0;
        }
        if (warningFlashCounter == 0) {      // update when old flags was processed
            warningFlags = 0;
            if (feature(FEATURE_VBAT) && getBatteryState() != BATTERY_OK)
                warningFlags |= 1 << WARNING_LOW_BATTERY;
            if (feature(FEATURE_FAILSAFE) && failsafeIsActive())
                warningFlags |= 1 << WARNING_FAILSAFE;
            if (!ARMING_FLAG(ARMED) && !ARMING_FLAG(OK_TO_ARM))
                warningFlags |= 1 << WARNING_ARMING_DISABLED;
        }
        *timer += LED_STRIP_HZ(10);
    }

    if (warningFlags) {
        const hsvColor_t *warningColor =  &HSV(BLACK);

        bool colorOn = (warningFlashCounter % 2) == 0;   // w_w_
        warningFlags_e warningId = warningFlashCounter / 4;
        if(warningFlags & (1 << warningId)) {
            switch(warningId) {
                case WARNING_ARMING_DISABLED:
                    warningColor = colorOn ? &HSV(GREEN)  : &HSV(BLACK);
                    break;
                case WARNING_LOW_BATTERY:
                    warningColor = colorOn ? &HSV(RED)    : &HSV(BLACK);
                    break;
                case WARNING_FAILSAFE:
                    warningColor = colorOn ? &HSV(YELLOW) : &HSV(BLUE);
                    break;
                default:;
            }
        }

        for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
            const ledConfig_t *ledConfig = ledConfigs(ledIndex);
            if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_WARNING)))
                continue;

            setLedHsv(ledIndex, warningColor);
        }
    }
}

#ifdef GPS
static void applyLedGpsLayer(bool updateNow, uint32_t *timer)
{
    static uint8_t gpsFlashCounter = 0;
    static uint8_t gpsPauseCounter = 0;
    const uint8_t blinkPauseLength = 4;

    if (updateNow) {
        if (gpsPauseCounter > 0) {
            gpsPauseCounter--;
        } else if (gpsFlashCounter >= GPS_numSat) {
            gpsFlashCounter = 0;
            gpsPauseCounter = blinkPauseLength;
        } else {
            gpsFlashCounter++;
            gpsPauseCounter = 1;
        }
        *timer += LED_STRIP_HZ(2.5);
    }

    const hsvColor_t *gpsColor;

    if (GPS_numSat == 0 || !sensors(SENSOR_GPS)) {
        gpsColor = getSC(LED_SCOLOR_GPSNOSATS);
    } else {
        bool colorOn = gpsPauseCounter == 0;  // each interval starts with pause
        if(STATE(GPS_FIX)) {
            gpsColor = colorOn ? getSC(LED_SCOLOR_GPSLOCKED) : getSC(LED_SCOLOR_BACKGROUND);
        } else {
            gpsColor = colorOn ? getSC(LED_SCOLOR_GPSNOLOCK) : getSC(LED_SCOLOR_GPSNOSATS);
        }
    }

    for (int i = 0; i < ledCount; ++i) {
        const ledConfig_t *ledConfig = ledConfigs(i);
        if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_GPS)))
            continue;

        setLedHsv(i, gpsColor);
    }
}

#endif


#define INDICATOR_DEADBAND 25

static void applyLedIndicatorLayer(bool updateNow, uint32_t *timer)
{
    static uint8_t flashCounter = 0;

    if(updateNow) {
        if (!rxIsReceivingSignal()) {
            *timer += LED_STRIP_HZ(5);  // try again soon
        } else {
            // calculate update frequency
            int scale = MAX(ABS(rcCommand[ROLL]), ABS(rcCommand[PITCH]));  // 0 - 500
            scale += (50 - INDICATOR_DEADBAND);  // start increasing frequency right after deadband
            *timer += LED_STRIP_HZ(5) * 50 / MAX(50, scale);   // 5 - 50Hz update, 2.5 - 25Hz blink

            flashCounter = !flashCounter;
        }
    }
    const hsvColor_t *flashColor = flashCounter ? &HSV(ORANGE) : &HSV(BLACK); // TODO - use user color?

    quadrant_e quadrants = 0;
    if (rcCommand[ROLL] > INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_NORTH_EAST | QUADRANT_SOUTH_EAST;
    } else if (rcCommand[ROLL] < -INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_NORTH_WEST | QUADRANT_SOUTH_WEST;
    }
    if (rcCommand[PITCH] > INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_NORTH_EAST | QUADRANT_NORTH_WEST;
    } else if (rcCommand[PITCH] < -INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_SOUTH_EAST | QUADRANT_SOUTH_WEST;
    }

    for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        const ledConfig_t *ledConfig = ledConfigs(ledIndex);
        if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_INDICATOR)))
            continue;

        if(getLedQuadrant(ledIndex) & quadrants)
            setLedHsv(ledIndex, flashColor);
    }
}

#define ROTATION_SEQUENCE_LED_COUNT 6 // 2 on, 4 off
#define ROTATION_SEQUENCE_LED_WIDTH 2 // 2 on

static void updateLedRingCounts(void)
{
    int seqLen;
    // try to split in segments/rings of exactly ROTATION_SEQUENCE_LED_COUNT leds
    if ((ledRingCount % ROTATION_SEQUENCE_LED_COUNT) == 0) {
        seqLen = ROTATION_SEQUENCE_LED_COUNT;
    } else {
        seqLen = ledRingCount;
        // else split up in equal segments/rings of at most ROTATION_SEQUENCE_LED_COUNT leds
        // TODO - improve partitioning (15 leds -> 3x5)
        while ((seqLen > ROTATION_SEQUENCE_LED_COUNT) && ((seqLen % 2) == 0)) {
            seqLen /= 2;
        }
    }
    ledRingSeqLen = seqLen;
}

static void applyLedThrustRingLayer(bool updateNow, uint32_t *timer)
{
    static uint8_t rotationPhase;
    int ledRingIndex = 0;

    if(updateNow) {
        rotationPhase = rotationPhase > 0 ? rotationPhase - 1 : ledRingSeqLen - 1;

        int scale = ARMING_FLAG(ARMED) ? scaleRange(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX, 10, 100) : 10;
        *timer += LED_STRIP_HZ(5) * 10 / scale;  // 5 - 50Hz update rate
    }

    for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        const ledConfig_t *ledConfig = ledConfigs(ledIndex);
        if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_THRUST_RING)))
            continue;

        bool applyColor;
        if (ARMING_FLAG(ARMED)) {
            applyColor = (ledRingIndex + rotationPhase) % ledRingSeqLen < ROTATION_SEQUENCE_LED_WIDTH;
        } else {
            applyColor = !(ledRingIndex % 2); // alternating pattern
        }

        const hsvColor_t *ringColor = applyColor ? colors(ledConfig->color) : &HSV(BLACK);
        setLedHsv(ledIndex, ringColor);

        ledRingIndex++;
    }
}

// blink twice, then wait
static void applyLedBlinkLayer(bool updateNow, uint32_t *timer)
{
    static uint8_t blinkCounter = 0;
    const int blinkCycleLength = 20;

    if (updateNow) {
        blinkCounter++;
        if (blinkCounter >= blinkCycleLength) {
            blinkCounter = 0;
        }
        *timer += LED_STRIP_HZ(10);
    }

    bool ledOn = (blinkCounter & 1) == 0 && blinkCounter < 4;  // b_b_____...

    for (int i = 0; i < ledCount; ++i) {
        const ledConfig_t *ledConfig = ledConfigs(i);
        if (!(ledConfig->flags & LED_FLAG_FUNCTION(LED_FUNCTION_BLINK)))
            continue;

        const hsvColor_t *blinkColor = ledOn ? colors(ledConfig->color) : getSC(LED_SCOLOR_BLINKBACKGROUND);
        setLedHsv(i, blinkColor);
    }
}


#ifdef USE_LED_ANIMATION

static void applyLedAnimationLayer(bool updateNow, uint32_t *timer)
{
    static uint8_t frameCounter = 0;
    const int animationFrames = ledGridHeight;
    if(updateNow) {
        frameCounter = (frameCounter + 1 < animationFrames) ? frameCounter + 1 : 0;
        *timer += LED_STRIP_HZ(20);
    }

    if (ARMING_FLAG(ARMED))
        return;

    int previousRow = frameCounter > 0 ? frameCounter - 1 : animationFrames - 1;
    int currentRow = frameCounter;
    int nextRow = (frameCounter + 1 < animationFrames) ? frameCounter + 1 : 0;

    for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        const ledConfig_t *ledConfig = ledConfigs(ledIndex);

        if (ledGetY(ledConfig) == previousRow) {
            setLedHsv(ledIndex, getSC(LED_SCOLOR_ANIMATION));
            scaleLedValue(ledIndex, 50);
        } else if (ledGetY(ledConfig) == currentRow) {
            setLedHsv(ledIndex, getSC(LED_SCOLOR_ANIMATION));
        } else if (ledGetY(ledConfig) == nextRow) {
            scaleLedValue(ledIndex, 50);
        }
    }
}
#endif

typedef enum {
    timIndicator,
    timBlink,
    timWarning,
#ifdef GPS
    timGps,
#endif
    timRotation,
#ifdef USE_LED_ANIMATION
    timAnimation,
#endif
    timTimerCount
} timId_e;

static uint32_t timerVal[timTimerCount];

// function to apply layer.
// function must replan self using timer pointer
// when updateNow is true (timer triggered), state must be updated first,
//  before calculating led state. Otherwise update started by different trigger
//  may modify LED state.
typedef void applyLayerFn_timed(bool updateNow, uint32_t* timer);
typedef void applyLayerFn(void);

static const struct {
    int8_t timId;                          // timer id for update, -1 if none
    union {
        applyLayerFn *apply;               // function to apply layer unconditionally
        applyLayerFn_timed *applyTimed;    // apply with timer
    } f;
} layerTable[] = {
    // LAYER 1
    { -1,             .f.apply      = &applyLedModeLayer },
    { -1,             .f.apply      = &applyLedHueLayer },
    // LAYER 2
    {timWarning,      .f.applyTimed = &applyLedWarningLayer},
#ifdef GPS
    {timGps,          .f.applyTimed = &applyLedGpsLayer},
#endif
    // LAYER 3
    {timIndicator,    .f.applyTimed = &applyLedIndicatorLayer},
    // LAYER 4
    {timBlink,        .f.applyTimed = &applyLedBlinkLayer},
#ifdef USE_LED_ANIMATION
    {timAnimation,    .f.applyTimed = &applyLedAnimationLayer},
#endif
    {timRotation,     .f.applyTimed = &applyLedThrustRingLayer},
};

void updateLedStrip(void)
{

    if (!(ledStripInitialised && isWS2811LedStripReady())) {
        return;
    }

    if (rcModeIsActive(BOXLEDLOW)) {
        if (ledStripEnabled) {
            ledStripDisable();
            ledStripEnabled = false;
        }
        return;
    }
    ledStripEnabled = true;

    uint32_t now = micros();

    // test all led timers, setting corresponding bits
    uint32_t timActive = 0;
    for(timId_e timId = 0; timId < timTimerCount; timId++) {
        if(cmp32(now, timerVal[timId]) < 0)
            continue;  // not ready yet
        timActive |= 1 << timId;
        // sanitize timer value, so that it can be safely incremented. Handles inital timerVal value.
        // max delay is limited to 5s
        if(cmp32(now, timerVal[timId]) >= LED_STRIP_MS(100) || cmp32(now, timerVal[timId]) < LED_STRIP_HZ(5000) ) {
            timerVal[timId] = now;
        }
    }

    if (!timActive)
        return;          // no change this update, keep old state

    // apply all layers; triggered timed functions has to update timers
    for(unsigned i = 0; i < ARRAYLEN(layerTable); i++) {
        int timId = layerTable[i].timId;
        if(timId >= 0) {
            uint32_t *timer = &timerVal[timId];
            bool updateNow = timActive & (1 << timId);
            (*layerTable[i].f.applyTimed)(updateNow, timer);
        } else {
            (*layerTable[i].f.apply)();
        }
    }

    ws2811UpdateStrip();
}

bool parseColor(int index, const char *colorConfig)
{
    const char *remainingCharacters = colorConfig;

    hsvColor_t *color = colors(index);

    bool result = true;
    static const uint16_t hsv_limit[HSV_COLOR_COMPONENT_COUNT] = {
        [HSV_HUE] = HSV_HUE_MAX,
        [HSV_SATURATION] = HSV_SATURATION_MAX,
        [HSV_VALUE] = HSV_VALUE_MAX,
    };
    for (int componentIndex = 0; result && componentIndex < HSV_COLOR_COMPONENT_COUNT; componentIndex++) {
        int val = atoi(remainingCharacters);
        if(val > hsv_limit[componentIndex]) {
            result = false;
            break;
        }
        switch (componentIndex) {
            case HSV_HUE:
                color->h = val;
                break;
            case HSV_SATURATION:
                color->s = val;
                break;
            case HSV_VALUE:
                color->v = val;
                break;
        }
        remainingCharacters = strchr(remainingCharacters, ',');
        if (remainingCharacters) {
            remainingCharacters++;  // skip separator
        } else {
            if (componentIndex < HSV_COLOR_COMPONENT_COUNT - 1) {
                result = false;
            }
        }
    }

    if (!result) {
        memset(color, 0, sizeof(*color));
    }

    return result;
}

/*
 * Redefine a color in a mode.
 * */
bool setModeColor(ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex)
{
    // check color
    if(colorIndex < 0 || colorIndex >= LED_CONFIGURABLE_COLOR_COUNT)
        return false;
    if(modeIndex < LED_MODE_COUNT) {  // modeIndex_e is unsigned, so one-sided test is enough
        if(modeColorIndex < 0 || modeColorIndex >= LED_DIRECTION_COUNT)
            return false;
        modeColors(modeIndex)->color[modeColorIndex] = colorIndex;
    } else if(modeIndex == LED_SPECIAL) {
        if(modeColorIndex < 0 || modeColorIndex >= LED_SPECIAL_COLOR_COUNT)
            return false;
        specialColors(0)->color[modeColorIndex] = colorIndex;
    } else {
        return false;
    }
    return true;
}

void pgResetFn_colors(hsvColor_t *instance)
{
    // copy hsv colors as default
    BUILD_BUG_ON(ARRAYLEN(*colors_arr()) <= ARRAYLEN(hsv));

    for (unsigned colorIndex = 0; colorIndex < ARRAYLEN(hsv); colorIndex++) {
        *instance++ = hsv[colorIndex];
    }
}

void pgResetFn_modeColors(modeColorIndexes_t *instance)
{
    memcpy_fn(instance, &defaultModeColors, sizeof(defaultModeColors));
}

void pgResetFn_specialColors(specialColorIndexes_t *instance)
{
    memcpy_fn(instance, &defaultSpecialColors, sizeof(defaultSpecialColors));
}


void ledStripInit(void)
{
    ledStripInitialised = false;
}

void ledStripEnable(void)
{
    reevalulateLedConfig();
    ledStripInitialised = true;

    ws2811LedStripInit();
}

static void ledStripDisable(void)
{
	setStripColor(&HSV(BLACK));

	ws2811UpdateStrip();
}
#endif
