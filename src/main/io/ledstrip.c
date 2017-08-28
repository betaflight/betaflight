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

#ifdef LED_STRIP

#include "build/build_config.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/string_light.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/light_ws2811strip.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "io/ledstrip.h"
#include "io/beeper.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/gps.h"

#include "navigation/navigation.h"

#include "rx/rx.h"
#include "sensors/acceleration.h"
#include "sensors/diagnostics.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/pitotmeter.h"

#include "telemetry/telemetry.h"


extern uint16_t rssi; // FIXME dependency on mw.c

PG_REGISTER_WITH_RESET_FN(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 0);

static bool ledStripInitialised = false;
static bool ledStripEnabled = true;

static void ledStripDisable(void);

//#define USE_LED_ANIMATION

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

STATIC_UNIT_TESTED ledCounts_t ledCounts;

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

void pgResetFn_ledStripConfig(ledStripConfig_t *instance)
{
    memset(instance->ledConfigs, 0, LED_MAX_STRIP_LENGTH * sizeof(ledConfig_t));
    // copy hsv colors as default
    memset(instance->colors, 0, ARRAYLEN(hsv) * sizeof(hsvColor_t));
    BUILD_BUG_ON(LED_CONFIGURABLE_COLOR_COUNT < ARRAYLEN(hsv));
    for (unsigned colorIndex = 0; colorIndex < ARRAYLEN(hsv); colorIndex++) {
        instance->colors[colorIndex] = hsv[colorIndex];
    }
    memcpy_fn(&instance->modeColors, &defaultModeColors, sizeof(defaultModeColors));
    memcpy_fn(&instance->specialColors, &defaultSpecialColors, sizeof(defaultSpecialColors));
}

static int scaledThrottle;

static void updateLedRingCounts(void);

STATIC_UNIT_TESTED void determineLedStripDimensions(void)
{
    int maxX = 0;
    int maxY = 0;

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];

        maxX = MAX(ledGetX(ledConfig), maxX);
        maxY = MAX(ledGetY(ledConfig), maxY);
    }
    ledGridWidth = maxX + 1;
    ledGridHeight = maxY + 1;
}

STATIC_UNIT_TESTED void determineOrientationLimits(void)
{
    highestYValueForNorth = MIN((ledGridHeight / 2) - 1, 0);
    lowestYValueForSouth = (ledGridHeight + 1) / 2;
    highestXValueForWest = MIN((ledGridWidth / 2) - 1, 0);
    lowestXValueForEast = (ledGridWidth + 1) / 2;
}

STATIC_UNIT_TESTED void updateLedCount(void)
{
    int count = 0, countRing = 0, countScanner= 0;

    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];

        if (!(*ledConfig))
            break;

        count++;

        if (ledGetFunction(ledConfig) == LED_FUNCTION_THRUST_RING)
            countRing++;

        if (ledGetOverlayBit(ledConfig, LED_OVERLAY_LARSON_SCANNER))
            countScanner++;
    }

    ledCounts.count = count;
    ledCounts.ring = countRing;
    ledCounts.larson = countScanner;
}

void reevaluateLedConfig(void)
{
    updateLedCount();
    determineLedStripDimensions();
    determineOrientationLimits();
    updateLedRingCounts();
}

// get specialColor by index
static const hsvColor_t* getSC(ledSpecialColorIds_e index)
{
    return &ledStripConfig()->colors[ledStripConfig()->specialColors.color[index]];
}

static const char directionCodes[LED_DIRECTION_COUNT] = { 'N', 'E', 'S', 'W', 'U', 'D' };
static const char baseFunctionCodes[LED_BASEFUNCTION_COUNT]   = { 'C', 'F', 'A', 'L', 'S', 'G', 'R' };
static const char overlayCodes[LED_OVERLAY_COUNT]   = { 'T', 'O', 'B', 'N', 'I', 'W' };

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

    ledConfig_t *ledConfig = &ledStripConfigMutable()->ledConfigs[ledIndex];
    memset(ledConfig, 0, sizeof(ledConfig_t));

    int x = 0, y = 0, color = 0;   // initialize to prevent warnings
    int baseFunction = 0;
    int overlay_flags = 0;
    int direction_flags = 0;

    for (enum parseState_e parseState = 0; parseState < PARSE_STATE_COUNT; parseState++) {
        char chunk[CHUNK_BUFFER_SIZE];
        {
            char chunkSeparator = chunkSeparators[parseState];
            int chunkIndex = 0;
            while (*config  && *config != chunkSeparator && chunkIndex < (CHUNK_BUFFER_SIZE - 1)) {
                chunk[chunkIndex++] = *config++;
            }
            chunk[chunkIndex++] = 0; // zero-terminate chunk
            if (*config != chunkSeparator) {
                return false;
            }
            config++;   // skip separator
        }
        switch (parseState) {
            case X_COORDINATE:
                x = fastA2I(chunk);
                break;
            case Y_COORDINATE:
                y = fastA2I(chunk);
                break;
            case DIRECTIONS:
                for (char *ch = chunk; *ch; ch++) {
                    for (ledDirectionId_e dir = 0; dir < LED_DIRECTION_COUNT; dir++) {
                        if (directionCodes[dir] == *ch) {
                            direction_flags |= LED_FLAG_DIRECTION(dir);
                            break;
                        }
                    }
                }
                break;
            case FUNCTIONS:
                for (char *ch = chunk; *ch; ch++) {
                    for (ledBaseFunctionId_e fn = 0; fn < LED_BASEFUNCTION_COUNT; fn++) {
                        if (baseFunctionCodes[fn] == *ch) {
                            baseFunction = fn;
                            break;
                        }
                    }

                    for (ledOverlayId_e ol = 0; ol < LED_OVERLAY_COUNT; ol++) {
                        if (overlayCodes[ol] == *ch) {
                            overlay_flags |= LED_FLAG_OVERLAY(ol);
                            break;
                        }
                    }
                }
                break;
            case RING_COLORS:
                color = fastA2I(chunk);
                if (color >= LED_CONFIGURABLE_COLOR_COUNT)
                    color = 0;
                break;
            case PARSE_STATE_COUNT:; // prevent warning
        }
    }

    *ledConfig = DEFINE_LED(x, y, color, direction_flags, baseFunction, overlay_flags, 0);

    reevaluateLedConfig();

    return true;
}

void generateLedConfig(ledConfig_t *ledConfig, char *ledConfigBuffer, size_t bufferSize)
{
    char directions[LED_DIRECTION_COUNT + 1];
    char baseFunctionOverlays[LED_OVERLAY_COUNT + 2];

    memset(ledConfigBuffer, 0, bufferSize);

    char *dptr = directions;
    for (ledDirectionId_e dir = 0; dir < LED_DIRECTION_COUNT; dir++) {
        if (ledGetDirectionBit(ledConfig, dir)) {
            *dptr++ = directionCodes[dir];
        }
    }
    *dptr = 0;

    char *fptr = baseFunctionOverlays;
    *fptr++ = baseFunctionCodes[ledGetFunction(ledConfig)];

    for (ledOverlayId_e ol = 0; ol < LED_OVERLAY_COUNT; ol++) {
        if (ledGetOverlayBit(ledConfig, ol)) {
            *fptr++ = overlayCodes[ol];
        }
    }
    *fptr = 0;

    // TODO - check buffer length
    sprintf(ledConfigBuffer, "%u,%u:%s:%s:%u", ledGetX(ledConfig), ledGetY(ledConfig), directions, baseFunctionOverlays, ledGetColor(ledConfig));
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
    const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];

    int x = ledGetX(ledConfig);
    int y = ledGetY(ledConfig);

    int quad = 0;
    if (y <= highestYValueForNorth)
        quad |= QUADRANT_NORTH;
    else if (y >= lowestYValueForSouth)
        quad |= QUADRANT_SOUTH;
    if (x >= lowestXValueForEast)
        quad |= QUADRANT_EAST;
    else if (x <= highestXValueForWest)
        quad |= QUADRANT_WEST;

    if ((quad & (QUADRANT_NORTH | QUADRANT_SOUTH))
       && (quad & (QUADRANT_EAST | QUADRANT_WEST)) ) { // is led  in one of NE/SE/NW/SW?
        quad |= 1 << (4 + ((quad & QUADRANT_SOUTH) ? 1 : 0) + ((quad & QUADRANT_WEST) ? 2 : 0));
    } else {
        quad |= QUADRANT_NOTDIAG;
    }

    if ((quad & (QUADRANT_NORTH | QUADRANT_SOUTH | QUADRANT_EAST | QUADRANT_WEST)) == 0)
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

static const hsvColor_t* getDirectionalModeColor(const int ledIndex, const modeColorIndexes_t *modeColors)
{
    const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];

    quadrant_e quad = getLedQuadrant(ledIndex);
    for (unsigned i = 0; i < ARRAYLEN(directionQuadrantMap); i++) {
        ledDirectionId_e dir = directionQuadrantMap[i].dir;
        quadrant_e quadMask = directionQuadrantMap[i].quadrantMask;

        if (ledGetDirectionBit(ledConfig, dir) && (quad & quadMask))
            return &ledStripConfig()->colors[modeColors->color[dir]];
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
    {HEADING_MODE,  LED_MODE_MAG},
#ifdef BARO
    {NAV_ALTHOLD_MODE, LED_MODE_BARO},
#endif
    {HORIZON_MODE,  LED_MODE_HORIZON},
    {ANGLE_MODE,    LED_MODE_ANGLE},
    {0,             LED_MODE_ORIENTATION},
};

static void applyLedFixedLayers()
{
    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];
        hsvColor_t color = *getSC(LED_SCOLOR_BACKGROUND);

        int fn = ledGetFunction(ledConfig);
        int hOffset = HSV_HUE_MAX;

        switch (fn) {
            case LED_FUNCTION_COLOR:
                color = ledStripConfig()->colors[ledGetColor(ledConfig)];
                break;

            case LED_FUNCTION_FLIGHT_MODE:
                for (unsigned i = 0; i < ARRAYLEN(flightModeToLed); i++)
                    if (!flightModeToLed[i].flightMode || FLIGHT_MODE(flightModeToLed[i].flightMode)) {
                        const hsvColor_t *directionalColor = getDirectionalModeColor(ledIndex, &ledStripConfig()->modeColors[flightModeToLed[i].ledMode]);
                        if (directionalColor) {
                            color = *directionalColor;
                        }

                        break; // stop on first match
                    }
                break;

            case LED_FUNCTION_ARM_STATE:
                color = ARMING_FLAG(ARMED) ? *getSC(LED_SCOLOR_ARMED) : *getSC(LED_SCOLOR_DISARMED);
                break;

            case LED_FUNCTION_BATTERY:
                color = HSV(RED);
                hOffset += scaleRange(calculateBatteryCapacityRemainingPercentage(), 0, 100, -30, 120);
                break;

            case LED_FUNCTION_RSSI:
                color = HSV(RED);
                hOffset += scaleRange(rssi * 100, 0, 1023, -30, 120);
                break;

            default:
                break;
        }

        if (ledGetOverlayBit(ledConfig, LED_OVERLAY_THROTTLE)) {
            hOffset += ((scaledThrottle - 10) * 4) / 3;
        }

        color.h = (color.h + hOffset) % (HSV_HUE_MAX + 1);

        setLedHsv(ledIndex, &color);

    }
}

static void applyLedHsv(uint32_t mask, const hsvColor_t *color)
{
    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];
        if ((*ledConfig & mask) == mask)
            setLedHsv(ledIndex, color);
    }
}

typedef enum {
    WARNING_ARMING_DISABLED,
    WARNING_LOW_BATTERY,
    WARNING_FAILSAFE,
    WARNING_HW_ERROR,
} warningFlags_e;

static void applyLedWarningLayer(bool updateNow, timeUs_t *timer)
{
    static uint8_t warningFlashCounter = 0;
    static uint8_t warningFlags = 0;          // non-zero during blinks

    if (updateNow) {
        // keep counter running, so it stays in sync with blink
        warningFlashCounter++;
        warningFlashCounter &= 0xF;

        if (warningFlashCounter == 0) {      // update when old flags was processed
            warningFlags = 0;
            if (feature(FEATURE_VBAT) && getBatteryState() != BATTERY_OK)
                warningFlags |= 1 << WARNING_LOW_BATTERY;
            if (failsafeIsActive())
                warningFlags |= 1 << WARNING_FAILSAFE;
            if (!ARMING_FLAG(ARMED) && isArmingDisabled())
                warningFlags |= 1 << WARNING_ARMING_DISABLED;
            if (!isHardwareHealthy())
                warningFlags |= 1 << WARNING_HW_ERROR;
        }
        *timer += LED_STRIP_HZ(10);
    }

    if (warningFlags) {
        const hsvColor_t *warningColor = NULL;

        bool colorOn = (warningFlashCounter % 2) == 0;   // w_w_
        warningFlags_e warningId = warningFlashCounter / 4;
        if (warningFlags & (1 << warningId)) {
            switch (warningId) {
                case WARNING_ARMING_DISABLED:
                    warningColor = colorOn ? &HSV(GREEN)  : &HSV(BLACK);
                    break;
                case WARNING_LOW_BATTERY:
                    warningColor = colorOn ? &HSV(RED)    : &HSV(BLACK);
                    break;
                case WARNING_FAILSAFE:
                    warningColor = colorOn ? &HSV(YELLOW) : &HSV(BLUE);
                    break;
                case WARNING_HW_ERROR:
                    warningColor = colorOn ? &HSV(BLUE) : &HSV(BLACK);
                    break;
                default:;
            }
        }
        if (warningColor)
            applyLedHsv(LED_MOV_OVERLAY(LED_FLAG_OVERLAY(LED_OVERLAY_WARNING)), warningColor);
    }
}

static void applyLedBatteryLayer(bool updateNow, timeUs_t *timer)
{
    static bool flash = false;

    int state;
    int timeOffset = 1;

    if (updateNow) {
       state = getBatteryState();

       switch (state) {
           case BATTERY_OK:
               flash = false;
               timeOffset = 1;
               break;
           case BATTERY_WARNING:
               timeOffset = 2;
               break;
           default:
               timeOffset = 8;
               break;
       }
       flash = !flash;
    }

    *timer += LED_STRIP_HZ(timeOffset);

    if (!flash) {
       const hsvColor_t *bgc = getSC(LED_SCOLOR_BACKGROUND);
       applyLedHsv(LED_MOV_FUNCTION(LED_FUNCTION_BATTERY), bgc);
    }
}

static void applyLedRssiLayer(bool updateNow, timeUs_t *timer)
{
    static bool flash = false;

    int state;
    int timeOffset = 0;

    if (updateNow) {
       state = (rssi * 100) / 1023;

       if (state > 50) {
           flash = false;
           timeOffset = 1;
       } else if (state > 20) {
           timeOffset = 2;
       } else {
           timeOffset = 8;
       }
       flash = !flash;
    }


    *timer += LED_STRIP_HZ(timeOffset);

    if (!flash) {
       const hsvColor_t *bgc = getSC(LED_SCOLOR_BACKGROUND);
       applyLedHsv(LED_MOV_FUNCTION(LED_FUNCTION_RSSI), bgc);
    }
}

#ifdef GPS
static void applyLedGpsLayer(bool updateNow, timeUs_t *timer)
{
    static uint8_t gpsFlashCounter = 0;
    static uint8_t gpsPauseCounter = 0;
    const uint8_t blinkPauseLength = 4;

    if (updateNow) {
        if (gpsPauseCounter > 0) {
            gpsPauseCounter--;
        } else if (gpsFlashCounter >= (gpsSol.numSat - 1)) {
            gpsFlashCounter = 0;
            gpsPauseCounter = blinkPauseLength;
        } else {
            gpsFlashCounter++;
            gpsPauseCounter = 1;
        }
        *timer += LED_STRIP_HZ(2.5);
    }

    const hsvColor_t *gpsColor;

    if (gpsSol.numSat == 0 || !sensors(SENSOR_GPS)) {
        gpsColor = getSC(LED_SCOLOR_GPSNOSATS);
    } else {
        bool colorOn = gpsPauseCounter == 0;  // each interval starts with pause
        if (STATE(GPS_FIX)) {
            gpsColor = colorOn ? getSC(LED_SCOLOR_GPSLOCKED) : getSC(LED_SCOLOR_BACKGROUND);
        } else {
            gpsColor = colorOn ? getSC(LED_SCOLOR_GPSNOLOCK) : getSC(LED_SCOLOR_GPSNOSATS);
        }
    }

    applyLedHsv(LED_MOV_FUNCTION(LED_FUNCTION_GPS), gpsColor);
}

#endif

#define INDICATOR_DEADBAND 25

static void applyLedIndicatorLayer(bool updateNow, timeUs_t *timer)
{
    static bool flash = 0;

    if (updateNow) {
        if (rxIsReceivingSignal()) {
            // calculate update frequency
            int scale = MAX(ABS(rcCommand[ROLL]), ABS(rcCommand[PITCH]));  // 0 - 500
            scale += (50 - INDICATOR_DEADBAND);  // start increasing frequency right after deadband
            *timer += LED_STRIP_HZ(5) * 50 / MAX(50, scale);   // 5 - 50Hz update, 2.5 - 25Hz blink

            flash = !flash;
        } else {
            *timer += LED_STRIP_HZ(5);  // try again soon
        }
    }

    if (!flash)
        return;

    const hsvColor_t *flashColor = &HSV(ORANGE); // TODO - use user color?

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

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];
        if (ledGetOverlayBit(ledConfig, LED_OVERLAY_INDICATOR)) {
            if (getLedQuadrant(ledIndex) & quadrants)
                setLedHsv(ledIndex, flashColor);
        }
    }
}

#define ROTATION_SEQUENCE_LED_COUNT 6 // 2 on, 4 off
#define ROTATION_SEQUENCE_LED_WIDTH 2 // 2 on

static void updateLedRingCounts(void)
{
    int seqLen;
    // try to split in segments/rings of exactly ROTATION_SEQUENCE_LED_COUNT leds
    if ((ledCounts.ring % ROTATION_SEQUENCE_LED_COUNT) == 0) {
        seqLen = ROTATION_SEQUENCE_LED_COUNT;
    } else {
        seqLen = ledCounts.ring;
        // else split up in equal segments/rings of at most ROTATION_SEQUENCE_LED_COUNT leds
        // TODO - improve partitioning (15 leds -> 3x5)
        while ((seqLen > ROTATION_SEQUENCE_LED_COUNT) && ((seqLen % 2) == 0)) {
            seqLen /= 2;
        }
    }
    ledCounts.ringSeqLen = seqLen;
}

static void applyLedThrustRingLayer(bool updateNow, timeUs_t *timer)
{
    static uint8_t rotationPhase;
    int ledRingIndex = 0;

    if (updateNow) {
        rotationPhase = rotationPhase > 0 ? rotationPhase - 1 : ledCounts.ringSeqLen - 1;

        int scale = scaledThrottle; // ARMING_FLAG(ARMED) ? scaleRange(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX, 10, 100) : 10;
        *timer += LED_STRIP_HZ(5) * 10 / scale;  // 5 - 50Hz update rate
    }

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_THRUST_RING) {

            bool applyColor;
            if (ARMING_FLAG(ARMED)) {
                applyColor = (ledRingIndex + rotationPhase) % ledCounts.ringSeqLen < ROTATION_SEQUENCE_LED_WIDTH;
            } else {
                applyColor = !(ledRingIndex % 2); // alternating pattern
            }

            if (applyColor) {
                const hsvColor_t *ringColor = &ledStripConfig()->colors[ledGetColor(ledConfig)];
                setLedHsv(ledIndex, ringColor);
            }

            ledRingIndex++;
        }
    }
}

typedef struct larsonParameters_s {
    uint8_t currentBrightness;
    int8_t currentIndex;
    int8_t direction;
} larsonParameters_t;

static int brightnessForLarsonIndex(larsonParameters_t *larsonParameters, uint8_t larsonIndex)
{
    int offset = larsonIndex - larsonParameters->currentIndex;
    static const int larsonLowValue = 8;

    if (ABS(offset) > 1)
        return (larsonLowValue);

    if (offset == 0)
        return (larsonParameters->currentBrightness);

    if (larsonParameters->direction == offset) {
        return (larsonParameters->currentBrightness - 127);
    }

    return (255 - larsonParameters->currentBrightness);

}

static void larsonScannerNextStep(larsonParameters_t *larsonParameters, int delta)
{
    if (larsonParameters->currentBrightness > (255 - delta)) {
        larsonParameters->currentBrightness = 127;
        if (larsonParameters->currentIndex >= ledCounts.larson || larsonParameters->currentIndex < 0) {
            larsonParameters->direction = -larsonParameters->direction;
        }
        larsonParameters->currentIndex += larsonParameters->direction;
    } else {
        larsonParameters->currentBrightness += delta;
    }
}

static void applyLarsonScannerLayer(bool updateNow, timeUs_t *timer)
{
    static larsonParameters_t larsonParameters = { 0, 0, 1 };

    if (updateNow) {
        larsonScannerNextStep(&larsonParameters, 15);
        *timer += LED_STRIP_HZ(60);
    }

    int scannerLedIndex = 0;
    for (unsigned i = 0; i < ledCounts.count; i++) {

        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[i];

        if (ledGetOverlayBit(ledConfig, LED_OVERLAY_LARSON_SCANNER)) {
            hsvColor_t ledColor;
            getLedHsv(i, &ledColor);
            ledColor.v = brightnessForLarsonIndex(&larsonParameters, scannerLedIndex);
            setLedHsv(i, &ledColor);
            scannerLedIndex++;
        }
    }
}

// blink twice, then wait ; either always or just when landing
static void applyLedBlinkLayer(bool updateNow, timeUs_t *timer)
{
    const uint16_t blinkPattern = 0x8005; // 0b1000000000000101;
    static uint16_t blinkMask;

    if (updateNow) {
        blinkMask = blinkMask >> 1;
        if (blinkMask <= 1)
            blinkMask = blinkPattern;

        *timer += LED_STRIP_HZ(10);
    }

    bool ledOn = (blinkMask & 1);  // b_b_____...
    if (!ledOn) {
        for (int i = 0; i < ledCounts.count; ++i) {
            const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[i];

            if (ledGetOverlayBit(ledConfig, LED_OVERLAY_BLINK) ||
                    (ledGetOverlayBit(ledConfig, LED_OVERLAY_LANDING_FLASH) && scaledThrottle < 55 && scaledThrottle > 10)) {
                setLedHsv(i, getSC(LED_SCOLOR_BLINKBACKGROUND));
            }
        }
    }
}

#ifdef USE_LED_ANIMATION
static void applyLedAnimationLayer(bool updateNow, timeUs_t *timer)
{
    static uint8_t frameCounter = 0;
    const int animationFrames = ledGridHeight;
    if (updateNow) {
        frameCounter = (frameCounter + 1 < animationFrames) ? frameCounter + 1 : 0;
        *timer += LED_STRIP_HZ(20);
    }

    if (ARMING_FLAG(ARMED))
        return;

    int previousRow = frameCounter > 0 ? frameCounter - 1 : animationFrames - 1;
    int currentRow = frameCounter;
    int nextRow = (frameCounter + 1 < animationFrames) ? frameCounter + 1 : 0;

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];

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
    timBlink,
    timLarson,
    timBattery,
    timRssi,
#ifdef GPS
    timGps,
#endif
    timWarning,
    timIndicator,
#ifdef USE_LED_ANIMATION
    timAnimation,
#endif
    timRing,
    timTimerCount
} timId_e;

static timeUs_t timerVal[timTimerCount];

// function to apply layer.
// function must replan self using timer pointer
// when updateNow is true (timer triggered), state must be updated first,
//  before calculating led state. Otherwise update started by different trigger
//  may modify LED state.
typedef void applyLayerFn_timed(bool updateNow, timeUs_t *timer);

static applyLayerFn_timed* layerTable[] = {
    [timBlink] = &applyLedBlinkLayer,
    [timLarson] = &applyLarsonScannerLayer,
    [timBattery] = &applyLedBatteryLayer,
    [timRssi] = &applyLedRssiLayer,
#ifdef GPS
    [timGps] = &applyLedGpsLayer,
#endif
    [timWarning] = &applyLedWarningLayer,
    [timIndicator] = &applyLedIndicatorLayer,
#ifdef USE_LED_ANIMATION
    [timAnimation] = &applyLedAnimationLayer,
#endif
    [timRing] = &applyLedThrustRingLayer
};

void ledStripUpdate(timeUs_t currentTimeUs)
{
    if (!(ledStripInitialised && isWS2811LedStripReady())) {
        return;
    }

    if (IS_RC_MODE_ACTIVE(BOXLEDLOW)) {
        if (ledStripEnabled) {
            ledStripDisable();
            ledStripEnabled = false;
        }
        return;
    }
    ledStripEnabled = true;

    // test all led timers, setting corresponding bits
    uint32_t timActive = 0;
    for (timId_e timId = 0; timId < timTimerCount; timId++) {
        // sanitize timer value, so that it can be safely incremented. Handles inital timerVal value.
        // max delay is limited to 5s
        int32_t delta = cmpTimeUs(currentTimeUs, timerVal[timId]);
        if (delta < 0 && delta > -LED_STRIP_MS(5000))
            continue;  // not ready yet
        timActive |= 1 << timId;
        if (delta >= LED_STRIP_MS(100) || delta < 0) {
            timerVal[timId] = currentTimeUs;
        }
    }

    if (!timActive)
        return;          // no change this update, keep old state

    // apply all layers; triggered timed functions has to update timers

    scaledThrottle = ARMING_FLAG(ARMED) ? scaleRange(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX, 10, 100) : 10;

    applyLedFixedLayers();

    for (timId_e timId = 0; timId < ARRAYLEN(layerTable); timId++) {
        timeUs_t *timer = &timerVal[timId];
        bool updateNow = timActive & (1 << timId);
        (*layerTable[timId])(updateNow, timer);
    }
    ws2811UpdateStrip();
}

bool parseColor(int index, const char *colorConfig)
{
    const char *remainingCharacters = colorConfig;

    hsvColor_t *color = &ledStripConfigMutable()->colors[index];

    bool result = true;
    static const uint16_t hsv_limit[HSV_COLOR_COMPONENT_COUNT] = {
        [HSV_HUE] = HSV_HUE_MAX,
        [HSV_SATURATION] = HSV_SATURATION_MAX,
        [HSV_VALUE] = HSV_VALUE_MAX,
    };
    for (int componentIndex = 0; result && componentIndex < HSV_COLOR_COMPONENT_COUNT; componentIndex++) {
        int val = fastA2I(remainingCharacters);
        if (val > hsv_limit[componentIndex]) {
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
    if (colorIndex < 0 || colorIndex >= LED_CONFIGURABLE_COLOR_COUNT)
        return false;
    if (modeIndex < LED_MODE_COUNT) {  // modeIndex_e is unsigned, so one-sided test is enough
        if (modeColorIndex < 0 || modeColorIndex >= LED_DIRECTION_COUNT)
            return false;
        ledStripConfigMutable()->modeColors[modeIndex].color[modeColorIndex] = colorIndex;
    } else if (modeIndex == LED_SPECIAL) {
        if (modeColorIndex < 0 || modeColorIndex >= LED_SPECIAL_COLOR_COUNT)
            return false;
        ledStripConfigMutable()->specialColors.color[modeColorIndex] = colorIndex;
    } else {
        return false;
    }
    return true;
}

void ledStripInit(void)
{
    ledStripInitialised = false;
}

void ledStripEnable(void)
{
    reevaluateLedConfig();
    ledStripInitialised = true;

    ws2811LedStripInit();
}

static void ledStripDisable(void)
{
    setStripColor(&HSV(BLACK));

    ws2811UpdateStrip();
}
#endif
