/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include <stddef.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "build/build_config.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/light_ws2811strip.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "flight/position.h"

#include "io/beeper.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/vtx.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

#define COLOR_UNDEFINED 255

static bool ledStripEnabled = false;

#define HZ_TO_US(hz) ((int32_t)((1000 * 1000) / (hz)))

#define MAX_TIMER_DELAY (5 * 1000 * 1000)

#define TASK_LEDSTRIP_RATE_WAIT_HZ 500    // Scheduling rate waiting for a timer to fire
#define TASK_LEDSTRIP_RATE_FAST_HZ 100000 // Reschedule as fast as possible

#define LED_OVERLAY_RAINBOW_RATE_HZ 60
#define LED_OVERLAY_LARSON_RATE_HZ 60
#define LED_OVERLAY_BLINK_RATE_HZ 10
#define LED_OVERLAY_VTX_RATE_HZ 5
#define LED_OVERLAY_INDICATOR_RATE_HZ 5
#define LED_OVERLAY_WARNING_RATE_HZ 10

#define LED_TASK_MARGIN                 1
// Decay the estimated max task duration by 1/(1 << LED_EXEC_TIME_SHIFT) on every invocation
#define LED_EXEC_TIME_SHIFT             7

#define PROFILE_COLOR_UPDATE_INTERVAL_US 1e6f  // normally updates when color changes but this is a 1 second forced update

#define VISUAL_BEEPER_COLOR COLOR_WHITE

#define BEACON_FAILSAFE_PERIOD_MS 250      // 4Hz
#define BEACON_FAILSAFE_ON_PERCENT 50      // 50% duty cycle

const hsvColor_t hsv[] = {
    //                        H    S    V
    [COLOR_BLACK] =        {  0,   0,   0}, // LED is off
    [COLOR_WHITE] =        {  0, 255, 255}, // for white, S must be 255 and V must be 255, H is ignored
    [COLOR_RED] =          {  0,   0, 255}, // for full colour S must be 0 and V must be 255
    [COLOR_ORANGE] =       { 15,   0, 255},
    [COLOR_YELLOW] =       { 50,   0, 255},
    [COLOR_LIME_GREEN] =   { 100,  0, 255},
    [COLOR_GREEN] =        {115,   0, 255},
    [COLOR_MINT_GREEN] =   {125,   0, 255},
    [COLOR_CYAN] =         {180,   0, 255},
    [COLOR_LIGHT_BLUE] =   {210,   0, 255},
    [COLOR_BLUE] =         {240,   0, 255},
    [COLOR_DARK_VIOLET] =  {270,   0, 255},
    [COLOR_MAGENTA] =      {300,   0, 255},
    [COLOR_DEEP_PINK] =    {330,   0, 255},
};
// macro to save typing on default colors
#define HSV(color) (hsv[COLOR_ ## color])



#define VTX_HUE_START_FREQ 5658     //  R1, returns solid red at R1
#define VTX_HUE_STOP_FREQ 5900      // frequency just below R8  at 5917,  above which Hue does not exceed VTX_HUE_MAX
#define VTX_HUE_MAX 355             // Maximum Hue value. Hue is circular in degrees, with red at 0 and 360.  345 is a strong magenta that is easily distinguished from red.

#ifdef USE_VTX_COMMON
static hsvColor_t getHsvFromVtxFrequency(uint16_t freq)
{
// assign LowBand and RaceBand Vtx channels to colors, with the same color for the same channel number in each band.

    hsvColor_t color = HSV(BLACK);
    const unsigned bandRange = 296; // width of the L or R bands;  8 channels per band, each 37MHz wide

    if (freq < VTX_SETTINGS_MIN_FREQUENCY_MHZ) {
        // Invalid channel or frequency value, e.g., user hasn't set band or channel
        return color;
        // turn LEDs off
    } else if (freq < VTX_HUE_START_FREQ - bandRange) {
        return HSV(WHITE);
        // frequency too low to map; exit and show white
    }
    else {
        if (freq < VTX_HUE_START_FREQ) {
            // fold frequencies below RaceBand up to equivalent RaceBand frequencies for Hue assignment 
            freq += bandRange;
        }
    if (freq > VTX_HUE_STOP_FREQ) {
            freq = VTX_HUE_STOP_FREQ;
            // Clamp incoming frequencies to an upper limit around R8
    }

        color.s = 0;
        color.v = 255;
        // for strong colours in betaflight, S must be 0 and V must be 255
        color.h = scaleRange(freq, VTX_HUE_START_FREQ, VTX_HUE_STOP_FREQ, 0, VTX_HUE_MAX);
        // scale Hue from 0 (red) to VTX_HUE_MAX, linearly across frequency range
        if (color.h < 165) {
    color.h = (color.h * 9) / 10;
    // warm up the yellow, pull cyan a bit further from blue, and keep a decent green
}
        return color;
    }
}
#endif


PG_REGISTER_WITH_RESET_FN(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 6);

// Default LED strip brightness (percent). A target may lower this, e.g. when a
// single bright addressable LED is used as a status indicator.
#ifndef LED_STRIP_DEFAULT_BRIGHTNESS
#define LED_STRIP_DEFAULT_BRIGHTNESS 100
#endif

void pgResetFn_ledStripConfig(ledStripConfig_t *ledStripConfig)
{
    ledStripConfig->ledstrip_visual_beeper = 1;
#ifdef USE_LED_STRIP_STATUS_MODE
    ledStripConfig->ledstrip_profile = LED_PROFILE_STATUS;
#else
    ledStripConfig->ledstrip_profile = LED_PROFILE_RACE;
#endif
    ledStripConfig->ledstrip_race_color = COLOR_ORANGE;
    ledStripConfig->ledstrip_beacon_color = COLOR_WHITE;
    ledStripConfig->ledstrip_beacon_period_ms = 500;    // 0.5 second (2hz)
    ledStripConfig->ledstrip_beacon_percent = 50;       // 50% duty cycle
    ledStripConfig->ledstrip_beacon_armed_only = false; // blink always
    ledStripConfig->ledstrip_visual_beeper_color = VISUAL_BEEPER_COLOR;
    ledStripConfig->ledstrip_brightness = LED_STRIP_DEFAULT_BRIGHTNESS;
    ledStripConfig->ledstrip_rainbow_delta = 0;
    ledStripConfig->ledstrip_rainbow_freq = 120;
    ledStripConfig->ledstrip_larson_freq = LED_LARSON_FREQ_DEFAULT;
    ledStripConfig->ledstrip_blink_period_ms = LED_BLINK_PERIOD_MS_DEFAULT;
    ledStripConfig->ledstrip_blink_on_ms = LED_BLINK_ON_MS_DEFAULT;
    ledStripConfig->ledstrip_blink_pattern = LED_BLINK_PATTERN_ALTERNATE;
    ledStripConfig->ledstrip_blink_flash_ms = LED_BLINK_FLASH_MS_DEFAULT;
    ledStripConfig->ledstrip_blink_gap_ms = LED_BLINK_GAP_MS_DEFAULT;
    ledStripConfig->ledstrip_blink_pause_ms = LED_BLINK_PAUSE_MS_DEFAULT;
#ifndef UNIT_TEST
#ifdef LED_STRIP_PIN
    ledStripConfig->ioTag = IO_TAG(LED_STRIP_PIN);
#else
    ledStripConfig->ioTag = IO_TAG_NONE;
#endif
#endif
}

#ifdef USE_LED_STRIP_STATUS_MODE

#if LED_STRIP_MAX_LENGTH > WS2811_LED_STRIP_LENGTH
# error "Led strip length must match driver"
#endif

typedef enum {
    LED_PROFILE_SLOW,
    LED_PROFILE_FAST,
    LED_PROFILE_ADVANCE
} ledProfileSequence_t;

const hsvColor_t *colors;
const modeColorIndexes_t *modeColors;
specialColorIndexes_t specialColors;

STATIC_UNIT_TESTED uint8_t ledGridRows;
// grid offsets
STATIC_UNIT_TESTED int8_t highestYValueForNorth;
STATIC_UNIT_TESTED int8_t lowestYValueForSouth;
STATIC_UNIT_TESTED int8_t highestXValueForWest;
STATIC_UNIT_TESTED int8_t lowestXValueForEast;

STATIC_UNIT_TESTED ledCounts_t ledCounts;

static const modeColorIndexes_t defaultModeColors[] = {
    //                          NORTH             EAST               SOUTH            WEST             UP          DOWN
    [LED_MODE_ORIENTATION] = {{ COLOR_WHITE,      COLOR_DARK_VIOLET, COLOR_RED,       COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_HEADFREE]    = {{ COLOR_LIME_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_HORIZON]     = {{ COLOR_BLUE,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_ANGLE]       = {{ COLOR_CYAN,       COLOR_DARK_VIOLET, COLOR_YELLOW,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
    [LED_MODE_MAG]         = {{ COLOR_MINT_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE,    COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE }},
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

PG_REGISTER_WITH_RESET_FN(ledStripProfilesConfig_t, ledStripProfilesConfig, PG_LED_STRIP_STATUS_MODE_CONFIG, 7);

static const char * const defaultLedProfileNames[LED_PROFILE_COUNT] = {
    "RACE",
    "BEACON",
#ifdef USE_LED_STRIP_STATUS_MODE
    "STATUS",
#endif
};

static void initDefaultLedProfileNames(ledStripProfilesConfig_t *ledStripProfilesConfig)
{
    for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
        strncpy(ledStripProfilesConfig->profileNames[profileIndex], defaultLedProfileNames[profileIndex], MAX_LED_PROFILE_NAME_LENGTH);
        ledStripProfilesConfig->profileNames[profileIndex][MAX_LED_PROFILE_NAME_LENGTH] = '\0';
    }
}

static void pgResetFn_ledStripStatusModeProfile(ledStripStatusModeConfig_t *ledStripStatusModeConfig)
{
    memset(ledStripStatusModeConfig->ledConfigs, 0, LED_STRIP_MAX_LENGTH * sizeof(ledConfig_t));
#ifdef LED_STRIP_DEFAULT_LED0
    // A target may seed a default LED (e.g. a single onboard status LED) so the
    // strip drives the hardware out of reset, before any user-configured layout.
    ledStripStatusModeConfig->ledConfigs[0] = LED_STRIP_DEFAULT_LED0;
#endif
    // copy hsv colors as default
    memset(ledStripStatusModeConfig->colors, 0, ARRAYLEN(hsv) * sizeof(hsvColor_t));
    STATIC_ASSERT(LED_CONFIGURABLE_COLOR_COUNT >= ARRAYLEN(hsv), LED_CONFIGURABLE_COLOR_COUNT_invalid);
    for (unsigned colorIndex = 0; colorIndex < ARRAYLEN(hsv); colorIndex++) {
        ledStripStatusModeConfig->colors[colorIndex] = hsv[colorIndex];
    }
    memcpy_fn(&ledStripStatusModeConfig->modeColors, &defaultModeColors, sizeof(defaultModeColors));
    memcpy_fn(&ledStripStatusModeConfig->specialColors, &defaultSpecialColors, sizeof(defaultSpecialColors));
    ledStripStatusModeConfig->ledstrip_aux_channel = THROTTLE;
    ledStripStatusModeConfig->profile_brightness = LED_STRIP_PROFILE_BRIGHTNESS_USE_MASTER;
    ledStripStatusModeConfig->profile_larson_freq = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_rainbow_delta = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_rainbow_freq = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_blink_period = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_blink_on_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_blink_pattern = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_blink_flash_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_blink_gap_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
    ledStripStatusModeConfig->profile_blink_pause_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
}

static void initSolidColorProfile(ledStripStatusModeConfig_t *profile, colorId_e color, bool blink)
{
    pgResetFn_ledStripStatusModeProfile(profile);

    const ledConfig_t solidColorLed = LED_MOV_FUNCTION(LED_FUNCTION_COLOR) | LED_MOV_COLOR(color)
        | (blink ? LED_MOV_OVERLAY(LED_FLAG_OVERLAY(LED_OVERLAY_BLINK)) : 0);

    for (int ledIndex = 0; ledIndex < LED_STRIP_MAX_LENGTH; ledIndex++) {
        profile->ledConfigs[ledIndex] = solidColorLed;
    }
}

void pgResetFn_ledStripProfilesConfig(ledStripProfilesConfig_t *ledStripProfilesConfig)
{
    pgResetFn_ledStripStatusModeProfile(&ledStripProfilesConfig->profiles[LED_PROFILE_STATUS]);
    initSolidColorProfile(&ledStripProfilesConfig->profiles[LED_PROFILE_RACE], COLOR_ORANGE, false);
    initSolidColorProfile(&ledStripProfilesConfig->profiles[LED_PROFILE_BEACON], COLOR_WHITE, true);
    initDefaultLedProfileNames(ledStripProfilesConfig);
}

const ledStripStatusModeConfig_t *ledStripProfileConfig(uint8_t profile)
{
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }
    return &ledStripProfilesConfig()->profiles[profile];
}

ledStripStatusModeConfig_t *ledStripProfileConfigMutable(uint8_t profile)
{
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }
    return &ledStripProfilesConfigMutable()->profiles[profile];
}

const char *ledStripProfileName(uint8_t profile)
{
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const char *name = ledStripProfilesConfig()->profileNames[profile];
    if (name[0] != '\0') {
        return name;
    }

    return defaultLedProfileNames[profile];
}

char *ledStripProfileNameMutable(uint8_t profile)
{
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    return ledStripProfilesConfigMutable()->profileNames[profile];
}

const ledStripStatusModeConfig_t *ledStripActiveProfileConfig(void)
{
    return ledStripProfileConfig(ledStripConfig()->ledstrip_profile);
}

ledStripStatusModeConfig_t *ledStripActiveProfileConfigMutable(void)
{
    return ledStripProfileConfigMutable(ledStripConfig()->ledstrip_profile);
}

void updateActiveLedProfilePointers(void)
{
    const ledStripStatusModeConfig_t *profile = ledStripActiveProfileConfig();
    colors = profile->colors;
    modeColors = profile->modeColors;
    specialColors = profile->specialColors;
}

void syncActiveLedProfileConfig(void)
{
    updateActiveLedProfilePointers();
    reevaluateLedConfig();
    resetLedStripProfileRenderState();
}

uint8_t migrateLedBlinkPattern(uint8_t pattern)
{
    if (pattern == LED_BLINK_PATTERN_DOUBLE_DEPRECATED) {
        return LED_BLINK_PATTERN_BEACON;
    }

    return pattern;
}

static void convertLegacyPeriodOnToFlashPause(uint16_t periodMs, uint16_t onMs, uint16_t *flashMs, uint16_t *pauseMs)
{
    if (periodMs < 50) {
        periodMs = LED_BLINK_PERIOD_MS_DEFAULT;
    }

    if (onMs < 1) {
        onMs = LED_BLINK_ON_MS_DEFAULT;
    }

    if (onMs > periodMs) {
        onMs = periodMs;
    }

    *flashMs = onMs;
    *pauseMs = periodMs - onMs;

    if (*flashMs < LED_BLINK_FLASH_MS_MIN) {
        *flashMs = LED_BLINK_FLASH_MS_MIN;
    } else if (*flashMs > 300) {
        *flashMs = 300;
    }

    if (*pauseMs < LED_BLINK_PAUSE_MS_MIN_ALTERNATE) {
        *pauseMs = LED_BLINK_PAUSE_MS_MIN_ALTERNATE;
    } else if (*pauseMs > 2000) {
        *pauseMs = 2000;
    }
}

static void migrateMasterAlternateBlinkFromLegacyPeriodOn(void)
{
    const uint8_t pattern = migrateLedBlinkPattern(ledStripConfig_System.ledstrip_blink_pattern);

    if (pattern != LED_BLINK_PATTERN_ALTERNATE) {
        return;
    }

    const uint16_t periodMs = ledStripConfig_System.ledstrip_blink_period_ms;
    const uint16_t onMs = ledStripConfig_System.ledstrip_blink_on_ms;
    const bool legacyTimingCustom = (periodMs != LED_BLINK_PERIOD_MS_DEFAULT) || (onMs != LED_BLINK_ON_MS_DEFAULT);
    const bool flashPauseAtDefaults =
        (ledStripConfig_System.ledstrip_blink_flash_ms == LED_BLINK_FLASH_MS_DEFAULT) &&
        (ledStripConfig_System.ledstrip_blink_pause_ms == LED_BLINK_PAUSE_MS_DEFAULT);

    if (legacyTimingCustom && flashPauseAtDefaults) {
        convertLegacyPeriodOnToFlashPause(
            periodMs,
            onMs,
            &ledStripConfig_System.ledstrip_blink_flash_ms,
            &ledStripConfig_System.ledstrip_blink_pause_ms
        );
    }
}

static void migrateProfileAlternateBlinkFromLegacyPeriodOn(ledStripStatusModeConfig_t *profile)
{
    const uint8_t pattern = profile->profile_blink_pattern;

    if (pattern != 0 && pattern != LED_BLINK_PATTERN_ALTERNATE) {
        return;
    }

    if (profile->profile_blink_flash_ms != 0 || profile->profile_blink_pause_ms != 0) {
        return;
    }

    if (profile->profile_blink_period == 0 && profile->profile_blink_on_ms == 0) {
        return;
    }

    const uint16_t periodMs = profile->profile_blink_period > 0 ? profile->profile_blink_period : LED_BLINK_PERIOD_MS_DEFAULT;
    const uint16_t onMs = profile->profile_blink_on_ms > 0 ? profile->profile_blink_on_ms : LED_BLINK_ON_MS_DEFAULT;

    convertLegacyPeriodOnToFlashPause(periodMs, onMs, &profile->profile_blink_flash_ms, &profile->profile_blink_pause_ms);
}

bool loadLedStripConfig(const void *from, int size, int version)
{
    pgResetInstance(&ledStripConfig_Registry, (uint8_t *)&ledStripConfig_System);

    if (version == pgVersion(&ledStripConfig_Registry)) {
        const int take = MIN(size, (int)sizeof(ledStripConfig_t));
        memcpy(&ledStripConfig_System, from, take);
        ledStripConfig_System.ledstrip_blink_pattern = migrateLedBlinkPattern(ledStripConfig_System.ledstrip_blink_pattern);
        migrateMasterAlternateBlinkFromLegacyPeriodOn();
        return true;
    }

    if (version == 5) {
        const size_t periodOffset = offsetof(ledStripConfig_t, ledstrip_blink_period_ms);
        const size_t oldSize = offsetof(ledStripConfig_t, ledstrip_blink_flash_ms);

        if (size >= (int)oldSize) {
            memcpy(&ledStripConfig_System, from, periodOffset);
            const uint8_t *oldConfig = (const uint8_t *)from;
            const uint16_t periodMs = *(const uint16_t *)(oldConfig + periodOffset);
            const uint8_t blinkPercent = oldConfig[periodOffset + sizeof(uint16_t)];
            const uint8_t blinkPattern = oldConfig[periodOffset + sizeof(uint16_t) + 1];
            uint16_t onMs = (uint16_t)((uint32_t)periodMs * blinkPercent / 100);

            if (onMs < 1) {
                onMs = 1;
            }

            ledStripConfig_System.ledstrip_blink_period_ms = periodMs;
            ledStripConfig_System.ledstrip_blink_on_ms = onMs;
            ledStripConfig_System.ledstrip_blink_pattern = migrateLedBlinkPattern(blinkPattern);
            ledStripConfig_System.ledstrip_blink_flash_ms = *(const uint16_t *)(oldConfig + periodOffset + sizeof(uint16_t) + 2);
            ledStripConfig_System.ledstrip_blink_gap_ms = *(const uint16_t *)(oldConfig + periodOffset + sizeof(uint16_t) + 4);
            ledStripConfig_System.ledstrip_blink_pause_ms = *(const uint16_t *)(oldConfig + periodOffset + sizeof(uint16_t) + 6);
            migrateMasterAlternateBlinkFromLegacyPeriodOn();
            return true;
        }
    }

    if (version == 4) {
        const size_t oldSize = offsetof(ledStripConfig_t, ledstrip_blink_flash_ms);

        if (size >= (int)oldSize) {
            memcpy(&ledStripConfig_System, from, oldSize);
            return true;
        }
    }

    return false;
}

bool loadLedStripProfilesConfig(const void *from, int size, int version)
{
    pgResetInstance(&ledStripProfilesConfig_Registry, (uint8_t *)&ledStripProfilesConfig_System);

    if (version == pgVersion(&ledStripProfilesConfig_Registry)) {
        const int take = MIN(size, (int)sizeof(ledStripProfilesConfig_t));
        memcpy(&ledStripProfilesConfig_System, from, take);
        for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
            ledStripStatusModeConfig_t *profile = &ledStripProfilesConfig_System.profiles[profileIndex];
            profile->profile_blink_pattern = migrateLedBlinkPattern(profile->profile_blink_pattern);
            migrateProfileAlternateBlinkFromLegacyPeriodOn(profile);
        }
        return true;
    }

    if (version == 6) {
        const size_t periodOffset = offsetof(ledStripStatusModeConfig_t, profile_blink_period);
        const size_t oldProfileSize = offsetof(ledStripStatusModeConfig_t, profile_blink_flash_ms);
        const size_t oldProfilesSize = oldProfileSize * LED_PROFILE_COUNT;

        if (size >= (int)oldProfilesSize) {
            for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
                const uint8_t *oldProfile = (const uint8_t *)from + profileIndex * oldProfileSize;
                ledStripStatusModeConfig_t *profile = &ledStripProfilesConfig_System.profiles[profileIndex];

                memcpy(profile, oldProfile, periodOffset);
                const uint16_t periodMs = *(const uint16_t *)(oldProfile + periodOffset);
                const uint8_t blinkPercent = oldProfile[periodOffset + sizeof(uint16_t)];
                const uint8_t blinkPattern = oldProfile[periodOffset + sizeof(uint16_t) + 1];
                uint16_t onMs = (uint16_t)((uint32_t)periodMs * blinkPercent / 100);

                if (onMs < 1) {
                    onMs = 1;
                }

                profile->profile_blink_period = periodMs;
                profile->profile_blink_pattern = migrateLedBlinkPattern(blinkPattern);

                if (blinkPercent == 0 || periodMs == 0) {
                    profile->profile_blink_on_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                } else {
                    profile->profile_blink_on_ms = onMs;
                }

                profile->profile_blink_flash_ms = *(const uint16_t *)(oldProfile + periodOffset + sizeof(uint16_t) + 2);
                profile->profile_blink_gap_ms = *(const uint16_t *)(oldProfile + periodOffset + sizeof(uint16_t) + 4);
                profile->profile_blink_pause_ms = *(const uint16_t *)(oldProfile + periodOffset + sizeof(uint16_t) + 6);
                migrateProfileAlternateBlinkFromLegacyPeriodOn(profile);
            }

            if (size >= (int)(oldProfilesSize + sizeof(ledStripProfilesConfig_System.profileNames))) {
                memcpy(
                    ledStripProfilesConfig_System.profileNames,
                    (const uint8_t *)from + oldProfilesSize,
                    sizeof(ledStripProfilesConfig_System.profileNames)
                );
            } else {
                initDefaultLedProfileNames(&ledStripProfilesConfig_System);
            }
            return true;
        }
    }

    if (version == 5) {
        const size_t oldProfileSize = offsetof(ledStripStatusModeConfig_t, profile_blink_flash_ms);
        const size_t oldProfilesSize = oldProfileSize * LED_PROFILE_COUNT;

        if (size >= (int)oldProfilesSize) {
            for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
                memcpy(
                    &ledStripProfilesConfig_System.profiles[profileIndex],
                    (const uint8_t *)from + profileIndex * oldProfileSize,
                    oldProfileSize
                );
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_flash_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_gap_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_pause_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
            }

            for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
                ledStripStatusModeConfig_t *profile = &ledStripProfilesConfig_System.profiles[profileIndex];
                profile->profile_blink_pattern = migrateLedBlinkPattern(profile->profile_blink_pattern);
                migrateProfileAlternateBlinkFromLegacyPeriodOn(profile);
            }

            if (size >= (int)(oldProfilesSize + sizeof(ledStripProfilesConfig_System.profileNames))) {
                memcpy(
                    ledStripProfilesConfig_System.profileNames,
                    (const uint8_t *)from + oldProfilesSize,
                    sizeof(ledStripProfilesConfig_System.profileNames)
                );
            } else {
                initDefaultLedProfileNames(&ledStripProfilesConfig_System);
            }
            return true;
        }
    }

    if (version == 4) {
        const size_t oldProfileSize = offsetof(ledStripStatusModeConfig_t, profile_blink_period);
        const size_t oldProfilesSize = oldProfileSize * LED_PROFILE_COUNT;

        if (size >= (int)oldProfilesSize) {
            for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
                memcpy(
                    &ledStripProfilesConfig_System.profiles[profileIndex],
                    (const uint8_t *)from + profileIndex * oldProfileSize,
                    oldProfileSize
                );
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_period = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_on_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_pattern = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
            }

            if (size >= (int)(oldProfilesSize + sizeof(ledStripProfilesConfig_System.profileNames))) {
                memcpy(
                    ledStripProfilesConfig_System.profileNames,
                    (const uint8_t *)from + oldProfilesSize,
                    sizeof(ledStripProfilesConfig_System.profileNames)
                );
            } else {
                initDefaultLedProfileNames(&ledStripProfilesConfig_System);
            }
            return true;
        }
    }

    if (version == 3) {
        const size_t oldProfileSize = offsetof(ledStripStatusModeConfig_t, profile_larson_freq);
        const size_t oldProfilesSize = oldProfileSize * LED_PROFILE_COUNT;

        if (size >= (int)oldProfilesSize) {
            for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
                memcpy(
                    &ledStripProfilesConfig_System.profiles[profileIndex],
                    (const uint8_t *)from + profileIndex * oldProfileSize,
                    oldProfileSize
                );
                ledStripProfilesConfig_System.profiles[profileIndex].profile_larson_freq = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_rainbow_delta = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_rainbow_freq = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_period = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_on_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_pattern = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
            }

            if (size >= (int)(oldProfilesSize + sizeof(ledStripProfilesConfig_System.profileNames))) {
                memcpy(
                    ledStripProfilesConfig_System.profileNames,
                    (const uint8_t *)from + oldProfilesSize,
                    sizeof(ledStripProfilesConfig_System.profileNames)
                );
            } else {
                initDefaultLedProfileNames(&ledStripProfilesConfig_System);
            }
            return true;
        }
    }

    if (version == 2) {
        const size_t oldProfileSize = offsetof(ledStripStatusModeConfig_t, profile_brightness);
        const size_t oldProfilesSize = oldProfileSize * LED_PROFILE_COUNT;

        if (size >= (int)oldProfilesSize) {
            for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
                memcpy(
                    &ledStripProfilesConfig_System.profiles[profileIndex],
                    (const uint8_t *)from + profileIndex * oldProfileSize,
                    oldProfileSize
                );
                ledStripProfilesConfig_System.profiles[profileIndex].profile_brightness = LED_STRIP_PROFILE_BRIGHTNESS_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_larson_freq = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_rainbow_delta = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_rainbow_freq = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_period = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_on_ms = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
                ledStripProfilesConfig_System.profiles[profileIndex].profile_blink_pattern = LED_STRIP_PROFILE_OVERLAY_USE_MASTER;
            }

            if (size >= (int)(oldProfilesSize + sizeof(ledStripProfilesConfig_System.profileNames))) {
                memcpy(
                    ledStripProfilesConfig_System.profileNames,
                    (const uint8_t *)from + oldProfilesSize,
                    sizeof(ledStripProfilesConfig_System.profileNames)
                );
            } else {
                initDefaultLedProfileNames(&ledStripProfilesConfig_System);
            }
            return true;
        }
    }

    if (version == 1 && size >= (int)(offsetof(ledStripStatusModeConfig_t, profile_brightness) * LED_PROFILE_COUNT)) {
        const size_t oldProfileSize = offsetof(ledStripStatusModeConfig_t, profile_brightness);
        for (unsigned profileIndex = 0; profileIndex < LED_PROFILE_COUNT; profileIndex++) {
            memcpy(
                &ledStripProfilesConfig_System.profiles[profileIndex],
                (const uint8_t *)from + profileIndex * oldProfileSize,
                oldProfileSize
            );
            ledStripProfilesConfig_System.profiles[profileIndex].profile_brightness = LED_STRIP_PROFILE_BRIGHTNESS_USE_MASTER;
        }
        initDefaultLedProfileNames(&ledStripProfilesConfig_System);
        return true;
    }

    if (version == 0 && size >= (int)offsetof(ledStripStatusModeConfig_t, profile_brightness)) {
        const size_t oldProfileSize = offsetof(ledStripStatusModeConfig_t, profile_brightness);
        memcpy(&ledStripProfilesConfig_System.profiles[LED_PROFILE_STATUS], from, MIN(size, (int)oldProfileSize));
        ledStripProfilesConfig_System.profiles[LED_PROFILE_STATUS].profile_brightness = LED_STRIP_PROFILE_BRIGHTNESS_USE_MASTER;
        initSolidColorProfile(&ledStripProfilesConfig_System.profiles[LED_PROFILE_RACE], ledStripConfig()->ledstrip_race_color, false);
        initSolidColorProfile(&ledStripProfilesConfig_System.profiles[LED_PROFILE_BEACON], ledStripConfig()->ledstrip_beacon_color, true);
        initDefaultLedProfileNames(&ledStripProfilesConfig_System);
        return true;
    }

    return false;
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

STATIC_UNIT_TESTED void updateDimensions(void)
{
    int maxX = 0;
    int minX = LED_XY_MASK;
    int maxY = 0;
    int minY = LED_XY_MASK;

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];

        int ledX = ledGetX(ledConfig);
        maxX = MAX(ledX, maxX);
        minX = MIN(ledX, minX);
        int ledY = ledGetY(ledConfig);
        maxY = MAX(ledY, maxY);
        minY = MIN(ledY, minY);
    }

    ledGridRows = maxY - minY + 1;

    if (minX < maxX) {
        lowestXValueForEast = (minX + maxX) / 2 + 1;
        highestXValueForWest = (minX + maxX - 1) / 2;
    } else {
        lowestXValueForEast = LED_XY_MASK / 2;
        highestXValueForWest = lowestXValueForEast - 1;
    }
    if (minY < maxY) {
        lowestYValueForSouth = (minY + maxY) / 2 + 1;
        highestYValueForNorth = (minY + maxY - 1) / 2;
    } else {
        lowestYValueForSouth = LED_XY_MASK / 2;
        highestYValueForNorth = lowestYValueForSouth - 1;
    }

}

enum ledBarIds {
   LED_BAR_GPS,
   LED_BAR_BATTERY,
   LED_BAR_COUNT
};
static uint8_t ledBarStates[LED_BAR_COUNT] = {0};

static void updateLedBars(void)
{
    memset(ledBarStates, 0, sizeof(ledBarStates));
    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];
        int fn = ledGetFunction(ledConfig);
        switch (fn) {
#ifdef USE_GPS
            case LED_FUNCTION_GPS_BAR:
                ledBarStates[LED_BAR_GPS]++;
                break;
#endif
            case LED_FUNCTION_BATTERY_BAR:
                ledBarStates[LED_BAR_BATTERY]++;
                break;
            default:
                break;
            }
        }
}

STATIC_UNIT_TESTED void updateLedCount(void)
{
    int count = 0, countRing = 0, countScanner= 0;

    for (int ledIndex = 0; ledIndex < LED_STRIP_MAX_LENGTH; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];

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
    setUsedLedCount(ledCounts.count);
}

void reevaluateLedConfig(void)
{
    updateLedCount();
    updateDimensions();
    updateLedRingCounts();
    updateRequiredOverlay();
    updateLedBars();
}

// get specialColor by index
static const hsvColor_t* getSC(ledSpecialColorIds_e index)
{
    return &ledStripActiveProfileConfig()->colors[ledStripActiveProfileConfig()->specialColors.color[index]];
}

static const char directionCodes[LED_DIRECTION_COUNT] = {
    [LED_DIRECTION_NORTH] = 'N',
    [LED_DIRECTION_EAST] = 'E',
    [LED_DIRECTION_SOUTH] = 'S',
    [LED_DIRECTION_WEST] = 'W',
    [LED_DIRECTION_UP] = 'U',
    [LED_DIRECTION_DOWN] = 'D'
};
static const char baseFunctionCodes[LED_BASEFUNCTION_COUNT] = {
    [LED_FUNCTION_COLOR] = 'C',
    [LED_FUNCTION_FLIGHT_MODE] = 'F',
    [LED_FUNCTION_ARM_STATE] = 'A',
    [LED_FUNCTION_BATTERY] = 'L',
    [LED_FUNCTION_RSSI] = 'S',
    [LED_FUNCTION_GPS] = 'G',
    [LED_FUNCTION_THRUST_RING] = 'R',
    [LED_FUNCTION_GPS_BAR] = 'P',
    [LED_FUNCTION_BATTERY_BAR] = 'E',
    [LED_FUNCTION_ALTITUDE] = 'U'
};
static const char overlayCodes[LED_OVERLAY_COUNT] = {
    [LED_OVERLAY_THROTTLE] = 'T',
    [LED_OVERLAY_RAINBOW] = 'Y',
    [LED_OVERLAY_LARSON_SCANNER] = 'O',
    [LED_OVERLAY_BLINK] = 'B',
    [LED_OVERLAY_VTX] = 'V',
    [LED_OVERLAY_INDICATOR] = 'I',
    [LED_OVERLAY_WARNING] = 'W'
};

#define CHUNK_BUFFER_SIZE 11
bool parseLedStripConfig(int ledIndex, const char *config)
{
    if (ledIndex >= LED_STRIP_MAX_LENGTH)
        return false;

    enum parseState_e {
        X_COORDINATE,
        Y_COORDINATE,
        DIRECTIONS,
        FUNCTIONS,
        RING_COLORS,
        PARSE_STATE_COUNT
    };
    static const char chunkSeparators[PARSE_STATE_COUNT] = {',', ':', ':', ':', '\0'};

    ledConfig_t *ledConfig = &ledStripActiveProfileConfigMutable()->ledConfigs[ledIndex];
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
                x = atoi(chunk);
                break;
            case Y_COORDINATE:
                y = atoi(chunk);
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
                color = atoi(chunk);
                if (color >= LED_CONFIGURABLE_COLOR_COUNT)
                    color = 0;
                break;
            case PARSE_STATE_COUNT:; // prevent warning
        }
    }

    *ledConfig = DEFINE_LED(x, y, color, direction_flags, baseFunction, overlay_flags);

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
    tfp_sprintf(ledConfigBuffer, "%u,%u:%s:%s:%u", ledGetX(ledConfig), ledGetY(ledConfig), directions, baseFunctionOverlays, ledGetColor(ledConfig));
}

typedef enum {
    // the ordering is important, see below how NSEW is mapped to  NE/SE/NW/SW
    QUADRANT_NORTH      = 1 << 0,
    QUADRANT_SOUTH      = 1 << 1,
    QUADRANT_EAST       = 1 << 2,
    QUADRANT_WEST       = 1 << 3,
} quadrant_e;

static quadrant_e getLedQuadrant(const int ledIndex)
{
    const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];

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

    return quad;
}

static const hsvColor_t* getDirectionalModeColor(const int ledIndex, const modeColorIndexes_t *modeColors)
{
    const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];
    const int ledDirection = ledGetDirection(ledConfig);

    for (unsigned i = 0; i < LED_DIRECTION_COUNT; i++) {
        if (ledDirection & (1 << i)) {
            return &ledStripActiveProfileConfig()->colors[modeColors->color[i]];
        }
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
#ifdef USE_MAG
    {MAG_MODE,      LED_MODE_MAG},
#endif
    {HORIZON_MODE,  LED_MODE_HORIZON},
    {ANGLE_MODE,    LED_MODE_ANGLE},
    {0,             LED_MODE_ORIENTATION},
};

static void applyLedFixedLayers(void)
{
    uint8_t ledBarCounters[LED_BAR_COUNT] = {0};

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];
        hsvColor_t color = *getSC(LED_SCOLOR_BACKGROUND);

        int fn = ledGetFunction(ledConfig);
        int hOffset = HSV_HUE_MAX + 1;

        switch (fn) {
        case LED_FUNCTION_COLOR:
            color = ledStripActiveProfileConfig()->colors[ledGetColor(ledConfig)];

            hsvColor_t nextColor = ledStripActiveProfileConfig()->colors[(ledGetColor(ledConfig) + 1 + LED_CONFIGURABLE_COLOR_COUNT) % LED_CONFIGURABLE_COLOR_COUNT];
            hsvColor_t previousColor = ledStripActiveProfileConfig()->colors[(ledGetColor(ledConfig) - 1 + LED_CONFIGURABLE_COLOR_COUNT) % LED_CONFIGURABLE_COLOR_COUNT];

            if (ledGetOverlayBit(ledConfig, LED_OVERLAY_THROTTLE)) {   //smooth fade with selected Aux channel of all HSV values from previousColor through color to nextColor
                const int auxInput = rcData[ledStripActiveProfileConfig()->ledstrip_aux_channel];
                int centerPWM = (PWM_RANGE_MIN + PWM_RANGE_MAX) / 2;
                if (auxInput < centerPWM) {
                    color.h = scaleRange(auxInput, PWM_RANGE_MIN, centerPWM, previousColor.h, color.h);
                    color.s = scaleRange(auxInput, PWM_RANGE_MIN, centerPWM, previousColor.s, color.s);
                    color.v = scaleRange(auxInput, PWM_RANGE_MIN, centerPWM, previousColor.v, color.v);
                } else {
                    color.h = scaleRange(auxInput, centerPWM, PWM_RANGE_MAX, color.h, nextColor.h);
                    color.s = scaleRange(auxInput, centerPWM, PWM_RANGE_MAX, color.s, nextColor.s);
                    color.v = scaleRange(auxInput, centerPWM, PWM_RANGE_MAX, color.v, nextColor.v);
                }
            }

            break;

        case LED_FUNCTION_FLIGHT_MODE:
            for (unsigned i = 0; i < ARRAYLEN(flightModeToLed); i++)
                if (!flightModeToLed[i].flightMode || FLIGHT_MODE(flightModeToLed[i].flightMode)) {
                    const hsvColor_t *directionalColor = getDirectionalModeColor(ledIndex, &ledStripActiveProfileConfig()->modeColors[flightModeToLed[i].ledMode]);
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
        case LED_FUNCTION_BATTERY_BAR:
            color = HSV(RED);
            hOffset += MAX(scaleRange(calculateBatteryPercentageRemaining(), 0, 100, -30, 120), 0);
            break;

#ifdef USE_GPS
        case LED_FUNCTION_GPS_BAR:
            {
                uint8_t minSats = 8;
#ifdef USE_GPS_RESCUE
                minSats = gpsRescueConfig()->minSats;
#endif
                if (gpsSol.numSat == 0 || !sensors(SENSOR_GPS)) {
                    color = HSV(RED);
                } else {
                    if (gpsSol.numSat >= minSats) {
                        color = HSV(GREEN);
                    } else {
                        color = HSV(RED);
                        hOffset += MAX(scaleRange(gpsSol.numSat, 0, minSats, -30, 120), 0);
                    }
                }
                break;
            }
#endif

#if defined(USE_BARO) || defined(USE_GPS)
        case LED_FUNCTION_ALTITUDE:
            color = ledStripActiveProfileConfig()->colors[ledGetColor(ledConfig)];
            hOffset += MAX(scaleRange(getEstimatedAltitudeCm(), 0, 500, -30, 120), 0);
            break;
#endif

        case LED_FUNCTION_RSSI:
            color = HSV(RED);
            hOffset += MAX(scaleRange(getRssiPercent(), 0, 100, -30, 120), 0);
            break;

        default:
            break;
        }

        if ((fn != LED_FUNCTION_COLOR) && ledGetOverlayBit(ledConfig, LED_OVERLAY_THROTTLE)) {
            const int auxInput = rcData[ledStripActiveProfileConfig()->ledstrip_aux_channel];
            hOffset += scaleRange(auxInput, PWM_RANGE_MIN, PWM_RANGE_MAX, 0, HSV_HUE_MAX + 1);
        }
        color.h = (color.h + hOffset) % (HSV_HUE_MAX + 1);

        switch (fn) {
#ifdef USE_GPS
            case LED_FUNCTION_GPS_BAR:
                if (ledBarCounters[LED_BAR_GPS] < gpsSol.numSat || ledBarCounters[LED_BAR_GPS] == 0) {
                    ledBarCounters[LED_BAR_GPS]++;
                    setLedHsv(ledIndex, &color);
                } else {
                    setLedHsv(ledIndex, getSC(LED_SCOLOR_BACKGROUND));
                }
                break;
#endif
            case LED_FUNCTION_BATTERY_BAR:
                if (ledBarCounters[LED_BAR_BATTERY] < (calculateBatteryPercentageRemaining() * ledBarStates[LED_BAR_BATTERY]) / 100 || ledBarCounters[LED_BAR_BATTERY] == 0) {
                    ledBarCounters[LED_BAR_BATTERY]++;
                    setLedHsv(ledIndex, &color);
                } else {
                    setLedHsv(ledIndex, getSC(LED_SCOLOR_BACKGROUND));
                }
                break;

            default:
                setLedHsv(ledIndex, &color);
                break;
        }
    }
}

static void applyLedHsv(uint32_t mask, const hsvColor_t *color)
{
    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];
        if ((*ledConfig & mask) == mask)
            setLedHsv(ledIndex, color);
    }
}

typedef enum {
    WARNING_ARMING_DISABLED,
    WARNING_LOW_BATTERY,
    WARNING_FAILSAFE,
    WARNING_CRASHFLIP_ACTIVE,
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
            if (batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE && getBatteryState() != BATTERY_OK) {
                warningFlags |= 1 << WARNING_LOW_BATTERY;
            }
            if (failsafeIsActive()) {
                warningFlags |= 1 << WARNING_FAILSAFE;
            }
            if (!ARMING_FLAG(ARMED) && isArmingDisabled()) {
                warningFlags |= 1 << WARNING_ARMING_DISABLED;
            }
            if (isCrashFlipModeActive()) {
                warningFlags |= 1 << WARNING_CRASHFLIP_ACTIVE;
            }
        }
        *timer += HZ_TO_US(LED_OVERLAY_WARNING_RATE_HZ);
    }

    const hsvColor_t *warningColor = NULL;

    if (warningFlags) {
        bool colorOn = (warningFlashCounter % 2) == 0;   // w_w_
        warningFlags_e warningId = warningFlashCounter / 4;
        if (warningFlags & (1 << warningId)) {
            switch (warningId) {
                case WARNING_ARMING_DISABLED:
                    warningColor = colorOn ? &HSV(GREEN) : &HSV(BLACK);
                    break;
                case WARNING_CRASHFLIP_ACTIVE:
                    warningColor = colorOn ? &HSV(MAGENTA) : &HSV(BLACK);
                    break;
                case WARNING_LOW_BATTERY:
                    warningColor = colorOn ? &HSV(RED) : &HSV(BLACK);
                    break;
                case WARNING_FAILSAFE:
                    warningColor = colorOn ? &HSV(YELLOW) : &HSV(BLUE);
                    break;
                default:;
            }
        }
    } else {
        if (isBeeperOn()) {
            warningColor = &hsv[ledStripConfig()->ledstrip_visual_beeper_color];
        }
    }

    if (warningColor) {
        applyLedHsv(LED_MOV_OVERLAY(LED_FLAG_OVERLAY(LED_OVERLAY_WARNING)), warningColor);
    }
}

#ifdef USE_VTX_COMMON


static void applyLedVtxLayer(bool updateNow, timeUs_t *timer)
{
    static uint16_t frequency = 0;
    static uint8_t power = 255;
    static unsigned vtxStatus = UINT32_MAX;
    static uint8_t showSettings = false;
    static uint16_t lastCheck = 0;
    static bool blink = false;

    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (!vtxDevice) {
        return;
    }

    uint8_t band = 255, channel = 255;

    if (updateNow) {
        // keep counter running, so it stays in sync with vtx
        vtxCommonGetBandAndChannel(vtxDevice, &band, &channel);
        vtxCommonGetPowerIndex(vtxDevice, &power);
        vtxCommonGetStatus(vtxDevice, &vtxStatus);

        frequency = vtxCommonLookupFrequency(vtxDevice, band, channel);

        // check if last vtx values have changed.
        uint16_t check = ((vtxStatus & VTX_STATUS_PIT_MODE) ? 1 : 0) + (power << 1) + (band << 4) + (channel << 8);
        if (!showSettings && check != lastCheck) {
            // display settings for 3 seconds.
            showSettings = 15;
        }
        lastCheck = check; // quick way to check if any settings changed.

        if (showSettings) {
            showSettings--;
        }
        blink = !blink;
        *timer += HZ_TO_US(LED_OVERLAY_VTX_RATE_HZ);
    }

    if (showSettings) { // show settings
        uint8_t vtxLedCount = 0;
        for (int i = 0; i < ledCounts.count && vtxLedCount < 6; ++i) {
            const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[i];
            if (ledGetOverlayBit(ledConfig, LED_OVERLAY_VTX)) {
                hsvColor_t color = {0, 0, 0};
                if (vtxLedCount == 0) {
                    color.h = HSV(GREEN).h;
                    color.s = HSV(GREEN).s;
                    color.v = blink ? 15 : 0; // blink received settings
                } else if (vtxLedCount > 0 && power >= vtxLedCount && !(vtxStatus & VTX_STATUS_PIT_MODE)) { // show power
                    color.h = HSV(ORANGE).h;
                    color.s = HSV(ORANGE).s;
                    color.v = blink ? 15 : 0; // blink received settings
                } else { // turn rest off
                    color.h = HSV(BLACK).h;
                    color.s = HSV(BLACK).s;
                    color.v = HSV(BLACK).v;
                }
                setLedHsv(i, &color);
                ++vtxLedCount;
            }
        }
    }
    else { // calculate the VTX color based on frequency

        hsvColor_t color = getHsvFromVtxFrequency(frequency);
        if (vtxStatus & VTX_STATUS_PIT_MODE) {
            color.v = blink ? color.v : 15;
            // if getHsvFromVtxFrequency returns a color, blink down to a pale color when in pit mode, but
            // if black is returned,  blink up from off (0,0,0) to pale red (0,0,15)
        }
        applyLedHsv(LED_MOV_OVERLAY(LED_FLAG_OVERLAY(LED_OVERLAY_VTX)), &color);
    }
}
#endif

static void applyLedBatteryLayer(bool updateNow, timeUs_t *timer)
{
    static bool flash = false;

    int timerDelayUs = HZ_TO_US(1);

    if (updateNow) {
        switch (getBatteryState()) {
            case BATTERY_OK:
                flash = true;
                timerDelayUs = HZ_TO_US(1);

                break;
            case BATTERY_WARNING:
                flash = !flash;
                timerDelayUs = HZ_TO_US(2);

                break;
            default:
                flash = !flash;
                timerDelayUs = HZ_TO_US(8);

                break;
        }

        *timer += timerDelayUs;
    }

    if (!flash) {
       const hsvColor_t *bgc = getSC(LED_SCOLOR_BACKGROUND);
       applyLedHsv(LED_MOV_FUNCTION(LED_FUNCTION_BATTERY), bgc);
    }
}

static void applyLedRssiLayer(bool updateNow, timeUs_t *timer)
{
    static bool flash = false;

    int timerDelay = HZ_TO_US(1);

    if (updateNow) {
        int state = getRssiPercent();

        if (state > 50) {
            flash = true;
            timerDelay = HZ_TO_US(1);
        } else if (state > 20) {
            flash = !flash;
            timerDelay = HZ_TO_US(2);
        } else {
            flash = !flash;
            timerDelay = HZ_TO_US(8);
        }

        *timer += timerDelay;
    }

    if (!flash) {
        const hsvColor_t *bgc = getSC(LED_SCOLOR_BACKGROUND);
        applyLedHsv(LED_MOV_FUNCTION(LED_FUNCTION_RSSI), bgc);
    }
}

#ifdef USE_GPS
static void applyLedGpsLayer(bool updateNow, timeUs_t *timer)
{

    static uint8_t gpsPauseCounter = 0;
    const uint8_t blinkPauseLength = 4;

    if (updateNow) {
        static uint8_t gpsFlashCounter = 0;
        if (gpsPauseCounter > 0) {
            gpsPauseCounter--;
        } else if (gpsFlashCounter >= gpsSol.numSat) {
            gpsFlashCounter = 0;
            gpsPauseCounter = blinkPauseLength;
        } else {
            gpsFlashCounter++;
            gpsPauseCounter = 1;
        }
        *timer += HZ_TO_US(2.5f);
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
        if (isRxReceivingSignal()) {
            // calculate update frequency
            int scale = MAX(fabsf(rcCommand[ROLL]), fabsf(rcCommand[PITCH]));  // 0 - 500
            scale = scale - INDICATOR_DEADBAND;  // start increasing frequency right after deadband
            *timer += HZ_TO_US(5 + (45 * scale) / (500 - INDICATOR_DEADBAND));   // 5 - 50Hz update, 2.5 - 25Hz blink

            flash = !flash;
        } else {
            *timer += HZ_TO_US(LED_OVERLAY_INDICATOR_RATE_HZ);
        }
    }

    if (!flash)
        return;

    const hsvColor_t *flashColor = &HSV(ORANGE); // TODO - use user color?

    quadrant_e quadrants = 0;
    if (rcCommand[ROLL] > INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_EAST;
    } else if (rcCommand[ROLL] < -INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_WEST;
    }
    if (rcCommand[PITCH] > INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_NORTH;
    } else if (rcCommand[PITCH] < -INDICATOR_DEADBAND) {
        quadrants |= QUADRANT_SOUTH;
    }

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];
        if (ledGetOverlayBit(ledConfig, LED_OVERLAY_INDICATOR)) {
            if (getLedQuadrant(ledIndex) & quadrants)
                setLedHsv(ledIndex, flashColor);
        }
    }
}

static void applyLedThrustRingLayer(bool updateNow, timeUs_t *timer)
{
    static uint8_t rotationPhase;
    int ledRingIndex = 0;

    if (updateNow) {
        rotationPhase = rotationPhase > 0 ? rotationPhase - 1 : ledCounts.ringSeqLen - 1;

        const int scaledThrottle = ARMING_FLAG(ARMED) ? scaleRange(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100) : 0;
        *timer += HZ_TO_US(5 + (45 * scaledThrottle) / 100);  // 5 - 50Hz update rate
    }

    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_THRUST_RING) {

            bool applyColor;
            if (ARMING_FLAG(ARMED)) {
                applyColor = (ledRingIndex + rotationPhase) % ledCounts.ringSeqLen < ROTATION_SEQUENCE_LED_WIDTH;
            } else {
                applyColor = !(ledRingIndex % 2); // alternating pattern
            }

            if (applyColor) {
                const hsvColor_t *ringColor = &ledStripActiveProfileConfig()->colors[ledGetColor(ledConfig)];
                setLedHsv(ledIndex, ringColor);
            }

            ledRingIndex++;
        }
    }
}

static void applyRainbowLayer(bool updateNow, timeUs_t *timer)
{
    //use offset as a fixed point number
    static int offset = 0;

    if (updateNow) {
        offset += ledStripProfileGetRainbowFreq(ledStripConfig()->ledstrip_profile);
        *timer += HZ_TO_US(LED_OVERLAY_RAINBOW_RATE_HZ);
    }
    uint8_t rainbowLedIndex = 0;
    const uint16_t rainbowDelta = ledStripProfileGetRainbowDelta(ledStripConfig()->ledstrip_profile);

    for (unsigned i = 0; i < ledCounts.count; i++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[i];
        if (ledGetOverlayBit(ledConfig, LED_OVERLAY_RAINBOW)) {
            hsvColor_t ledColor;
            ledColor.h = (offset / LED_OVERLAY_RAINBOW_RATE_HZ + rainbowLedIndex * rainbowDelta) % (HSV_HUE_MAX + 1);
            ledColor.s = 0;
            ledColor.v = HSV_VALUE_MAX;
            setLedHsv(i, &ledColor);
            rainbowLedIndex++;
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

    if (abs(offset) > 1)
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
        const uint16_t larsonFreq = ledStripProfileGetLarsonFreq(ledStripConfig()->ledstrip_profile);
        larsonScannerNextStep(&larsonParameters, larsonFreq);
        *timer += HZ_TO_US(LED_OVERLAY_LARSON_RATE_HZ);
    }

    int scannerLedIndex = 0;
    for (unsigned i = 0; i < ledCounts.count; i++) {

        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[i];

        if (ledGetOverlayBit(ledConfig, LED_OVERLAY_LARSON_SCANNER)) {
            hsvColor_t ledColor;
            getLedHsv(i, &ledColor);
            ledColor.v = brightnessForLarsonIndex(&larsonParameters, scannerLedIndex);
            setLedHsv(i, &ledColor);
            scannerLedIndex++;
        }
    }
}

// blink overlay: configurable period, duty, and pattern per profile
static bool ledBlinkLayerIsLedOn(uint8_t profile)
{
    const uint8_t pattern = ledStripProfileGetBlinkPattern(profile);

    if (pattern == LED_BLINK_PATTERN_BEACON) {
        const uint16_t flashMs = ledStripProfileGetBlinkFlashMs(profile);
        const uint16_t gapMs = ledStripProfileGetBlinkGapMs(profile);
        const uint16_t pauseMs = ledStripProfileGetBlinkPauseMs(profile);
        const uint32_t cycleMs = (uint32_t)flashMs + gapMs + flashMs + pauseMs;

        if (cycleMs < 50) {
            return true;
        }

        const uint32_t phase = millis() % cycleMs;
        const uint32_t flashEnd = flashMs;
        const uint32_t gapEnd = flashEnd + gapMs;
        const uint32_t secondFlashEnd = gapEnd + flashMs;

        if (phase < flashEnd) {
            return true;
        }

        if (phase < gapEnd) {
            return false;
        }

        if (phase < secondFlashEnd) {
            return true;
        }

        return false;
    }

    if (pattern == LED_BLINK_PATTERN_ALTERNATE) {
        const uint16_t flashMs = ledStripProfileGetBlinkFlashMs(profile);
        const uint16_t pauseMs = ledStripProfileGetBlinkPauseMs(profile);
        const uint32_t cycleMs = (uint32_t)flashMs + pauseMs;

        if (cycleMs < 50) {
            return true;
        }

        return (millis() % cycleMs) < flashMs;
    }

    return true;
}

static void applyLedBlinkLayer(bool updateNow, timeUs_t *timer)
{
    if (updateNow) {
        const uint8_t profile = ledStripConfig()->ledstrip_profile;
        const uint8_t pattern = ledStripProfileGetBlinkPattern(profile);
        uint16_t periodMs;

        if (pattern == LED_BLINK_PATTERN_BEACON) {
            periodMs = ledStripProfileGetBlinkFlashMs(profile)
                + ledStripProfileGetBlinkGapMs(profile)
                + ledStripProfileGetBlinkFlashMs(profile)
                + ledStripProfileGetBlinkPauseMs(profile);
        } else if (pattern == LED_BLINK_PATTERN_ALTERNATE) {
            periodMs = ledStripProfileGetBlinkFlashMs(profile)
                + ledStripProfileGetBlinkPauseMs(profile);
        } else {
            periodMs = 500;
        }

        uint16_t rateHz = LED_OVERLAY_BLINK_RATE_HZ;

        if (periodMs >= 50) {
            const uint16_t periodRateHz = 4000 / periodMs;

            if (periodRateHz > rateHz) {
                rateHz = periodRateHz;
            }
        }

        if (pattern == LED_BLINK_PATTERN_BEACON || pattern == LED_BLINK_PATTERN_ALTERNATE) {
            const uint16_t flashMs = ledStripProfileGetBlinkFlashMs(profile);

            if (flashMs >= 10) {
                const uint16_t flashRateHz = 4000 / flashMs;

                if (flashRateHz > rateHz) {
                    rateHz = flashRateHz;
                }
            }
        }

        *timer += HZ_TO_US(rateHz);
    }

    if (!ledBlinkLayerIsLedOn(ledStripConfig()->ledstrip_profile)) {
        for (int i = 0; i < ledCounts.count; ++i) {
            const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[i];

            if (ledGetOverlayBit(ledConfig, LED_OVERLAY_BLINK)) {
                setLedHsv(i, getSC(LED_SCOLOR_BLINKBACKGROUND));
            }
        }
    }
}

// In reverse order of priority
typedef enum {
    timRainbow,
    timBlink,
    timLarson,
    timRing,
    timIndicator,
#ifdef USE_VTX_COMMON
    timVtx,
#endif
#ifdef USE_GPS
    timGps,
#endif
    timBattery,
    timRssi,
    timWarning,
    timTimerCount
} timId_e;

static timeUs_t timerVal[timTimerCount];
static uint16_t disabledTimerMask;

#if defined(USE_LED_STRIP_STATUS_MODE)
static struct {
    timId_e timId;
    uint32_t timActive;
    bool fixedLayersApplied;
    bool applyProfile;
    bool multipassProfile;
    bool multipassUpdate;
    timeUs_t updateStartTimeUs;
    uint16_t ledStateDurationFractionUs[2];
} ledStripRenderState = {
    .applyProfile = true,
};

void resetLedStripProfileRenderState(void)
{
    ledStripRenderState.timId = 0;
    ledStripRenderState.timActive = 0;
    ledStripRenderState.fixedLayersApplied = false;
    ledStripRenderState.applyProfile = true;
    ledStripRenderState.multipassProfile = false;
    ledStripRenderState.multipassUpdate = false;
    ledStripRenderState.updateStartTimeUs = 0;
    memset(timerVal, 0, sizeof(timerVal));
}
#endif

STATIC_ASSERT(timTimerCount <= sizeof(disabledTimerMask) * 8, disabledTimerMask_too_small);

// function to apply layer.
// function must replan self using timer pointer
// when updateNow is true (timer triggered), state must be updated first,
//  before calculating led state. Otherwise update started by different trigger
//  may modify LED state.
typedef void applyLayerFn_timed(bool updateNow, timeUs_t *timer);

static applyLayerFn_timed* layerTable[] = {
    [timRainbow] = &applyRainbowLayer,
    [timBlink] = &applyLedBlinkLayer,
    [timLarson] = &applyLarsonScannerLayer,
    [timBattery] = &applyLedBatteryLayer,
    [timRssi] = &applyLedRssiLayer,
#ifdef USE_GPS
    [timGps] = &applyLedGpsLayer,
#endif
    [timWarning] = &applyLedWarningLayer,
#ifdef USE_VTX_COMMON
    [timVtx] = &applyLedVtxLayer,
#endif
    [timIndicator] = &applyLedIndicatorLayer,
    [timRing] = &applyLedThrustRingLayer
};

static bool isOverlayTypeUsed(ledOverlayId_e overlayType)
{
    for (int ledIndex = 0; ledIndex < ledCounts.count; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripActiveProfileConfig()->ledConfigs[ledIndex];
        if (ledGetOverlayBit(ledConfig, overlayType)) {
            return true;
        }
    }
    return false;
}

void updateRequiredOverlay(void)
{
    disabledTimerMask = 0;
    disabledTimerMask |= !isOverlayTypeUsed(LED_OVERLAY_RAINBOW) << timRainbow;
    disabledTimerMask |= !isOverlayTypeUsed(LED_OVERLAY_BLINK) << timBlink;
    disabledTimerMask |= !isOverlayTypeUsed(LED_OVERLAY_LARSON_SCANNER) << timLarson;
    disabledTimerMask |= !isOverlayTypeUsed(LED_OVERLAY_WARNING) << timWarning;
#ifdef USE_VTX_COMMON
    disabledTimerMask |= !isOverlayTypeUsed(LED_OVERLAY_VTX) << timVtx;
#endif
    disabledTimerMask |= !isOverlayTypeUsed(LED_OVERLAY_INDICATOR) << timIndicator;
}

bool ledStripProfileUsesSimpleRenderer(const ledStripStatusModeConfig_t *profile)
{
    const ledConfig_t reference = profile->ledConfigs[0];

    if (ledGetFunction(&reference) != LED_FUNCTION_COLOR) {
        return false;
    }

    if (ledGetDirection(&reference) != 0) {
        return false;
    }

    for (int ledIndex = 1; ledIndex < LED_STRIP_MAX_LENGTH; ledIndex++) {
        if (profile->ledConfigs[ledIndex] != reference) {
            return false;
        }
    }

    return true;
}

void syncSimpleLedProfilesFromConfig(void)
{
    ledStripStatusModeConfig_t *raceProfile = ledStripProfileConfigMutable(LED_PROFILE_RACE);
    if (ledStripProfileUsesSimpleRenderer(raceProfile)) {
        initSolidColorProfile(raceProfile, ledStripConfig()->ledstrip_race_color, false);
    }

    ledStripStatusModeConfig_t *beaconProfile = ledStripProfileConfigMutable(LED_PROFILE_BEACON);
    if (ledStripProfileUsesSimpleRenderer(beaconProfile)) {
        initSolidColorProfile(beaconProfile, ledStripConfig()->ledstrip_beacon_color, true);
    }
}

static ledProfileSequence_t applySimpleProfile(timeUs_t currentTimeUs)
{
    static timeUs_t colorUpdateTimeUs = 0;
    static hsvColor_t previousHsv = { 0, 0, 0 };

    hsvColor_t currentHsv = hsv[COLOR_BLACK];
    const ledStripStatusModeConfig_t *profile = ledStripActiveProfileConfig();
    const ledConfig_t *ledConfig = &profile->ledConfigs[0];
    const uint8_t profileColorIndex = ledGetColor(ledConfig);
    const hsvColor_t visualBeeperHsv = hsv[ledStripConfig()->ledstrip_visual_beeper_color];

    unsigned flashPeriod = 0;
    unsigned onPercent = 0;

    switch (ledStripConfig()->ledstrip_profile) {
    case LED_PROFILE_RACE:
        if (profileColorIndex == COLOR_BLACK) {
#ifdef USE_VTX_COMMON
            const vtxDevice_t *vtxDevice = vtxCommonDevice();
            if (vtxDevice) {
                uint8_t const band = vtxSettingsConfig()->band;
                uint8_t const channel = vtxSettingsConfig()->channel;
                uint16_t vtxFrequency = VTX_SETTINGS_MIN_FREQUENCY_MHZ;

                if (band && channel) {
                    vtxFrequency = vtxCommonLookupFrequency(vtxDevice, band, channel);
                } else {
                    vtxFrequency = vtxSettingsConfig()->freq;
                }

                currentHsv = getHsvFromVtxFrequency(vtxFrequency);
            }
#endif
        } else {
            currentHsv = profile->colors[profileColorIndex];
        }

        if (IS_RC_MODE_ACTIVE(BOXBEEPERON) || failsafeIsActive()) {
            flashPeriod = BEACON_FAILSAFE_PERIOD_MS;
            onPercent = BEACON_FAILSAFE_ON_PERCENT;

            const unsigned onPeriod = flashPeriod * onPercent / 100;

            if (onPeriod > 0 && (millis() % flashPeriod) < onPeriod) {
                currentHsv = visualBeeperHsv;
            }
        } else {
            if (ledStripConfig()->ledstrip_visual_beeper && isBeeperOn()) {
                currentHsv = visualBeeperHsv;
            }
        }
        break;

    case LED_PROFILE_BEACON:
        flashPeriod = ledStripConfig()->ledstrip_beacon_period_ms;
        onPercent = ledStripConfig()->ledstrip_beacon_percent;

        currentHsv = profile->colors[profileColorIndex];

        if ((millis() % flashPeriod) < (flashPeriod * onPercent / 100)) {
            // LED stays on during the on-period; currentHsv already set
        } else {
            currentHsv = hsv[COLOR_BLACK];
        }
        break;

    default:
        break;
    }

    const bool updateLedStripColor =
        currentHsv.h != previousHsv.h ||
        currentHsv.s != previousHsv.s ||
        currentHsv.v != previousHsv.v ||
        (cmpTimeUs(currentTimeUs, colorUpdateTimeUs) >= 0);

    if (updateLedStripColor) {
        setStripColor(&currentHsv);

        previousHsv = currentHsv;
        colorUpdateTimeUs = currentTimeUs + PROFILE_COLOR_UPDATE_INTERVAL_US;

        return LED_PROFILE_ADVANCE;
    }

    return LED_PROFILE_SLOW;
}

static ledProfileSequence_t applyStatusProfile(timeUs_t now)
{
    timeUs_t startTime = micros();

    if (!ledStripRenderState.timActive) {
        // apply all layers; triggered timed functions has to update timers
        // test all led timers, setting corresponding bits
        for (timId_e timId = 0; timId < timTimerCount; timId++) {
            if (!(disabledTimerMask & (1 << timId))) {
                // sanitize timer value, so that it can be safely incremented. Handles inital timerVal value.
                const timeDelta_t delta = cmpTimeUs(now, timerVal[timId]);
                // max delay is limited to 5s
                if (delta > MAX_TIMER_DELAY) {
                    // Restart the interval on this timer; catches start condition following initialisation
                    timerVal[timId] = now;
                }

                if (delta >= 0) {
                    ledStripRenderState.timActive |= 1 << timId;
                }
            }
        }

        if (!ledStripRenderState.timActive) {
            return LED_PROFILE_SLOW;          // no change this update, keep old state
        }
    }

    if (!ledStripRenderState.fixedLayersApplied) {
        applyLedFixedLayers();
        ledStripRenderState.fixedLayersApplied = true;
    }

    for (; ledStripRenderState.timId < ARRAYLEN(layerTable); ledStripRenderState.timId++) {
        timeUs_t *timer = &timerVal[ledStripRenderState.timId];
        bool updateNow = ledStripRenderState.timActive & (1 << ledStripRenderState.timId);
        (*layerTable[ledStripRenderState.timId])(updateNow, timer);
        if (cmpTimeUs(micros(), startTime) > LED_TARGET_UPDATE_US) {
            // Come back and complete this quickly
            ledStripRenderState.timId++;
            return LED_PROFILE_FAST;
        }
    }

    // Reset state for next iteration
    ledStripRenderState.timActive = 0;
    ledStripRenderState.fixedLayersApplied = false;
    ledStripRenderState.timId = 0;

    return LED_PROFILE_ADVANCE;
}

bool parseColor(int index, const char *colorConfig)
{
    const char *remainingCharacters = colorConfig;

    hsvColor_t *color = &ledStripActiveProfileConfigMutable()->colors[index];

    bool result = true;
    static const uint16_t hsv_limit[HSV_COLOR_COMPONENT_COUNT] = {
        [HSV_HUE] = HSV_HUE_MAX,
        [HSV_SATURATION] = HSV_SATURATION_MAX,
        [HSV_VALUE] = HSV_VALUE_MAX,
    };
    for (int componentIndex = 0; result && componentIndex < HSV_COLOR_COMPONENT_COUNT; componentIndex++) {
        int val = atoi(remainingCharacters);
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
        ledStripActiveProfileConfigMutable()->modeColors[modeIndex].color[modeColorIndex] = colorIndex;
    } else if (modeIndex == LED_SPECIAL) {
        if (modeColorIndex < 0 || modeColorIndex >= LED_SPECIAL_COLOR_COUNT)
            return false;
        ledStripActiveProfileConfigMutable()->specialColors.color[modeColorIndex] = colorIndex;
    } else if (modeIndex == LED_AUX_CHANNEL) {
        if (modeColorIndex < 0 || modeColorIndex >= 1)
            return false;
        ledStripActiveProfileConfigMutable()->ledstrip_aux_channel = colorIndex;
    } else {
        return false;
    }
    return true;
}

void ledStripEnable(void)
{
    ws2811LedStripEnable();

    ledStripEnabled = true;
}

void ledStripDisable(void)
{
    if (ledStripEnabled) {
        ledStripEnabled = false;
        setStripColor(&HSV(BLACK));

        // Multiple calls may be required as normally broken into multiple parts
        while (!ws2811UpdateStrip(ledStripGetActiveBrightness()));
    }
}

void ledStripInit(void)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    syncActiveLedProfileConfig();
#endif

    ws2811LedStripInit(ledStripConfig()->ioTag, (ledStripFormatRGB_e)ledStripConfig()->ledstrip_grb_rgb);
}


timeUs_t executeTimeUs;
void ledStripUpdate(timeUs_t currentTimeUs) {
    bool ledCurrentState = ledStripRenderState.applyProfile;

    if (ledStripRenderState.updateStartTimeUs != 0) {
        // The LED task rate is considered to be the rate at which updates are sent to the LEDs as a consequence
        // of the layer timers firing
        schedulerIgnoreTaskExecRate();
    }

    if (!isWS2811LedStripReady()) {
        // Call schedulerIgnoreTaskExecTime() unless data is being processed
        schedulerIgnoreTaskExecTime();
        return;
    }

    if (ledStripEnabled && IS_RC_MODE_ACTIVE(BOXLEDLOW)) {
        ledStripDisable();
    } else if (!IS_RC_MODE_ACTIVE(BOXLEDLOW)) {
        ledStripEnable();
    }

    if (ledStripEnabled) {
        if (ledStripRenderState.applyProfile) {
            ledProfileSequence_t ledProfileSequence = LED_PROFILE_SLOW;

            if (ledStripRenderState.updateStartTimeUs == 0) {
                ledStripRenderState.updateStartTimeUs = currentTimeUs;
            }

            switch (ledStripConfig()->ledstrip_profile) {
            case LED_PROFILE_STATUS:
                ledProfileSequence = applyStatusProfile(currentTimeUs);
                break;
            case LED_PROFILE_RACE:
            case LED_PROFILE_BEACON:
                if (ledStripProfileUsesSimpleRenderer(ledStripActiveProfileConfig())) {
                    ledProfileSequence = applySimpleProfile(currentTimeUs);
                } else {
                    ledProfileSequence = applyStatusProfile(currentTimeUs);
                }
                break;
            default:
                break;
            }

            if (ledProfileSequence == LED_PROFILE_SLOW) {
                // No timer was ready so no work was done
                schedulerIgnoreTaskExecTime();
                // Reschedule waiting for a timer to trigger a LED state change
                rescheduleTask(TASK_SELF, TASK_PERIOD_HZ(TASK_LEDSTRIP_RATE_WAIT_HZ));
            } else {
                if (ledProfileSequence == LED_PROFILE_ADVANCE) {
                    // The state leading to advancing from applying the profile layers to updating the DMA buffer is always short
                    if (ledStripRenderState.multipassProfile) {
                        schedulerIgnoreTaskExecTime();
                        ledStripRenderState.multipassProfile = false;
                    }
                    // The profile is now fully applied
                    ledStripRenderState.applyProfile = false;
                } else {
                    ledStripRenderState.multipassProfile = true;
                }
                // Reschedule for a fast period to update the DMA buffer
                rescheduleTask(TASK_SELF, TASK_PERIOD_HZ(TASK_LEDSTRIP_RATE_FAST_HZ));
            }
        } else {
            // Profile is applied, so now update the LEDs
            if (ws2811UpdateStrip(ledStripGetActiveBrightness())) {
                // Final pass updating the DMA buffer is always short
                if (ledStripRenderState.multipassUpdate) {
                    schedulerIgnoreTaskExecTime();
                    ledStripRenderState.multipassUpdate = false;
                }

                ledStripRenderState.applyProfile = true;

                timeDelta_t lastUpdateDurationUs = cmpTimeUs(currentTimeUs, ledStripRenderState.updateStartTimeUs);

                lastUpdateDurationUs %= TASK_PERIOD_HZ(TASK_LEDSTRIP_RATE_HZ);
                rescheduleTask(TASK_SELF, cmpTimeUs(TASK_PERIOD_HZ(TASK_LEDSTRIP_RATE_HZ), lastUpdateDurationUs));

                ledStripRenderState.updateStartTimeUs = 0;
            } else {
                ledStripRenderState.multipassUpdate = true;
            }
        }
    }

    if (!schedulerGetIgnoreTaskExecTime()) {
        executeTimeUs = micros() - currentTimeUs;
        if (executeTimeUs > (ledStripRenderState.ledStateDurationFractionUs[ledCurrentState] >> LED_EXEC_TIME_SHIFT)) {
            ledStripRenderState.ledStateDurationFractionUs[ledCurrentState] = executeTimeUs << LED_EXEC_TIME_SHIFT;
        } else if (ledStripRenderState.ledStateDurationFractionUs[ledCurrentState] > 0) {
            // Slowly decay the max time
            ledStripRenderState.ledStateDurationFractionUs[ledCurrentState]--;
        }
    }

    schedulerSetNextStateTime((ledStripRenderState.ledStateDurationFractionUs[ledStripRenderState.applyProfile] >> LED_EXEC_TIME_SHIFT) + LED_TASK_MARGIN);
}

uint8_t getLedProfile(void)
{
    return ledStripConfig()->ledstrip_profile;
}

void setLedProfile(uint8_t profile)
{
    if (profile >= LED_PROFILE_COUNT) {
        return;
    }

    if (profile != ledStripConfig()->ledstrip_profile) {
        ledStripConfigMutable()->ledstrip_profile = profile;
    }

#ifdef USE_LED_STRIP_STATUS_MODE
    syncActiveLedProfileConfig();
#endif
}

#endif // USE_LED_STRIP_STATUS_MODE

uint16_t ledStripProfileGetLarsonFreq(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileLarsonFreq = ledStripProfileConfig(profile)->profile_larson_freq;
    if (profileLarsonFreq > 0) {
        return profileLarsonFreq;
    }
#endif
    UNUSED(profile);

    const uint16_t masterLarsonFreq = ledStripConfig()->ledstrip_larson_freq;
    if (masterLarsonFreq > 0) {
        return masterLarsonFreq;
    }

    return LED_LARSON_FREQ_DEFAULT;
}

uint16_t ledStripProfileGetRainbowDelta(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileRainbowDelta = ledStripProfileConfig(profile)->profile_rainbow_delta;
    if (profileRainbowDelta > 0) {
        return profileRainbowDelta;
    }
#endif
    UNUSED(profile);
    return ledStripConfig()->ledstrip_rainbow_delta;
}

uint16_t ledStripProfileGetRainbowFreq(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileRainbowFreq = ledStripProfileConfig(profile)->profile_rainbow_freq;
    if (profileRainbowFreq > 0) {
        return profileRainbowFreq;
    }
#endif
    UNUSED(profile);

    const uint16_t masterRainbowFreq = ledStripConfig()->ledstrip_rainbow_freq;
    if (masterRainbowFreq > 0) {
        return masterRainbowFreq;
    }

    return 1;
}

uint16_t ledStripProfileGetBlinkPeriod(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileBlinkPeriod = ledStripProfileConfig(profile)->profile_blink_period;
    if (profileBlinkPeriod > 0) {
        return profileBlinkPeriod;
    }
#endif
    UNUSED(profile);

    const uint16_t masterBlinkPeriod = ledStripConfig()->ledstrip_blink_period_ms;
    if (masterBlinkPeriod >= 50) {
        return masterBlinkPeriod;
    }

    return LED_BLINK_PERIOD_MS_DEFAULT;
}

uint16_t ledStripProfileGetBlinkOnMs(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileBlinkOnMs = ledStripProfileConfig(profile)->profile_blink_on_ms;
    if (profileBlinkOnMs > 0) {
        return profileBlinkOnMs;
    }
#endif
    UNUSED(profile);

    const uint16_t masterBlinkOnMs = ledStripConfig()->ledstrip_blink_on_ms;
    if (masterBlinkOnMs > 0) {
        return masterBlinkOnMs;
    }

    return LED_BLINK_ON_MS_DEFAULT;
}

uint8_t ledStripProfileGetBlinkPattern(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint8_t profileBlinkPattern = ledStripProfileConfig(profile)->profile_blink_pattern;
    if (profileBlinkPattern >= LED_BLINK_PATTERN_ALTERNATE) {
        return migrateLedBlinkPattern(profileBlinkPattern);
    }
#endif
    UNUSED(profile);

    const uint8_t masterBlinkPattern = ledStripConfig()->ledstrip_blink_pattern;
    if (masterBlinkPattern >= LED_BLINK_PATTERN_ALTERNATE) {
        return migrateLedBlinkPattern(masterBlinkPattern);
    }

    return LED_BLINK_PATTERN_ALTERNATE;
}

uint16_t ledStripProfileGetBlinkFlashMs(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileBlinkFlashMs = ledStripProfileConfig(profile)->profile_blink_flash_ms;
    if (profileBlinkFlashMs >= LED_BLINK_FLASH_MS_MIN) {
        return profileBlinkFlashMs;
    }
#endif
    UNUSED(profile);

    const uint16_t masterBlinkFlashMs = ledStripConfig()->ledstrip_blink_flash_ms;
    if (masterBlinkFlashMs >= LED_BLINK_FLASH_MS_MIN) {
        return masterBlinkFlashMs;
    }

    return LED_BLINK_FLASH_MS_DEFAULT;
}

uint16_t ledStripProfileGetBlinkGapMs(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileBlinkGapMs = ledStripProfileConfig(profile)->profile_blink_gap_ms;
    if (profileBlinkGapMs >= 20) {
        return profileBlinkGapMs;
    }
#endif
    UNUSED(profile);

    const uint16_t masterBlinkGapMs = ledStripConfig()->ledstrip_blink_gap_ms;
    if (masterBlinkGapMs >= 20) {
        return masterBlinkGapMs;
    }

    return LED_BLINK_GAP_MS_DEFAULT;
}

uint16_t ledStripProfileGetBlinkPauseMs(uint8_t profile)
{
    uint16_t pauseMs = LED_BLINK_PAUSE_MS_DEFAULT;

#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint16_t profileBlinkPauseMs = ledStripProfileConfig(profile)->profile_blink_pause_ms;
    if (profileBlinkPauseMs >= LED_BLINK_PAUSE_MS_MIN_ALTERNATE) {
        pauseMs = profileBlinkPauseMs;
    } else {
        const uint16_t masterBlinkPauseMs = ledStripConfig()->ledstrip_blink_pause_ms;
        if (masterBlinkPauseMs >= LED_BLINK_PAUSE_MS_MIN_ALTERNATE) {
            pauseMs = masterBlinkPauseMs;
        }
    }
#else
    UNUSED(profile);

    const uint16_t masterBlinkPauseMs = ledStripConfig()->ledstrip_blink_pause_ms;
    if (masterBlinkPauseMs >= LED_BLINK_PAUSE_MS_MIN_ALTERNATE) {
        pauseMs = masterBlinkPauseMs;
    }
#endif

    const uint8_t pattern = ledStripProfileGetBlinkPattern(profile);
    const uint16_t minPauseMs = (pattern == LED_BLINK_PATTERN_ALTERNATE)
        ? LED_BLINK_PAUSE_MS_MIN_ALTERNATE
        : LED_BLINK_PAUSE_MS_MIN;

    if (pauseMs < minPauseMs) {
        pauseMs = minPauseMs;
    }

    return pauseMs;
}

uint8_t ledStripProfileGetBrightness(uint8_t profile)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        profile = LED_PROFILE_STATUS;
    }

    const uint8_t profileBrightness = ledStripProfileConfig(profile)->profile_brightness;
    if (profileBrightness >= 5) {
        return profileBrightness;
    }
#endif
    UNUSED(profile);
    return ledStripConfig()->ledstrip_brightness;
}

void ledStripProfileSetBrightness(uint8_t profile, uint8_t brightness)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    if (profile >= LED_PROFILE_COUNT) {
        return;
    }

    if (brightness >= 5 && brightness <= 100) {
        ledStripProfileConfigMutable(profile)->profile_brightness = brightness;
    }
#else
    UNUSED(profile);
    setLedBrightness(brightness);
#endif
}

uint8_t ledStripGetActiveBrightness(void)
{
#if defined(USE_LED_STRIP_STATUS_MODE)
    return ledStripProfileGetBrightness(ledStripConfig()->ledstrip_profile);
#else
    return ledStripConfig()->ledstrip_brightness;
#endif
}

uint8_t getLedBrightness(void)
{
    return ledStripGetActiveBrightness();
}

void setLedBrightness(uint8_t brightness)
{
    if (brightness < 5) {
        brightness = 5;
    }
    if (brightness > 100) {
        return;
    }

#if defined(USE_LED_STRIP_STATUS_MODE)
    ledStripProfileSetBrightness(ledStripConfig()->ledstrip_profile, brightness);
#else
    ledStripConfigMutable()->ledstrip_brightness = brightness;
#endif
}
#endif
