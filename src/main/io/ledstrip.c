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

#include <common/maths.h>

#include "drivers/light_ws2811strip.h"
#include "drivers/system.h"

#include "sensors/battery.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "rx/rx.h"
#include "io/rc_controls.h"

#include "io/ledstrip.h"

#define LED_RED    {255, 0,   0  }
#define LED_GREEN  {0,   255, 0  }
#define LED_BLUE   {0,   0,   255}
#define LED_CYAN   {0,   255, 255}
#define LED_YELLOW {255, 255, 0  }
#define LED_ORANGE {255, 128, 0  }
#define LED_PINK   {255, 0,   128}
#define LED_PURPLE {192, 64,  255}

static const rgbColor24bpp_t stripOrientation[] =
{
    {LED_GREEN},
    {LED_GREEN},
    {LED_GREEN},
    {LED_GREEN},
    {LED_GREEN},
    {LED_RED},
    {LED_RED},
    {LED_RED},
    {LED_RED},
    {LED_RED}
};

static const rgbColor24bpp_t stripHorizon[] =
{
    {LED_BLUE},
    {LED_BLUE},
    {LED_BLUE},
    {LED_BLUE},
    {LED_BLUE},
    {LED_YELLOW},
    {LED_YELLOW},
    {LED_YELLOW},
    {LED_YELLOW},
    {LED_YELLOW}
};

static const rgbColor24bpp_t stripAngle[] =
{
    {LED_CYAN},
    {LED_CYAN},
    {LED_CYAN},
    {LED_CYAN},
    {LED_CYAN},
    {LED_YELLOW},
    {LED_YELLOW},
    {LED_YELLOW},
    {LED_YELLOW},
    {LED_YELLOW}
};

static const rgbColor24bpp_t stripMag[] =
{
    {LED_PURPLE},
    {LED_PURPLE},
    {LED_PURPLE},
    {LED_PURPLE},
    {LED_PURPLE},
    {LED_ORANGE},
    {LED_ORANGE},
    {LED_ORANGE},
    {LED_ORANGE},
    {LED_ORANGE}
};

static const rgbColor24bpp_t stripHeadfree[] =
{
    {LED_PINK},
    {LED_PINK},
    {LED_PINK},
    {LED_PINK},
    {LED_PINK},
    {LED_ORANGE},
    {LED_ORANGE},
    {LED_ORANGE},
    {LED_ORANGE},
    {LED_ORANGE}
};

static const rgbColor24bpp_t stripReds[] =
{
    {{ 32,   0,   0}},
    {{ 96,   0,   0}},
    {{160,   0,   0}},
    {{224,   0,   0}},
    {{255,   0,   0}},
    {{255,   0,   0}},
    {{224,   0,   0}},
    {{160,   0,   0}},
    {{ 96,   0,   0}},
    {{ 32,   0,   0}},
};

uint32_t nextIndicatorFlashAt = 0;
uint32_t nextBatteryFlashAt = 0;

#define LED_STRIP_10HZ ((1000 * 1000) / 10)
#define LED_STRIP_5HZ ((1000 * 1000) / 5)

void updateLedStrip(void)
{
    if (!isWS2811LedStripReady()) {
        return;
    }

    uint32_t now = micros();

    bool indicatorFlashNow = (int32_t)(now - nextIndicatorFlashAt) >= 0L;
    bool batteryFlashNow = (int32_t)(now - nextBatteryFlashAt) >= 0L;

    if (!(batteryFlashNow || indicatorFlashNow)) {
        return;
    }

    static uint8_t indicatorFlashState = 0;
    static uint8_t batteryFlashState = 0;

    static const rgbColor24bpp_t *flashColor;

    // LAYER 1

    if (f.ARMED) {
        setStripColors(stripOrientation);
    } else {
        setStripColors(stripReds);
    }

    if (f.HEADFREE_MODE) {
        setStripColors(stripHeadfree);
#ifdef MAG
    } else if (f.MAG_MODE) {
        setStripColors(stripMag);
#endif
    } else if (f.HORIZON_MODE) {
        setStripColors(stripHorizon);
    } else if (f.ANGLE_MODE) {
        setStripColors(stripAngle);
    }

    // LAYER 2

    if (batteryFlashNow) {
        nextBatteryFlashAt = now + LED_STRIP_10HZ;

        if (batteryFlashState == 0) {
            batteryFlashState = 1;
        } else {
            batteryFlashState = 0;
        }
    }

    if (batteryFlashState == 1 && feature(FEATURE_VBAT) && shouldSoundBatteryAlarm()) {
        setStripColor(&black);
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

    if (indicatorFlashState == 0) {
        flashColor = &orange;
    } else {
        flashColor = &black;
    }
    if (rcCommand[ROLL] < -50) {
        setLedColor(0, flashColor);
        setLedColor(9, flashColor);
    }
    if (rcCommand[ROLL] > 50) {
        setLedColor(4, flashColor);
        setLedColor(5, flashColor);
    }
    if (rcCommand[PITCH] > 50) {
        setLedColor(0, flashColor);
        setLedColor(4, flashColor);
    }
    if (rcCommand[PITCH] < -50) {
        setLedColor(5, flashColor);
        setLedColor(9, flashColor);
    }

    ws2811UpdateStrip();
}
