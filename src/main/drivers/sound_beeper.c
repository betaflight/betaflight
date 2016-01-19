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
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "system.h"
#include "gpio.h"

#include "sound_beeper.h"


#ifdef BEEPER

void (*systemBeepPtr)(bool onoff) = NULL;

static uint16_t beeperPin;

static void beepNormal(bool onoff)
{
    if (onoff) {
        digitalLo(BEEP_GPIO, beeperPin);
    } else {
        digitalHi(BEEP_GPIO, beeperPin);
    }
}

static void beepInverted(bool onoff)
{
    if (onoff) {
        digitalHi(BEEP_GPIO, beeperPin);
    } else {
        digitalLo(BEEP_GPIO, beeperPin);
    }
}
#endif

void systemBeep(bool onoff)
{
#ifndef BEEPER
    UNUSED(onoff);
#else
    systemBeepPtr(onoff);
#endif
}

#ifdef CC3D
void beeperInit(beeperConfig_t *config, uint8_t use_buzzer_p6) {
	if (use_buzzer_p6) {
        beeperPin = Pin_2;
	} else {
        beeperPin = BEEP_PIN;
	}
#else
void beeperInit(beeperConfig_t *config) {
	beeperPin = BEEP_PIN;
#endif
#ifndef BEEPER
    UNUSED(config);
#else
    initBeeperHardware(config);
    if (config->isInverted)
        systemBeepPtr = beepInverted;
    else
        systemBeepPtr = beepNormal;
    BEEP_OFF;
#endif
}
