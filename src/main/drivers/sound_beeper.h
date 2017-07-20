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

#pragma once

#include "drivers/io_types.h"

#ifdef BEEPER
#define BEEP_TOGGLE              systemBeepToggle()
#define BEEP_OFF                 systemBeep(false)
#define BEEP_ON                  systemBeep(true)
#else
#define BEEP_TOGGLE do {} while (0)
#define BEEP_OFF    do {} while (0)
#define BEEP_ON     do {} while (0)
#endif

typedef struct beeperDevConfig_s {
    ioTag_t ioTag;
    unsigned isInverted : 1;
    unsigned isOD : 1;
} beeperDevConfig_t;

void systemBeep(bool on);
void systemBeepToggle(void);
void beeperInit(const beeperDevConfig_t *beeperConfig);

