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

#pragma once

#include "pg/pg.h"
#include "drivers/io_types.h"
#include "common/utils.h"

#define STATUS_LED_COUNT 3

// Helpful macros

#define LED0_TOGGLE              ledToggle(0)
#define LED0_OFF                 ledSet(0, false)
#define LED0_ON                  ledSet(0, true)

#define LED1_TOGGLE              ledToggle(1)
#define LED1_OFF                 ledSet(1, false)
#define LED1_ON                  ledSet(1, true)

#define LED2_TOGGLE              ledToggle(2)
#define LED2_OFF                 ledSet(2, false)
#define LED2_ON                  ledSet(2, true)

// use dummy functions for unittest
#if defined(UNIT_TEST) || defined(USE_VIRTUAL_LED)

// ledInit is mising intentionally
static inline void ledToggle(int led) { UNUSED(led); }
static inline void ledSet(int led, bool state) { UNUSED(led); UNUSED(state); }

#else

typedef struct statusLedConfig_s {
    ioTag_t ioTags[STATUS_LED_COUNT];
    uint8_t inversion;
} statusLedConfig_t;

PG_DECLARE(statusLedConfig_t, statusLedConfig);

void ledInit(const statusLedConfig_t *statusLedConfig);
void ledToggle(int led);
void ledSet(int led, bool state);

#endif
