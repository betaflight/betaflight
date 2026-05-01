/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ESP32-S3 minimal timer framework.
 *
 * The ESP32 uses LEDC for PWM and RMT for DShot/LED, not general-purpose
 * timers. This file provides the empty timerHardware array and stub
 * functions required when USE_TIMER is defined, enabling shared code
 * paths (cli.c, config.c) that reference timer functions.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TIMER

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/timer.h"

// Empty timer hardware array - no pins mapped to timers on ESP32.
// All PWM output is handled by LEDC/RMT peripherals directly.
const timerHardware_t fullTimerHardware[1] = { { 0 } };

void timerInit(void)
{
    // NOOP - ESP32 uses LEDC/RMT instead of general-purpose timers
}

const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex)
{
    UNUSED(ioTag);
    UNUSED(timerIndex);
    return NULL;
}

int8_t timerGetTIMNumber(const timerHardware_t *timHw)
{
    UNUSED(timHw);
    return -1;
}

int8_t timerGetNumberByIndex(uint8_t index)
{
    UNUSED(index);
    return -1;
}

int8_t timerGetIndexByNumber(uint8_t number)
{
    UNUSED(number);
    return -1;
}

#endif // USE_TIMER
