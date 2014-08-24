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

#include "config/runtime_config.h"

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;

// each entry in the array is a bitmask, 3 bits per aux channel (only aux 1 to 4), aux1 is first, each bit corresponds to an rc channel reading
// bit 1 - stick LOW
// bit 2 - stick MIDDLE
// bit 3 - stick HIGH
// an option is enabled when ANY channel has an appropriate reading corresponding to the bit.
// an option is disabled when NO channel has an appropriate reading corresponding to the bit.
// example: 110000000001 - option is only enabled when AUX1 is LOW or AUX4 is MEDIUM or HIGH.
uint8_t rcOptions[CHECKBOX_ITEM_COUNT];

static uint32_t enabledSensors = 0;

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}
